#include "ui_subghztx.hpp"

using namespace portapack;
using namespace ui;

namespace ui::external_app::subghztx {

// =====================================================================
// SubGhzTxView
// =====================================================================

void SubGhzTxView::focus() {
    button_browse.focus();
}

void SubGhzTxView::set_ready() {
    ready_signal = true;
}

SubGhzTxView::SubGhzTxView(NavigationView& nav)
    : nav_{nav} {
    add_children({&field_frequency, &tx_view,
                  &text_filename, &text_protocol, &text_status,
                  &button_browse, &button_startstop});

    button_browse.on_select = [this](Button&) {
        auto v = nav_.push<FileLoadView>(".sub");
        v->push_dir(subghz_dir);
        v->on_changed = [this](std::filesystem::path p) {
            on_file_changed(p);
        };
    };

    button_startstop.on_select = [this](Button&) {
        if (is_running) {
            // Button held during TX — flag to repeat after this send completes.
            should_repeat_ = true;
        } else {
            should_repeat_ = false;
            start();
        }
    };
}

SubGhzTxView::~SubGhzTxView() {
    stop();
}

bool SubGhzTxView::on_file_changed(std::filesystem::path new_path) {
    stop();
    pulses.clear();
    filename = "";
    parsed_valid = false;

    auto _opt = subfile::parse_file(new_path);
    parsed_valid = _opt.has_value();
    if (!parsed_valid) {
        text_filename.set("File: parse error");
        text_protocol.set("Protocol: -");
        text_status.set("Status: parse fail");
        return false;
    }
    parsed = std::move(*_opt);

    text_filename.set("File: " + new_path.filename().string());
    text_protocol.set("Protocol: " + parsed.protocol +
                      " @ " + to_string_dec_uint(parsed.frequency_hz / 1000) + " kHz");

    // dispatch() handles RAW (returns raw_data), BinRAW, and key-based protocols.
    auto disp = subfile::dispatch(parsed, /*repeat=*/5);
    if (!disp.has_value() || disp->pulses.empty()) {
        text_status.set("Status: unsupported protocol");
        return false;
    }
    pulses = std::move(disp->pulses);

    fm_mode = (parsed.preset == subfile::Preset::FM238 ||
               parsed.preset == subfile::Preset::FM12K ||
               parsed.preset == subfile::Preset::FM476);

    transmitter_model.set_target_frequency(parsed.frequency_hz);
    filename = new_path;
    text_status.set("Status: ready, " + to_string_dec_uint(pulses.size()) + " pulses");
    return true;
}

bool SubGhzTxView::start() {
    if (filename.empty()) {
        text_status.set("Status: load a file first");
        return false;
    }

    baseband::run_prepared_image(portapack::memory::map::m4_code.base());
    baseband::set_subcar_app_mode(1);  // switch ARFC combined baseband to TX mode (no-op on standalone POSK)
    transmitter_model.set_target_frequency(parsed.frequency_hz);
    transmitter_model.set_sampling_rate(SUBGHZTX_OOK_SAMPLERATE);
    transmitter_model.set_baseband_bandwidth(1'750'000);
    transmitter_model.set_rf_amp(true);
    transmitter_model.set_tx_gain(35);
    transmitter_model.enable();
    baseband::set_bitstream_config(SUBGHZTX_FM_DEVIATION, fm_mode ? 1 : 0);

    button_startstop.set_text("Stop");

    auto done_cb = [](uint32_t) {
        TXProgressMessage m{};
        m.done = true;
        EventDispatcher::send_message(m);
    };

    if (pulses.empty()) { stop(); return false; }
    text_status.set(fm_mode ? "Status: TX (FM)" : "Status: TX (AM)");
    play_thread = std::make_unique<PulsePlayThread>(
        pulses, 1024, 4, &ready_signal, done_cb);

    is_running = true;
    return true;
}

void SubGhzTxView::stop() {
    // Shutdown baseband FIRST — unblocks BufferExchange::get() in play threads
    // so chThdWait() in their destructors doesn't deadlock.
    transmitter_model.disable();
    baseband::shutdown();
    if (play_thread) play_thread.reset();
    is_running = false;
    ready_signal = false;
    button_startstop.set_text("Start");
    text_status.set("Status: idle");
}

void SubGhzTxView::on_tx_progress(bool done) {
    if (done && is_running) {
        bool repeat = should_repeat_;
        should_repeat_ = false;
        stop();
        if (repeat) start();
    }
}

// =====================================================================
// PulsePlayThread — streams in-memory pulse vector to baseband.
// =====================================================================

PulsePlayThread::PulsePlayThread(std::vector<int32_t> pulses,
                                 size_t read_size, size_t buffer_count,
                                 bool* ready_signal,
                                 std::function<void(uint32_t)> done_cb)
    : config{read_size, buffer_count},
      pulses_{std::move(pulses)},
      ready_sig_{ready_signal},
      done_cb_{std::move(done_cb)} {
    thread_ = chThdCreateFromHeap(nullptr, 4 * 1024, NORMALPRIO,
                                  PulsePlayThread::static_fn, this);
}

PulsePlayThread::~PulsePlayThread() {
    if (thread_) {
        chThdTerminate(thread_);
        if (thread_->p_refs) chThdWait(thread_);
        thread_ = nullptr;
    }
}

msg_t PulsePlayThread::static_fn(void* arg) {
    auto self = static_cast<PulsePlayThread*>(arg);
    auto rc = self->run();
    if (self->done_cb_) self->done_cb_(rc);
    chThdExit(0);
    return 0;
}

uint32_t PulsePlayThread::run() {
    struct BasebandReplay {
        BasebandReplay(ReplayConfig* c) { baseband::replay_start(c); }
    } replay{&config};
    BufferExchange buffers{&config};

    // Wait for FIFO to be allocated, signaled by ready_sig_.
    while (!(*ready_sig_) && !chThdShouldTerminate()) chThdSleep(100);

    size_t cursor = 0;

    // Prefill phase.
    while (!buffers.empty() && !chThdShouldTerminate()) {
        auto buf = buffers.get_prefill();
        if (buf == nullptr) { buffers.put_app(buf); continue; }
        size_t slots = config.read_size / 4;
        auto out = reinterpret_cast<int32_t*>(buf->data());
        for (size_t i = 0; i < slots; i++) {
            out[i] = (cursor < pulses_.size()) ? pulses_[cursor++] : 0;
        }
        buf->set_size(config.read_size);
        buffers.put(buf);
    }
    baseband::set_fifo_data(nullptr);
    // Don't return early — wait for M4 to consume the prefill data.

    // Main streaming — drain zero buffers until M4 has played all signal data.
    // buffers.get() blocks until M4 returns one consumed buffer; after buffer_count
    // drain iterations all prefill data has been played by the M4.
    size_t drain = 0;
    while (!chThdShouldTerminate()) {
        auto buf = buffers.get();
        size_t slots = buf->capacity() / 4;
        auto out = reinterpret_cast<int32_t*>(buf->data());
        for (size_t i = 0; i < slots; i++) {
            out[i] = (cursor < pulses_.size()) ? pulses_[cursor++] : 0;
        }
        buf->set_size(buf->capacity());
        buffers.put(buf);
        if (cursor >= pulses_.size()) {
            if (++drain >= config.buffer_count) return END_OF_PULSES;
        }
    }
    return TERMINATED;
}


}  // namespace ui::external_app::subghztx
