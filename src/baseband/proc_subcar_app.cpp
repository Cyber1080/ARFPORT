#include "proc_subcar_app.hpp"
#include "sine_table_int8.hpp"
#include "portapack_shared_memory.hpp"
#include "event_m4.hpp"
#include "utility.hpp"

// ── Lifecycle ────────────────────────────────────────────────────────────────

SubCarAppProcessor::SubCarAppProcessor() {
    protoList = new SubCarProtos();
    rx_thread.start();   // default: RX mode
}

SubCarAppProcessor::~SubCarAppProcessor() {
    if (protoList) { delete protoList; protoList = nullptr; }
}

void SubCarAppProcessor::switch_mode(uint8_t new_mode) {
    if (new_mode == current_mode) return;
    if (current_mode == 0) {
        rx_thread.stop();
    } else {
        tx_thread.stop();
        tx_configured = false;
        stream.reset();
    }
    current_mode = new_mode;
    if (current_mode == 0) {
        rx_configured = false;
        rx_thread.start();
    } else {
        tx_thread.start();
    }
}

// ── Message handler ───────────────────────────────────────────────────────────

void SubCarAppProcessor::on_message(const Message* const message) {
    switch (message->id) {
        // RX messages
        case Message::ID::SubGhzFPRxConfigure:
            configure_rx(*reinterpret_cast<const SubGhzFPRxConfigureMessage*>(message));
            break;
        // TX messages
        case Message::ID::ReplayConfig:
            tx_configured = false;
            configure_replay(*reinterpret_cast<const ReplayConfigMessage*>(message));
            break;
        case Message::ID::FIFOData:
            tx_configured = true;
            txprogress_message.progress = -4;
            shared_memory.application_queue.push(txprogress_message);
            break;
        case Message::ID::StreamTXConfiguration:
            configure_streamtx(*reinterpret_cast<const StreamTXConfigurationMessage*>(message));
            break;
        // Mode switch
        case Message::ID::SubCarAppMode:
            switch_mode(reinterpret_cast<const SubCarAppModeMessage*>(message)->mode);
            break;
        default:
            break;
    }
}

// ── execute() dispatcher ─────────────────────────────────────────────────────

void SubCarAppProcessor::execute(const buffer_c8_t& buffer) {
    if (current_mode == 0)
        execute_rx(buffer);
    else
        execute_tx(buffer);
}

// ── RX ───────────────────────────────────────────────────────────────────────

void SubCarAppProcessor::configure_rx(const SubGhzFPRxConfigureMessage& msg) {
    if (modulation != msg.modulation) {
        if (protoList) delete protoList;
        protoList = new SubCarProtos();
    }
    modulation   = msg.modulation;
    baseband_fs  = msg.sampling_rate;
    rx_thread.set_sampling_rate(baseband_fs);
    nsPerDecSamp = 1'000'000'000 / baseband_fs * 8;
    decim_0.configure(taps_200k_wfm_decim_0.taps);
    decim_1.configure(taps_200k_wfm_decim_1.taps);
    rx_configured = true;
}

void SubCarAppProcessor::execute_rx(const buffer_c8_t& buffer) {
    if (!rx_configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    feed_channel_stats(decim_1_out);

    for (size_t i = 0; i < decim_1_out.count; i++) {
        threshold = (low_estimate + high_estimate) / 2;
        int32_t const hysteresis = threshold / 8;
        int16_t re = decim_1_out.p[i].real();
        int16_t im = decim_1_out.p[i].imag();
        uint32_t mag = (((uint32_t)re * (uint32_t)re) + ((uint32_t)im * (uint32_t)im)) >> 10;

        if (modulation == 0) {
            int32_t const ook_low_delta = mag - low_estimate;
            bool meashl = currentHiLow;
            if (sig_state == STATE_IDLE) {
                if (mag > (threshold + hysteresis)) {
                    meashl = true; sig_state = STATE_PULSE; numg = 0;
                } else {
                    meashl = false;
                    low_estimate += ook_low_delta / OOK_EST_LOW_RATIO;
                    low_estimate += ((ook_low_delta > 0) ? 1 : -1);
                    high_estimate = 1.35 * low_estimate;
                    high_estimate = std::max(high_estimate, min_high_level);
                    high_estimate = std::min(high_estimate, (uint32_t)OOK_MAX_HIGH_LEVEL);
                }
            } else if (sig_state == STATE_PULSE) {
                ++numg; if (numg > 100) numg = 100;
                if (mag < (threshold - hysteresis)) {
                    sig_state = (numg < 3) ? STATE_GAP : STATE_GAP_START;
                    numg = 0; meashl = false;
                } else {
                    high_estimate += mag / OOK_EST_HIGH_RATIO - high_estimate / OOK_EST_HIGH_RATIO;
                    high_estimate = std::max(high_estimate, min_high_level);
                    high_estimate = std::min(high_estimate, (uint32_t)OOK_MAX_HIGH_LEVEL);
                    meashl = true;
                }
            } else if (sig_state == STATE_GAP_START) {
                ++numg;
                if (mag > (threshold + hysteresis)) { sig_state = STATE_PULSE; meashl = true; }
                else if (numg >= 3) { sig_state = STATE_GAP; meashl = false; }
            } else if (sig_state == STATE_GAP) {
                ++numg;
                if (mag > (threshold + hysteresis)) { numg = 0; sig_state = STATE_PULSE; meashl = true; }
                else meashl = false;
            }
            if (meashl == currentHiLow && currentDuration < 30'000'000) {
                currentDuration += nsPerDecSamp;
            } else {
                if (currentDuration >= 30'000'000) sig_state = STATE_IDLE;
                if (protoList) protoList->feed(currentHiLow, currentDuration / 1000);
                currentDuration = nsPerDecSamp;
                currentHiLow = meashl;
            }
        }
        if (modulation == 1) {
            int32_t discrim = ((int32_t)im * fm_state.last_re) - ((int32_t)re * fm_state.last_im);
            fm_state.last_re = re; fm_state.last_im = im;
            fm_state.smoothed_discrim += (discrim - fm_state.smoothed_discrim) >> 4;
            if (mag > (threshold / 2)) {
                const int32_t fm_hysteresis = 2000;
                bool new_level = fm_state.current_logic_level;
                if (fm_state.smoothed_discrim > fm_hysteresis) new_level = true;
                else if (fm_state.smoothed_discrim < -fm_hysteresis) new_level = false;
                if (new_level == fm_state.current_logic_level) {
                    fm_state.buffer_count++;
                } else {
                    int32_t duration_us = (fm_state.buffer_count * nsPerDecSamp) / 1000;
                    if (duration_us > 15 && protoList)
                        protoList->feed(fm_state.current_logic_level, duration_us);
                    fm_state.current_logic_level = new_level;
                    fm_state.buffer_count = 1;
                }
            } else {
                fm_state.buffer_count = 0;
            }
        }
    }
}

// ── TX ───────────────────────────────────────────────────────────────────────

void SubCarAppProcessor::write_tx_sample(const buffer_c8_t& buffer, bool bit_value, size_t i) {
    int8_t re = 0, im = 0;
    if (tx_mode == 0) {
        if (bit_value) {
            phase = (phase + 200);
            sphase = phase + (64 << 18);
            re = sine_table_i8[(sphase & 0x03FC0000) >> 18];
            im = sine_table_i8[(phase  & 0x03FC0000) >> 18];
        }
    } else if (tx_mode == 1) {
        if (bit_value) phase += deviation_delta;
        else           phase -= deviation_delta;
        sphase = phase + (64 << 18);
        re = sine_table_i8[(sphase & 0x03FC0000) >> 18];
        im = sine_table_i8[(phase  & 0x03FC0000) >> 18];
    }
    buffer.p[i] = {re, im};
}

void SubCarAppProcessor::execute_tx(const buffer_c8_t& buffer) {
    if (!tx_configured || !stream) return;
    for (size_t i = 0; i < buffer.count; i++) {
        if (rem_samples <= curr_samples) {
            int32_t sample = -13346;
            rem_samples = 0; curr_samples = 0;
            size_t readed = stream->read(&sample, 4);
            if (readed == 0) {
                txprogress_message.progress = -10;
                shared_memory.application_queue.push(txprogress_message);
            } else {
                if (sample == endsignals[readerrs + 1]) {
                    readerrs++;
                    if (readerrs == 2) {
                        tx_configured = false;
                        txprogress_message.done = true;
                        txprogress_message.progress = 100;
                        curr_hilow = false;
                        shared_memory.application_queue.push(txprogress_message);
                    }
                } else {
                    if (sample < 0) { rem_samples = SUBCAR_APP_TX_FS * ((-sample) / 1000000.0); curr_hilow = false; }
                    else            { rem_samples = SUBCAR_APP_TX_FS * (sample    / 1000000.0); curr_hilow = true;  }
                }
            }
        }
        ++curr_samples;
        write_tx_sample(buffer, curr_hilow, i);
    }
}

void SubCarAppProcessor::configure_replay(const ReplayConfigMessage& msg) {
    if (msg.config) {
        txprogress_message.progress = -2;
        shared_memory.application_queue.push(txprogress_message);
        stream = std::make_unique<StreamOutput>(msg.config);
        RequestSignalMessage sig{RequestSignalMessage::Signal::FillRequest};
        shared_memory.application_queue.push(sig);
    } else {
        txprogress_message.progress = -3;
        shared_memory.application_queue.push(txprogress_message);
        stream.reset();
    }
}

void SubCarAppProcessor::configure_streamtx(const StreamTXConfigurationMessage& msg) {
    tx_mode = msg.mode;
    deviation = msg.deviation;
    uint64_t big_calc = (uint64_t)msg.deviation << 26;
    deviation_delta = (uint32_t)(big_calc / SUBCAR_APP_TX_FS);
}

int main() {
    EventDispatcher event_dispatcher{std::make_unique<SubCarAppProcessor>()};
    event_dispatcher.run();
    return 0;
}
