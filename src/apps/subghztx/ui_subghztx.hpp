/*
 * SubGhzTx — protocol-aware Sub-GHz transmit app for Mayhem.
 *
 * Drops into firmware/application/external/subghztx/.
 * Companion to (not replacement for) the existing FlipperTx, which only
 * supports RAW/BinRAW. SubGhzTx adds protocol-aware dispatch via the
 * encoder layer (Steps 4-7), so .sub files with `Protocol: Princeton`,
 * `CAME`, `Holtek`, `Holtek_HT12X`, `SMC5326`, `GateTX`, `Ansonic`,
 * `Intertechno_V3`, `MegaCode`, `KeeLoq`, etc. transmit verbatim from
 * a captured key.
 *
 * Architecture: the View parses the .sub file at load-time, dispatches
 * to the matching encoder, and stores the resulting int32_t pulse
 * vector in memory. The play thread streams from this vector to the
 * baseband, so no per-pulse file I/O on the M4 critical path.
 */

#ifndef __UI_SUBGHZTX_HPP__
#define __UI_SUBGHZTX_HPP__

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_transmitter.hpp"
#include "ui_freq_field.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "ui_fileman.hpp"
#include "baseband_api.hpp"
#include "buffer_exchange.hpp"
#include "flipper_subfile.hpp"
#include "sub_file_parser.hpp"
#include "sub_file_dispatch.hpp"
#include "fproto_encoder_base.hpp"
#include "fproto_encoder_extra.hpp"
#include "fproto_enc_keeloq.hpp"

#include <vector>
#include <memory>

using namespace ui;

namespace ui::external_app::subghztx {

#define SUBGHZTX_OOK_SAMPLERATE 2280000U
#define SUBGHZTX_FM_DEVIATION    60000U

class PulsePlayThread;

class SubGhzTxView : public View {
   public:
    SubGhzTxView(NavigationView& nav);
    ~SubGhzTxView();

    void focus() override;
    std::string title() const override { return "SubGhzTx"; }

   private:
    NavigationView& nav_;
    TxRadioState radio_state_{
        433'920'000, 1'750'000, SUBGHZTX_OOK_SAMPLERATE};

    TxFrequencyField field_frequency{{0, 0}, nav_};
    TransmitterView2 tx_view{{11 * 8, 0}, /*short_ui*/ true};

    app_settings::SettingsManager settings_{
        "tx_subghztx", app_settings::Mode::TX};

    Text text_filename{
        {0, 2 * 16, screen_width, 16}, "File: -"};
    Text text_protocol{
        {0, 3 * 16, screen_width, 16}, "Protocol: -"};
    Text text_status{
        {0, 4 * 16, screen_width, 16}, "Status: idle"};

    Button button_browse{
        {1, 6 * 16, 96, 24}, "Browse"};
    Button button_startstop{
        {110, 6 * 16, 96, 24}, "Start"};

    bool is_running{false};
    bool ready_signal{false};
    bool should_repeat_{false};
    std::filesystem::path filename{};
    subfile::SubFile parsed{};
    bool parsed_valid{false};
    std::vector<int32_t> pulses{};
    bool fm_mode{false};

    std::unique_ptr<PulsePlayThread> play_thread{};

    bool on_file_changed(std::filesystem::path new_path);
    bool start();
    void stop();
    void on_tx_progress(bool done);
    void set_ready();

    MessageHandlerRegistration handler_tx_progress{
        Message::ID::TXProgress,
        [this](const Message* const p) {
            on_tx_progress(static_cast<const TXProgressMessage*>(p)->done);
        }};

    MessageHandlerRegistration handler_fifo_signal{
        Message::ID::RequestSignal,
        [this](const Message* const p) {
            const auto m = static_cast<const RequestSignalMessage*>(p);
            if (m->signal == RequestSignalMessage::Signal::FillRequest) set_ready();
        }};
};

// Streams an in-memory pulse vector to the baseband bit-stream TX image.
class PulsePlayThread {
   public:
    PulsePlayThread(std::vector<int32_t> pulses,
                    size_t read_size,
                    size_t buffer_count,
                    bool* ready_signal,
                    std::function<void(uint32_t)> done_cb);
    ~PulsePlayThread();

    PulsePlayThread(const PulsePlayThread&) = delete;
    PulsePlayThread& operator=(const PulsePlayThread&) = delete;

    enum { END_OF_PULSES = 0, TERMINATED = 1 };

   private:
    ReplayConfig config;
    std::vector<int32_t> pulses_;
    bool* ready_sig_;
    std::function<void(uint32_t)> done_cb_;
    Thread* thread_{nullptr};

    static msg_t static_fn(void* arg);
    uint32_t run();
};


}  // namespace ui::external_app::subghztx

#endif
