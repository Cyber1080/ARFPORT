/*
 * Copyright (C) 2026 HTotoo
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
/*
   This and The other files related to this is based on a lot of great people's work. https://github.com/RocketGod-git/ProtoPirate Check the repo, and the credits inside.
*/

#ifndef __UI_SubCar_H__
#define __UI_SubCar_H__

#define SD_NO_SERIAL 0xFFFFFFFF
#define SD_NO_BTN 0xFF
#define SD_NO_CNT 0xFF

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_freq_field.hpp"
#include "app_settings.hpp"
#include "radio_state.hpp"
#include "utility.hpp"
#include "log_file.hpp"
#include "recent_entries.hpp"
#include "baseband_api.hpp"
#include "buffer_exchange.hpp"

#include "../../baseband/fprotos/subcartypes.hpp"

#include <vector>
#include <functional>

using namespace ui;

namespace ui::subcarrx {

#define SUBCAR_TX_SAMPLERATE 2280000U
#define SUBCAR_TX_DEVIATION  60000U

// Streams an in-memory signed-us pulse vector to the combined ARFC baseband TX.
class SubCarPulsePlayThread {
   public:
    SubCarPulsePlayThread(std::vector<int32_t> pulses, size_t read_size, size_t buffer_count,
                          bool* ready_signal, std::function<void(uint32_t)> done_cb);
    ~SubCarPulsePlayThread();
    SubCarPulsePlayThread(const SubCarPulsePlayThread&) = delete;
    SubCarPulsePlayThread& operator=(const SubCarPulsePlayThread&) = delete;
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

struct SubCarRecentEntry {
    using Key = uint64_t;
    static constexpr Key invalid_key = 0x0fffffff;
    uint8_t sensorType = FPC_Invalid;
    uint16_t bits = 0;
    uint16_t age = 0;  // updated on each seconds, show how long the signal was last seen
    uint64_t data = 0;
    uint64_t data2 = 0;
    SubCarRecentEntry() {}
    SubCarRecentEntry(
        uint8_t sensorType,
        uint64_t data = 0,
        uint64_t data2 = 0,
        uint16_t bits = 0)
        : sensorType{sensorType},
          bits{bits},
          data{data},
          data2{data2} {
    }
    Key key() const {
        return (data ^ ((static_cast<uint64_t>(sensorType) & 0xFF) << 0));  // should be optimized...
    }
    void inc_age(int delta) {
        if (UINT16_MAX - delta > age) age += delta;
    }
    void reset_age() {
        age = 0;
    }

    std::string to_csv();
};

class SubCarLogger {
   public:
    Optional<File::Error> append(const std::filesystem::path& filename) {
        return log_file.append(filename);
    }

    void log_data(SubCarRecentEntry& data);
    void write_header() {
        log_file.write_entry(";Type; Bits; Data;");
    }

   private:
    LogFile log_file{};
};
using SubCarRecentEntries = RecentEntries<SubCarRecentEntry>;
using SubCarRecentEntriesView = RecentEntriesView<SubCarRecentEntries>;

class SubCarView : public View {
   public:
    SubCarView(NavigationView& nav);
    ~SubCarView();

    void focus() override;

    std::string title() const override { return "SubCar"; };
    static const char* getSensorTypeName(FPROTO_SUBCAR_SENSOR type);
    static std::string pad_string_with_spaces(int snakes);

   private:
    void on_tick_second();
    void on_data(const SubCarDataMessage* data);

    NavigationView& nav_;
    RxRadioState radio_state_{
        433'920'000 /* frequency */,
        1'750'000 /* bandwidth */,
        4'000'000 /* sampling rate */,
        ReceiverModel::Mode::AMAudio};
    bool logging = false;
    uint8_t modulation = 0;
    app_settings::SettingsManager settings_{
        "rx_subcar",
        app_settings::Mode::RX,
        {
            {"log"sv, &logging},
            {"modulationmode"sv, &modulation},
        }};

    SubCarRecentEntries recent{};

    RFAmpField field_rf_amp{
        {UI_POS_X(13), UI_POS_Y(0)}};
    LNAGainField field_lna{
        {UI_POS_X(15), UI_POS_Y(0)}};
    VGAGainField field_vga{
        {UI_POS_X(18), UI_POS_Y(0)}};
    RSSI rssi{
        {UI_POS_X(21), 0, UI_POS_WIDTH_REMAINING(24), 4}};
    Channel channel{
        {UI_POS_X(21), 5, UI_POS_WIDTH_REMAINING(24), 4},
    };
    RxFrequencyField field_frequency{
        {UI_POS_X(0), UI_POS_Y(0)},
        nav_};

    SignalToken signal_token_tick_second{};

    Button button_clear_list{
        {UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(7), UI_POS_HEIGHT(2)},
        "Clear"};

    Checkbox check_log{
        {UI_POS_X(8), UI_POS_Y(1)},
        3,
        "Log",
        true};

    Labels labels{
        {{UI_POS_X(15), UI_POS_Y(1)}, "Mode:", Theme::getInstance()->fg_light->foreground},
    };
    ui::OptionsField options_mode{
        {UI_POS_X(22), UI_POS_Y(1)},
        3,
        {{"AM", 0}, {"FM", 1}}};

    static constexpr auto header_height = 3 * 16;

    std::unique_ptr<SubCarLogger> logger{};

    ui::RecentEntriesColumns columns{{
        {"Type", 0},
        {"Bits", 4},
        {"Age", 3},
    }};
    SubCarRecentEntriesView recent_entries_view{columns, recent};

    void on_freqchg(int64_t freq);
    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};

    MessageHandlerRegistration message_handler_packet{
        Message::ID::SubCarData,
        [this](Message* const p) {
            const auto message = static_cast<const SubCarDataMessage*>(p);
            this->on_data(message);
        }};
};

class SubCarRecentEntryDetailView : public View {
   public:
    SubCarRecentEntryDetailView(NavigationView& nav, const SubCarRecentEntry& entry);

    void update_data();
    void focus() override;

   private:
    NavigationView& nav_;
    SubCarRecentEntry entry_{};

    uint32_t serial = 0;
    std::string btn = "";
    uint32_t cnt = SD_NO_CNT;

    Text text_type{{UI_POS_X(0), UI_POS_Y(1), UI_POS_WIDTH(15), UI_POS_HEIGHT(1)}, "?"};
    Text text_id{{UI_POS_X(6), UI_POS_Y(2), UI_POS_WIDTH(10), UI_POS_HEIGHT(1)}, "?"};

    Console console{
        {UI_POS_X(0), UI_POS_Y(4), UI_POS_MAXWIDTH, screen_height - (4 * 16) - 36}};

    Labels labels{
        {{UI_POS_X(0), UI_POS_Y(0)}, "Type:", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(2)}, "Serial: ", Theme::getInstance()->fg_light->foreground},
        {{UI_POS_X(0), UI_POS_Y(3)}, "Data:", Theme::getInstance()->fg_light->foreground},
    };

    Button button_done{
        {screen_width - 96 - 4, screen_height - 32 - 12, 96, 32},
        "Done"};

    Button button_emulate{
        {4, screen_height - 32 - 12, 110, 32},
        "Emulate TX"};

    void parseProtocol();

    // ── Emulate (counter-sync TX) ──────────────────────────────────────────
    bool tx_running{false};
    bool ready_signal{false};
    std::vector<int32_t> tx_pulses{};
    std::unique_ptr<SubCarPulsePlayThread> play_thread{};
    void start_emulate();
    void stop_emulate();
    void on_tx_progress(int progress, bool done);
    void set_ready() {
        ready_signal = true;
    }

    MessageHandlerRegistration handler_tx_progress{
        Message::ID::TXProgress,
        [this](const Message* const p) {
            const auto m = static_cast<const TXProgressMessage*>(p);
            on_tx_progress(m->progress, m->done);
        }};
    MessageHandlerRegistration handler_fifo_signal{
        Message::ID::RequestSignal,
        [this](const Message* const p) {
            const auto m = static_cast<const RequestSignalMessage*>(p);
            if (m->signal == RequestSignalMessage::Signal::FillRequest) set_ready();
        }};
};

}  // namespace ui::subcarrx

#endif /*__UI_SubCar_H__*/
