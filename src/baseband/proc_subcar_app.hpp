#pragma once
/*
 * SubCar App — Combined RX/TX M4 baseband processor (tag: ARFC)
 * RX mode: OOK/FM demodulation + SubCarProtos decoder (same as PSCD)
 * TX mode: OOK/2FSK stream output (same as POSK/bintstream)
 * Mode switch via SubCarAppModeMessage; threads swap Direction at runtime.
 */

#ifndef __PROC_SUBCAR_APP_H__
#define __PROC_SUBCAR_APP_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"
#include "message.hpp"
#include "dsp_decimate.hpp"
#include "stream_output.hpp"
#include <array>
#include <memory>

#pragma GCC push_options
#pragma GCC optimize("Os")
#include "fprotos/subcarprotos.hpp"
#pragma GCC pop_options

#define OOK_EST_HIGH_RATIO 3
#define OOK_EST_LOW_RATIO  5
#define OOK_MAX_HIGH_LEVEL 450000
#define SUBCAR_APP_RX_FS   4000000U
#define SUBCAR_APP_TX_FS   2280000U
#define SUBCAR_APP_FM_DEV  60000U

class SubCarAppProcessor : public BasebandProcessor {
   public:
    SubCarAppProcessor();
    ~SubCarAppProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    // ── Mode ────────────────────────────────────────────────────────────────
    uint8_t current_mode = 0;  // 0=RX, 1=TX
    void switch_mode(uint8_t new_mode);

    // ── RX state (mirrors SubCarProcessor) ──────────────────────────────────
    enum RxState { STATE_IDLE = 0, STATE_PULSE, STATE_GAP_START, STATE_GAP } sig_state = STATE_IDLE;
    uint32_t low_estimate  = 100;
    uint32_t high_estimate = 12000;
    uint32_t min_high_level = 10;
    uint8_t  numg = 0;
    size_t   baseband_fs = SUBCAR_APP_RX_FS;
    uint32_t nsPerDecSamp = 0;
    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{dst.data(), dst.size()};
    dsp::decimate::FIRC8xR16x24FS4Decim4 decim_0{};
    dsp::decimate::FIRC16xR16x16Decim2   decim_1{};
    uint32_t currentDuration = 0;
    uint32_t threshold = 0x0630;
    bool     currentHiLow = false;
    bool     rx_configured = false;
    struct DemodFMState {
        bool     current_logic_level = false;
        uint32_t buffer_count = 0;
        int16_t  last_re = 0;
        int16_t  last_im = 0;
        int32_t  smoothed_discrim = 0;
    } fm_state{};
    uint8_t modulation = 0;
    FProtoListGeneral* protoList = nullptr;
    void configure_rx(const SubGhzFPRxConfigureMessage& msg);
    void execute_rx(const buffer_c8_t& buffer);

    // ── TX state (mirrors BinaryTimedProcessorStreamed) ──────────────────────
    uint32_t rem_samples = 0;
    uint32_t curr_samples = 0;
    bool     curr_hilow = false;
    uint32_t phase{0}, sphase{0};
    bool     tx_configured = false;
    uint8_t  tx_mode = 0;
    uint32_t deviation = SUBCAR_APP_FM_DEV;
    uint32_t deviation_delta = (SUBCAR_APP_FM_DEV * 4294967296ULL) / SUBCAR_APP_TX_FS;
    int32_t  endsignals[3] = {0, 42069, 613379};
    uint8_t  readerrs = 0;
    std::unique_ptr<StreamOutput> stream{};
    TXProgressMessage txprogress_message{};
    void write_tx_sample(const buffer_c8_t& buffer, bool bit_value, size_t i);
    void execute_tx(const buffer_c8_t& buffer);
    void configure_replay(const ReplayConfigMessage& msg);
    void configure_streamtx(const StreamTXConfigurationMessage& msg);

    // ── Threads (only one active at a time) ─────────────────────────────────
    BasebandThread rx_thread{SUBCAR_APP_RX_FS, this, baseband::Direction::Receive,    /*auto_start=*/false};
    BasebandThread tx_thread{SUBCAR_APP_TX_FS, this, baseband::Direction::Transmit,   /*auto_start=*/false};
    RSSIThread rssi_thread{};
};

#endif
