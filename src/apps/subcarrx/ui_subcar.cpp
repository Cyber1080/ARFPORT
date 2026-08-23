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

#include "ui_subcar.hpp"
#include "audio.hpp"
#include "baseband_api.hpp"
#include "string_format.hpp"
#include "file_path.hpp"
#include "portapack_persistent_memory.hpp"

using namespace portapack;
using namespace ui;

#pragma GCC optimize("Os")  // MEASURE: size-optimize the M4 UI code to fit the 32KB region

namespace ui::subcarrx {

std::string SubCarRecentEntry::to_csv() {
    std::string csv = ";";
    csv += SubCarView::getSensorTypeName((FPROTO_SUBCAR_SENSOR)sensorType);
    csv += ";" + to_string_dec_uint(bits) + ";";
    csv += to_string_hex(data, 64 / 4) + ";" + to_string_hex(data2, 64 / 4);
    return csv;
}

void SubCarLogger::log_data(SubCarRecentEntry& data) {
    log_file.write_entry(data.to_csv());
}

void SubCarRecentEntryDetailView::update_data() {
    // process protocol data
    parseProtocol();
    // set text elements
    text_type.set(SubCarView::getSensorTypeName((FPROTO_SUBCAR_SENSOR)entry_.sensorType));

    text_id.set("0x" + to_string_hex(serial));
    if (entry_.bits > 0) console.writeln("Bits: " + to_string_dec_uint(entry_.bits));
    if (!btn.empty()) console.writeln("Btn: " + btn);
    if (cnt != SD_NO_CNT) console.writeln("Cnt: " + to_string_dec_uint(cnt));
    if (entry_.data != 0) console.writeln("Data : " + to_string_hex(entry_.data));
    if (entry_.data2 != 0) console.writeln("Data2: " + to_string_hex(entry_.data2));
}

SubCarRecentEntryDetailView::SubCarRecentEntryDetailView(NavigationView& nav, const SubCarRecentEntry& entry)
    : nav_{nav},
      entry_{entry} {
    add_children({&button_done,
                  &button_emulate,
                  &text_type,
                  &text_id,
                  &console,
                  &labels});

    button_done.on_select = [&nav](const ui::Button&) {
        nav.pop();
    };
    button_emulate.on_select = [this](const ui::Button&) {
        if (tx_running) { stop_emulate(); return; }
        if (entry_.sensorType == FPC_FORDV0 || entry_.sensorType == FPC_KIAV5) {
            start_emulate();
        } else {
            console.writeln("Emulate: Ford V0 / Kia V5 only");
        }
    };
    update_data();
}

void SubCarRecentEntryDetailView::focus() {
    button_done.focus();
}

// ===================================================================
// Ford V0 counter-sync encoder (ported from D4C1/ARF ford_v0.c)
// ===================================================================
namespace {
static const uint8_t ford_v0_crc_matrix[64] = {
    0xDA, 0xB5, 0x55, 0x6A, 0xAA, 0xAA, 0xAA, 0xD5,
    0xB6, 0x6C, 0xCC, 0xD9, 0x99, 0x99, 0x99, 0xB3,
    0x71, 0xE3, 0xC3, 0xC7, 0x87, 0x87, 0x87, 0x8F,
    0x0F, 0xE0, 0x3F, 0xC0, 0x7F, 0x80, 0x7F, 0x80,
    0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0x80,
    0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
    0x23, 0x12, 0x94, 0x84, 0x35, 0xF4, 0x55, 0x84};

static uint8_t fv0_popcount8(uint8_t x) {
    uint8_t c = 0;
    while (x) { c += x & 1; x >>= 1; }
    return c;
}
static uint8_t fv0_calc_crc(uint8_t* buf) {
    uint8_t crc = 0;
    for (int row = 0; row < 8; row++) {
        uint8_t xs = 0;
        for (int col = 0; col < 8; col++)
            xs ^= (ford_v0_crc_matrix[row * 8 + col] & buf[col + 1]);
        if (fv0_popcount8(xs) & 1) crc |= (1 << row);
    }
    return crc;
}
static uint8_t fv0_crc_for_tx(uint64_t key1, uint8_t chk) {
    uint8_t buf[16] = {0};
    for (int i = 0; i < 8; ++i) buf[i] = (uint8_t)(key1 >> (56 - i * 8));
    buf[8] = chk;
    return fv0_calc_crc(buf) ^ 0x80;
}
static uint8_t fv0_chk_from_buf(uint8_t* buf) {
    uint8_t s = 0;
    for (int i = 1; i <= 7; i++) s += buf[i];
    return s;
}
// Recover serial/button/count from a decoded key1/key2 (inverse of encode).
static void fv0_decode(uint64_t key1, uint16_t key2, uint32_t* serial, uint8_t* button, uint32_t* count) {
    uint8_t buf[13] = {0};
    for (int i = 0; i < 8; ++i) buf[i] = (uint8_t)(key1 >> (56 - i * 8));
    buf[8] = (uint8_t)(key2 >> 8);
    buf[9] = (uint8_t)(key2 & 0xFF);
    uint8_t tmp = buf[8], parity = 0, parity_any = (tmp != 0);
    while (tmp) { parity ^= (tmp & 1); tmp >>= 1; }
    buf[11] = parity_any ? parity : 0;
    uint8_t xb, lim;
    if (buf[11]) { xb = buf[7]; lim = 7; } else { xb = buf[6]; lim = 6; }
    for (int i = 1; i < lim; ++i) buf[i] ^= xb;
    if (buf[11] == 0) buf[7] ^= xb;
    uint8_t ob7 = buf[7];
    buf[7] = (ob7 & 0xAA) | (buf[6] & 0x55);
    uint8_t mixed = (buf[6] & 0xAA) | (ob7 & 0x55);
    buf[6] = mixed;
    uint32_t sle = ((uint32_t)buf[1]) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    *serial = ((sle & 0xFF) << 24) | (((sle >> 8) & 0xFF) << 16) | (((sle >> 16) & 0xFF) << 8) | ((sle >> 24) & 0xFF);
    *button = (buf[5] >> 4) & 0x0F;
    *count = ((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];
}
static void fv0_encode(uint64_t orig_key1, uint32_t serial, uint8_t button, uint32_t count, uint8_t* chk_out, uint64_t* key1_out) {
    uint8_t buf[13] = {0};
    buf[0] = (uint8_t)(orig_key1 >> 56);
    buf[1] = (serial >> 24) & 0xFF; buf[2] = (serial >> 16) & 0xFF;
    buf[3] = (serial >> 8) & 0xFF;  buf[4] = serial & 0xFF;
    buf[5] = (button << 4) | ((count >> 16) & 0x0F);
    buf[6] = (count >> 8) & 0xFF;   buf[7] = count & 0xFF;
    uint8_t checksum = fv0_chk_from_buf(buf);
    uint8_t ob7 = buf[7], ob6 = buf[6];
    buf[7] = (ob7 & 0xAA) | (ob6 & 0x55);
    buf[6] = (ob6 & 0xAA) | (ob7 & 0x55);
    *chk_out = checksum;
    buf[8] = checksum;
    uint8_t tmp = buf[8], parity = 0, parity_any = (tmp != 0);
    while (tmp) { parity ^= (tmp & 1); tmp >>= 1; }
    uint8_t b11 = parity_any ? parity : 0;
    uint8_t xb, lim;
    if (b11) { xb = buf[7]; lim = 7; } else { xb = buf[6]; lim = 6; }
    if (b11 == 0) buf[7] ^= xb;
    for (int i = 1; i < lim; i++) buf[i] ^= xb;
    *key1_out = 0;
    for (int i = 0; i < 8; i++) *key1_out = (*key1_out << 8) | buf[i];
}

// Build the OOK pulse stream (signed us: +high / -low) for counter+1 frame, N repeats.
static std::vector<int32_t> build_ford_v0_pulses(uint64_t key1, uint16_t key2) {
    uint32_t serial = 0, count = 0; uint8_t button = 0;
    fv0_decode(key1, key2, &serial, &button, &count);
    count = (count + 1) & 0xFFFFF;  // counter-sync increment
    uint8_t chk = 0; uint64_t new_key1 = 0;
    fv0_encode(key1, serial, button, count, &chk, &new_key1);
    uint8_t crc = fv0_crc_for_tx(new_key1, chk);
    uint16_t new_key2 = ((uint16_t)chk << 8) | crc;

    const int32_t te_s = 250, te_l = 500, gap = 3500;
    uint64_t k1_inv = ~new_key1;
    uint16_t k2_inv = ~new_key2;
    std::vector<int32_t> p;
    p.reserve(240 * 4);
    for (int rep = 0; rep < 4; rep++) {
        for (int i = 0; i < 20; i++) { p.push_back(+te_s); p.push_back(-te_l); p.push_back(+te_l); }
        p.push_back(+te_s);
        p.push_back(-gap);
        for (int i = 63; i >= 0; i--) {
            if ((k1_inv >> i) & 1) { p.push_back(+te_s); p.push_back(-te_s); }
            else { p.push_back(-te_s); p.push_back(+te_s); }
        }
        for (int i = 15; i >= 0; i--) {
            if ((k2_inv >> i) & 1) { p.push_back(+te_s); p.push_back(-te_s); }
            else { p.push_back(-te_s); p.push_back(+te_s); }
        }
        p.push_back(-20000);  // inter-frame gap
    }
    // End sentinels so the baseband plays the whole frame then reports real done
    // (matches endsignals[1]/[2] in proc_subcar_app). Without these the M4 feed-done
    // stops the transmitter mid-frame -> trimmed signal.
    p.push_back(42069);
    p.push_back(613379);
    return p;
}

// ── Kia V5 (custom mixer cipher; 2FSK 433.92, Manchester 400/800) ─────────────
// Ported from Flipper-ARF/ProtoPirate kia_v5.c. Counter-sync: decode counter,
// +1, re-encrypt, rebuild the 64-bit frame + 2-bit CRC, Manchester-modulate.
static const uint8_t kia_v5_key[8] = {0x53, 0x54, 0x46, 0x52, 0x4b, 0x45, 0x30, 0x30};

static uint64_t kia_bit_reverse_64(uint64_t input) {
    uint64_t output = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t byte = (input >> (i * 8)) & 0xFF, rev = 0;
        for (int b = 0; b < 8; b++)
            if (byte & (1 << b)) rev |= (1 << (7 - b));
        output |= ((uint64_t)rev << ((7 - i) * 8));
    }
    return output;
}

static uint8_t kia_v5_calc_crc(uint64_t data) {
    uint8_t crc = 0;
    for (int i = 63; i >= 0; i--) {
        const uint8_t bit = (data >> i) & 1U;
        const uint8_t shifted_out = (crc >> 1U) & 1U;
        crc = (uint8_t)(((crc & 1U) << 1U) | bit);
        if (shifted_out) crc ^= 3U;
    }
    return (uint8_t)(crc & 3U);
}

static uint16_t kia_v5_mixer_decode(uint32_t encrypted) {
    uint8_t s0 = encrypted & 0xFF, s1 = (encrypted >> 8) & 0xFF;
    uint8_t s2 = (encrypted >> 16) & 0xFF, s3 = (encrypted >> 24) & 0xFF;
    int ri = 1;
    for (size_t i = 0; i < 18; i++) {
        uint8_t r = kia_v5_key[ri] & 0xFF;
        int steps = 8;
        while (steps > 0) {
            uint8_t base;
            if ((s3 & 0x40) == 0) base = (s3 & 0x02) == 0 ? 0x74 : 0x2E;
            else base = (s3 & 0x02) == 0 ? 0x3A : 0x5C;
            if (s2 & 0x08) base = (((base >> 4) & 0x0F) | ((base & 0x0F) << 4)) & 0xFF;
            if (s1 & 0x01) base = ((base & 0x3F) << 2) & 0xFF;
            if (s0 & 0x01) base = (base << 1) & 0xFF;
            uint8_t temp = (s3 ^ s1) & 0xFF;
            s3 = ((s3 & 0x7F) << 1) & 0xFF; if (s2 & 0x80) s3 |= 0x01;
            s2 = ((s2 & 0x7F) << 1) & 0xFF; if (s1 & 0x80) s2 |= 0x01;
            s1 = ((s1 & 0x7F) << 1) & 0xFF; if (s0 & 0x80) s1 |= 0x01;
            s0 = ((s0 & 0x7F) << 1) & 0xFF;
            uint8_t chk = (base ^ (r ^ temp)) & 0xFF; if (chk & 0x80) s0 |= 0x01;
            r = ((r & 0x7F) << 1) & 0xFF;
            steps--;
        }
        ri = (ri - 1) & 0x7;
    }
    return (s0 + (s1 << 8)) & 0xFFFF;
}

static uint32_t kia_v5_mixer_encode(uint32_t serial, uint16_t counter, uint8_t button) {
    uint8_t s0 = (uint8_t)(((serial >> 8) & 0x0FU) | ((button & 0x0FU) << 4));
    uint8_t s1 = (uint8_t)((counter >> 8) & 0xFFU);
    uint8_t s2 = (uint8_t)(serial & 0xFFU);
    uint8_t s3 = (uint8_t)(counter & 0xFFU);
    int ks_idx = 0;
    for (int round_i = 0; round_i < 18; round_i++) {
        uint8_t r = kia_v5_key[ks_idx] & 0xFFU;
        ks_idx = (ks_idx + 1) & 0x07;
        uint8_t running_d = s3;
        for (int step = 0; step < 8; step++) {
            uint8_t base;
            if ((s0 & 0x80U) == 0) base = (s0 & 0x04U) == 0 ? 0x74U : 0x2EU;
            else base = (s0 & 0x04U) == 0 ? 0x3AU : 0x5CU;
            if (s2 & 0x10U) base = (uint8_t)(((base >> 4) & 0x0FU) | ((base & 0x0FU) << 4));
            if (s1 & 0x02U) base = (uint8_t)((base & 0x3FU) << 2);
            uint8_t base_final = base;
            if (running_d & 0x02U) base_final = (uint8_t)((base & 0x7FU) << 1);
            const bool carry_b = (s1 & 0x01U) != 0;
            const bool carry_c = (s2 & 0x01U) != 0;
            const bool carry_a = (s0 & 0x01U) != 0;
            uint8_t new_d = (uint8_t)(running_d >> 1); if (carry_b) new_d |= 0x80U;
            running_d ^= s2;
            s1 = (uint8_t)(s1 >> 1); if (carry_c) s1 |= 0x80U;
            s2 = (uint8_t)(s2 >> 1); if (carry_a) s2 |= 0x80U;
            const uint8_t feedback = (uint8_t)(((running_d ^ r) << 7) ^ base_final);
            s0 = (uint8_t)(s0 >> 1); if (feedback & 0x80U) s0 |= 0x80U;
            r = (uint8_t)(r >> 1);
            running_d = new_d;
        }
        s3 = running_d;
    }
    return ((uint32_t)s0 << 24) | ((uint32_t)s2 << 16) | ((uint32_t)s1 << 8) | (uint32_t)s3;
}

// Build the counter+1 Kia V5 frame as a signed-us Manchester pulse stream.
// frame64_orig = entry_.data2 (the stored 64-bit frame from the decoder).
static std::vector<int32_t> build_kia_v5_pulses(uint64_t frame64_orig) {
    uint64_t yek = kia_bit_reverse_64(frame64_orig);
    uint32_t serial = (uint32_t)((yek >> 32) & 0x0FFFFFFF);
    uint8_t button = (uint8_t)((yek >> 60) & 0x0F);
    uint32_t enc = (uint32_t)(yek & 0xFFFFFFFF);
    uint16_t cnt = (uint16_t)((kia_v5_mixer_decode(enc) + 1) & 0xFFFF);
    uint32_t enc2 = kia_v5_mixer_encode(serial, cnt, button);
    uint64_t yek2 = (yek & 0xFFFFFFFF00000000ULL) | enc2;
    uint64_t frame = kia_bit_reverse_64(yek2);
    uint8_t crc = kia_v5_calc_crc(frame);

    const int32_t s = 400, l = 800;
    std::vector<int32_t> p;
    p.reserve(8 * 320);
    for (int rep = 0; rep < 8; rep++) {
        for (int i = 0; i < 80; i++) { p.push_back(+s); p.push_back(-s); }  // preamble
        p.push_back(-s); p.push_back(+l); p.push_back(-s); p.push_back(+s);  // sync + align
        for (int b = 63; b >= 0; b--) {                                     // 64 data bits MSB-first
            if ((frame >> b) & 1ULL) { p.push_back(-s); p.push_back(+s); }
            else { p.push_back(+s); p.push_back(-s); }
        }
        p.push_back(+s); p.push_back(-s);                                    // separator
        if ((crc >> 1) & 1) { p.push_back(-s); p.push_back(+s); } else { p.push_back(+s); p.push_back(-s); }
        if (crc & 1)        { p.push_back(-s); p.push_back(+s); } else { p.push_back(+s); p.push_back(-s); }
        p.push_back(-s); p.push_back(+s);                                    // end marker
        p.push_back(-20000);                                                 // inter-frame gap
    }
    p.push_back(42069);
    p.push_back(613379);
    return p;
}
}  // namespace

// ── SubCarPulsePlayThread (streams pulse vector to ARFC TX baseband) ──────────
SubCarPulsePlayThread::SubCarPulsePlayThread(std::vector<int32_t> pulses, size_t read_size,
                                             size_t buffer_count, bool* ready_signal,
                                             std::function<void(uint32_t)> done_cb)
    : config{read_size, buffer_count},
      pulses_{std::move(pulses)},
      ready_sig_{ready_signal},
      done_cb_{std::move(done_cb)} {
    thread_ = chThdCreateFromHeap(nullptr, 4 * 1024, NORMALPRIO, SubCarPulsePlayThread::static_fn, this);
}
SubCarPulsePlayThread::~SubCarPulsePlayThread() {
    if (thread_) {
        chThdTerminate(thread_);
        chThdWait(thread_);  // unconditional: reclaims the from-heap thread stack
        thread_ = nullptr;
    }
}
msg_t SubCarPulsePlayThread::static_fn(void* arg) {
    auto self = static_cast<SubCarPulsePlayThread*>(arg);
    auto rc = self->run();
    if (self->done_cb_) self->done_cb_(rc);
    chThdExit(0);
    return 0;
}
uint32_t SubCarPulsePlayThread::run() {
    struct BasebandReplay {
        BasebandReplay(ReplayConfig* c) { baseband::replay_start(c); }
    } replay{&config};
    // Wait for the M0 StreamOutput ctor to populate config->fifo_buffers_* and
    // signal FillRequest BEFORE building BufferExchange, otherwise it caches
    // nullptr FIFO pointers and never delivers buffers (the TXp:-10 empty-stream bug).
    while (!(*ready_sig_) && !chThdShouldTerminate()) chThdSleep(100);
    if (chThdShouldTerminate()) return TERMINATED;
    BufferExchange buffers{&config};
    size_t cursor = 0;
    while (!buffers.empty() && !chThdShouldTerminate()) {
        auto buf = buffers.get_prefill();
        if (buf == nullptr) { buffers.put_app(buf); continue; }
        size_t slots = config.read_size / 4;
        auto out = reinterpret_cast<int32_t*>(buf->data());
        for (size_t i = 0; i < slots; i++) out[i] = (cursor < pulses_.size()) ? pulses_[cursor++] : 0;
        buf->set_size(config.read_size);
        buffers.put(buf);
    }
    baseband::set_fifo_data(nullptr);
    size_t drain = 0;
    while (!chThdShouldTerminate()) {
        auto buf = buffers.get();
        size_t slots = buf->capacity() / 4;
        auto out = reinterpret_cast<int32_t*>(buf->data());
        for (size_t i = 0; i < slots; i++) out[i] = (cursor < pulses_.size()) ? pulses_[cursor++] : 0;
        buf->set_size(buf->capacity());
        buffers.put(buf);
        if (cursor >= pulses_.size()) { if (++drain >= config.buffer_count) return END_OF_PULSES; }
    }
    return TERMINATED;
}

// ── Emulate (counter-sync TX) ────────────────────────────────────────────────
void SubCarRecentEntryDetailView::start_emulate() {
    uint32_t tx_deviation = SUBCAR_TX_DEVIATION;
    uint8_t tx_mode = 0;  // 0 = OOK/AM, 1 = 2FSK/FM
    if (entry_.sensorType == FPC_KIAV5) {
        tx_pulses = build_kia_v5_pulses(entry_.data2);
        tx_mode = 1;               // Kia V5 is 2FSK
        tx_deviation = 47600;      // ~Dev476 (Kia/Hyundai); tune to 433.92 in RX first
        if (tx_pulses.empty()) { console.writeln("Emulate: encode failed"); return; }
        console.writeln("TX Kia V5 cnt+1");
    } else {
        tx_pulses = build_ford_v0_pulses(entry_.data, (uint16_t)entry_.data2);
        if (tx_pulses.empty()) { console.writeln("Emulate: encode failed"); return; }
        console.writeln("TX Ford V0 cnt+1");
    }
    button_emulate.set_text("Stop TX");
    tx_running = true;
    ready_signal = false;
    receiver_model.disable();
    baseband::set_subcar_app_mode(1);  // ARFC -> TX
    transmitter_model.set_target_frequency(receiver_model.target_frequency());
    transmitter_model.set_sampling_rate(SUBCAR_TX_SAMPLERATE);
    transmitter_model.set_baseband_bandwidth(1'750'000);
    transmitter_model.set_rf_amp(true);
    transmitter_model.set_tx_gain(35);
    transmitter_model.enable();
    baseband::set_bitstream_config(tx_deviation, tx_mode);
    auto done_cb = [](uint32_t) {
        TXProgressMessage m{};
        m.done = true;
        EventDispatcher::send_message(m);
    };
    play_thread = std::make_unique<SubCarPulsePlayThread>(tx_pulses, 1024, 4, &ready_signal, done_cb);
}

void SubCarRecentEntryDetailView::stop_emulate() {
    transmitter_model.disable();
    if (play_thread) play_thread.reset();
    baseband::set_subcar_app_mode(0);  // ARFC -> RX
    receiver_model.enable();
    tx_running = false;
    ready_signal = false;
    button_emulate.set_text("Emulate TX");
    std::vector<int32_t>().swap(tx_pulses);  // free the pulse buffer
    console.writeln("Emulate: done");
}

void SubCarRecentEntryDetailView::on_tx_progress(int progress, bool done) {
    // Only stop on the baseband's REAL done (progress 100, after end sentinels).
    // The M4 feed-done arrives as progress 0 and must NOT stop us early (trim).
    (void)progress;
    if (done && progress == 100 && tx_running) stop_emulate();
}

void SubCarView::focus() {
    field_frequency.focus();
}

SubCarView::SubCarView(NavigationView& nav)
    : nav_{nav} {
    add_children({&rssi,
                  &channel,
                  &field_rf_amp,
                  &field_lna,
                  &field_vga,
                  &field_frequency,
                  &button_clear_list,
                  &check_log,
                  &labels,
                  &options_mode,
                  &recent_entries_view});

    baseband::run_image(portapack::spi_flash::image_tag_subcar);
    logger = std::make_unique<SubCarLogger>();

    button_clear_list.on_select = [this](Button&) {
        recent.clear();
        recent_entries_view.set_dirty();
    };
    field_frequency.set_step(10000);
    check_log.on_select = [this](Checkbox&, bool v) {
        logging = v;
        if (logger && logging) {
            logger->append(logs_dir.string() + "/SubCarLOG_" + to_string_timestamp(rtc_time::now()) + ".CSV");
            logger->write_header();
        }
    };
    check_log.set_value(logging);
    const Rect content_rect{0, header_height, screen_width, screen_height - header_height};
    recent_entries_view.set_parent_rect(content_rect);
    recent_entries_view.on_select = [this](const SubCarRecentEntry& entry) {
        nav_.push<SubCarRecentEntryDetailView>(entry);
    };

    options_mode.on_change = [this](size_t, int32_t v) {
        modulation = v;
        chThdSleepMilliseconds(100);  // wait for the baseband thread to process the previous config, to avoid glitchy output when switching modes
        baseband::set_subghzd_config(modulation, receiver_model.sampling_rate());
    };
    signal_token_tick_second = rtc_time::signal_tick_second += [this]() {
        on_tick_second();
    };
    options_mode.set_selected_index(modulation, true);
    receiver_model.enable();
}

void SubCarView::on_tick_second() {
    for (auto& entry : recent) {
        entry.inc_age(1);
    }
    recent_entries_view.set_dirty();
}

void SubCarView::on_data(const SubCarDataMessage* data) {
    SubCarRecentEntry key{data->sensorType, data->data, data->data2, data->bits};
    if (logger && logging) {
        logger->log_data(key);
    }
    auto matching_recent = find(recent, key.key());
    if (matching_recent != std::end(recent)) {
        // Found within. Move to front of list, increment counter.
        (*matching_recent).reset_age();
        recent.push_front(*matching_recent);
        recent.erase(matching_recent);
    } else {
        recent.emplace_front(key);
        truncate_entries(recent, 64);
    }
    recent_entries_view.set_dirty();
}

SubCarView::~SubCarView() {
    rtc_time::signal_tick_second -= signal_token_tick_second;
    receiver_model.disable();
    baseband::shutdown();
}

const char* SubCarView::getSensorTypeName(FPROTO_SUBCAR_SENSOR type) {
    switch (type) {
        case FPC_SUZUKI:
            return "Suzuki";
        case FPC_VW:
            return "VW";
        case FPC_SUBARU:
            return "Subaru";
        case FPC_KIAV5:
            return "Kia V5";
        case FPC_KIAV3V4:
            return "Kia V3/V4";
        case FPC_KIAV2:
            return "Kia V2";
        case FPC_KIAV1:
            return "Kia V1";
        case FPC_KIAV0:
            return "Kia V0";
        case FPC_FORDV0:
            return "Ford V0";
        case FPC_FIATV0:
            return "Fiat V0";
        case FPC_BMWV0:
            return "BMW V0";
        case FPC_HONDA:
            return "Honda";
        case FPC_FORDV1:
            return "Ford V1";
        case FPC_MAZDAV0:
            return "Mazda V0";
        case FPC_MITSUBISHIV0:
            return "Mitsubishi V0";
        case FPC_KIAV7:
            return "Kia V7";
        case FPC_FIATV1:
            return "Fiat V1";
        case FPC_PORSCHEV0:
            return "Porsche V0";
        case FPC_CHRYSLERV0:
            return "Chrysler V0";
        case FPC_BMWCAS4:
            return "BMW CAS4";
        case FPC_HYUNDAIKIA_V0:
            return "Hyundai/Kia V0";
        case FPC_HYUNDAISANTAFEV0:
            return "Hyundai SFe V0";
        case FPC_FORDV2:
            return "Ford V2";
        case FPC_FORDV3:
            return "Ford V3";
        case FPC_LAND_ROVER_V0:
            return "Land Rover V0";
        case FPC_PSA:
            return "PSA Group";
        case FPC_SCHER_KHAN:
            return "Scher-Khan";
        case FPC_STAR_LINE:
            return "Star Line";
        case FPC_HONDA_STATIC:
            return "Honda Static";
        case FPC_TOYOTA:
            return "Toyota";
            /* case FPC_KIAV6:
                 return "Kia V6";*/

        case FPC_Invalid:
        default:
            return "Unknown";
    }
}

std::string SubCarView::pad_string_with_spaces(int snakes) {
    if (snakes <= 0) return std::string();  // guard: negative/underflow would allocate a huge string and crash
    return std::string(snakes, ' ');
}

void SubCarView::on_freqchg(int64_t freq) {
    field_frequency.set_value(freq);
}

void subaru_decode_count(const uint8_t* KB, uint16_t* count) {
    uint8_t lo = 0;
    if ((KB[4] & 0x40) == 0)
        lo |= 0x01;
    if ((KB[4] & 0x80) == 0)
        lo |= 0x02;
    if ((KB[5] & 0x01) == 0)
        lo |= 0x04;
    if ((KB[5] & 0x02) == 0)
        lo |= 0x08;
    if ((KB[6] & 0x01) == 0)
        lo |= 0x10;
    if ((KB[6] & 0x02) == 0)
        lo |= 0x20;
    if ((KB[5] & 0x40) == 0)
        lo |= 0x40;
    if ((KB[5] & 0x80) == 0)
        lo |= 0x80;

    uint8_t REG_SH1 = (KB[7] << 4) & 0xF0;
    if (KB[5] & 0x04)
        REG_SH1 |= 0x04;
    if (KB[5] & 0x08)
        REG_SH1 |= 0x08;
    if (KB[6] & 0x80)
        REG_SH1 |= 0x02;
    if (KB[6] & 0x40)
        REG_SH1 |= 0x01;

    uint8_t REG_SH2 = ((KB[6] << 2) & 0xF0) | ((KB[7] >> 4) & 0x0F);

    uint8_t SER0 = KB[3];
    uint8_t SER1 = KB[1];
    uint8_t SER2 = KB[2];

    uint8_t total_rot = 4 + lo;
    for (uint8_t i = 0; i < total_rot; ++i) {
        uint8_t t_bit = (SER0 >> 7) & 1;
        SER0 = ((SER0 << 1) & 0xFE) | ((SER1 >> 7) & 1);
        SER1 = ((SER1 << 1) & 0xFE) | ((SER2 >> 7) & 1);
        SER2 = ((SER2 << 1) & 0xFE) | t_bit;
    }

    uint8_t T1 = SER1 ^ REG_SH1;
    uint8_t T2 = SER2 ^ REG_SH2;

    uint8_t hi = 0;
    if ((T1 & 0x10) == 0)
        hi |= 0x04;
    if ((T1 & 0x20) == 0)
        hi |= 0x08;
    if ((T2 & 0x80) == 0)
        hi |= 0x02;
    if ((T2 & 0x40) == 0)
        hi |= 0x01;
    if ((T1 & 0x01) == 0)
        hi |= 0x40;
    if ((T1 & 0x02) == 0)
        hi |= 0x80;
    if ((T2 & 0x08) == 0)
        hi |= 0x20;
    if ((T2 & 0x04) == 0)
        hi |= 0x10;
    *count = ((hi << 8) | lo) & 0xFFFF;
}

void SubCarRecentEntryDetailView::parseProtocol() {
    btn = "";
    cnt = SD_NO_CNT;
    serial = 0;

    if (entry_.sensorType == FPC_Invalid) return;

    if (entry_.sensorType == FPC_SUZUKI) {
        uint32_t data_high = (uint32_t)(entry_.data >> 32);
        uint32_t data_low = (uint32_t)entry_.data;
        serial = ((data_high & 0xFFF) << 16) | (data_low >> 16);
        uint8_t buttonid = (data_low >> 12) & 0xF;
        cnt = (data_high << 4) >> 16;
        btn = to_string_dec_uint(buttonid);
        return;
    }
    if (entry_.sensorType == FPC_VW) {
        // uint32_t key_high = (entry_.data >> 32) & 0xFFFFFFFF;
        uint32_t key_low = entry_.data & 0xFFFFFFFF;
        serial = key_low;  // trimmed to 32 bits for VW
        uint8_t check = entry_.data2 & 0xFF;
        uint8_t btnid = (check >> 4) & 0xF;
        switch (btnid) {
            case 0x1:
                btn = "UNLOCK";
                break;
            case 0x2:
                btn = "LOCK";
                break;
            case 0x3:
                btn = "Un+Lk";
                break;
            case 0x4:
                btn = "TRUNK";
                break;
            case 0x5:
                btn = "Un+Tr";
                break;
            case 0x6:
                btn = "Lk+Tr";
                break;
            case 0x7:
                btn = "Un+Lk+Tr";
                break;
            case 0x8:
                btn = "PANIC";
                break;
            default:
                btn = "Unknown";
                break;
        }
    }
    if (entry_.sensorType == FPC_SUBARU) {
        uint8_t* data_bytes = (uint8_t*)&entry_.data;
        serial = ((uint32_t)data_bytes[1] << 16) | ((uint32_t)data_bytes[2] << 8) | data_bytes[3];
        uint8_t button = data_bytes[0] & 0x0F;
        btn = to_string_dec_uint(button);
        uint16_t cnttmp = 0;
        subaru_decode_count(data_bytes, &cnttmp);
        cnt = cnttmp;
    }

    if (entry_.sensorType == FPC_KIAV5) {
        uint64_t yek = 0;
        for (int i = 0; i < 8; i++) {
            uint8_t byte = (entry_.data2 >> (i * 8)) & 0xFF;
            uint8_t reversed = 0;
            for (int b = 0; b < 8; b++) {
                if (byte & (1 << b))
                    reversed |= (1 << (7 - b));
            }
            yek |= ((uint64_t)reversed << ((7 - i) * 8));
        }
        serial = (uint32_t)((yek >> 32) & 0x0FFFFFFF);
        uint8_t button = (uint8_t)((yek >> 60) & 0x0F);
        uint32_t encrypted = (uint32_t)(yek & 0xFFFFFFFF);
        btn = to_string_dec_uint(button);
        // decode
        uint8_t keystore_bytes[] = {0x53, 0x54, 0x46, 0x52, 0x4b, 0x45, 0x30, 0x30};
        uint8_t s0 = (encrypted & 0xFF);
        uint8_t s1 = (encrypted >> 8) & 0xFF;
        uint8_t s2 = (encrypted >> 16) & 0xFF;
        uint8_t s3 = (encrypted >> 24) & 0xFF;
        int round_index = 1;
        for (size_t i = 0; i < 18; i++) {
            uint8_t r = keystore_bytes[round_index] & 0xFF;
            int steps = 8;
            while (steps > 0) {
                uint8_t base;
                if ((s3 & 0x40) == 0) {
                    base = (s3 & 0x02) == 0 ? 0x74 : 0x2E;
                } else {
                    base = (s3 & 0x02) == 0 ? 0x3A : 0x5C;
                }

                if (s2 & 0x08) {
                    base = (((base >> 4) & 0x0F) | ((base & 0x0F) << 4)) & 0xFF;
                }
                if (s1 & 0x01) {
                    base = ((base & 0x3F) << 2) & 0xFF;
                }
                if (s0 & 0x01) {
                    base = (base << 1) & 0xFF;
                }

                uint8_t temp = (s3 ^ s1) & 0xFF;
                s3 = ((s3 & 0x7F) << 1) & 0xFF;
                if (s2 & 0x80) {
                    s3 |= 0x01;
                }
                s2 = ((s2 & 0x7F) << 1) & 0xFF;
                if (s1 & 0x80) {
                    s2 |= 0x01;
                }
                s1 = ((s1 & 0x7F) << 1) & 0xFF;
                if (s0 & 0x80) {
                    s1 |= 0x01;
                }
                s0 = ((s0 & 0x7F) << 1) & 0xFF;

                uint8_t chk = (base ^ (r ^ temp)) & 0xFF;
                if (chk & 0x80) {
                    s0 |= 0x01;
                }
                r = ((r & 0x7F) << 1) & 0xFF;
                steps--;
            }
            round_index = (round_index - 1) & 0x7;
        }
        cnt = (s0 + (s1 << 8)) & 0xFFFF;
    }

    if (entry_.sensorType == FPC_KIAV3V4) {
        // not decrypted!
        serial = (uint32_t)entry_.data;
        // uint8_t button = entry_.data2 & 0xFF;
        btn = "?";  // to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_KIAV2) {
        serial = (uint32_t)((entry_.data >> 20) & 0xFFFFFFFF);
        uint8_t button = (uint8_t)((entry_.data >> 16) & 0x0F);
        uint16_t raw_count = (uint16_t)((entry_.data >> 4) & 0xFFF);
        cnt = ((raw_count >> 4) | (raw_count << 8)) & 0xFFF;
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_KIAV1) {
        serial = (uint32_t)((entry_.data >> 24) & 0xFFFFFFFF);
        uint8_t button = (uint8_t)((entry_.data >> 16) & 0xFF);
        cnt = (uint8_t)((entry_.data >> 4) & 0xF) << 8 | ((entry_.data >> 8) & 0xFF);
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_KIAV0) {
        serial = (uint32_t)((entry_.data >> 12) & 0x0FFFFFFF);
        uint8_t button = (entry_.data >> 8) & 0x0F;
        cnt = (entry_.data >> 40) & 0xFFFF;
        btn = to_string_dec_uint(button);
    }
    if (entry_.sensorType == FPC_FORDV0) {
        uint8_t buf[13] = {0};

        for (int i = 0; i < 8; ++i) {
            buf[i] = (uint8_t)(entry_.data >> (56 - i * 8));
        }

        buf[8] = (uint8_t)(entry_.data2 >> 8);
        buf[9] = (uint8_t)(entry_.data2 & 0xFF);
        uint8_t tmp = buf[8];
        uint8_t parity = 0;
        uint8_t parity_any = (tmp != 0);
        while (tmp) {
            parity ^= (tmp & 1);
            tmp >>= 1;
        }
        buf[11] = parity_any ? parity : 0;
        uint8_t xor_byte;
        uint8_t limit;
        if (buf[11]) {
            xor_byte = buf[7];
            limit = 7;
        } else {
            xor_byte = buf[6];
            limit = 6;
        }

        for (int idx = 1; idx < limit; ++idx) {
            buf[idx] ^= xor_byte;
        }

        if (buf[11] == 0) {
            buf[7] ^= xor_byte;
        }

        uint8_t orig_b7 = buf[7];
        buf[7] = (orig_b7 & 0xAA) | (buf[6] & 0x55);
        uint8_t mixed = (buf[6] & 0xAA) | (orig_b7 & 0x55);
        buf[12] = mixed;
        buf[6] = mixed;

        uint32_t serial_le = ((uint32_t)buf[1]) |
                             ((uint32_t)buf[2] << 8) |
                             ((uint32_t)buf[3] << 16) |
                             ((uint32_t)buf[4] << 24);

        serial = ((serial_le & 0xFF) << 24) |
                 (((serial_le >> 8) & 0xFF) << 16) |
                 (((serial_le >> 16) & 0xFF) << 8) |
                 ((serial_le >> 24) & 0xFF);

        uint8_t button = (buf[5] >> 4) & 0x0F;

        cnt = ((buf[5] & 0x0F) << 16) |
              (buf[6] << 8) |
              buf[7];
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_TOYOTA) {
        // [63:32]=hop(rolling)  [31:4]=serial(28)  [3:0]=button ; data2: 0=Corolla433 1=Tundra315
        serial = (uint32_t)((entry_.data >> 4) & 0x0FFFFFFF);
        uint8_t button = (uint8_t)(entry_.data & 0x0F);
        btn = to_string_dec_uint(button);
        // KeeLoq hop code (no plain counter); model shown via data2
    }

    if (entry_.sensorType == FPC_FIATV0) {
        serial = (uint32_t)(entry_.data & 0xFFFFFFFF);
        cnt = (uint32_t)((entry_.data >> 32) & 0xFFFFFFFF);
        uint8_t button = (uint8_t)(entry_.data2 & 0xFF);
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_BMWV0) {
        serial = (uint32_t)((entry_.data >> 12) & 0x0FFFFFFF);
        uint8_t button = (entry_.data >> 8) & 0x0F;
        cnt = (entry_.data >> 40) & 0xFFFF;
        btn = to_string_dec_uint(button);
    }

    // Honda: raw 80-bit NRZ frame, bit-field positions are estimates.
    // Byte 0 (decode_data2 [7:0])  = type/manufacturer byte
    // Bits [35:8]  of decode_data  = fixed serial ID (28 bits, estimate)
    // Bits [39:36] of decode_data  = button code (4 bits, estimate)
    // Bits [55:40] of decode_data  = rolling counter (16 bits, estimate)
    // Bits [79:56] = encrypted block — shown in raw hex via decode_data2
    // NOTE: calibrate te_short in c-honda.hpp if data looks scrambled.
    if (entry_.sensorType == FPC_HONDA) {
        serial = (uint32_t)((entry_.data >> 8) & 0x0FFFFFFFUL);   // bits [35:8]
        uint8_t button = (uint8_t)((entry_.data >> 36) & 0xFUL);  // bits [39:36]
        cnt = (uint32_t)((entry_.data >> 40) & 0xFFFFUL);         // bits [55:40]
        btn = to_string_hex(button, 1);
    }

    /*if (entry_.sensorType == FPC_KIAV6) {
        // not decrypted!
        serial = 0;
        btn = "?";
        cnt = 0;
    }*/

    if (entry_.sensorType == FPC_FORDV1) {
        // 136-bit Manchester FM; bytes[0..6] in data, bytes[7..15] in data2
        // No public field mapping — upper 4 bytes as candidate serial
        serial = (uint32_t)(entry_.data >> 32);
    }


    if (entry_.sensorType == FPC_MAZDAV0) {
        // 64-bit Manchester; checksum = sum of bytes[0..6] == byte[7]
        // No public field mapping — upper 4 bytes as candidate serial
        serial = (uint32_t)(entry_.data >> 32);
    }

    if (entry_.sensorType == FPC_MITSUBISHIV0) {
        // 96-bit PWM OOK; bytes[0..7] in data, bytes[8..11] in data2
        serial = (uint32_t)(entry_.data >> 32);
    }

    if (entry_.sensorType == FPC_KIAV7) {
        // [0x4C][cnt_hi][cnt_lo][serial:28][btn:4][crc8]
        serial = (uint32_t)((entry_.data >> 12) & 0x0FFFFFFFUL);
        uint8_t button = (uint8_t)((entry_.data >> 8) & 0xFUL);
        cnt = (uint32_t)((entry_.data >> 40) & 0xFFFFUL);
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_FIATV1) {
        // [pre*2B][serial*4B][btn:4|epoch:4][cnt:5|scr:2|fix:1][enc*5B][crc8]
        // decode_data bits[47:16]=serial, [15:12]=btn, [7:3]=cnt
        serial = (uint32_t)((entry_.data >> 16) & 0xFFFFFFFFUL);
        uint8_t button = (uint8_t)((entry_.data >> 12) & 0xFUL);
        cnt = (uint8_t)((entry_.data >> 3) & 0x1FUL);
        btn = to_string_dec_uint(button);
    }

    if (entry_.sensorType == FPC_PORSCHEV0) {
        // 64-bit rolling code; no public frame field map
        serial = (uint32_t)(entry_.data >> 32);
    }

    if (entry_.sensorType == FPC_CHRYSLERV0) {
        // 80-bit PWM; bytes[0..7] in data, bytes[8..9] in data2
        // Bytes 1-4 used in XOR validation — candidate serial
        serial = (uint32_t)((entry_.data >> 24) & 0xFFFFFFFFUL);
        uint8_t button = (uint8_t)((entry_.data >> 16) & 0xFFUL);
        btn = to_string_hex(button, 2);
    }

    if (entry_.sensorType == FPC_BMWCAS4) {
        // 64-bit Manchester; byte[0]=0x30 (fixed), byte[6]=0xC5 (fixed)
        // Bytes 1-4 = candidate key ID
        serial = (uint32_t)((entry_.data >> 24) & 0xFFFFFFFFUL);
    }

    if (entry_.sensorType == FPC_HYUNDAIKIA_V0) {
        // [serial:32][button_mask:16][checksum:16]
        serial = (uint32_t)(entry_.data >> 32);
        uint16_t bmask = (uint16_t)((entry_.data >> 16) & 0xFFFFUL);
        btn = to_string_hex(bmask, 4);
    }

    if (entry_.sensorType == FPC_HYUNDAISANTAFEV0) {
        // [rolling:32][serial:24][counter:8] in data; [button:8][crc8] in data2
        serial = (uint32_t)((entry_.data >> 8) & 0xFFFFFFUL);
        uint8_t button = (uint8_t)((entry_.data2 >> 8) & 0xFFUL);
        cnt = (uint8_t)(entry_.data & 0xFFUL);
        btn = to_string_hex(button, 2);
    }

    return;
}

}  // namespace ui::subcarrx

namespace ui {

template <>
void RecentEntriesTable<ui::subcarrx::SubCarRecentEntries>::draw(
    const Entry& entry,
    const Rect& target_rect,
    Painter& painter,
    const Style& style,
    ui::RecentEntriesColumns& columns) {
    std::string line{};
    line.reserve(30);

    line = ui::subcarrx::SubCarView::getSensorTypeName((FPROTO_SUBCAR_SENSOR)entry.sensorType);
    line = line + " " + to_string_hex(entry.data << 32);
    line.resize(columns.at(0).second, ' ');
    std::string ageStr = to_string_dec_uint(entry.age);
    std::string bitsStr = to_string_dec_uint(entry.bits);
    line += ui::subcarrx::SubCarView::pad_string_with_spaces(5 - (int)bitsStr.length()) + bitsStr;
    line += ui::subcarrx::SubCarView::pad_string_with_spaces(4 - (int)ageStr.length()) + ageStr;

    line.resize(target_rect.width() / 8, ' ');
    painter.draw_string(target_rect.location(), style, line);
}

}  // namespace ui
