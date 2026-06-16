#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Toyota KeeLoq dual-variant decoder. Ported from D4C1-Labs/Flipper-ARF toyota.c.
//  Variant A: Corolla 433MHz, PWM pairs (LS=0 / SL=1), te 400/800us, 68 bits.
//  Variant B: Tundra 315MHz, NRZ, preamble ~200/390us + sync ~1938us, midpoint 287us, 67 bits.
//  decode_data : [63:32]=hop  [31:4]=serial(28)  [3:0]=button
//  decode_data2: variant (0=Corolla, 1=Tundra)
class FProtoSubCarToyota : public FProtoSubCarBase {
   public:
    FProtoSubCarToyota() {
        sensorType = FPC_TOYOTA;
        te_short = 200;
        te_long = 390;
        te_delta = 120;
        min_count_bit_for_found = 60;
    }

    void feed(bool level, uint32_t duration) {
        if (parser_step == StepReset) {
            if (!level) return;
            bool fits_b = (DURATION_DIFF(duration, 200u) < 120u) && (duration < 310u);
            bool fits_a = (DURATION_DIFF(duration, 400u) < 175u) && (duration >= 310u);
            if (fits_b) {
                variant = 1;
                tlast = duration;
                have_high = true;
                preamble_count = 0;
                parser_step = StepPreambleB;
            } else if (fits_a) {
                variant = 0;
                tlast = duration;
                have_high = true;
                preamble_count = 0;
                parser_step = StepPreambleA;
            }
            return;
        }
        if (variant == 1)
            feed_b(level, duration);
        else
            feed_a(level, duration);
    }

   private:
    enum { StepReset = 0, StepPreambleA, StepDataA, StepPreambleB, StepDataB };

    uint64_t bits_lo = 0;
    uint8_t bits_hi = 0;
    uint8_t bit_count = 0;
    uint32_t tlast = 0;
    bool have_high = false;
    uint16_t preamble_count = 0;
    uint8_t variant = 0;
    uint32_t hop = 0, serial = 0;
    uint8_t button = 0;

    static bool a_short(uint32_t d) { return DURATION_DIFF(d, 400u) < 175u; }
    static bool a_long(uint32_t d) { return DURATION_DIFF(d, 800u) < 175u; }
    static bool b_short(uint32_t d) { return DURATION_DIFF(d, 200u) < 120u; }
    static bool b_long(uint32_t d) { return DURATION_DIFF(d, 390u) < 120u; }

    void reset_parser() {
        parser_step = StepReset;
        tlast = 0;
        bits_lo = 0;
        bits_hi = 0;
        bit_count = 0;
        have_high = false;
        preamble_count = 0;
        hop = 0;
        serial = 0;
        button = 0;
        // variant intentionally NOT reset (detected once per session)
    }

    void push_bit(uint8_t bit) {
        uint8_t carry = (uint8_t)(bits_lo >> 63) & 1;
        bits_hi = (bits_hi << 1) | carry;
        bits_lo = (bits_lo << 1) | (bit & 1);
        bit_count++;
    }

    uint32_t extract(uint8_t offset, uint8_t length) {
        uint32_t result = 0;
        uint8_t total = bit_count;
        for (uint8_t i = 0; i < length; i++) {
            int8_t pos = (int8_t)(total - 1) - (int8_t)(offset + i);
            uint8_t b = 0;
            if (pos >= 64)
                b = (bits_hi >> (pos - 64)) & 1;
            else if (pos >= 0)
                b = (bits_lo >> pos) & 1;
            result = (result << 1) | b;
        }
        return result;
    }

    void decode_and_fire() {
        if (bit_count < 60) return;
        hop = extract(0, 32);
        serial = extract(32, 28);
        button = (uint8_t)extract(60, 4);
        decode_data = ((uint64_t)hop << 32) | ((uint64_t)serial << 4) | (button & 0x0F);
        decode_data2 = variant;
        data_count_bit = bit_count;
        if (callback) fire_callback();
    }

    // Variant A — Corolla 433MHz, PWM pairs
    void feed_a(bool level, uint32_t duration) {
        if (parser_step == StepPreambleA) {
            if (level) {
                tlast = duration;
                have_high = true;
                return;
            }
            if (!have_high) { reset_parser(); return; }
            have_high = false;
            bool hs = a_short(tlast), hl = a_long(tlast);
            bool ls = a_short(duration), ll = a_long(duration);
            if (hs && ls) { preamble_count++; return; }
            if (preamble_count < 6) { reset_parser(); return; }
            bits_lo = 0; bits_hi = 0; bit_count = 0;
            if (hl && ls) push_bit(0);
            else if (hs && ll) push_bit(1);
            parser_step = StepDataA;
            return;
        }
        if (parser_step == StepDataA) {
            if (level) {
                if (a_short(duration) || a_long(duration)) {
                    tlast = duration;
                    have_high = true;
                } else {
                    if (bit_count >= 60) decode_and_fire();
                    reset_parser();
                }
                return;
            }
            if (!have_high) return;
            have_high = false;
            bool hs = a_short(tlast), hl = a_long(tlast);
            bool ls = a_short(duration), ll = a_long(duration);
            if (hl && ls) push_bit(0);
            else if (hs && ll) push_bit(1);
            else { if (bit_count >= 60) decode_and_fire(); reset_parser(); return; }
            if (bit_count >= 68) { decode_and_fire(); reset_parser(); }
        }
    }

    // Variant B — Tundra 315MHz, NRZ
    void feed_b(bool level, uint32_t duration) {
        if (parser_step == StepPreambleB) {
            if (level) {
                if (b_short(duration)) { tlast = duration; have_high = true; }
                else reset_parser();
                return;
            }
            if (!have_high) { reset_parser(); return; }
            have_high = false;
            if (duration >= 1500u && duration <= 2600u) {
                if (preamble_count >= 6) {
                    bits_lo = 0; bits_hi = 0; bit_count = 0; have_high = false;
                    parser_step = StepDataB;
                } else reset_parser();
                return;
            }
            if (b_long(duration)) { preamble_count++; return; }
            reset_parser();
            return;
        }
        if (parser_step == StepDataB) {
            if (duration >= 1500u) {
                if (bit_count >= 60) decode_and_fire();
                reset_parser();
                return;
            }
            uint8_t bit = (duration > 287u) ? 1 : 0;
            push_bit(bit);
            if (bit_count >= 67) { decode_and_fire(); reset_parser(); }
        }
    }
};
