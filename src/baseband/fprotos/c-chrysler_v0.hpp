#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Chrysler V0 — PWM OOK, 80 bits
// Found on: PT Cruiser, Dodge, Jeep (~2004-2010), 315/433 MHz
// Bit period ~4000µs: 1 = 600µs HI + 3400µs LO; 0 = 300µs HI + 3700µs LO
// Preamble: 24 zero-bits (300µs HI + 3700µs LO each); gap > 10000µs; then 80-bit data.
// Validation: XOR check bytes (byte5 vs byte1, bytes 6-9 vs 1-4 with fixed masks).
// Source: FoxFW chrysler.c, ported to ARF subcarbase format.

typedef enum {
    ChryslerV0StepReset = 0,
    ChryslerV0StepPreamble,
    ChryslerV0StepGap,
    ChryslerV0StepData,
} ChryslerV0Step;

#define CHRYSLER_BIT_PERIOD    4000u
#define CHRYSLER_BIT_TOLERANCE  800u
#define CHRYSLER_PREAMBLE_MIN    15u
#define CHRYSLER_PREAMBLE_GAP  10000u
#define CHRYSLER_DATA_BITS       80u
#define CHRYSLER_SHORT_MAX      450u
#define CHRYSLER_LONG_MIN       450u

class FProtoSubCarChryslerV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarChryslerV0() {
        sensorType = FPC_CHRYSLERV0;
        te_short = 300;
        te_long = 600;
        te_delta = 150;
        min_count_bit_for_found = CHRYSLER_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case ChryslerV0StepReset:
                if (level && duration <= CHRYSLER_SHORT_MAX && duration > 100u) {
                    te_last = duration;
                    preamble_count = 1;
                    parser_step = ChryslerV0StepPreamble;
                }
                break;

            case ChryslerV0StepPreamble:
                if (!level) {
                    uint32_t total = te_last + duration;
                    if (DURATION_DIFF(total, CHRYSLER_BIT_PERIOD) < CHRYSLER_BIT_TOLERANCE &&
                        te_last <= CHRYSLER_SHORT_MAX) {
                        preamble_count++;
                    } else if (duration > CHRYSLER_PREAMBLE_GAP &&
                               preamble_count >= CHRYSLER_PREAMBLE_MIN) {
                        parser_step = ChryslerV0StepGap;
                    } else {
                        parser_step = ChryslerV0StepReset;
                    }
                } else {
                    if (duration <= CHRYSLER_SHORT_MAX && duration > 100u) {
                        te_last = duration;
                    } else {
                        parser_step = ChryslerV0StepReset;
                    }
                }
                break;

            case ChryslerV0StepGap:
                if (level) {
                    te_last = duration;
                    decode_count_bit = 0;
                    memset(raw_bytes, 0, sizeof(raw_bytes));
                    parser_step = ChryslerV0StepData;
                } else {
                    parser_step = ChryslerV0StepReset;
                }
                break;

            case ChryslerV0StepData:
                if (level) {
                    te_last = duration;
                } else {
                    uint32_t total = te_last + duration;
                    if (DURATION_DIFF(total, CHRYSLER_BIT_PERIOD) < CHRYSLER_BIT_TOLERANCE) {
                        bool bit_val = (te_last >= CHRYSLER_LONG_MIN);
                        if (decode_count_bit < CHRYSLER_DATA_BITS) {
                            uint8_t bi = (uint8_t)(decode_count_bit / 8);
                            uint8_t bp = (uint8_t)(7u - (decode_count_bit % 8u));
                            if (bit_val) raw_bytes[bi] |= (uint8_t)(1u << bp);
                            decode_count_bit++;
                        }
                        if (decode_count_bit == CHRYSLER_DATA_BITS) {
                            if (validate()) publish();
                            parser_step = ChryslerV0StepReset;
                        }
                    } else {
                        if (decode_count_bit >= CHRYSLER_DATA_BITS && validate()) publish();
                        parser_step = ChryslerV0StepReset;
                    }
                }
                break;
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t raw_bytes[10] = {};

    bool validate() {
        uint8_t* d = raw_bytes;
        uint8_t msb = (d[0] >> 7) & 1u;
        // Check byte 5
        if (msb == 0) { if (d[5] != (uint8_t)(d[1] ^ 0xC3u)) return false; }
        else          { if (d[5] != d[1]) return false; }
        // Check byte 6 vs byte 1 XOR mask
        uint8_t b1_xor_b6 = d[1] ^ d[6];
        if (msb == 0) { if (b1_xor_b6 != 0x04u && b1_xor_b6 != 0x08u) return false; }
        else          { if (b1_xor_b6 != 0x62u) return false; }
        // Check bytes 7-9 vs 2-4 XOR masks
        if (msb == 0) {
            if (d[2]^d[7] != 0x63u || d[3]^d[8] != 0x59u || d[4]^d[9] != 0x46u) return false;
        } else {
            if (d[2]^d[7] != 0x9Au || d[3]^d[8] != 0xC6u) return false;
            uint8_t m4 = d[4]^d[9];
            if (m4 != 0x20u && m4 != 0x10u) return false;
        }
        return true;
    }

    void publish() {
        uint8_t* d = raw_bytes;
        decode_data = 0;
        for (int i = 0; i < 8; i++)
            decode_data = (decode_data << 8) | d[i];
        decode_data2 = ((uint64_t)d[8] << 8) | d[9];
        data_count_bit = CHRYSLER_DATA_BITS;
        if (callback) fire_callback();
    }
};
