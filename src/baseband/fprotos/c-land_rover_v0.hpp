#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Land Rover V0 — PWM-differential, 81 bits (64-bit key + 16-bit tail + 1 extra bit)
// te_short=250µs, te_long=500µs, sync=750µs @ 433 MHz FM
// Preamble: ≥64 alternating short pairs; then sync high+low 750µs; then data.
// Ported from protopirate land_rover_v0.c

#define LAND_ROVER_V0_TE_SHORT     250U
#define LAND_ROVER_V0_TE_LONG      500U
#define LAND_ROVER_V0_TE_DELTA     100U
#define LAND_ROVER_V0_SYNC_US      750U
#define LAND_ROVER_V0_SYNC_DELTA   120U
#define LAND_ROVER_V0_MIN_PREAMBLE 64U
#define LAND_ROVER_V0_MIN_BITS     81U
#define LAND_ROVER_V0_SIG_UNLOCK   0xA285E3UL
#define LAND_ROVER_V0_SIG_LOCK     0xC20363UL

typedef enum {
    LandRoverV0StepReset = 0,
    LandRoverV0StepPreambleLow,
    LandRoverV0StepPreambleHigh,
    LandRoverV0StepSyncLow,
    LandRoverV0StepData,
} LandRoverV0Step;

class FProtoSubCarLandRoverV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarLandRoverV0() {
        sensorType = FPC_LAND_ROVER_V0;
        te_short = LAND_ROVER_V0_TE_SHORT;
        te_long = LAND_ROVER_V0_TE_LONG;
        te_delta = LAND_ROVER_V0_TE_DELTA;
        min_count_bit_for_found = LAND_ROVER_V0_MIN_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch(parser_step) {
            case LandRoverV0StepReset:
                if(level && is_short(duration)) {
                    preamble_count = 0;
                    parser_step = LandRoverV0StepPreambleLow;
                }
                break;

            case LandRoverV0StepPreambleLow:
                if(!level && is_short(duration)) {
                    preamble_count++;
                    parser_step = LandRoverV0StepPreambleHigh;
                } else {
                    parser_step = LandRoverV0StepReset;
                }
                break;

            case LandRoverV0StepPreambleHigh:
                if(level && is_short(duration)) {
                    parser_step = LandRoverV0StepPreambleLow;
                } else if(level && is_sync(duration) && preamble_count >= LAND_ROVER_V0_MIN_PREAMBLE) {
                    parser_step = LandRoverV0StepSyncLow;
                } else {
                    parser_step = LandRoverV0StepReset;
                }
                break;

            case LandRoverV0StepSyncLow:
                if(!level && is_sync(duration)) {
                    // Reset decoder state
                    memset(raw, 0, sizeof(raw));
                    bit_count = 0;
                    extra_bit = false;
                    previous_bit = true;
                    boundary_pad_skipped = false;
                    pending_short = false;
                    // First bit is always 1 (start-of-frame marker)
                    add_decoded_bit(true);
                    parser_step = LandRoverV0StepData;
                } else {
                    parser_step = LandRoverV0StepReset;
                }
                break;

            case LandRoverV0StepData:
                if(!process_transition(level, duration)) {
                    parser_step = LandRoverV0StepReset;
                    break;
                }
                if(bit_count == LAND_ROVER_V0_MIN_BITS) {
                    if(finish_frame()) {
                        if(callback) fire_callback();
                    }
                    parser_step = LandRoverV0StepReset;
                }
                break;
        }
        te_last = duration;
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t raw[10] = {};    // 80 bits key+tail
    uint8_t bit_count = 0;
    bool extra_bit = false;
    bool previous_bit = true;
    bool boundary_pad_skipped = false;
    bool pending_short = false;

    bool is_short(uint32_t d) { return DURATION_DIFF(d, LAND_ROVER_V0_TE_SHORT) < (int32_t)LAND_ROVER_V0_TE_DELTA; }
    bool is_long(uint32_t d)  { return DURATION_DIFF(d, LAND_ROVER_V0_TE_LONG)  < (int32_t)LAND_ROVER_V0_TE_DELTA; }
    bool is_sync(uint32_t d)  { return DURATION_DIFF(d, LAND_ROVER_V0_SYNC_US)  < (int32_t)LAND_ROVER_V0_SYNC_DELTA; }

    bool add_decoded_bit(bool bit) {
        if(bit_count < 80U) {
            const uint8_t byte_index = bit_count / 8U;
            const uint8_t bit_index = 7U - (bit_count % 8U);
            if(bit) raw[byte_index] |= (uint8_t)(1U << bit_index);
        } else if(bit_count == 80U) {
            extra_bit = bit;
        } else {
            return false;
        }
        bit_count++;
        return true;
    }

    bool process_transition(bool level, uint32_t duration) {
        if(!boundary_pad_skipped) {
            if(level && is_short(duration)) {
                boundary_pad_skipped = true;
                return true;
            }
            boundary_pad_skipped = true;
        }

        if(pending_short) {
            if(!previous_bit && !level && is_short(duration)) {
                pending_short = false;
                return add_decoded_bit(false);
            } else if(previous_bit && level && is_short(duration)) {
                pending_short = false;
                return add_decoded_bit(true);
            }
            return false;
        }

        if(!previous_bit) {
            if(level && is_long(duration)) {
                previous_bit = true;
                return add_decoded_bit(true);
            } else if(level && is_short(duration)) {
                pending_short = true;
                return true;
            }
            return false;
        }

        if(!level && is_long(duration)) {
            previous_bit = false;
            return add_decoded_bit(false);
        } else if(!level && is_short(duration)) {
            pending_short = true;
            return true;
        }

        return false;
    }

    static uint8_t calculate_check(uint32_t count) {
        const uint8_t c0 = ((count>>1)^(count>>2)^(count>>3)^(count>>4)^(count>>6)) & 1U;
        const uint8_t c1 = ((count>>0)^(count>>2)^(count>>3)^(count>>4)^(count>>5)^(count>>6)^1U) & 1U;
        const uint8_t c2 = ((count>>1)^(count>>3)^(count>>4)^(count>>5)^(count>>6)) & 1U;
        return (uint8_t)(c0 | (c1<<1) | (c2<<2));
    }

    static uint16_t calculate_tail(uint32_t count) {
        const uint8_t tail_msb = ((count>>0)^(count>>2)^(count>>4)^(count>>5)) & 1U;
        return tail_msb ? 0xFFFFU : 0x7FFFU;
    }

    bool finish_frame() {
        // Build 64-bit key from raw bytes 0-7
        uint64_t key = 0;
        for(int i = 0; i < 8; i++) key = (key << 8) | raw[i];
        uint16_t tail = ((uint16_t)raw[8] << 8) | raw[9];

        uint8_t key_bytes[8];
        for(int i = 7; i >= 0; i--) { key_bytes[i] = (uint8_t)(key & 0xFF); key >>= 8; }
        // Restore correct byte order
        for(int i = 0; i < 8; i++) key_bytes[i] = raw[i];

        const uint32_t count = ((uint32_t)key_bytes[6] << 1) | ((key_bytes[7] >> 7) & 1U);
        const uint8_t expected_check = calculate_check(count);
        const uint16_t expected_tail = calculate_tail(count);

        const bool check_ok = ((key_bytes[7] & 0x78U) == 0U) && ((key_bytes[7] & 0x07U) == expected_check);
        const bool tail_ok = (tail == expected_tail) && extra_bit;

        if(!check_ok || !tail_ok) return false;

        // Reconstruct key as uint64
        uint64_t k = 0;
        for(int i = 0; i < 8; i++) k = (k << 8) | key_bytes[i];

        decode_data = k;
        decode_data2 = ((uint64_t)tail << 1) | (extra_bit ? 1U : 0U);
        data_count_bit = LAND_ROVER_V0_MIN_BITS;
        return true;
    }
};
