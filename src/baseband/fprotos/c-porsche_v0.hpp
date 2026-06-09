#pragma once
#include "subcarbase.hpp"

// Porsche Touareg/Cayenne V0 — PWM OOK, 64 bits
// Also covers: VW Touareg (same hardware platform)
// te_short=1680µs, te_long=3370µs, te_delta=500µs
// Sync: ≥15 alternating 3370µs LOW/HIGH pairs; then gap (5930µs).
// Data: on HIGH edge, pair (te_last=LOW, duration=HIGH):
//   long_LOW + short_HIGH = bit 1
//   short_LOW + long_HIGH = bit 0
// Rolling code protocol; raw frame stored for analysis.
// Original impl by @lupettohf (protopirate), ported to ARF subcarbase format.

typedef enum {
    PorscheV0StepReset = 0,
    PorscheV0StepSync,
    PorscheV0StepGap,
    PorscheV0StepData,
} PorscheV0Step;

#define PORSCHE_V0_BIT_COUNT  64u
#define PORSCHE_V0_SYNC_MIN   15u
#define PC_TE_GAP             5930u
#define PC_GAP_DELTA          600u

class FProtoSubCarPorscheV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarPorscheV0() {
        sensorType = FPC_PORSCHEV0;
        te_short = 1680;
        te_long = 3370;
        te_delta = 500;
        min_count_bit_for_found = PORSCHE_V0_BIT_COUNT;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case PorscheV0StepReset:
                // Wait for first long LOW (sync start)
                if (!level && DURATION_DIFF(duration, te_long) < te_delta) {
                    sync_count = 1;
                    parser_step = PorscheV0StepSync;
                }
                break;

            case PorscheV0StepSync:
                if (DURATION_DIFF(duration, te_long) < te_delta) {
                    sync_count++;
                    // Sync is pairs of long pulses; count both polarities
                } else if (sync_count >= PORSCHE_V0_SYNC_MIN &&
                           DURATION_DIFF(duration, PC_TE_GAP) < PC_GAP_DELTA) {
                    // Gap detected after sufficient sync
                    parser_step = PorscheV0StepGap;
                } else {
                    parser_step = PorscheV0StepReset;
                }
                break;

            case PorscheV0StepGap:
                // Expect the other half of the gap pulse, then start data
                if (DURATION_DIFF(duration, PC_TE_GAP) < PC_GAP_DELTA) {
                    decode_data = 0;
                    decode_count_bit = 0;
                    parser_step = PorscheV0StepData;
                } else {
                    parser_step = PorscheV0StepReset;
                }
                break;

            case PorscheV0StepData:
                if (level) {
                    // On HIGH: check (te_last=prev LOW duration, duration=HIGH)
                    bool bit_value;
                    if (DURATION_DIFF(te_last, te_long)  < te_delta &&
                        DURATION_DIFF(duration, te_short) < te_delta) {
                        bit_value = true;  // long_low + short_high = 1
                    } else if (DURATION_DIFF(te_last, te_short) < te_delta &&
                               DURATION_DIFF(duration, te_long)  < te_delta) {
                        bit_value = false; // short_low + long_high = 0
                    } else {
                        parser_step = PorscheV0StepReset;
                        break;
                    }
                    decode_data = (decode_data << 1) | (bit_value ? 1u : 0u);
                    decode_count_bit++;
                    if (decode_count_bit >= PORSCHE_V0_BIT_COUNT) {
                        data_count_bit = PORSCHE_V0_BIT_COUNT;
                        if (callback) callback(this);
                        parser_step = PorscheV0StepReset;
                    }
                } else {
                    te_last = duration; // store LOW duration for next HIGH
                }
                break;
        }
    }

   private:
    uint16_t sync_count = 0;
};
