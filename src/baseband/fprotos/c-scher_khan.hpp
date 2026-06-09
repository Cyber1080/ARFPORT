#pragma once
#include "subcarbase.hpp"

// Scher-Khan / MAGICAR — PWM, 35/51/57/63/64/81/82 bits
// te_short=750µs, te_long=1100µs @ 433 MHz FM
// Preamble: ≥2 double-short header pairs; start bit = single short pair
// Ported from protopirate scher_khan.c

#define SCHER_KHAN_TE_SHORT    750U
#define SCHER_KHAN_TE_LONG     1100U
#define SCHER_KHAN_TE_DELTA    160U
#define SCHER_KHAN_MIN_BITS    35U

typedef enum {
    ScherKhanStepReset = 0,
    ScherKhanStepCheckPreamble,
    ScherKhanStepSaveDuration,
    ScherKhanStepCheckDuration,
} ScherKhanStep;

class FProtoSubCarScherKhan : public FProtoSubCarBase {
   public:
    FProtoSubCarScherKhan() {
        sensorType = FPC_SCHER_KHAN;
        te_short = SCHER_KHAN_TE_SHORT;
        te_long = SCHER_KHAN_TE_LONG;
        te_delta = SCHER_KHAN_TE_DELTA;
        min_count_bit_for_found = SCHER_KHAN_MIN_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch(parser_step) {
            case ScherKhanStepReset:
                if(level && DURATION_DIFF(duration, te_short * 2) < (int32_t)te_delta) {
                    parser_step = ScherKhanStepCheckPreamble;
                    te_last = duration;
                    header_count = 0;
                }
                break;

            case ScherKhanStepCheckPreamble:
                if(level) {
                    if(DURATION_DIFF(duration, te_short * 2) < (int32_t)te_delta ||
                       DURATION_DIFF(duration, te_short) < (int32_t)te_delta) {
                        te_last = duration;
                    } else {
                        parser_step = ScherKhanStepReset;
                    }
                } else if(DURATION_DIFF(duration, te_short * 2) < (int32_t)te_delta ||
                           DURATION_DIFF(duration, te_short) < (int32_t)te_delta) {
                    if(DURATION_DIFF(te_last, te_short * 2) < (int32_t)te_delta) {
                        // Found header pair
                        header_count++;
                    } else if(DURATION_DIFF(te_last, te_short) < (int32_t)te_delta) {
                        // Found start bit
                        if(header_count >= 2) {
                            parser_step = ScherKhanStepSaveDuration;
                            decode_data = 0;
                            bit_count_local = 1;
                        } else {
                            parser_step = ScherKhanStepReset;
                        }
                    } else {
                        parser_step = ScherKhanStepReset;
                    }
                } else {
                    parser_step = ScherKhanStepReset;
                }
                break;

            case ScherKhanStepSaveDuration:
                if(level) {
                    if(duration >= (te_delta * 2UL + te_long)) {
                        // Stop bit found
                        parser_step = ScherKhanStepReset;
                        if(bit_count_local >= SCHER_KHAN_MIN_BITS) {
                            decode_data = raw_decode_data;
                            data_count_bit = bit_count_local;
                            if(callback) callback(this);
                        }
                        raw_decode_data = 0;
                        bit_count_local = 0;
                    } else {
                        te_last = duration;
                        parser_step = ScherKhanStepCheckDuration;
                    }
                } else {
                    parser_step = ScherKhanStepReset;
                }
                break;

            case ScherKhanStepCheckDuration:
                if(!level) {
                    if(DURATION_DIFF(te_last, te_short) < (int32_t)te_delta &&
                       DURATION_DIFF(duration, te_short) < (int32_t)te_delta) {
                        raw_decode_data = (raw_decode_data << 1) | 0;
                        bit_count_local++;
                        parser_step = ScherKhanStepSaveDuration;
                    } else if(DURATION_DIFF(te_last, te_long) < (int32_t)te_delta &&
                               DURATION_DIFF(duration, te_long) < (int32_t)te_delta) {
                        raw_decode_data = (raw_decode_data << 1) | 1;
                        bit_count_local++;
                        parser_step = ScherKhanStepSaveDuration;
                    } else {
                        parser_step = ScherKhanStepReset;
                    }
                } else {
                    parser_step = ScherKhanStepReset;
                }
                break;
        }
    }

   private:
    uint16_t header_count = 0;
    uint64_t raw_decode_data = 0;
    uint8_t bit_count_local = 0;
};
