#pragma once
#include "subcarbase.hpp"

// Star Line (Pandora) — PWM KeeLoq, 64 bits
// te_short=250µs, te_long=500µs @ 433 MHz AM
// Preamble: >4 double-long pulses (te_long*2); data: short=0, long=1 PWM
// KeeLoq decryption is deferred — raw 64-bit word stored in decode_data.
// Ported from protopirate star_line.c

#define STAR_LINE_TE_SHORT    250U
#define STAR_LINE_TE_LONG     500U
#define STAR_LINE_TE_DELTA    120U
#define STAR_LINE_MIN_BITS    64U

typedef enum {
    StarLineStepReset = 0,
    StarLineStepCheckPreamble,
    StarLineStepSaveDuration,
    StarLineStepCheckDuration,
} StarLineStep;

class FProtoSubCarStarLine : public FProtoSubCarBase {
   public:
    FProtoSubCarStarLine() {
        sensorType = FPC_STAR_LINE;
        te_short = STAR_LINE_TE_SHORT;
        te_long = STAR_LINE_TE_LONG;
        te_delta = STAR_LINE_TE_DELTA;
        min_count_bit_for_found = STAR_LINE_MIN_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch(parser_step) {
            case StarLineStepReset:
                if(level) {
                    if(DURATION_DIFF(duration, te_long * 2) < (int32_t)(te_delta * 2)) {
                        parser_step = StarLineStepCheckPreamble;
                        header_count++;
                    } else if(header_count > 4) {
                        raw_decode_data = 0;
                        bit_count_local = 0;
                        te_last = duration;
                        parser_step = StarLineStepCheckDuration;
                    }
                } else {
                    header_count = 0;
                }
                break;

            case StarLineStepCheckPreamble:
                if(!level && DURATION_DIFF(duration, te_long * 2) < (int32_t)(te_delta * 2)) {
                    // Good preamble low
                    parser_step = StarLineStepReset;
                } else {
                    header_count = 0;
                    parser_step = StarLineStepReset;
                }
                break;

            case StarLineStepSaveDuration:
                if(level) {
                    if(duration >= (te_long + te_delta)) {
                        // Stop bit
                        parser_step = StarLineStepReset;
                        if(bit_count_local >= STAR_LINE_MIN_BITS &&
                           bit_count_local <= STAR_LINE_MIN_BITS + 2) {
                            if(decode_data != raw_decode_data) {
                                decode_data = raw_decode_data;
                                data_count_bit = STAR_LINE_MIN_BITS;
                                if(callback) callback(this);
                            }
                        }
                        raw_decode_data = 0;
                        bit_count_local = 0;
                        header_count = 0;
                    } else {
                        te_last = duration;
                        parser_step = StarLineStepCheckDuration;
                    }
                } else {
                    parser_step = StarLineStepReset;
                }
                break;

            case StarLineStepCheckDuration:
                if(!level) {
                    if(DURATION_DIFF(te_last, te_short) < (int32_t)te_delta &&
                       DURATION_DIFF(duration, te_short) < (int32_t)te_delta) {
                        if(bit_count_local < STAR_LINE_MIN_BITS) {
                            raw_decode_data = (raw_decode_data << 1) | 0;
                        }
                        bit_count_local++;
                        parser_step = StarLineStepSaveDuration;
                    } else if(DURATION_DIFF(te_last, te_long) < (int32_t)te_delta &&
                               DURATION_DIFF(duration, te_long) < (int32_t)te_delta) {
                        if(bit_count_local < STAR_LINE_MIN_BITS) {
                            raw_decode_data = (raw_decode_data << 1) | 1;
                        }
                        bit_count_local++;
                        parser_step = StarLineStepSaveDuration;
                    } else {
                        parser_step = StarLineStepReset;
                    }
                } else {
                    parser_step = StarLineStepReset;
                }
                break;
        }
    }

   private:
    uint16_t header_count = 0;
    uint64_t raw_decode_data = 0;
    uint8_t bit_count_local = 0;
};
