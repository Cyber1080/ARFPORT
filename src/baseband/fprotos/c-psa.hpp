#pragma once
#include "subcarbase.hpp"
#include <cstring>

// PSA (Peugeot/Citroen/DS) — Manchester FM, 128 bits (64-bit Key1 + 16-bit Key2)
// te_short=250µs, te_long=500µs @ 433 MHz AM/FM
// Two sub-variants: 250µs TE (state 1/2) and 125µs TE (state 3/4)
// On decode: Key1 stored in decode_data (64-bit), Key2 low 16 bits in decode_data2.
// Crypto (XOR/TEA) is omitted — raw keys are reported for app-side decryption.
// Ported from protopirate psa.c

#define PSA_TE_SHORT         250U
#define PSA_TE_LONG          500U
#define PSA_TE_DELTA         100U
#define PSA_TE_SHORT_125     125U
#define PSA_TE_LONG_250      250U
#define PSA_TE_LONG_300      300U
#define PSA_TE_END_1000      1000U
#define PSA_TE_END_500       500U
#define PSA_TOLERANCE_99     99U
#define PSA_TOLERANCE_100    100U
#define PSA_TOLERANCE_49     49U
#define PSA_TOLERANCE_50     50U
#define PSA_PATTERN_THR_1    0x46U
#define PSA_PATTERN_THR_2    0x45U
#define PSA_MAX_BITS         0x79U
#define PSA_KEY1_BITS        0x40U
#define PSA_KEY2_BITS        0x50U

typedef enum {
    PSAState0 = 0,
    PSAState1,
    PSAState2,
    PSAState3,
    PSAState4,
} PSADecoderState;

class FProtoSubCarPSA : public FProtoSubCarBase {
   public:
    FProtoSubCarPSA() {
        sensorType = FPC_PSA;
        te_short = PSA_TE_SHORT;
        te_long = PSA_TE_LONG;
        te_delta = PSA_TE_DELTA;
        min_count_bit_for_found = 128;
    }

    void feed(bool level, uint32_t duration) {
        uint32_t tolerance;
        uint32_t new_state = state;
        uint32_t prev_dur = prev_duration;

        switch(state) {
            case PSAState0:
                if(!level) { prev_duration = duration; return; }

                if(duration < PSA_TE_SHORT) {
                    tolerance = PSA_TE_SHORT - duration;
                    if(tolerance > PSA_TOLERANCE_99) {
                        // Check 125µs path
                        uint32_t t125;
                        if(duration < PSA_TE_SHORT_125) t125 = PSA_TE_SHORT_125 - duration;
                        else                             t125 = duration - PSA_TE_SHORT_125;
                        if(t125 > 40 || duration > 180) { prev_duration = duration; return; }
                        new_state = PSAState3;
                    } else {
                        new_state = PSAState1;
                    }
                } else {
                    tolerance = duration - PSA_TE_SHORT;
                    if(tolerance > PSA_TOLERANCE_99) { prev_duration = duration; return; }
                    new_state = PSAState1;
                }

                decode_data_low = 0; decode_data_high = 0;
                pattern_counter = 0; decode_count_local = 0;
                prev_duration = duration;
                FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);
                break;

            case PSAState1:
                if(level) { prev_duration = duration; return; }

                if(duration < PSA_TE_SHORT) {
                    tolerance = PSA_TE_SHORT - duration;
                    if(tolerance < PSA_TOLERANCE_100) {
                        uint32_t prev_diff = abs_diff(prev_dur, PSA_TE_SHORT);
                        if(prev_diff <= PSA_TOLERANCE_99) pattern_counter++;
                        prev_duration = duration;
                        return;
                    }
                } else {
                    tolerance = duration - PSA_TE_SHORT;
                    if(tolerance < PSA_TOLERANCE_100) {
                        uint32_t prev_diff = abs_diff(prev_dur, PSA_TE_SHORT);
                        if(prev_diff <= PSA_TOLERANCE_99) pattern_counter++;
                        prev_duration = duration;
                        return;
                    } else {
                        uint32_t long_diff = abs_diff(duration, PSA_TE_LONG);
                        if(long_diff < 100) {
                            if(pattern_counter > PSA_PATTERN_THR_1) {
                                new_state = PSAState2;
                                decode_data_low = 0; decode_data_high = 0;
                                decode_count_local = 0;
                                FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);
                                state = new_state;
                            }
                            pattern_counter = 0;
                            prev_duration = duration;
                            return;
                        }
                    }
                }
                new_state = PSAState0;
                pattern_counter = 0;
                break;

            case PSAState2:
                if(decode_count_local >= PSA_MAX_BITS) { new_state = PSAState0; break; }

                // End-of-frame detection: long HIGH when decode_count == KEY2_BITS
                if(level && decode_count_local == PSA_KEY2_BITS) {
                    if(duration >= 800) {
                        uint32_t end_diff = abs_diff(duration, PSA_TE_END_1000);
                        if(end_diff <= 199) {
                            if(((key1_high >> 16) & 0xF) == 0xA) {
                                commit_frame();
                            }
                            new_state = PSAState0;
                            state = new_state;
                            return;
                        }
                    }
                }

                {
                    uint8_t manchester_input = 0;
                    bool should_process = false;

                    if(duration < PSA_TE_SHORT) {
                        tolerance = PSA_TE_SHORT - duration;
                        if(tolerance >= PSA_TOLERANCE_100) { prev_duration = duration; return; }
                        manchester_input = (uint8_t)(((level ^ 1) & 0x7f) << 1);
                        should_process = true;
                    } else {
                        tolerance = duration - PSA_TE_SHORT;
                        if(tolerance < PSA_TOLERANCE_100) {
                            manchester_input = (uint8_t)(((level ^ 1) & 0x7f) << 1);
                            should_process = true;
                        } else if(duration < PSA_TE_LONG) {
                            uint32_t diff_from_250 = duration - PSA_TE_SHORT;
                            uint32_t diff_from_500 = PSA_TE_LONG - duration;
                            if(diff_from_500 < 150 || diff_from_250 > diff_from_500) {
                                manchester_input = level ? 4 : 6;
                                should_process = true;
                            } else if(diff_from_250 < 150) {
                                manchester_input = (uint8_t)(((level ^ 1) & 0x7f) << 1);
                                should_process = true;
                            } else if(duration >= 350 && duration <= 400) {
                                manchester_input = level ? 4 : 6;
                                should_process = true;
                            }
                        } else {
                            uint32_t long_diff = duration - PSA_TE_LONG;
                            if(long_diff < 100) {
                                manchester_input = level ? 4 : 6;
                                should_process = true;
                            } else if(!level) {
                                // Large gap — reset
                                new_state = PSAState0; pattern_counter = 0;
                                break;
                            }
                        }
                    }

                    if(should_process && decode_count_local < PSA_KEY2_BITS) {
                        bool decoded_bit = false;
                        if(FProtoGeneral::manchester_advance(manchester_state, (ManchesterEvent)manchester_input, &manchester_state, &decoded_bit)) {
                            uint32_t carry = (decode_data_low >> 31) & 1;
                            decode_data_low = (decode_data_low << 1) | (decoded_bit ? 1U : 0U);
                            decode_data_high = (decode_data_high << 1) | carry;
                            decode_count_local++;
                            if(decode_count_local == PSA_KEY1_BITS) {
                                key1_low = decode_data_low;
                                key1_high = decode_data_high;
                                decode_data_low = 0; decode_data_high = 0;
                            }
                        }
                    }

                    if(!level) { prev_duration = duration; return; }

                    if(!should_process) {
                        uint32_t end_diff = abs_diff(duration, PSA_TE_END_1000);
                        if(end_diff <= 199 && decode_count_local == PSA_KEY2_BITS &&
                           ((key1_high >> 16) & 0xF) == 0xA) {
                            commit_frame();
                            new_state = PSAState0;
                            state = new_state;
                            return;
                        }
                    }
                }
                break;

            case PSAState3:
                if(duration >= 250) {
                    if(duration >= PSA_TE_LONG_250 && duration < PSA_TE_LONG_300) {
                        if(pattern_counter > PSA_PATTERN_THR_2) {
                            new_state = PSAState4;
                            decode_data_low = 0; decode_data_high = 0;
                            decode_count_local = 0;
                            FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);
                            state = new_state;
                            pattern_counter = 0;
                            prev_duration = duration;
                            return;
                        }
                    }
                    new_state = PSAState0; pattern_counter = 0;
                    break;
                }
                {
                    uint32_t t125 = abs_diff(duration, PSA_TE_SHORT_125);
                    if(t125 < PSA_TOLERANCE_50) {
                        uint32_t prev_diff = abs_diff(prev_dur, PSA_TE_SHORT_125);
                        if(prev_diff <= PSA_TOLERANCE_49) pattern_counter++;
                        else pattern_counter = 0;
                        prev_duration = duration;
                        return;
                    }
                    new_state = PSAState0; pattern_counter = 0;
                }
                break;

            case PSAState4:
                if(decode_count_local >= PSA_MAX_BITS) { new_state = PSAState0; break; }

                if(!level) {
                    uint8_t manchester_input;
                    bool decoded_bit = false;

                    if(duration < PSA_TE_SHORT_125) {
                        uint32_t t = PSA_TE_SHORT_125 - duration;
                        if(t > PSA_TOLERANCE_49) { prev_duration = duration; return; }
                        manchester_input = (uint8_t)(((level ^ 1) & 0x7f) << 1);
                    } else {
                        uint32_t t = duration - PSA_TE_SHORT_125;
                        if(t < PSA_TOLERANCE_50) {
                            manchester_input = (uint8_t)(((level ^ 1) & 0x7f) << 1);
                        } else if(duration >= PSA_TE_LONG_250 && duration < PSA_TE_LONG_300) {
                            manchester_input = level ? 4 : 6;
                        } else {
                            prev_duration = duration; return;
                        }
                    }

                    if(FProtoGeneral::manchester_advance(manchester_state, (ManchesterEvent)manchester_input, &manchester_state, &decoded_bit)) {
                        uint32_t carry = (decode_data_low >> 31) & 1;
                        decode_data_low = (decode_data_low << 1) | (decoded_bit ? 1U : 0U);
                        decode_data_high = (decode_data_high << 1) | carry;
                        decode_count_local++;
                        if(decode_count_local == PSA_KEY1_BITS) {
                            key1_low = decode_data_low;
                            key1_high = decode_data_high;
                            decode_data_low = 0; decode_data_high = 0;
                        }
                    }
                } else if(level) {
                    uint32_t end_diff = abs_diff(duration, PSA_TE_END_500);
                    if(end_diff <= 99) {
                        if(decode_count_local == PSA_KEY2_BITS && ((key1_high >> 16) & 0xF) == 0xA) {
                            commit_frame();
                        }
                        new_state = PSAState0;
                        state = new_state;
                        return;
                    }
                }
                break;
        }

        state = new_state;
        prev_duration = duration;
    }

   private:
    uint32_t state = PSAState0;
    uint32_t prev_duration = 0;
    uint32_t decode_data_low = 0;
    uint32_t decode_data_high = 0;
    uint8_t decode_count_local = 0;
    uint32_t key1_low = 0;
    uint32_t key1_high = 0;
    uint32_t key2_low = 0;
    uint16_t pattern_counter = 0;
    ManchesterState manchester_state = ManchesterStateMid1;

    static uint32_t abs_diff(uint32_t a, uint32_t b) {
        return (a >= b) ? (a - b) : (b - a);
    }

    void commit_frame() {
        key2_low = decode_data_low;
        // Pack: decode_data = 64-bit Key1, decode_data2 = Key2 low 16 bits
        decode_data = ((uint64_t)key1_high << 32) | key1_low;
        decode_data2 = (uint64_t)(key2_low & 0xFFFF);
        data_count_bit = 128;
        if(callback) fire_callback();
        decode_data_low = 0;
        decode_data_high = 0;
        decode_count_local = 0;
    }
};
