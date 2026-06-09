#pragma once
#include "subcarbase.hpp"
#include <cstring>

// BMW CAS4 — Manchester OOK, 64 bits
// te_short=500µs, te_long=1000µs, te_delta=150µs
// Preamble: ≥10 pulses in [300,700]µs; then gap ≥1800µs.
// Validation: byte[0]==0x30, byte[6]==0xC5.
// Source: FoxFW bmw_cas4.c, ported to ARF subcarbase format.

typedef enum {
    BmwCas4StepReset = 0,
    BmwCas4StepPreamble,
    BmwCas4StepData,
} BmwCas4Step;

#define BMW_CAS4_DATA_BITS      64u
#define BMW_CAS4_PREAMBLE_MIN   10u
#define BMW_CAS4_PULSE_MIN     300u
#define BMW_CAS4_PULSE_MAX     700u
#define BMW_CAS4_GAP_MIN      1800u
#define BMW_CAS4_BYTE0_MARKER 0x30u
#define BMW_CAS4_BYTE6_MARKER 0xC5u

class FProtoSubCarBmwCas4 : public FProtoSubCarBase {
   public:
    FProtoSubCarBmwCas4() {
        sensorType = FPC_BMWCAS4;
        te_short = 500;
        te_long = 1000;
        te_delta = 150;
        min_count_bit_for_found = BMW_CAS4_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case BmwCas4StepReset:
                if (level && duration >= BMW_CAS4_PULSE_MIN &&
                    duration <= BMW_CAS4_PULSE_MAX) {
                    preamble_count = 1;
                    te_last = duration;
                    parser_step = BmwCas4StepPreamble;
                }
                break;

            case BmwCas4StepPreamble:
                if (duration >= BMW_CAS4_PULSE_MIN && duration <= BMW_CAS4_PULSE_MAX) {
                    preamble_count++;
                    te_last = duration;
                } else if (!level && duration >= BMW_CAS4_GAP_MIN) {
                    if (preamble_count >= BMW_CAS4_PREAMBLE_MIN) {
                        decode_count_bit = 0;
                        memset(raw_bytes, 0, sizeof(raw_bytes));
                        FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset,
                                                          &manchester_state, NULL);
                        parser_step = BmwCas4StepData;
                    } else {
                        parser_step = BmwCas4StepReset;
                    }
                } else {
                    parser_step = BmwCas4StepReset;
                }
                break;

            case BmwCas4StepData: {
                if (decode_count_bit >= BMW_CAS4_DATA_BITS) {
                    parser_step = BmwCas4StepReset;
                    break;
                }
                ManchesterEvent event = ManchesterEventReset;
                uint32_t diff = DURATION_DIFF(duration, (uint32_t)te_short);
                if (diff < (uint32_t)te_delta) {
                    event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
                } else {
                    diff = DURATION_DIFF(duration, (uint32_t)te_long);
                    if (diff < (uint32_t)te_delta)
                        event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;
                }

                if (event != ManchesterEventReset) {
                    bool db;
                    if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                          &manchester_state, &db)) {
                        uint8_t bi = (uint8_t)(decode_count_bit / 8);
                        uint8_t bp = (uint8_t)(7u - (decode_count_bit % 8u));
                        if (db) raw_bytes[bi] |= (uint8_t)(1u << bp);
                        decode_count_bit++;
                        if (decode_count_bit == BMW_CAS4_DATA_BITS) {
                            if (raw_bytes[0] == BMW_CAS4_BYTE0_MARKER &&
                                raw_bytes[6] == BMW_CAS4_BYTE6_MARKER) {
                                decode_data = 0;
                                for (int i = 0; i < 8; i++)
                                    decode_data = (decode_data << 8) | raw_bytes[i];
                                data_count_bit = BMW_CAS4_DATA_BITS;
                                if (callback) callback(this);
                            }
                            parser_step = BmwCas4StepReset;
                        }
                    }
                } else {
                    parser_step = BmwCas4StepReset;
                }
                te_last = duration;
                break;
            }
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t raw_bytes[8] = {};
    ManchesterState manchester_state = ManchesterStateStart1;
};
