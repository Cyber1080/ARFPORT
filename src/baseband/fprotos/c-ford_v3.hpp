#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Ford V3 — Manchester FM, 104 bits (13 bytes)
// te_short=240µs, te_long=480µs @ 433 MHz FM
// Preamble: ≥30 short pulses; then one long pulse starts data.
// Ported from protopirate ford_v3.c

#define FORD_V3_TE_SHORT      240U
#define FORD_V3_TE_LONG       480U
#define FORD_V3_TE_DELTA      60U
#define FORD_V3_DATA_BITS     104U
#define FORD_V3_DATA_BYTES    13U
#define FORD_V3_PREAMBLE_MIN  30U

typedef enum {
    FordV3DecoderStepReset = 0,
    FordV3DecoderStepPreamble,
    FordV3DecoderStepData,
} FordV3DecoderStep;

class FProtoSubCarFordV3 : public FProtoSubCarBase {
   public:
    FProtoSubCarFordV3() {
        sensorType = FPC_FORDV3;
        te_short = FORD_V3_TE_SHORT;
        te_long = FORD_V3_TE_LONG;
        te_delta = FORD_V3_TE_DELTA;
        min_count_bit_for_found = FORD_V3_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch(parser_step) {
            case FordV3DecoderStepReset:
                if(dur_is_short(duration)) {
                    reset_data();
                    preamble_count = 1;
                    parser_step = FordV3DecoderStepPreamble;
                }
                break;

            case FordV3DecoderStepPreamble:
                if(dur_is_short(duration)) {
                    preamble_count++;
                } else if(preamble_count >= FORD_V3_PREAMBLE_MIN && dur_is_long(duration)) {
                    // First long pulse: seed Manchester state and emit first bit
                    manchester_state = ManchesterStateMid1;
                    ManchesterEvent event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
                    bool data_bit = false;
                    if(FProtoGeneral::manchester_advance(manchester_state, event, &manchester_state, &data_bit)) {
                        add_bit(data_bit);
                    }
                    parser_step = FordV3DecoderStepData;
                } else {
                    parser_step = FordV3DecoderStepReset;
                }
                break;

            case FordV3DecoderStepData: {
                if(!dur_is_short(duration) && !dur_is_long(duration)) {
                    // Gap — emit if we have enough bits
                    emit_if_ready();
                    parser_step = FordV3DecoderStepReset;
                    break;
                }

                ManchesterEvent event;
                if(level)
                    event = dur_is_short(duration) ? ManchesterEventShortHigh : ManchesterEventLongHigh;
                else
                    event = dur_is_short(duration) ? ManchesterEventShortLow : ManchesterEventLongLow;

                bool data_bit = false;
                if(FProtoGeneral::manchester_advance(manchester_state, event, &manchester_state, &data_bit)) {
                    add_bit(data_bit);
                    if(bit_count >= FORD_V3_DATA_BITS) {
                        emit_if_ready();
                        parser_step = FordV3DecoderStepReset;
                    }
                }
                break;
            }
        }
    }

   private:
    uint8_t raw_bytes[FORD_V3_DATA_BYTES] = {};
    uint8_t bit_count = 0;
    uint16_t preamble_count = 0;
    ManchesterState manchester_state = ManchesterStateMid1;

    bool dur_is_short(uint32_t d) {
        return DURATION_DIFF(d, FORD_V3_TE_SHORT) < (int32_t)FORD_V3_TE_DELTA;
    }
    bool dur_is_long(uint32_t d) {
        return DURATION_DIFF(d, FORD_V3_TE_LONG) < (int32_t)FORD_V3_TE_DELTA;
    }

    void reset_data() {
        memset(raw_bytes, 0, sizeof(raw_bytes));
        bit_count = 0;
        preamble_count = 0;
        FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);
    }

    void add_bit(bool bit) {
        if(bit_count >= FORD_V3_DATA_BITS) return;
        const uint8_t byte_index = bit_count / 8U;
        const uint8_t bit_in_byte = 7U - (bit_count % 8U);
        if(bit) raw_bytes[byte_index] |= (uint8_t)(1U << bit_in_byte);
        bit_count++;
    }

    void emit_if_ready() {
        if(bit_count < FORD_V3_DATA_BITS) return;

        // Pack bytes 0-7 into decode_data, remaining into decode_data2
        decode_data = 0;
        for(uint8_t i = 0; i < 8; i++)
            decode_data = (decode_data << 8) | raw_bytes[i];
        decode_data2 = 0;
        for(uint8_t i = 8; i < FORD_V3_DATA_BYTES; i++)
            decode_data2 = (decode_data2 << 8) | raw_bytes[i];

        data_count_bit = FORD_V3_DATA_BITS;
        if(callback) fire_callback();
    }
};
