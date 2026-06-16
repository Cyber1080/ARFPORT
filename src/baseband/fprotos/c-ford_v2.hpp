#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Ford V2 — Manchester FM, 104 bits (13 bytes)
// te_short=200µs, te_long=400µs @ 433 MHz FM
// Preamble: ≥64 alternating short pulses; sync word: 0x7FA7 (inverted 0x7FA8)
// Ported from protopirate ford_v2.c

#define FORD_V2_TE_SHORT    200U
#define FORD_V2_TE_LONG     400U
#define FORD_V2_TE_DELTA    260U
#define FORD_V2_PREAMBLE_MIN 64U
#define FORD_V2_DATA_BYTES  13U
#define FORD_V2_DATA_BITS   104U
#define FORD_V2_SYNC_0      0x7FU
#define FORD_V2_SYNC_1      0xA7U
#define FORD_V2_SYNC_BITS   16U
#define FORD_V2_INTER_BURST_GAP_US 15000U
#define FORD_V2_POST_SYNC_DECODE_COUNT_BIT 16U

typedef enum {
    FordV2DecoderStepReset = 0,
    FordV2DecoderStepPreamble,
    FordV2DecoderStepSync,
    FordV2DecoderStepData,
} FordV2DecoderStep;

class FProtoSubCarFordV2 : public FProtoSubCarBase {
   public:
    FProtoSubCarFordV2() {
        sensorType = FPC_FORDV2;
        te_short = FORD_V2_TE_SHORT;
        te_long = FORD_V2_TE_LONG;
        te_delta = FORD_V2_TE_DELTA;
        min_count_bit_for_found = FORD_V2_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch(parser_step) {
            case FordV2DecoderStepReset:
                if(dur_is_short(duration)) {
                    preamble_count = 1;
                    parser_step = FordV2DecoderStepPreamble;
                }
                break;

            case FordV2DecoderStepPreamble:
                if(dur_is_short(duration)) {
                    if(preamble_count < 0xFFFFU) preamble_count++;
                } else if(!level && dur_is_long(duration)) {
                    if(preamble_count >= FORD_V2_PREAMBLE_MIN) {
                        enter_sync(level, duration);
                    } else {
                        do_reset();
                    }
                } else {
                    do_reset();
                }
                break;

            case FordV2DecoderStepSync:
            case FordV2DecoderStepData:
                if(dur_is_short(duration) || dur_is_long(duration)) {
                    feed_manchester(level, duration);
                } else {
                    if(duration >= FORD_V2_INTER_BURST_GAP_US) {
                        do_reset();
                    } else {
                        do_reset();
                    }
                }
                te_last = duration;
                break;
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t raw_bytes[FORD_V2_DATA_BYTES] = {};
    uint8_t byte_count = 0;
    uint16_t sync_shift = 0;
    uint8_t sync_bit_count = 0;
    uint32_t decode_buf = 0;
    uint8_t decode_buf_bits = 0;
    ManchesterState manchester_state = ManchesterStateMid1;

    // sync_shift target: ~(0x7F<<8 | 0xA7) = 0x8058 inverted per protopirate logic
    // protopirate: ford_v2_sync_shift16_inv = ~((0x7F<<8)|0xA7) = 0x8058
    static const uint16_t SYNC_TARGET = (uint16_t)(~(((uint16_t)FORD_V2_SYNC_0 << 8) | (uint16_t)FORD_V2_SYNC_1));

    bool dur_is_short(uint32_t d) {
        return DURATION_DIFF(d, FORD_V2_TE_SHORT) < (int32_t)FORD_V2_TE_DELTA;
    }
    bool dur_is_long(uint32_t d) {
        return DURATION_DIFF(d, FORD_V2_TE_LONG) < (int32_t)FORD_V2_TE_DELTA;
    }

    void do_reset() {
        parser_step = FordV2DecoderStepReset;
        preamble_count = 0;
        byte_count = 0;
        sync_shift = 0;
        sync_bit_count = 0;
        decode_buf = 0;
        decode_buf_bits = 0;
        memset(raw_bytes, 0, sizeof(raw_bytes));
        FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);
    }

    // Feed one Manchester event into the sync or data path
    void feed_manchester(bool level, uint32_t duration) {
        ManchesterEvent ev;
        if(dur_is_short(duration))
            ev = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        else
            ev = level ? ManchesterEventLongHigh : ManchesterEventLongLow;

        if(parser_step == FordV2DecoderStepSync) {
            feed_sync_event(ev);
        } else {
            feed_data_event(ev);
        }
    }

    void feed_sync_event(ManchesterEvent ev) {
        bool data_bit;
        if(!FProtoGeneral::manchester_advance(manchester_state, ev, &manchester_state, &data_bit))
            return;

        sync_shift = (uint16_t)((sync_shift << 1) | (data_bit ? 1U : 0U));
        if(sync_bit_count < FORD_V2_SYNC_BITS) sync_bit_count++;

        if(sync_bit_count >= FORD_V2_SYNC_BITS && sync_shift == SYNC_TARGET) {
            // Enter data phase: pre-load sync bytes
            memset(raw_bytes, 0, sizeof(raw_bytes));
            raw_bytes[0] = FORD_V2_SYNC_0;
            raw_bytes[1] = FORD_V2_SYNC_1;
            byte_count = 2;
            decode_buf = 0;
            decode_buf_bits = FORD_V2_POST_SYNC_DECODE_COUNT_BIT;
            parser_step = FordV2DecoderStepData;
        }
    }

    void feed_data_event(ManchesterEvent ev) {
        bool data_bit;
        if(!FProtoGeneral::manchester_advance(manchester_state, ev, &manchester_state, &data_bit))
            return;

        // Data bits are inverted per protopirate
        data_bit = !data_bit;

        decode_buf = (decode_buf << 1) | (data_bit ? 1U : 0U);
        decode_buf_bits++;

        if((decode_buf_bits & 7U) == 0U) {
            uint8_t byte_val = (uint8_t)(decode_buf & 0xFFU);
            if(byte_count < FORD_V2_DATA_BYTES) {
                raw_bytes[byte_count++] = byte_val;
            }
            decode_buf = 0;

            if(byte_count == FORD_V2_DATA_BYTES) {
                commit_frame();
                do_reset();
            }
        }
    }

    void enter_sync(bool level, uint32_t duration) {
        parser_step = FordV2DecoderStepSync;
        byte_count = 0;
        sync_shift = 0;
        sync_bit_count = 0;
        decode_buf = 0;
        decode_buf_bits = 0;
        memset(raw_bytes, 0, sizeof(raw_bytes));

        FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset, &manchester_state, NULL);

        // Feed the transition pulse that ended preamble
        ManchesterEvent ev;
        if(dur_is_short(duration))
            ev = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
        else if(dur_is_long(duration))
            ev = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
        else { do_reset(); return; }

        if(ev == ManchesterEventShortLow || ev == ManchesterEventLongLow)
            manchester_state = ManchesterStateMid0;

        feed_sync_event(ev);
    }

    bool commit_frame() {
        if(raw_bytes[0] != FORD_V2_SYNC_0 || raw_bytes[1] != FORD_V2_SYNC_1)
            return false;

        // Pack first 8 bytes into decode_data, remaining 5 into decode_data2
        decode_data = 0;
        for(uint8_t i = 0; i < 8; i++)
            decode_data = (decode_data << 8) | raw_bytes[i];

        decode_data2 = 0;
        for(uint8_t i = 8; i < FORD_V2_DATA_BYTES; i++)
            decode_data2 = (decode_data2 << 8) | raw_bytes[i];

        data_count_bit = FORD_V2_DATA_BITS;
        if(callback) fire_callback();
        return true;
    }
};
