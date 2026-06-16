#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Fiat V1 — Magneti Marelli BSI (PCF7946), Manchester OOK, 104 bits
// Two timing variants auto-detected from preamble averaging:
//   Type A (Panda):       te_short≈260µs, te_long≈520µs
//   Type B (Grande Punto): te_short≈100µs, te_long≈200µs
// Preamble: ≥48 pulses in [35,450]µs; gap > 4*te; sync HIGH in [4*te..12*te].
// Frame: [preamble_residue×2B][serial×4B][btn:4|epoch:4][cnt:5|scr:2|fix:1][encrypted×5B][crc8]
// CRC8: poly=0x01, init=0x03 over bytes[0..11] == bytes[12].
// Ported from protopirate (fiat_v1.c) to ARF subcarbase format.

typedef enum {
    FiatV1StepReset = 0,
    FiatV1StepPreamble,
    FiatV1StepSync,
    FiatV1StepData,
} FiatV1Step;

#define FIAT_V1_PREAMBLE_MIN     48u
#define FIAT_V1_PULSE_MIN_US     35u
#define FIAT_V1_PULSE_MAX_US     450u
#define FIAT_V1_DATA_BITS        104u
#define FIAT_V1_DATA_BYTES       13u

class FProtoSubCarFiatV1 : public FProtoSubCarBase {
   public:
    FProtoSubCarFiatV1() {
        sensorType = FPC_FIATV1;
        te_short = 260;
        te_long = 520;
        te_delta = 80;
        min_count_bit_for_found = FIAT_V1_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case FiatV1StepReset:
                if (level && duration >= FIAT_V1_PULSE_MIN_US &&
                    duration <= FIAT_V1_PULSE_MAX_US) {
                    preamble_count = 1;
                    te_sum = duration;
                    te_count = 1;
                    te_last = duration;
                    parser_step = FiatV1StepPreamble;
                }
                break;

            case FiatV1StepPreamble:
                if (duration >= FIAT_V1_PULSE_MIN_US && duration <= FIAT_V1_PULSE_MAX_US) {
                    preamble_count++;
                    te_sum += duration;
                    te_count++;
                    te_last = duration;
                } else if (!level) {
                    if (preamble_count >= FIAT_V1_PREAMBLE_MIN && te_count > 0) {
                        te_detected = te_sum / te_count;
                        if (duration > te_detected * 4u) {
                            parser_step = FiatV1StepSync;
                            te_last = duration;
                        } else {
                            parser_step = FiatV1StepReset;
                        }
                    } else {
                        parser_step = FiatV1StepReset;
                    }
                } else {
                    parser_step = FiatV1StepReset;
                }
                break;

            case FiatV1StepSync:
                if (level && duration >= te_detected * 4u && duration <= te_detected * 12u) {
                    // Sync HIGH accepted — prepare data
                    decode_count_bit = 0;
                    decode_data = 0;
                    decode_data2 = 0;
                    memset(raw_bytes, 0, sizeof(raw_bytes));
                    FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset,
                                                      &manchester_state, NULL);
                    manchester_state = ManchesterStateMid1;
                    te_last = duration;
                    parser_step = FiatV1StepData;
                } else {
                    parser_step = FiatV1StepReset;
                }
                break;

            case FiatV1StepData: {
                uint32_t tes = te_detected ? te_detected : (uint32_t)te_short;
                uint32_t tel = tes * 2u;
                uint32_t ted = tes / 2u;
                if (ted < 30u) ted = 30u;

                ManchesterEvent event = ManchesterEventReset;
                uint32_t diff = (duration > tes) ? (duration - tes) : (tes - duration);
                if (diff < ted) {
                    event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
                } else {
                    diff = (duration > tel) ? (duration - tel) : (tel - duration);
                    if (diff < ted)
                        event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;
                }

                bool frame_done = false;
                if (event != ManchesterEventReset) {
                    bool db;
                    if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                          &manchester_state, &db)) {
                        // Store into raw_bytes
                        uint8_t bi = (uint8_t)(decode_count_bit / 8);
                        uint8_t bp = (uint8_t)(7 - (decode_count_bit % 8));
                        if (bi < FIAT_V1_DATA_BYTES && db)
                            raw_bytes[bi] |= (uint8_t)(1u << bp);
                        // Pack into decode_data / decode_data2 (64+40 bits)
                        if (decode_count_bit < 64)
                            decode_data = (decode_data << 1) | (db ? 1u : 0u);
                        else
                            decode_data2 = (decode_data2 << 1) | (db ? 1u : 0u);
                        decode_count_bit++;
                        if (decode_count_bit >= FIAT_V1_DATA_BITS) frame_done = true;
                    }
                } else if (decode_count_bit >= (FIAT_V1_DATA_BITS - 2)) {
                    frame_done = true;
                } else {
                    parser_step = FiatV1StepReset;
                    break;
                }

                if (frame_done) {
                    if (decode_count_bit >= FIAT_V1_DATA_BITS) {
                        uint8_t calc = crc8(raw_bytes, 12);
                        if (calc == raw_bytes[12]) {
                            data_count_bit = FIAT_V1_DATA_BITS;
                            if (callback) fire_callback();
                        }
                    }
                    parser_step = FiatV1StepReset;
                }
                te_last = duration;
                break;
            }
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint32_t te_sum = 0;
    uint16_t te_count = 0;
    uint32_t te_detected = 0;
    uint8_t raw_bytes[FIAT_V1_DATA_BYTES] = {};
    ManchesterState manchester_state = ManchesterStateStart1;

    static uint8_t crc8(const uint8_t* data, size_t len) {
        uint8_t crc = 0x03u;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x01u) : (uint8_t)(crc << 1);
        }
        return crc;
    }
};
