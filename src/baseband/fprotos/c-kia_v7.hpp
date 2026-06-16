#pragma once
#include "subcarbase.hpp"

// Kia V7 — Manchester OOK, 64 bits
// te_short=250µs, te_long=500µs
// Preamble: ≥16 short-short pairs, then long-short sync transition.
// Frame (after bit inversion): [0x4C][cnt_hi][cnt_lo][serial×3.5B][btn:4][crc8]
// CRC8: poly=0x7F, init=0x4C over bytes[0..6].
// Ported from protopirate (kia_v7.c) to ARF subcarbase format.

typedef enum {
    KiaV7StepReset = 0,
    KiaV7StepPreamble,
    KiaV7StepSyncLow,
    KiaV7StepData,
} KiaV7Step;

#define KIA_V7_HEADER       0x4Cu
#define KIA_V7_KEY_BITS     64u
#define KIA_V7_PREAMBLE_MIN 16u

class FProtoSubCarKiaV7 : public FProtoSubCarBase {
   public:
    FProtoSubCarKiaV7() {
        sensorType = FPC_KIAV7;
        te_short = 250;
        te_long = 500;
        te_delta = 100;
        min_count_bit_for_found = KIA_V7_KEY_BITS;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;
        bool data = false;

        switch (parser_step) {
            case KiaV7StepReset:
                if (level && IS_SHORT(duration)) {
                    preamble_count = 0;
                    te_last = duration;
                    FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset,
                                                      &manchester_state, NULL);
                    parser_step = KiaV7StepPreamble;
                }
                break;

            case KiaV7StepPreamble:
                if (level) {
                    if (IS_LONG(duration) && IS_SHORT(te_last)) {
                        if (preamble_count >= KIA_V7_PREAMBLE_MIN) {
                            decode_data = 0;
                            decode_count_bit = 0;
                            // Seed decoder with the 4 bits implied by long-after-short
                            add_bit(1); add_bit(0); add_bit(1); add_bit(1);
                            te_last = duration;
                            parser_step = KiaV7StepSyncLow;
                        } else {
                            parser_step = KiaV7StepReset;
                        }
                    } else if (IS_SHORT(duration)) {
                        te_last = duration;
                    } else {
                        parser_step = KiaV7StepReset;
                    }
                } else {
                    if (IS_SHORT(duration) && IS_SHORT(te_last)) {
                        preamble_count++;
                    } else {
                        parser_step = KiaV7StepReset;
                    }
                }
                break;

            case KiaV7StepSyncLow:
                if (!level && IS_SHORT(duration) && IS_LONG(te_last)) {
                    te_last = duration;
                    parser_step = KiaV7StepData;
                } else {
                    parser_step = KiaV7StepReset;
                }
                break;

            case KiaV7StepData:
                if (IS_SHORT(duration)) {
                    event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
                } else if (IS_LONG(duration)) {
                    event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
                } else {
                    parser_step = KiaV7StepReset;
                    break;
                }

                if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                      &manchester_state, &data)) {
                    add_bit(data ? 1 : 0);
                }

                if (decode_count_bit == KIA_V7_KEY_BITS) {
                    uint64_t candidate = ~decode_data;
                    uint8_t hdr = (uint8_t)((candidate >> 56) & 0xFF);
                    if (hdr == KIA_V7_HEADER && crc_ok(candidate)) {
                        decode_data = candidate;
                        data_count_bit = KIA_V7_KEY_BITS;
                        if (callback) fire_callback();
                    }
                    parser_step = KiaV7StepReset;
                    FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset,
                                                      &manchester_state, NULL);
                }
                break;
        }
    }

   private:
    uint16_t preamble_count = 0;
    ManchesterState manchester_state = ManchesterStateStart1;

    inline bool IS_SHORT(uint32_t d) { return DURATION_DIFF(d, te_short) < te_delta; }
    inline bool IS_LONG(uint32_t d)  { return DURATION_DIFF(d, te_long)  < te_delta; }

    void add_bit(uint8_t b) {
        decode_data = (decode_data << 1) | b;
        decode_count_bit++;
    }

    static uint8_t crc8(const uint8_t* data, size_t len) {
        uint8_t crc = 0x4Cu;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x7Fu) : (uint8_t)(crc << 1);
        }
        return crc;
    }

    static bool crc_ok(uint64_t d) {
        uint8_t bytes[8];
        for (int i = 7; i >= 0; i--) { bytes[i] = (uint8_t)(d & 0xFF); d >>= 8; }
        return crc8(bytes, 7) == bytes[7];
    }
};
