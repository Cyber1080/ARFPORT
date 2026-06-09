#pragma once
#include "subcarbase.hpp"

// Mazda V0 — Manchester OOK, 64 bits
// te_short=250µs, te_long=500µs
// Also covers: Infiniti (identical protocol per PANDORA MAX firmware analysis)
// Preamble: ≥16 ones, then sync byte 0xD7; data bytes transmitted inverted on air.
// Checksum: sum of all 7 payload bytes = byte[7].
// Ported from protopirate (mazda_v0.c) to ARF subcarbase format.

typedef enum {
    MazdaV0StepReset = 0,
    MazdaV0StepPreamble,
    MazdaV0StepData,
} MazdaV0Step;

#define MAZDA_V0_SYNC_BYTE      0xD7u
#define MAZDA_V0_PREAMBLE_ONES  16

class FProtoSubCarMazdaV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarMazdaV0() {
        sensorType = FPC_MAZDAV0;
        te_short = 250;
        te_long = 500;
        te_delta = 120;  // 100 + 20 tolerance as in protopirate
        min_count_bit_for_found = 64;
    }

    void feed(bool level, uint32_t duration) {
        ManchesterEvent event = ManchesterEventReset;
        bool data = false;

        switch (parser_step) {
            case MazdaV0StepReset:
                // Wait for first short HIGH — start of preamble
                if (level && DURATION_DIFF(duration, te_short) < te_delta) {
                    decode_data = 0;
                    decode_count_bit = 0;
                    preamble_count = 0;
                    preamble_pattern = 0;
                    FProtoGeneral::manchester_advance(manchester_state, ManchesterEventReset,
                                                      &manchester_state, NULL);
                    manchester_state = ManchesterStateMid1;
                    parser_step = MazdaV0StepPreamble;
                }
                break;

            case MazdaV0StepPreamble:
                if (!get_event(duration, level, &event)) {
                    parser_step = MazdaV0StepReset;
                    break;
                }
                if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                      &manchester_state, &data)) {
                    preamble_pattern = (preamble_pattern << 1) | (data ? 1u : 0u);
                    if (data) {
                        preamble_count++;
                    } else if (preamble_count < (MAZDA_V0_PREAMBLE_ONES - 1)) {
                        preamble_count = 0;
                        preamble_pattern = 0;
                        break;
                    }
                    // Sync detected when last 8 bits == 0xD7 and enough ones seen
                    if ((preamble_pattern & 0xFFu) == MAZDA_V0_SYNC_BYTE &&
                        preamble_count >= (MAZDA_V0_PREAMBLE_ONES - 1)) {
                        decode_data = 0;
                        decode_count_bit = 0;
                        parser_step = MazdaV0StepData;
                    }
                }
                break;

            case MazdaV0StepData:
                if (!get_event(duration, level, &event)) {
                    parser_step = MazdaV0StepReset;
                    break;
                }
                if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                      &manchester_state, &data)) {
                    decode_data = (decode_data << 1) | (data ? 1u : 0u);
                    decode_count_bit++;

                    if (decode_count_bit == min_count_bit_for_found) {
                        // Data arrives inverted on air
                        decode_data = ~decode_data;
                        // Validate checksum: sum of bytes 0-6 == byte 7
                        if (checksum_ok(decode_data)) {
                            data_count_bit = (uint16_t)decode_count_bit;
                            if (callback) callback(this);
                        }
                        decode_data = 0;
                        decode_count_bit = 0;
                        preamble_count = 0;
                        preamble_pattern = 0;
                        parser_step = MazdaV0StepReset;
                    }
                }
                break;
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t preamble_pattern = 0;
    ManchesterState manchester_state = ManchesterStateStart1;

    bool get_event(uint32_t duration, bool level, ManchesterEvent* event) {
        if (DURATION_DIFF(duration, te_short) < te_delta) {
            *event = level ? ManchesterEventShortLow : ManchesterEventShortHigh;
            return true;
        }
        if (DURATION_DIFF(duration, te_long) < te_delta) {
            *event = level ? ManchesterEventLongLow : ManchesterEventLongHigh;
            return true;
        }
        return false;
    }

    static bool checksum_ok(uint64_t d) {
        uint8_t sum = 0;
        for (int i = 7; i >= 1; i--)
            sum += (uint8_t)((d >> (i * 8)) & 0xFF);
        return sum == (uint8_t)(d & 0xFF);
    }
};
