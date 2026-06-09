#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Mitsubishi V0 — PWM OOK, 96 bits
// te_short=250µs, te_long=500µs (short-high+long-low=1, long-high+short-low=0)
// Original decoder by @lupettohf (protopirate), ported to ARF subcarbase format.

typedef enum {
    MitsubishiV0StepReset = 0,
    MitsubishiV0StepDataSave,
    MitsubishiV0StepDataCheck,
} MitsubishiV0Step;

#define MITSUBISHI_V0_BIT_COUNT 96

class FProtoSubCarMitsubishiV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarMitsubishiV0() {
        sensorType = FPC_MITSUBISHIV0;
        te_short = 250;
        te_long = 500;
        te_delta = 100;
        min_count_bit_for_found = MITSUBISHI_V0_BIT_COUNT;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case MitsubishiV0StepReset:
                if (level) {
                    te_last = duration;
                    parser_step = MitsubishiV0StepDataCheck;
                }
                break;

            case MitsubishiV0StepDataSave:
                if (level) {
                    te_last = duration;
                    parser_step = MitsubishiV0StepDataCheck;
                } else {
                    reset_payload();
                    parser_step = MitsubishiV0StepReset;
                }
                break;

            case MitsubishiV0StepDataCheck:
                if (!level) {
                    if (collect_pair(te_last, duration)) {
                        if (bit_count >= MITSUBISHI_V0_BIT_COUNT) {
                            publish_frame();
                            reset_payload();
                            parser_step = MitsubishiV0StepReset;
                        } else {
                            parser_step = MitsubishiV0StepDataSave;
                        }
                    } else {
                        reset_payload();
                        parser_step = MitsubishiV0StepReset;
                    }
                } else {
                    te_last = duration;
                }
                break;
        }
    }

   private:
    uint16_t bit_count = 0;
    uint8_t raw_bytes[12] = {};

    void reset_payload() {
        bit_count = 0;
        memset(raw_bytes, 0, sizeof(raw_bytes));
    }

    bool collect_pair(uint32_t high, uint32_t low) {
        bool bit_value;
        if (DURATION_DIFF(high, te_short) < te_delta && DURATION_DIFF(low, te_long) < te_delta) {
            bit_value = true;
        } else if (DURATION_DIFF(high, te_long) < te_delta && DURATION_DIFF(low, te_short) < te_delta) {
            bit_value = false;
        } else {
            return false;
        }

        if (bit_count < MITSUBISHI_V0_BIT_COUNT) {
            if (bit_value) {
                uint8_t byte_idx = bit_count >> 3;
                uint8_t bit_pos = 7 - (bit_count & 7);
                raw_bytes[byte_idx] |= (1u << bit_pos);
            }
            bit_count++;
        }
        return true;
    }

    void unscramble(uint8_t* p) {
        for (uint8_t i = 0; i < 8; i++) p[i] = (uint8_t)~p[i];
        uint8_t hi = p[4], lo = p[5];
        uint8_t mask3 = ((hi & 0xAAu) | (lo & 0x55u)) ^ ((lo & 0xAAu) | (hi & 0x55u));
        for (uint8_t i = 0; i < 5; i++) p[i] ^= mask3;
    }

    void publish_frame() {
        uint8_t payload[12];
        memcpy(payload, raw_bytes, 12);
        unscramble(payload);

        // Pack into decode_data (bytes 0-7) and decode_data2 (bytes 8-11)
        decode_data = 0;
        for (int i = 0; i < 8; i++)
            decode_data = (decode_data << 8) | payload[i];
        decode_data2 = 0;
        for (int i = 8; i < 12; i++)
            decode_data2 = (decode_data2 << 8) | payload[i];

        data_count_bit = bit_count;
        if (callback) callback(this);
    }
};
