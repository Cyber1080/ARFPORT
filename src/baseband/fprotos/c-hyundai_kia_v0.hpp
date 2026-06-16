#pragma once
#include "subcarbase.hpp"

// Hyundai/Kia V0 — PWM OOK, 64 bits (fixed code, no rolling)
// Found on: Hyundai Accent, Rio, Elantra; Kia Rio (~2001-2008)
// Sync: ~312µs HI + ~10400µs LO
// Bit 1: ~728µs HI + ~312µs LO; Bit 0: ~312µs HI + ~728µs LO
// Frame: [serial:32][button_mask:16][checksum:16]
// Checksum: ~(serial[31:16] ^ serial[15:0] ^ button_mask)
// Source: FoxFW auto_rke_protocols.c (HKR section), ported to ARF subcarbase format.

typedef enum {
    HKV0StepReset = 0,
    HKV0StepSync,
    HKV0StepData,
} HKV0Step;

#define HKV0_DATA_BITS    64u
#define HKV0_SYNC_HI_US   312u
#define HKV0_SYNC_LO_US   10400u
#define HKV0_BIT1_HI_US   728u
#define HKV0_BIT0_HI_US   312u
#define HKV0_BIT1_LO_US   312u
#define HKV0_BIT0_LO_US   728u
#define HKV0_TOL_PCT       15u

class FProtoSubCarHyundaiKiaV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarHyundaiKiaV0() {
        sensorType = FPC_HYUNDAIKIA_V0;
        te_short = 312;
        te_long = 728;
        te_delta = 100;
        min_count_bit_for_found = HKV0_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case HKV0StepReset:
                // Sync starts with a short (~312µs) HIGH
                if (level && in_tol(duration, HKV0_SYNC_HI_US)) {
                    te_last = duration;
                    parser_step = HKV0StepSync;
                }
                break;

            case HKV0StepSync:
                // Expect long LOW (~10400µs) to confirm sync
                if (!level && in_tol(duration, HKV0_SYNC_LO_US)) {
                    decode_data = 0;
                    decode_count_bit = 0;
                    parser_step = HKV0StepData;
                } else {
                    parser_step = HKV0StepReset;
                }
                break;

            case HKV0StepData:
                if (level) {
                    te_last = duration;
                } else {
                    bool bit_val;
                    if (in_tol(te_last, HKV0_BIT1_HI_US) && in_tol(duration, HKV0_BIT1_LO_US)) {
                        bit_val = true;
                    } else if (in_tol(te_last, HKV0_BIT0_HI_US) && in_tol(duration, HKV0_BIT0_LO_US)) {
                        bit_val = false;
                    } else {
                        parser_step = HKV0StepReset;
                        break;
                    }
                    decode_data = (decode_data << 1) | (bit_val ? 1u : 0u);
                    decode_count_bit++;
                    if (decode_count_bit == HKV0_DATA_BITS) {
                        if (checksum_ok(decode_data)) {
                            data_count_bit = HKV0_DATA_BITS;
                            if (callback) fire_callback();
                        }
                        parser_step = HKV0StepReset;
                    }
                }
                break;
        }
    }

   private:
    // 15% tolerance check
    bool in_tol(uint32_t measured, uint32_t ref) {
        uint32_t d = DURATION_DIFF(measured, ref);
        return (d * 100u) <= (ref * HKV0_TOL_PCT);
    }

    static bool checksum_ok(uint64_t d) {
        uint32_t serial = (uint32_t)(d >> 32);
        uint16_t button = (uint16_t)((d >> 16) & 0xFFFFu);
        uint16_t rx_ck  = (uint16_t)(d & 0xFFFFu);
        uint16_t calc   = (uint16_t)~((uint16_t)(serial ^ (serial >> 16)) ^ button);
        return rx_ck == calc;
    }
};
