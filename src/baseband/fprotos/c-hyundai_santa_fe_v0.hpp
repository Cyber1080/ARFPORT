#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Hyundai Santa Fe V0 — PWM OOK, 80 bits
// Found on: Hyundai Santa Fe / Solaris 2013-2016 (TRW fob variant)
// Sync: ~375µs HI + ~12000µs LO
// Bit 1: ~375µs HI + ~125µs LO; Bit 0: ~125µs HI + ~375µs LO
// Frame: [rolling:32][serial:24][counter:8][button:8][crc8:8]
// CRC8: poly=0x31, init=0xFF over bytes[0..8]
// Source: FoxFW auto_rke_protocols.c (SantaFe section), ported to ARF subcarbase format.

typedef enum {
    SFeV0StepReset = 0,
    SFeV0StepSync,
    SFeV0StepData,
} SFeV0Step;

#define SFE_DATA_BITS     80u
#define SFE_DATA_BYTES    10u
#define SFE_SYNC_HI_US    375u
#define SFE_SYNC_LO_US    12000u
#define SFE_BIT1_HI_US    375u
#define SFE_BIT0_HI_US    125u
#define SFE_BIT1_LO_US    125u
#define SFE_BIT0_LO_US    375u
#define SFE_TOL_PCT        15u

class FProtoSubCarHyundaiSantaFeV0 : public FProtoSubCarBase {
   public:
    FProtoSubCarHyundaiSantaFeV0() {
        sensorType = FPC_HYUNDAISANTAFEV0;
        te_short = 125;
        te_long = 375;
        te_delta = 60;
        min_count_bit_for_found = SFE_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case SFeV0StepReset:
                // Sync starts with ~375µs HIGH (same as bit-1 HIGH; differentiated by next LOW)
                if (level && in_tol(duration, SFE_SYNC_HI_US)) {
                    te_last = duration;
                    parser_step = SFeV0StepSync;
                }
                break;

            case SFeV0StepSync:
                if (!level && in_tol(duration, SFE_SYNC_LO_US)) {
                    decode_count_bit = 0;
                    memset(raw_bytes, 0, sizeof(raw_bytes));
                    parser_step = SFeV0StepData;
                } else {
                    parser_step = SFeV0StepReset;
                }
                break;

            case SFeV0StepData:
                if (level) {
                    te_last = duration;
                } else {
                    bool bit_val;
                    if (in_tol(te_last, SFE_BIT1_HI_US) && in_tol(duration, SFE_BIT1_LO_US)) {
                        bit_val = true;
                    } else if (in_tol(te_last, SFE_BIT0_HI_US) && in_tol(duration, SFE_BIT0_LO_US)) {
                        bit_val = false;
                    } else {
                        parser_step = SFeV0StepReset;
                        break;
                    }
                    uint8_t bi = (uint8_t)(decode_count_bit / 8u);
                    uint8_t bp = (uint8_t)(7u - (decode_count_bit % 8u));
                    if (bit_val) raw_bytes[bi] |= (uint8_t)(1u << bp);
                    decode_count_bit++;
                    if (decode_count_bit >= SFE_DATA_BITS) {
                        if (crc_ok()) {
                            // Pack: decode_data = bytes 0-7, decode_data2 = bytes 8-9
                            decode_data = 0;
                            for (int i = 0; i < 8; i++)
                                decode_data = (decode_data << 8) | raw_bytes[i];
                            decode_data2 = ((uint64_t)raw_bytes[8] << 8) | raw_bytes[9];
                            data_count_bit = SFE_DATA_BITS;
                            if (callback) fire_callback();
                        }
                        parser_step = SFeV0StepReset;
                    }
                }
                break;
        }
    }

   private:
    uint8_t raw_bytes[SFE_DATA_BYTES] = {};

    bool in_tol(uint32_t measured, uint32_t ref) {
        uint32_t d = DURATION_DIFF(measured, ref);
        return (d * 100u) <= (ref * (uint32_t)SFE_TOL_PCT);
    }

    bool crc_ok() {
        uint8_t crc = 0xFFu;
        for (uint8_t i = 0; i < 9u; i++) {
            crc ^= raw_bytes[i];
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x31u) : (uint8_t)(crc << 1);
        }
        return crc == raw_bytes[9];
    }
};
