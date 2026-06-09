#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * MERCEDES car alarm encoder.
 * Proprietary rolling-code protocol.
 *
 * Timing: 380/760 µs
 * Modulation: FSK
 * Data: 64-bit rolling code
 * Repeat: 130+
 * Frequency: EU 433 MHz
 *
 * Mercedes-Benz uses custom rolling-code algorithm with
 * FSK modulation for all vehicle classes.
 */
class FProtoEncoderMercedes : public FProtoEncoderBase {
public:
    FProtoEncoderMercedes() {
        protocol_name = "MERCEDES";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 380;
        bit_count = 64;
        repeat_count = 130;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        if (input_len < 8 || input_len > 8) {
            return false;
        }

        std::memcpy(packet, input, 8);
        *pkt_len = 8;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 380;
        const uint32_t te_long = 760;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;

        for (int i = 0; i < 10; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        }

        for (uint8_t i = 0; i < 64; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
