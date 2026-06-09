#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * BMW car alarm encoder.
 * Proprietary rolling-code protocol with Manchester encoding.
 *
 * Timing: 360/720 µs
 * Modulation: Manchester
 * Data: 64-bit rolling code
 * Repeat: 140+
 * Frequency: EU 433 MHz
 *
 * BMW uses Manchester encoding for superior noise immunity
 * in luxury vehicles with extensive RF shielding.
 */
class FProtoEncoderBmw : public FProtoEncoderBase {
public:
    FProtoEncoderBmw() {
        protocol_name = "BMW";
        modulation = ProtoModulation::Manchester;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 360;
        bit_count = 64;
        repeat_count = 140;
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
        const uint32_t te_short = 360;
        const uint32_t te_long = 720;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;

        // BMW Manchester preamble
        for (int i = 0; i < 8; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        }

        for (uint8_t i = 0; i < 64; i++) {
            bool bit_val = bit_read(data, i);
            manchester_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
