#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * MAGICAR-868 gate opener encoder.
 * European 868 MHz variant of MAGICAR protocol.
 *
 * Timing: 250/500 µs (same as 433 MHz variant)
 * Modulation: FSK
 * Data: 64-bit rolling code payload
 * Repeat: 100+
 * Frequency: EU 868 MHz (European ISM band)
 *
 * MAGICAR-868 is the European market version, operating in
 * the 868 MHz band instead of 433 MHz. Identical timing/encoding.
 */
class FProtoEncoderMagicar868 : public FProtoEncoderBase {
public:
    FProtoEncoderMagicar868() {
        protocol_name = "MAGICAR-868";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_868;
        frequency_hz = 868350000;  // EU 868 MHz band
        symbol_duration_us = 250;
        bit_count = 64;
        repeat_count = 100;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        if (input_len < 7 || input_len > 8) {
            return false;
        }

        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;
        }

        *pkt_len = 8;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 250;
        const uint32_t te_long = 500;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;

        for (int i = 0; i < 12; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        }

        for (uint8_t i = 0; i < 64; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 6;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
