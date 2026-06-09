#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * VOLKSWAGEN (VW/Audi/Skoda) car alarm encoder.
 * Proprietary rolling-code protocol.
 *
 * Timing: 350/700 µs
 * Modulation: FSK
 * Data: 64-bit rolling code
 * Repeat: 120+
 * Frequency: EU 433 MHz
 *
 * VW Group manufacturers (Audi, Skoda, Seat) use similar rolling-code
 * algorithms with FSK modulation. Standard European 433 MHz.
 */
class FProtoEncoderVolkswagen : public FProtoEncoderBase {
public:
    FProtoEncoderVolkswagen() {
        protocol_name = "VOLKSWAGEN";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 350;
        bit_count = 64;
        repeat_count = 120;
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
        const uint32_t te_short = 350;
        const uint32_t te_long = 700;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;

        // VW preamble (9 sync bits)
        for (int i = 0; i < 9; i++) {
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
