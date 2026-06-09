#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * BENINCA-868 gate opener encoder.
 * European 868 MHz variant of BENINCA protocol.
 *
 * Timing: 320/640 µs adaptive
 * Modulation: Adaptive FSK
 * Data: 52-56 bits variable
 * Repeat: 50+
 * Frequency: EU 868 MHz
 */
class FProtoEncoderBeninca868 : public FProtoEncoderBase {
public:
    FProtoEncoderBeninca868() {
        protocol_name = "BENINCA-868";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_868;
        frequency_hz = 868350000;
        symbol_duration_us = 320;
        bit_count = 52;
        repeat_count = 50;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        if (input_len < 6 || input_len > 8) {
            return false;
        }

        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;
        }

        *pkt_len = input_len;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 320;
        const uint32_t te_long = 640;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;

        for (int i = 0; i < 6; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        }

        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 4;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
