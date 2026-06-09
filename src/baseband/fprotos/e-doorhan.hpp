#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * DOORHAN gate opener encoder.
 * Static protocol with OOK modulation.
 *
 * Timing: 800/1600 µs (2× HORMANN timing)
 * Modulation: OOK (On-Off Keying)
 * Data: 18-20 bit payload
 * Repeat: 10
 *
 * DOORHAN is a European gate opener protocol commonly found in
 * industrial and residential gate systems. The 2× timing
 * (compared to HORMANN) provides extra noise immunity.
 */
class FProtoEncoderDoorhan : public FProtoEncoderBase {
public:
    FProtoEncoderDoorhan() {
        protocol_name = "DOORHAN";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 800;  // te_short baseline
        bit_count = 20;             // 18-20 bits
        repeat_count = 10;          // Standard repetition
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // DOORHAN expects 2-4 bytes
        if (input_len < 2 || input_len > 4) {
            return false;
        }

        std::memcpy(packet, input, input_len);
        if (input_len < 4) {
            packet[input_len] = 0;
        }

        *pkt_len = input_len;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 800;   // µs (2× HORMANN)
        const uint32_t te_long = 1600;   // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;

        // === Preamble ===
        // DOORHAN starts with extended HIGH pulse (timing synchronization)
        durations[idx] = te_short * 16;
        levels[idx++] = 1;  // HIGH
        durations[idx] = te_short * 2;
        levels[idx++] = 0;  // LOW

        // === Data bits (OOK encoded) ===
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            ook_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer ===
        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
