#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * CAME gate opener encoder.
 * Simple static protocol with OOK modulation.
 *
 * Timing: 700/1400 µs (longest timing of all gate openers)
 * Modulation: OOK (On-Off Keying)
 * Data: 24-25 bit payload
 * Repeat: 5 (minimal repetition needed)
 *
 * CAME is known for simple, reliable gate opener systems.
 * Long timing windows reduce susceptibility to noise and
 * multi-path interference.
 */
class FProtoEncoderCame : public FProtoEncoderBase {
public:
    FProtoEncoderCame() {
        protocol_name = "CAME";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 700;  // te_short baseline (longest of gates)
        bit_count = 25;             // 24-25 bits
        repeat_count = 5;           // Minimal repetition
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // CAME expects 3-4 bytes
        if (input_len < 3 || input_len > 4) {
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
        const uint32_t te_short = 700;   // µs
        const uint32_t te_long = 1400;   // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;

        // === Preamble (start pulse) ===
        // CAME uses simple HIGH pulse to start
        durations[idx] = te_short * 8;
        levels[idx++] = 1;  // HIGH
        durations[idx] = te_short * 2;
        levels[idx++] = 0;  // LOW

        // === Data bits (OOK encoded) ===
        // Bit 1: te_long HIGH + te_short LOW
        // Bit 0: te_short HIGH + te_long LOW
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            ook_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer ===
        durations[idx] = te_short * 4;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        // CAME uses simple byte checksum
        return checksum_add_bytes(data, len);
    }
};
