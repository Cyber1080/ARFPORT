#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * HONDA car alarm encoder.
 * Rolling-code protocol with OOK modulation.
 *
 * Timing: 500/1000 µs
 * Modulation: OOK (On-Off Keying)
 * Data: 48-bit rolling code payload
 * Repeat: 10 (compact transmission)
 *
 * HONDA uses a proprietary rolling-code algorithm with OOK modulation.
 * The relatively short bit count and low repetition allows fast
 * transmission, useful for quick lock/unlock response.
 */
class FProtoEncoderHonda : public FProtoEncoderBase {
public:
    FProtoEncoderHonda() {
        protocol_name = "HONDA";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::US_315;  // Honda uses US 315 MHz
        frequency_hz = 315000000;
        symbol_duration_us = 500;  // te_short baseline
        bit_count = 48;             // 48-bit payload (smaller than others)
        repeat_count = 10;          // Compact repetition
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // HONDA expects 6-8 bytes
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
        const uint32_t te_short = 500;   // µs
        const uint32_t te_long = 1000;   // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;

        // === Preamble (HONDA start pattern) ===
        durations[idx] = te_short * 12;
        levels[idx++] = 1;  // HIGH
        durations[idx] = te_short * 2;
        levels[idx++] = 0;  // LOW

        // === Data bits (OOK encoded) ===
        // OOK: bit 1 = long_high + short_low
        //      bit 0 = short_high + long_low
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
        return checksum_add_bytes(data, len);
    }

private:
    /**
     * HONDA rolling code generation.
     * Uses OOK-compatible counter mixing.
     */
    uint64_t generate_rolling_code(uint32_t serial, uint8_t button, uint16_t counter) {
        uint64_t code = ((uint64_t)serial << 16) | counter;
        code ^= ((uint64_t)button << 32);
        return code;
    }
};
