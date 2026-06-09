#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * SUBARU car alarm encoder.
 * Rolling-code protocol with Manchester encoding.
 *
 * Timing: 400/800 µs
 * Modulation: Manchester (self-clocking)
 * Data: 64-bit rolling code payload
 * Repeat: 150+ (highest repetition for maximum reliability)
 *
 * SUBARU uses Manchester encoding for inherent clock recovery
 * and improved noise immunity. Longer repetition count ensures
 * reliable operation in vehicles with metallic body RF shielding.
 */
class FProtoEncoderSubaru : public FProtoEncoderBase {
public:
    FProtoEncoderSubaru() {
        protocol_name = "SUBARU";
        modulation = ProtoModulation::Manchester;
        frequency_band = ProtoFrequencyBand::US_315;  // Subaru US-market uses 315 MHz
        frequency_hz = 315000000;
        symbol_duration_us = 400;  // te_short baseline
        bit_count = 64;             // 64-bit rolling code
        repeat_count = 150;         // Maximum repetition for vehicle use
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // SUBARU expects 8 bytes (serial + button + counter + CRC)
        if (input_len < 8 || input_len > 8) {
            return false;
        }

        std::memcpy(packet, input, 8);
        *pkt_len = 8;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 400;   // µs
        const uint32_t te_long = 800;    // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = 64;

        // === Preamble (Manchester sync) ===
        // Manchester inherently provides timing information,
        // but preamble helps receiver lock onto signal
        for (int i = 0; i < 8; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (Manchester encoded) ===
        // Manchester: 0 → short_high + long_low
        //             1 → long_high + short_low
        // Guaranteed transition on every bit
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            manchester_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer (Manchester end state) ===
        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }

private:
    /**
     * SUBARU rolling code generation.
     * Uses Manchester-compatible counter mixing.
     */
    uint64_t generate_rolling_code(uint32_t serial, uint8_t button, uint16_t counter) {
        uint64_t code = ((uint64_t)serial << 20) | counter;
        code ^= ((uint64_t)button << 48);
        // Manchester-compatible CRC
        return crc16_pandora((uint8_t*)&code, 8);
    }
};
