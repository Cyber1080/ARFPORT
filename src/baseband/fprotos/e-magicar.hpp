#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * MAGICAR car alarm encoder.
 * Rolling-code protocol with FSK modulation.
 *
 * Timing: 250/500 µs (fast, low latency)
 * Modulation: FSK (Frequency Shift Keying)
 * Data: 64-bit rolling code payload
 * Repeat: 100+ (high repetition for reliability)
 *
 * MAGICAR is a budget car alarm system using rolling code
 * to prevent replay attacks. Shorter timing allows faster TX,
 * useful for panic button response time.
 */
class FProtoEncoderMagicar : public FProtoEncoderBase {
public:
    FProtoEncoderMagicar() {
        protocol_name = "MAGICAR";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 250;  // te_short baseline (fastest gate/car)
        bit_count = 64;             // 64-bit rolling code
        repeat_count = 100;         // High repetition
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // MAGICAR expects 7-8 bytes (serial + button + counter)
        if (input_len < 7 || input_len > 8) {
            return false;
        }

        // In real implementation, rolling code would be generated
        // from keystore using MAGICAR-specific encryption.
        // For ARFPORT, we'll use simplified counter-based generation.
        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;
        }

        *pkt_len = 8;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 250;   // µs (fastest protocol)
        const uint32_t te_long = 500;    // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = 64;

        // === Preamble (speed sync) ===
        // Fast alternating pattern for clock synchronization
        for (int i = 0; i < 12; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (FSK encoded) ===
        // FSK: fast modulation
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer ===
        durations[idx] = te_short * 6;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }

private:
    /**
     * MAGICAR rolling code generation.
     * Simplified version without manufacturer keystore.
     */
    uint64_t generate_rolling_code(uint32_t serial, uint8_t button, uint16_t counter) {
        uint64_t code = ((uint64_t)serial << 16) | counter;
        code ^= ((uint64_t)button << 32);
        return code;
    }
};
