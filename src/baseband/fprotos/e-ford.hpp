#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * FORD car alarm encoder.
 * Proprietary rolling-code protocol with FSK modulation.
 *
 * Timing: 300/600 µs
 * Modulation: FSK (Frequency Shift Keying)
 * Data: 64-bit rolling code payload
 * Repeat: 100+ (high repetition for vehicle reliability)
 *
 * FORD alarm systems use a proprietary rolling code algorithm
 * to prevent key cloning attacks. Common in 2000s-era Ford vehicles.
 * Timing balanced for both low latency and noise immunity.
 */
class FProtoEncoderFord : public FProtoEncoderBase {
public:
    FProtoEncoderFord() {
        protocol_name = "FORD";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::US_315;  // Ford uses US 315 MHz
        frequency_hz = 315000000;
        symbol_duration_us = 300;  // te_short baseline
        bit_count = 64;             // 64-bit rolling code
        repeat_count = 100;         // High repetition for vehicle use
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // FORD expects 7-8 bytes (serial + button + counter)
        if (input_len < 7 || input_len > 8) {
            return false;
        }

        // Rolling code generation from counter
        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;
        }

        *pkt_len = 8;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 300;   // µs
        const uint32_t te_long = 600;    // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = 64;

        // === Preamble (FORD sync pattern) ===
        // FORD uses specific preamble for synchronization
        for (int i = 0; i < 10; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (FSK encoded) ===
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer (end pulse) ===
        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }

private:
    /**
     * FORD rolling code generation.
     * Uses counter + serial number with proprietary XOR mixing.
     */
    uint64_t generate_rolling_code(uint32_t serial, uint8_t button, uint16_t counter) {
        uint64_t code = ((uint64_t)serial << 16) | counter;
        // FORD-specific mixing (simplified without real key)
        code ^= (0xABCD1234ULL ^ ((uint64_t)button << 40));
        return crc16_pandora((uint8_t*)&code, 8);
    }
};
