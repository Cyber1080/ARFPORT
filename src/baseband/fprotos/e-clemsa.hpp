#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * CLEMSA gate opener encoder.
 * Manchester-encoded protocol for robust reception.
 *
 * Timing: 320/640 µs
 * Modulation: Manchester encoding (self-clocking, noise-resistant)
 * Data: 80-bit payload
 * Repeat: 50+
 *
 * CLEMSA uses Manchester encoding which provides inherent clock
 * information and is highly resistant to noise and signal
 * reflections. Commonly used in high-interference environments.
 */
class FProtoEncoderClemsa : public FProtoEncoderBase {
public:
    FProtoEncoderClemsa() {
        protocol_name = "CLEMSA";
        modulation = ProtoModulation::Manchester;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 320;  // te_short baseline
        bit_count = 80;             // 80 bits (10 bytes)
        repeat_count = 50;          // Manchester needs more repetitions
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // CLEMSA expects 10 bytes of input data
        if (input_len < 10 || input_len > 10) {
            return false;
        }

        std::memcpy(packet, input, 10);
        *pkt_len = 10;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 320;   // µs
        const uint32_t te_long = 640;    // µs

        uint64_t data = 0;
        // Process up to 8 bytes from the packet
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < 64) ? (pkt_len * 8) : 64;

        // === Preamble (Manchester sync) ===
        // Manchester encoding provides its own clock signal
        // Use alternating bits to synchronize receiver PLL
        for (int i = 0; i < 8; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (Manchester encoded) ===
        // Manchester: 0 → short_high + long_low
        //             1 → long_high + short_low
        // This provides self-clocking and guaranteed transitions
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            manchester_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer (end pulse) ===
        durations[idx] = te_short * 8;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        // CLEMSA may use different checksum, but default to CRC16
        return crc16_pandora(data, len);
    }
};
