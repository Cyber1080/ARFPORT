#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * STARLINE car alarm encoder.
 * Dynamic protocol with FSK modulation and rolling code.
 *
 * Timing: Variable (adaptive windowing based on signal quality)
 * PANDORA analysis: phase timing 0xa8c-0xd48, 0x4b1-0x6a3
 * Data: 64-bit payload with rolling counter
 *
 * Modulation: FSK (two frequencies)
 * Button codes: 4-bit (0-15 commands)
 * Serial: 28-bit device ID
 * Counter: 16-bit rolling code counter
 *
 * STARLINE uses adaptive timing windows to handle poor reception:
 * - Primary window: ~window_short
 * - Secondary window: ~window_long (2x primary)
 * Tolerance: ±te_delta allows 20-30% timing variation
 */
class FProtoEncoderStarline : public FProtoEncoderBase {
public:
    FProtoEncoderStarline() {
        protocol_name = "STARLINE";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 480;  // Adaptive baseline (~0.48 ms)
        bit_count = 64;
        repeat_count = 150;  // High repetition for rolling code robustness
    }

    /**
     * Build packet from input data.
     * Input: [serial(4)] [button(1)] [counter(2)] = 7 bytes minimum
     *
     * @param input Input data
     * @param input_len Length of input
     * @param packet Output packet buffer (8 bytes)
     * @param pkt_len Output packet length
     * @return true if successful
     */
    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // Require 7-8 bytes
        if (input_len < 7 || input_len > 8) {
            return false;
        }

        // For STARLINE, input is serial + button + counter
        // Output is the same (rolling code would be pre-computed)
        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;
        }

        *pkt_len = 8;
        return true;
    }

    /**
     * Encode packet using FSK with adaptive windowing.
     * STARLINE timing is more flexible than KEELOQ, allowing
     * for signal quality adaptation.
     *
     * @param packet Input packet data (8 bytes)
     * @param pkt_len Packet length
     * @param durations Output: duration array
     * @param levels Output: level array
     * @param out_len Output: number of transitions
     */
    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        // STARLINE adaptive timing (from PANDORA analysis)
        const uint32_t te_short = 300;   // Primary window (~300 µs)
        const uint32_t te_long = 600;    // Secondary window (2x primary)
        const uint32_t te_delta = 100;   // ±100 µs tolerance

        // Convert packet to bit array
        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < 64) ? (pkt_len * 8) : 64;

        // === Preamble (adaptive sync pattern) ===
        // STARLINE uses a series of short pulses to establish timing
        for (int i = 0; i < 8; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (FSK encoded) ===
        // FSK: 0 → short_high+long_low, 1 → long_high+short_low
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer ===
        // End with silence and a final HIGH pulse
        durations[idx] = te_short * 8;  // ~2400 µs silence
        levels[idx++] = 0;
        durations[idx] = te_short;
        levels[idx++] = 1;  // Final pulse

        *out_len = idx;
    }

    /**
     * STARLINE uses CRC-16 for rolling code validation.
     */
    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }

private:
    /**
     * Adaptive timing window adjustment.
     * STARLINE can adjust timing based on signal quality or
     * receiver feedback (not typically used in TX-only mode).
     */
    uint32_t adapt_timing(uint32_t base_time, uint32_t quality_metric) {
        // quality_metric: 0-100 (0=poor, 100=excellent)
        // Poor quality → longer timing windows for better noise immunity
        if (quality_metric < 50) {
            return base_time + (base_time / 4);  // +25% timing window
        }
        return base_time;
    }
};
