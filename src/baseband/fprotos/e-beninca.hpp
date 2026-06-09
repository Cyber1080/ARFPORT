#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * BENINCA gate opener encoder.
 * Adaptive protocol with FSK modulation.
 *
 * Timing: 320/640 µs (adaptive windows)
 * Modulation: Adaptive FSK (frequency-shift with window adjustment)
 * Data: 52-56 bit variable payload
 * Repeat: 50+
 *
 * BENINCA uses adaptive timing windows to handle signal reflections
 * and environmental noise. The timing windows expand/contract based on
 * received signal quality (not applicable in TX-only mode).
 */
class FProtoEncoderBeninca : public FProtoEncoderBase {
public:
    FProtoEncoderBeninca() {
        protocol_name = "BENINCA";
        modulation = ProtoModulation::FSK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 320;  // te_short baseline
        bit_count = 52;             // 52-56 bits variable
        repeat_count = 50;          // Adaptive repetition
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // BENINCA expects 6-8 bytes of input data
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
        // Adaptive timing windows (BENINCA characteristic)
        const uint32_t te_short = 320;   // µs
        const uint32_t te_long = 640;    // µs

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;

        // === Preamble (sync pattern) ===
        // BENINCA uses alternating pattern for receiver synchronization
        for (int i = 0; i < 6; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (FSK encoded) ===
        // FSK: adaptive frequency based on bit value
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            fsk_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer ===
        durations[idx] = te_short * 4;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
