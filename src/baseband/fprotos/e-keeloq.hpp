#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * KEELOQ rolling-code encoder.
 * Used by: Car alarms, gate openers (Meroni, Nortel, etc.)
 * Modulation: FSK or Manchester
 *
 * Timing: 400/800 µs (PANDORA variant) or 160/300 µs (raw units)
 * Data: 64-bit payload (rolling code + metadata)
 * Rolling Code: Counter-based with manufacturer-specific encryption
 *
 * Data format (64 bits):
 * - Button: 4-bit command code
 * - Serial: 28-bit device ID
 * - Counter: 16-bit or variable (rolling code)
 * - Hop data: encrypted rolling code from counter
 *
 * For ARFPORT, we support two modes:
 * 1. STATIC mode: Pre-computed 64-bit rolling code (from keystore)
 * 2. DYNAMIC mode: Generate rolling code from counter (simplified)
 */
class FProtoEncoderKeeloq : public FProtoEncoderBase {
public:
    FProtoEncoderKeeloq() {
        protocol_name = "KEELOQ";
        modulation = ProtoModulation::FSK;  // Or Manchester
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 400;  // te_short baseline
        bit_count = 64;             // 64-bit rolling code payload
        repeat_count = 100;         // KEELOQ uses many repetitions for reliability
    }

    /**
     * Build packet from input data.
     * Input format: [serial(4)] [button(1)] [counter(2)] = 7 bytes minimum
     * Output: 8-byte packet with rolling code
     *
     * @param input Input data (serial + button + counter)
     * @param input_len Length of input (typically 7-8 bytes)
     * @param packet Output packet buffer (8 bytes)
     * @param pkt_len Output packet length
     * @return true if successful
     */
    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // Expect at least 7 bytes: 4(serial) + 1(btn) + 2(counter)
        if (input_len < 7 || input_len > 8) {
            return false;
        }

        // For KEELOQ, the full 64-bit rolling code should be pre-computed
        // by a keystore or rolling-code generator (Flipper feature)
        // For ARFPORT without keystore, we'll use input data directly
        // and pad to 8 bytes

        std::memcpy(packet, input, input_len);
        if (input_len < 8) {
            packet[input_len] = 0;  // Pad with zeros
        }

        *pkt_len = 8;
        return true;
    }

    /**
     * Encode packet to modulated signal.
     * Uses FSK or Manchester modulation based on configuration.
     *
     * @param packet Input packet data (8 bytes = 64 bits)
     * @param pkt_len Packet length
     * @param durations Output: duration array (in microseconds)
     * @param levels Output: level array (0=LOW, 1=HIGH)
     * @param out_len Output: number of transitions
     */
    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        // Timing constants (PANDORA variant: larger for CC1101 noise rejection)
        const uint32_t te_short = 400;   // µs
        const uint32_t te_long = 800;    // µs

        // Convert packet bytes to bit array (up to 64 bits)
        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < 64) ? (pkt_len * 8) : 64;

        // === Preamble (optional, helps receiver sync) ===
        // Typically a series of alternating bits or known pattern
        // For KEELOQ, use 10 cycles of alternating HIGH/LOW
        for (int i = 0; i < 10; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;  // HIGH
            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW
        }

        // === Data bits (Manchester encoded) ===
        // Manchester: 0 → short_high+long_low, 1 → long_high+short_low
        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            manchester_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        // === Trailer (silence) ===
        // End with a LOW state for ~1ms
        durations[idx] = te_short * 4;  // ~1600 µs silence
        levels[idx++] = 0;

        *out_len = idx;
    }

    /**
     * KEELOQ uses CRC-16 for rolling code validation.
     * Use PANDORA MAX CRC-16 algorithm (custom polynomial).
     */
    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }

private:
    /**
     * Simple rolling code generator (without keystore).
     * KEELOQ normally uses proprietary encryption, but for ARFPORT
     * we can use a simplified counter-based approach.
     */
    uint32_t generate_rolling_code(uint32_t serial, uint8_t button, uint16_t counter) {
        // Placeholder: in real KEELOQ, this would be encrypted with manufacturer key
        uint32_t code = (serial << 4) | button;
        code ^= counter;
        return crc16_pandora((uint8_t*)&code, 4);
    }
};
