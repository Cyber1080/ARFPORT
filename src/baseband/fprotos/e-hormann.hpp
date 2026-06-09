#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * HORMANN gate opener encoder.
 * Static protocol (no rolling code) with OOK modulation.
 *
 * Timing: 500/1000 µs (matches PANDORA MAX)
 * Data: 44-bit payload
 * Modulation: On-Off Keying (OOK)
 *
 * Pattern per transmission:
 * - Start: 24×te_short HIGH + 1×te_short LOW
 * - Data: 44 bits (MSB first), each encoded as:
 *   - Bit 1: te_long HIGH + te_short LOW
 *   - Bit 0: te_short HIGH + te_long LOW
 * - Repetitions: 20× (same pattern repeated for reliability)
 * - End: 24×te_short HIGH
 */
class FProtoEncoderHormann : public FProtoEncoderBase {
public:
    FProtoEncoderHormann() {
        protocol_name = "HORMANN";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 500;  // te_short baseline
        bit_count = 44;             // HORMANN uses 44-bit payload
        repeat_count = 10;          // Configured repeat count
    }

    /**
     * Build packet from input data.
     * For static protocols like HORMANN, input is directly copied to packet.
     *
     * @param input Input data (up to 8 bytes = 64 bits)
     * @param input_len Length of input
     * @param packet Output packet buffer
     * @param pkt_len Output packet length
     * @return true if successful
     */
    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        // Validate input length (max 8 bytes for uint64_t data field)
        if (input_len == 0 || input_len > 8) {
            return false;
        }

        // For static protocol, packet = input data
        std::memcpy(packet, input, input_len);
        *pkt_len = input_len;
        return true;
    }

    /**
     * Encode packet to modulated signal (duration + level pairs).
     * Generates the RF waveform for HORMANN OOK transmission.
     *
     * @param packet Input packet data
     * @param pkt_len Packet length
     * @param durations Output: duration array (in microseconds)
     * @param levels Output: level array (0=LOW, 1=HIGH)
     * @param out_len Output: number of transitions
     */
    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        // Timing constants
        const uint32_t te_short = 500;   // µs
        const uint32_t te_long = 1000;   // µs

        // Convert packet bytes to bit array (up to 64 bits)
        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        uint8_t count_bit = (pkt_len * 8 < bit_count) ? (pkt_len * 8) : bit_count;
        size_t idx = 0;

        // Encode 20 repetitions of the pattern (for transmission reliability)
        for (size_t repeat = 0; repeat < 20; repeat++) {
            // === Start bit pattern ===
            // 24×te_short HIGH + 1×te_short LOW
            durations[idx] = te_short * 24;
            levels[idx++] = 1;  // HIGH

            durations[idx] = te_short;
            levels[idx++] = 0;  // LOW

            // === Data bits (MSB first) ===
            // For each bit (starting from MSB):
            for (uint8_t i = count_bit; i > 0; i--) {
                bool bit_val = bit_read(data, i - 1);

                if (bit_val) {
                    // Bit 1: te_long HIGH + te_short LOW
                    durations[idx] = te_long;
                    levels[idx++] = 1;  // HIGH
                    durations[idx] = te_short;
                    levels[idx++] = 0;  // LOW
                } else {
                    // Bit 0: te_short HIGH + te_long LOW
                    durations[idx] = te_short;
                    levels[idx++] = 1;  // HIGH
                    durations[idx] = te_long;
                    levels[idx++] = 0;  // LOW
                }
            }
        }

        // === End bit pattern ===
        // 24×te_short HIGH
        durations[idx] = te_short * 24;
        levels[idx++] = 1;  // HIGH

        *out_len = idx;
    }

    /**
     * HORMANN is a static protocol with no checksum.
     * Override returns 0 (no validation needed).
     */
    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        (void)data;  // Unused
        (void)len;   // Unused
        return 0;    // Static protocol, no CRC
    }
};
