#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * GARAGE-DOOR generic encoder.
 * Simple garage door opener with static code.
 *
 * Timing: 600/1200 µs
 * Modulation: OOK
 * Data: 24-bit payload
 * Repeat: 3 (minimal, garage door response is immediate)
 * Frequency: US 315 MHz (standard for garage openers)
 *
 * Generic garage door opener protocol used by multiple
 * manufacturers. Very simple, no rolling code.
 */
class FProtoEncoderGarageDoor : public FProtoEncoderBase {
public:
    FProtoEncoderGarageDoor() {
        protocol_name = "GARAGE-DOOR";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::US_315;
        frequency_hz = 315000000;
        symbol_duration_us = 600;
        bit_count = 24;
        repeat_count = 3;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
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
        const uint32_t te_short = 600;
        const uint32_t te_long = 1200;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < 24) ? (pkt_len * 8) : 24;

        // Garage door preamble
        durations[idx] = te_short * 10;
        levels[idx++] = 1;
        durations[idx] = te_short;
        levels[idx++] = 0;

        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            ook_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 5;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return 0;  // Garage door openers typically have no checksum
    }
};
