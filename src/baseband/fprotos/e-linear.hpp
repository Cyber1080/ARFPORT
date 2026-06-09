#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * LINEAR/Liftmaster garage door encoder.
 * Proprietary protocol with rolling code.
 *
 * Timing: 500/1000 µs
 * Modulation: OOK
 * Data: 40-bit rolling code payload
 * Repeat: 10
 * Frequency: US 315 MHz
 *
 * Linear (now part of Nortek) manufactures Liftmaster garage doors
 * with proprietary rolling-code algorithm. Common in North America.
 */
class FProtoEncoderLinear : public FProtoEncoderBase {
public:
    FProtoEncoderLinear() {
        protocol_name = "LINEAR";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::US_315;
        frequency_hz = 315000000;
        symbol_duration_us = 500;
        bit_count = 40;
        repeat_count = 10;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        if (input_len < 5 || input_len > 8) {
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
        const uint32_t te_short = 500;
        const uint32_t te_long = 1000;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;
        uint8_t count_bit = (pkt_len * 8 < 40) ? (pkt_len * 8) : 40;

        durations[idx] = te_short * 13;
        levels[idx++] = 1;
        durations[idx] = te_short;
        levels[idx++] = 0;

        for (uint8_t i = 0; i < count_bit; i++) {
            bool bit_val = bit_read(data, i);
            ook_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 4;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return crc16_pandora(data, len);
    }
};
