#pragma once

#include "fproto_encoder_base.hpp"
#include <cstring>

/**
 * WIRELESS-SENSOR generic encoder.
 * Weather station / home automation sensor protocol.
 *
 * Timing: 450/900 µs
 * Modulation: OOK
 * Data: 32-bit sensor payload (temp/humidity/state)
 * Repeat: 5
 * Frequency: EU 433 MHz
 *
 * Generic wireless sensor protocol used by many manufacturers
 * for temperature, humidity, water leak, door/window sensors.
 */
class FProtoEncoderWirelessSensor : public FProtoEncoderBase {
public:
    FProtoEncoderWirelessSensor() {
        protocol_name = "WIRELESS-SENSOR";
        modulation = ProtoModulation::OOK;
        frequency_band = ProtoFrequencyBand::EU_433;
        frequency_hz = 433050000;
        symbol_duration_us = 450;
        bit_count = 32;
        repeat_count = 5;
    }

    bool build_packet(const uint8_t* input, size_t input_len,
                     uint8_t* packet, size_t* pkt_len) override {
        if (input_len < 4 || input_len > 4) {
            return false;
        }

        std::memcpy(packet, input, 4);
        *pkt_len = 4;
        return true;
    }

    void encode(const uint8_t* packet, size_t pkt_len,
               uint32_t* durations, uint8_t* levels, size_t* out_len) override {
        const uint32_t te_short = 450;
        const uint32_t te_long = 900;

        uint64_t data = 0;
        for (size_t i = 0; i < pkt_len && i < 8; i++) {
            data |= ((uint64_t)packet[i] << (i * 8));
        }

        size_t idx = 0;

        // Sensor preamble
        for (int i = 0; i < 5; i++) {
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        }

        for (uint8_t i = 0; i < 32; i++) {
            bool bit_val = bit_read(data, i);
            ook_encode_bit(bit_val, te_short, te_long, durations, levels, idx);
        }

        durations[idx] = te_short * 4;
        levels[idx++] = 0;

        *out_len = idx;
    }

    uint16_t calculate_checksum(const uint8_t* data, size_t len) override {
        return checksum_add_bytes(data, len);
    }
};
