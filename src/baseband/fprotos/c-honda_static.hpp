#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Honda Static — symbol-based Manchester, 64 bits (8 bytes)
// Short pulse: 28-98µs = 1 symbol; Long pulse: 61-191µs = 2 symbols
// Sync: ≥700µs resets symbol buffer. After ≥36 symbols: try to decode.
// Preamble = alternating transitions (≤19 consecutive same-level transitions).
// Ported from protopirate honda_static.c

#define HONDA_STATIC_BIT_COUNT       64U
#define HONDA_STATIC_MIN_SYMBOLS     36U
#define HONDA_STATIC_SHORT_BASE_US   28U
#define HONDA_STATIC_SHORT_SPAN_US   70U
#define HONDA_STATIC_LONG_BASE_US    61U
#define HONDA_STATIC_LONG_SPAN_US    130U
#define HONDA_STATIC_SYMBOL_CAPACITY 512U
#define HONDA_STATIC_PREAMBLE_MAX_TRANSITIONS 19U
#define HONDA_STATIC_SYMBOL_BYTE_COUNT ((HONDA_STATIC_SYMBOL_CAPACITY + 7U) / 8U)

class FProtoSubCarHondaStatic : public FProtoSubCarBase {
   public:
    FProtoSubCarHondaStatic() {
        sensorType = FPC_HONDA_STATIC;
        te_short = 28;
        te_long = 130;
        te_delta = 70;
        min_count_bit_for_found = HONDA_STATIC_BIT_COUNT;
    }

    void feed(bool level, uint32_t duration) {
        const uint8_t sym = level ? 1U : 0U;

        // Short pulse: one symbol
        if(duration >= HONDA_STATIC_SHORT_BASE_US &&
           (duration - HONDA_STATIC_SHORT_BASE_US) <= HONDA_STATIC_SHORT_SPAN_US) {
            if(symbols_count < HONDA_STATIC_SYMBOL_CAPACITY) {
                symbol_set(symbols_count, sym);
                symbols_count++;
            }
            return;
        }

        // Long pulse: two symbols
        if(duration >= HONDA_STATIC_LONG_BASE_US &&
           (duration - HONDA_STATIC_LONG_BASE_US) <= HONDA_STATIC_LONG_SPAN_US) {
            if((uint16_t)(symbols_count + 2U) <= HONDA_STATIC_SYMBOL_CAPACITY) {
                symbol_set(symbols_count, sym); symbols_count++;
                symbol_set(symbols_count, sym); symbols_count++;
            }
            return;
        }

        // Gap / sync — try to decode accumulated symbols
        if(symbols_count >= HONDA_STATIC_MIN_SYMBOLS) {
            if(!parse_symbols(true)) {
                parse_symbols(false);
            }
        }
        symbols_count = 0;
    }

   private:
    uint8_t symbols[HONDA_STATIC_SYMBOL_BYTE_COUNT] = {};
    uint16_t symbols_count = 0;

    void symbol_set(uint16_t index, uint8_t v) {
        const uint8_t byte_index = (uint8_t)(index >> 3U);
        const uint8_t shift = (uint8_t)(~index) & 0x07U;
        if(v) symbols[byte_index] |= (uint8_t)(1U << shift);
        else  symbols[byte_index] &= (uint8_t)~(1U << shift);
    }

    uint8_t symbol_get(uint16_t index) const {
        return (symbols[index >> 3U] >> ((uint8_t)(~index) & 0x07U)) & 1U;
    }

    // Read up to 32 bits from symbol stream using bit positions
    static uint8_t get_bits(const uint8_t* data, uint8_t start, uint8_t count) {
        uint32_t value = 0;
        for(uint8_t i = 0; i < count; i++) {
            const uint8_t bit_index = start + i;
            const uint8_t byte = data[bit_index >> 3U];
            const uint8_t shift = (uint8_t)(~bit_index) & 0x07U;
            value = (value << 1U) | ((byte >> shift) & 1U);
        }
        return (uint8_t)value;
    }
    static uint32_t get_bits_u32(const uint8_t* data, uint8_t start, uint8_t count) {
        uint32_t value = 0;
        for(uint8_t i = 0; i < count; i++) {
            const uint8_t bit_index = start + i;
            const uint8_t byte = data[bit_index >> 3U];
            const uint8_t shift = (uint8_t)(~bit_index) & 0x07U;
            value = (value << 1U) | ((byte >> shift) & 1U);
        }
        return value;
    }

    static bool is_valid_button(uint8_t button) {
        if(button > 9U) return false;
        return ((0x336U >> button) & 1U) != 0U;
    }
    static bool is_valid_serial(uint32_t serial) {
        return (serial != 0U) && (serial != 0x0FFFFFFFU);
    }

    // Manchester decode: scan for preamble end, then extract 64 bits
    bool manchester_pack_64(uint16_t start_pos, bool inverted, uint8_t packet[9], uint16_t* out_bits) {
        memset(packet, 0, 9);
        uint16_t pos = start_pos;
        uint16_t bit_count = 0;

        while((uint16_t)(pos + 1U) < symbols_count) {
            if(bit_count >= HONDA_STATIC_BIT_COUNT) break;
            const uint8_t a = symbol_get(pos);
            const uint8_t b = symbol_get(pos + 1U);
            if(a == b) { pos++; continue; }
            bool bit = inverted ? (a == 0U && b == 1U) : (a == 1U && b == 0U);
            if(bit) packet[bit_count >> 3U] |= (uint8_t)(1U << (((uint8_t)~bit_count) & 0x07U));
            bit_count++;
            pos += 2U;
        }
        if(out_bits) *out_bits = bit_count;
        return bit_count >= HONDA_STATIC_BIT_COUNT;
    }

    bool validate_forward(const uint8_t packet[9], uint32_t& serial_out, uint32_t& counter_out, uint8_t& button_out) {
        const uint8_t button = get_bits(packet, 0, 4);
        const uint32_t serial = get_bits_u32(packet, 4, 28);
        const uint32_t counter = get_bits_u32(packet, 32, 24);
        const uint8_t checksum = get_bits(packet, 56, 8);
        uint8_t checksum_calc = 0;
        for(size_t i = 0; i < 7; i++) checksum_calc ^= packet[i];
        if(checksum != checksum_calc) return false;
        if(!is_valid_button(button)) return false;
        if(!is_valid_serial(serial)) return false;
        button_out = button; serial_out = serial; counter_out = counter;
        return true;
    }

    bool validate_reverse(const uint8_t packet[9], uint32_t& serial_out, uint32_t& counter_out, uint8_t& button_out) {
        uint8_t reversed[9];
        for(size_t i = 0; i < 9; i++) {
            uint8_t b = packet[i];
            b = (uint8_t)(((b & 0xAAU) >> 1) | ((b & 0x55U) << 1));
            b = (uint8_t)(((b & 0xCCU) >> 2) | ((b & 0x33U) << 2));
            b = (uint8_t)(((b & 0xF0U) >> 4) | ((b & 0x0FU) << 4));
            reversed[i] = b;
        }
        const uint8_t button = get_bits(reversed, 0, 4);
        const uint32_t serial = get_bits_u32(reversed, 4, 28);
        const uint32_t counter = get_bits_u32(reversed, 32, 24);
        if(!is_valid_button(button)) return false;
        if(!is_valid_serial(serial)) return false;
        button_out = button; serial_out = serial; counter_out = counter;
        return true;
    }

    // Pack compact: button(4b) | serial(28b) | counter(24b) = 56 bits + checksum padding
    static uint64_t pack_compact(uint8_t button, uint32_t serial, uint32_t counter) {
        uint8_t compact[8] = {};
        compact[0] = button & 0x0FU;
        compact[1] = (uint8_t)(serial >> 20U);
        compact[2] = (uint8_t)(serial >> 12U);
        compact[3] = (uint8_t)(serial >> 4U);
        compact[4] = (uint8_t)(serial << 4U);
        compact[5] = (uint8_t)(counter >> 16U);
        compact[6] = (uint8_t)(counter >> 8U);
        compact[7] = (uint8_t)counter;
        uint64_t v = 0;
        for(int i = 0; i < 8; i++) v = (v << 8) | compact[i];
        return v;
    }

    bool parse_symbols(bool inverted) {
        uint16_t index = 1U;
        uint16_t transitions = 0U;

        while(index < symbols_count) {
            if(symbol_get(index) != symbol_get(index - 1U)) {
                transitions++;
            } else {
                if(transitions > HONDA_STATIC_PREAMBLE_MAX_TRANSITIONS) break;
                transitions = 0U;
            }
            index++;
        }
        if(index >= symbols_count) return false;

        // Skip any run of same-level at end of preamble
        while((uint16_t)(index + 1U) < symbols_count &&
              symbol_get(index) == symbol_get(index + 1U)) {
            index++;
        }
        const uint16_t data_start = index;

        uint8_t packet[9] = {};
        uint16_t bit_count = 0;
        if(!manchester_pack_64(data_start, inverted, packet, &bit_count)) return false;

        uint32_t serial = 0, counter = 0;
        uint8_t button = 0;

        if(validate_forward(packet, serial, counter, button)) {
            commit(button, serial, counter);
            return true;
        }
        if(inverted) return false;
        if(validate_reverse(packet, serial, counter, button)) {
            commit(button, serial, counter);
            return true;
        }
        return false;
    }

    void commit(uint8_t button, uint32_t serial, uint32_t counter) {
        decode_data = pack_compact(button, serial, counter);
        decode_data2 = 0;
        data_count_bit = HONDA_STATIC_BIT_COUNT;
        if(callback) fire_callback();
    }
};
