#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// Modulation types
enum class ProtoModulation : uint8_t {
    OOK = 0,        // On-Off Keying (simple on/off)
    FSK = 1,        // Frequency Shift Keying (2 frequencies)
    Manchester = 2, // Manchester encoding
    GFSK = 3        // Gaussian FSK
};

// Frequency bands
enum class ProtoFrequencyBand : uint8_t {
    EU_433 = 0,     // 433.05 MHz
    EU_868 = 1,     // 868.35 MHz
    EU_434 = 2,     // 434.42 MHz
    US_315 = 3      // 315.00 MHz
};

// Protocol flags (matching SubGhz convention)
typedef uint16_t ProtoFlags;
static constexpr ProtoFlags PROTO_FLAG_433 = (1 << 0);
static constexpr ProtoFlags PROTO_FLAG_868 = (1 << 1);
static constexpr ProtoFlags PROTO_FLAG_315 = (1 << 2);
static constexpr ProtoFlags PROTO_FLAG_DECODABLE = (1 << 3);
static constexpr ProtoFlags PROTO_FLAG_SENDABLE = (1 << 4);

// Level/Duration pair for modulation (same as SubGhz LevelDuration)
struct LevelDuration {
    uint32_t duration;  // Microseconds
    uint8_t level;      // 0=LOW, 1=HIGH
};

// Generic block data (from SubGhz)
struct ProtoBlockGeneric {
    uint32_t serial;           // Device ID
    uint8_t btn;               // Button/command (4-bit)
    uint32_t cnt;              // Counter/repeat
    uint64_t data;             // Full encoded data
    uint16_t data_count_bit;   // Total bits in data
};

// Encoder block (buffer for LevelDuration modulation)
struct ProtoBlockEncoder {
    LevelDuration* upload;      // Modulated signal: {duration, level} pairs
    size_t size_upload;         // Allocated size
    size_t count_upload;        // Current fill level
    uint32_t repeat;            // Transmission repeats
    bool is_running;            // Transmission active
};

/**
 * Base class for all RF protocol encoders.
 * Converts protocol data (serial, button, counter) → modulated RF signal.
 */
class FProtoEncoderBase {
   public:
    virtual ~FProtoEncoderBase() = default;

    // === Interface (must implement) ===

    /**
     * Build packet from input data.
     * @param input Protocol-specific input (e.g., button code)
     * @param input_len Input length
     * @param packet Output packet buffer
     * @param pkt_len Output packet length
     * @return true if successful
     */
    virtual bool build_packet(const uint8_t* input, size_t input_len,
                             uint8_t* packet, size_t* pkt_len) = 0;

    /**
     * Encode packet to modulated signal.
     * @param packet Input packet data
     * @param pkt_len Packet length
     * @param durations Output: duration array
     * @param levels Output: level array (0=LOW, 1=HIGH)
     * @param out_len Output: number of transitions
     */
    virtual void encode(const uint8_t* packet, size_t pkt_len,
                       uint32_t* durations, uint8_t* levels, size_t* out_len) = 0;

    /**
     * Calculate checksum/CRC (if applicable).
     * @param data Input data
     * @param len Data length
     * @return checksum value
     */
    virtual uint16_t calculate_checksum(const uint8_t* data, size_t len) {
        return 0;  // Override if needed
    }

    // === Properties ===
    ProtoModulation modulation = ProtoModulation::OOK;
    ProtoFrequencyBand frequency_band = ProtoFrequencyBand::EU_433;
    uint32_t frequency_hz = 433050000;
    uint32_t symbol_duration_us = 48;
    uint32_t sensor_type = 0;
    size_t bit_count = 64;
    size_t repeat_count = 1;

    const char* protocol_name = "Unknown";

   protected:
    // === Helpers for common operations ===

    /**
     * CRC-16 using PANDORA MAX algorithm.
     * Used by: KEELOQ, STARLINE, many others.
     */
    static uint16_t crc16_pandora(const uint8_t* data, size_t len) {
        uint16_t crc = 0;
        for (size_t i = 0; i < len; i++) {
            crc = (crc << 8 | crc >> 8) ^ data[i];
            crc = crc ^ (crc >> 4);
            crc = crc ^ (crc << 12);
            crc = crc ^ ((crc & 0xff) << 5);
        }
        return crc;
    }

    /**
     * Simple checksum: sum of all bytes.
     * Used by: STARLINE, CAME, others.
     */
    static uint8_t checksum_add_bytes(const uint8_t* data, size_t len) {
        uint32_t sum = 0;
        for (size_t i = 0; i < len; i++) {
            sum += data[i];
        }
        return (uint8_t)sum;
    }

    /**
     * Read bit from byte array.
     * @param data Byte array
     * @param bit Bit position (0 = LSB of first byte)
     * @return 0 or 1
     */
    static inline bool bit_read(uint64_t value, uint8_t bit) {
        return (value >> bit) & 1;
    }

    /**
     * Write bit to byte array.
     */
    static inline void bit_write(uint64_t& value, uint8_t bit, bool val) {
        if (val) {
            value |= (1ULL << bit);
        } else {
            value &= ~(1ULL << bit);
        }
    }

    /**
     * Manchester encode a bit.
     * 0 → {short_high, long_low}
     * 1 → {long_high, short_low}
     */
    static void manchester_encode_bit(bool bit, uint32_t te_short, uint32_t te_long,
                                     uint32_t* durations, uint8_t* levels, size_t& idx) {
        if (bit) {
            // Bit 1
            durations[idx] = te_long;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        } else {
            // Bit 0
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_long;
            levels[idx++] = 0;
        }
    }

    /**
     * FSK encode a bit.
     * 0 → {te_short_high, te_long_low}
     * 1 → {te_long_high, te_short_low}
     */
    static void fsk_encode_bit(bool bit, uint32_t te_short, uint32_t te_long,
                              uint32_t* durations, uint8_t* levels, size_t& idx) {
        if (bit) {
            // Bit 1
            durations[idx] = te_long;
            levels[idx++] = 1;
            durations[idx] = te_short;
            levels[idx++] = 0;
        } else {
            // Bit 0
            durations[idx] = te_short;
            levels[idx++] = 1;
            durations[idx] = te_long;
            levels[idx++] = 0;
        }
    }

    /**
     * OOK encode a bit (on-off keying).
     * Bit value determines pulse width, low period fixed.
     */
    static void ook_encode_bit(bool bit, uint32_t te_short, uint32_t te_long,
                              uint32_t* durations, uint8_t* levels, size_t& idx) {
        if (bit) {
            // Bit 1: long pulse
            durations[idx] = te_long;
            levels[idx++] = 1;
        } else {
            // Bit 0: short pulse
            durations[idx] = te_short;
            levels[idx++] = 1;
        }
        // Gap between bits
        durations[idx] = te_short;
        levels[idx++] = 0;
    }
};
