#pragma once
#include "subcarbase.hpp"
#include <cstring>

// Ford V1 — Manchester FM, 136 bits (17 bytes), CRC16-CCITT
// te_short=65µs, te_long=130µs @ 433 MHz FM
// Also covers: Jaguar, Land Rover (identical protocol per PANDORA MAX analysis)
// Preamble: ≥50 alternating 130µs pulses; sync: ≤2 short events before data.
// CRC16-CCITT (poly 0x1021, init 0x0000) over bytes[3..14] → bytes[15:16].
// Ported from protopirate (ford_v1.c) to ARF subcarbase format.

typedef enum {
    FordV1StepReset = 0,
    FordV1StepPreamble,
    FordV1StepSync,
    FordV1StepData,
} FordV1Step;

#define FORD_V1_DATA_BYTES    17
#define FORD_V1_DATA_BITS     136
#define FORD_V1_PREAMBLE_MIN  50
#define FORD_V1_DELTA_LONG    40u
#define FORD_V1_DELTA_SHORT   39u
#define FORD_V1_SILENCE_MULT  3u

class FProtoSubCarFordV1 : public FProtoSubCarBase {
   public:
    FProtoSubCarFordV1() {
        sensorType = FPC_FORDV1;
        te_short = 65;
        te_long = 130;
        te_delta = 39;
        min_count_bit_for_found = FORD_V1_DATA_BITS;
    }

    void feed(bool level, uint32_t duration) {
        switch (parser_step) {
            case FordV1StepReset:
                // Preamble starts with a long LOW
                if (!level && DURATION_DIFF(duration, te_long) < FORD_V1_DELTA_LONG) {
                    parser_step = FordV1StepPreamble;
                    preamble_count = 1;
                    te_last = duration;
                }
                break;

            case FordV1StepPreamble:
                if (DURATION_DIFF(duration, te_long) < FORD_V1_DELTA_LONG) {
                    preamble_count++;
                    te_last = duration;
                } else if (DURATION_DIFF(duration, te_short) < FORD_V1_DELTA_SHORT) {
                    if (preamble_count >= FORD_V1_PREAMBLE_MIN) {
                        // Transition to sync
                        sync_idx = 0;
                        sync_short_count = 1;
                        sync_events[0] = (uint8_t)(level ? ManchesterEventShortHigh
                                                          : ManchesterEventShortLow);
                        parser_step = FordV1StepSync;
                    } else {
                        parser_step = FordV1StepReset;
                    }
                } else {
                    if (preamble_count < FORD_V1_PREAMBLE_MIN)
                        parser_step = FordV1StepReset;
                }
                break;

            case FordV1StepSync: {
                uint8_t ev;
                bool is_short = false;
                if (DURATION_DIFF(duration, te_short) < FORD_V1_DELTA_SHORT) {
                    ev = (uint8_t)(level ? ManchesterEventShortHigh : ManchesterEventShortLow);
                    is_short = true;
                } else if (DURATION_DIFF(duration, te_long) < FORD_V1_DELTA_SHORT) {
                    ev = (uint8_t)(level ? ManchesterEventLongHigh : ManchesterEventLongLow);
                } else {
                    parser_step = FordV1StepPreamble;
                    break;
                }

                sync_idx++;
                if (is_short) sync_short_count++;

                if (sync_idx < 8) sync_events[sync_idx] = ev;

                if (sync_short_count > 2) {
                    // Enough shorts — start data
                    bit_count = 0;
                    byte_count = 0;
                    partial_byte = 0;
                    memset(raw_bytes, 0, sizeof(raw_bytes));

                    ManchesterState ms = ManchesterStateStart1;
                    if (sync_events[0] == (uint8_t)ManchesterEventShortLow)
                        ms = ManchesterStateMid0;
                    FProtoGeneral::manchester_advance(ms, ManchesterEventReset, &ms, NULL);
                    manchester_state = ms;

                    parser_step = FordV1StepData;

                    // Replay buffered sync events into data decoder
                    for (uint8_t i = 0; i <= sync_idx && i < 8; i++) {
                        bool db;
                        if (FProtoGeneral::manchester_advance(
                                manchester_state, (ManchesterEvent)sync_events[i],
                                &manchester_state, &db)) {
                            partial_byte = (uint8_t)((partial_byte << 1) | (db ? 1u : 0u));
                            bit_count++;
                            if ((bit_count & 7) == 0) {
                                if (byte_count < FORD_V1_DATA_BYTES)
                                    raw_bytes[byte_count++] = partial_byte;
                                partial_byte = 0;
                            }
                        }
                    }
                    break;
                }

                if (sync_idx >= 7)
                    parser_step = FordV1StepPreamble;
                break;
            }

            case FordV1StepData: {
                ManchesterEvent event = ManchesterEventReset;
                if (DURATION_DIFF(duration, te_short) < FORD_V1_DELTA_SHORT) {
                    event = level ? ManchesterEventShortHigh : ManchesterEventShortLow;
                } else if (DURATION_DIFF(duration, te_long) < FORD_V1_DELTA_SHORT) {
                    event = level ? ManchesterEventLongHigh : ManchesterEventLongLow;
                } else {
                    // Gap or odd pulse — try to finish if we have 16 bytes
                    try_finish();
                    break;
                }

                bool db;
                if (FProtoGeneral::manchester_advance(manchester_state, event,
                                                      &manchester_state, &db)) {
                    partial_byte = (uint8_t)((partial_byte << 1) | (db ? 1u : 0u));
                    bit_count++;
                    if ((bit_count & 7) == 0) {
                        if (byte_count < FORD_V1_DATA_BYTES)
                            raw_bytes[byte_count++] = partial_byte;
                        partial_byte = 0;
                        if (byte_count > 16) {
                            process_frame();
                            do_reset();
                        }
                    }
                }
                break;
            }
        }
    }

   private:
    uint16_t preamble_count = 0;
    uint8_t sync_idx = 0;
    uint8_t sync_short_count = 0;
    uint8_t sync_events[8] = {};
    uint8_t raw_bytes[FORD_V1_DATA_BYTES] = {};
    uint8_t byte_count = 0;
    uint16_t bit_count = 0;
    uint8_t partial_byte = 0;
    ManchesterState manchester_state = ManchesterStateStart1;

    void do_reset() {
        parser_step = FordV1StepReset;
        preamble_count = 0;
        sync_idx = 0;
        sync_short_count = 0;
        bit_count = 0;
        byte_count = 0;
        partial_byte = 0;
        memset(raw_bytes, 0, sizeof(raw_bytes));
    }

    static uint16_t crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0;
        for (size_t i = 0; i < len; i++) {
            crc ^= ((uint16_t)data[i] << 8);
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        return crc;
    }

    static void decode_payload(uint8_t* raw) {
        uint8_t endbyte = raw[8];
        bool parity_any = (endbyte != 0);
        uint8_t parity = 0;
        for (uint8_t tmp = endbyte; tmp; tmp >>= 1) parity ^= (tmp & 1);
        uint8_t flag = parity_any ? parity : 0;

        if (flag) {
            uint8_t xb = raw[7];
            for (int i = 1; i < 7; i++) raw[i] ^= xb;
        } else {
            uint8_t xb = raw[6];
            for (int i = 1; i < 6; i++) raw[i] ^= xb;
            raw[7] ^= xb;
        }
        uint8_t b6 = raw[6], b7 = raw[7];
        raw[6] = (b6 & 0xAAu) | (b7 & 0x55u);
        raw[7] = (b7 & 0xAAu) | (b6 & 0x55u);
    }

    bool validate_and_publish(uint8_t* raw) {
        uint16_t calc = crc16(&raw[3], 12);
        uint16_t recv = ((uint16_t)raw[15] << 8) | raw[16];
        if (recv != calc) {
            // Try bit-inverted frame
            for (int i = 0; i < FORD_V1_DATA_BYTES; i++) raw[i] = (uint8_t)~raw[i];
            calc = crc16(&raw[3], 12);
            recv = ((uint16_t)raw[15] << 8) | raw[16];
            if (recv != calc) return false;
        }

        uint8_t decoded[9];
        memcpy(decoded, &raw[6], 9);
        decode_payload(decoded);

        // Sanity cross-check
        if (decoded[3] != raw[5] || decoded[4] != raw[6]) return false;

        // Pack raw bytes into decode_data (bytes 0-6) and decode_data2 (bytes 7-16)
        decode_data = 0;
        for (int i = 0; i < 7; i++)
            decode_data = (decode_data << 8) | raw[i];
        decode_data2 = 0;
        for (int i = 7; i < 16; i++)
            decode_data2 = (decode_data2 << 8) | raw[i];

        data_count_bit = FORD_V1_DATA_BITS;
        if (callback) fire_callback();
        return true;
    }

    void process_frame() {
        validate_and_publish(raw_bytes);
    }

    void try_finish() {
        // Try with 0 or 1 missing bits in last byte (handles truncated frames)
        if (byte_count != 16) { do_reset(); return; }
        uint8_t shift = (uint8_t)(FORD_V1_DATA_BITS - bit_count);
        if (shift > 1) { do_reset(); return; }
        uint8_t variants = (uint8_t)(1u << shift);
        uint8_t saved[16];
        memcpy(saved, raw_bytes, 16);
        for (uint8_t v = 0; v < variants; v++) {
            memcpy(raw_bytes, saved, 16);
            raw_bytes[16] = (uint8_t)((partial_byte << shift) | v);
            if (validate_and_publish(raw_bytes)) break;
        }
        do_reset();
    }
};
