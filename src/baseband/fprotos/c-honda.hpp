#pragma once
#include "subcarbase.hpp"

// ============================================================
// Honda 2FSK FHSS decoder — 2019 Civic EX and related models
// FCC filing verified (Annex 3 / Annex 11 test reports)
//
// *** WHY CC1101 CANNOT DECODE THIS ***
// -----------------------------------------------
// This remote is FREQUENCY-HOPPING SPREAD SPECTRUM (FHSS).
// It alternates between two channels 523 kHz apart:
//   Channel 1 center: ~433.648 MHz  (FSK tones: 433.618 / 433.677 MHz)
//   Channel 2 center: ~434.171 MHz  (FSK tones: 434.139 / 434.202 MHz)
//
// CC1101 (Flipper Zero) is a single-channel receiver. PLL retune
// takes ~340 us — far too slow to follow the hops. The Flipper
// always misses every other hop and gets half a frame at best.
// No amount of register tuning fixes this; it is a fundamental
// architectural limitation of the single-VCO CC1101.
//
// *** WHY HACKRF / PORTAPACK CAN HANDLE IT ***
// -----------------------------------------------
// HackRF captures up to 20 MHz simultaneously. Centered at
// 433.920 MHz both channels fall within a 600 kHz window.
// The FM discriminator in proc_subcar processes whichever
// channel the current hop lands on — no retuning needed.
//
// Practical PortaPack tuning: 433.920 MHz, FM mode (modulation=1).
// At the current 8x decimation (500 kHz post-decim bandwidth) the
// channels are at +/-260 kHz from center — just at the edge.
// For reliable dual-channel capture: reduce proc_subcar decimation
// from 8x to 4x (skip decim_1) to get 1 MHz post-decim bandwidth.
//
// Physical layer (measured from FCC occupied-bandwidth report)
// -------------------------------------------------------------
//   Channel 1  : 433.648 MHz center
//   Channel 2  : 434.171 MHz center
//   Hop spacing: 523 kHz
//   Modulation : 2FSK (GFSK)
//   Deviation  : +-29 to +-32 kHz (read from FSK tone spacing in FCC plot)
//   Frame time : ~24.5 ms worst case (FCC periodic-operation test)
//   Symbol rate: ~3000-4000 baud => te_symbol ~250-333 us (estimated)
//
// *** CALIBRATION ***
// te_short is estimated. To verify: capture raw IQ at 433.92 MHz
// (>=1 MHz span), FM-discriminate offline, measure alternating
// preamble pulse width. Adjust HONDA_TE_SHORT in 20 us steps.
//
// Encoding: NRZ over 2FSK (same frame on both hop channels)
// ----------------------------------------------------------
// After FM discriminator (proc_subcar modulation=1):
//   level=true  => high-freq tone => logical '1'
//   level=false => low-freq  tone => logical '0'
//   duration    => N x te_short consecutive identical bits (run-length)
//
// Frame structure (community RE, unverified bit positions):
//   Preamble : >=40 alternating te_short pulses (101010...)
//   Sync gap : one LOW run of ~3-5x te_short
//   Data     : 80 raw NRZ bits (rolling code — not decryptable)
//              Shown as raw hex decode_data / decode_data2.
//
// CC1101 best-effort single-channel config (catches ~50% of hops):
//   FREQ2/1/0 = 0x10/0xB1/0xBC  (433.92 MHz, 26 MHz XTAL)
//   MDMCFG4   = 0xCA             (BW ~101 kHz, drate_e=10)
//   MDMCFG3   = 0x83             (~4800 baud estimate)
//   MDMCFG2   = 0x12             (GFSK, no Manchester, sync 16/16)
//   DEVIATN   = 0x34             (+-28.6 kHz, matches FCC measurement)
//   Even with correct registers CC1101 misses every other hop.
// ============================================================

#define HONDA_TE_SHORT       83u   // us — IQ-verified: 127samp @ 1.536MHz = 82.7us => 12096 baud
#define HONDA_TE_DELTA       40u   // us tolerance (~48%)
#define HONDA_PRESYNC_MIN   580u   // us — HIGH presync burst floor  (~7x te_short)
#define HONDA_PRESYNC_MAX   870u   // us — HIGH presync burst ceiling (~10x te_short)
#define HONDA_PREAMBLE_MIN   40u   // min alternating 1T pairs before presync
#define HONDA_POSTSYNC_MIN   60u   // min 1T pairs after presync before data
#define HONDA_BITS           80u   // NRZ bits to collect (first 10 bytes)

// Measured signal structure (from 2019 Civic EX IQ capture @ 1.536 MHz):
//   Symbol rate: ~12096 baud (NOT 4kbaud as earlier estimated)
//   Burst A (32ms): 384 alternating 1T pairs — raw preamble, no data
//   ~98ms silence
//   Burst B (31ms):
//     [0]    HIGH 8.6xTE (~714us) — presync marker
//     [1-89] alternating 1T       — post-sync preamble (~14.8ms)
//     [90+]  NRZ 1T/2T mix        — rolling-code data (80+ bits)
//   First 7 bytes fixed (device ID): 0xD3 4A AB 54 D2 B2 AB
//   Byte 31 = rolling counter (increments by 1 each press)

typedef enum {
    HondaDecoderStepReset = 0,
    HondaDecoderStepPreamble,
    HondaDecoderStepPostSync,   // counting 1T pairs after presync
    HondaDecoderStepData,
} HondaDecoderStep;

class FProtoSubCarHonda : public FProtoSubCarBase {
   public:
    FProtoSubCarHonda() {
        sensorType = FPC_HONDA;
        te_short = HONDA_TE_SHORT;
        te_long  = HONDA_TE_SHORT * 2;
        te_delta = HONDA_TE_DELTA;
        min_count_bit_for_found = HONDA_BITS;
    }

    // Append N copies of bit into the 80-bit shift register.
    // bits[0] first -> ends in MSB. decode_data = bits[63:0],
    // decode_data2 = bits[79:64].
    void honda_push_bits(bool bit, uint32_t count) {
        for (uint32_t i = 0; i < count; ++i) {
            if (decode_count_bit < 64) {
                decode_data = (decode_data << 1) | (bit ? 1u : 0u);
            } else if (decode_count_bit < 80) {
                decode_data2 = (decode_data2 << 1) | (bit ? 1u : 0u);
            }
            decode_count_bit++;
            if (decode_count_bit >= HONDA_BITS) return;
        }
    }

    void honda_reset() {
        parser_step      = HondaDecoderStepReset;
        preamble_count   = 0;
        postsync_count   = 0;
        last_level       = false;
        decode_data      = 0;
        decode_data2     = 0;
        decode_count_bit = 0;
    }

    // Start collecting NRZ data; `first_*` is the run that triggered the transition.
    void honda_start_data(bool first_level, uint32_t first_dur) {
        decode_data      = 0;
        decode_data2     = 0;
        decode_count_bit = 0;
        parser_step      = HondaDecoderStepData;
        // Push the triggering run's bits immediately.
        uint32_t sym = (first_dur + te_short / 2) / te_short;
        if (sym == 0) sym = 1;
        if (sym > HONDA_BITS) sym = HONDA_BITS;
        honda_push_bits(first_level, sym);
        if (decode_count_bit >= HONDA_BITS) {
            data_count_bit = decode_count_bit;
            if (callback) fire_callback();
            honda_reset();
        }
    }

    void feed(bool level, uint32_t duration) {
        // Pre-check: HIGH presync pulse overrides any state except Data.
        // Seen at start of every data burst (burst B), ~8.6x te_short.
        if (parser_step != HondaDecoderStepData &&
            level == true &&
            duration >= HONDA_PRESYNC_MIN &&
            duration <= HONDA_PRESYNC_MAX) {
            // Valid presync — transition to post-sync preamble counting.
            // Require we have seen at least some preamble first (burst A),
            // OR allow it if we're already in Preamble state.
            if (parser_step == HondaDecoderStepPreamble &&
                preamble_count >= HONDA_PREAMBLE_MIN) {
                postsync_count = 0;
                last_level     = true;   // presync was HIGH; first post-sync pulse is LOW
                parser_step    = HondaDecoderStepPostSync;
            } else {
                honda_reset();
            }
            return;
        }

        switch (parser_step) {

            case HondaDecoderStepReset:
                // Wait for the first 1T pulse to seed preamble counting.
                if (DURATION_DIFF(duration, te_short) < te_delta) {
                    preamble_count = 1;
                    last_level     = level;
                    parser_step    = HondaDecoderStepPreamble;
                }
                break;

            case HondaDecoderStepPreamble:
                if (DURATION_DIFF(duration, te_short) < te_delta) {
                    if (level != last_level) {
                        preamble_count++;
                        last_level = level;
                    } else {
                        honda_reset();
                    }
                } else if (preamble_count >= HONDA_PREAMBLE_MIN &&
                           DURATION_DIFF(duration, te_short * 2) < te_delta) {
                    // First exactly-2T pulse after enough preamble: burst A missed,
                    // but burst B mini-preamble sufficed. (Tight threshold avoids
                    // matching the ~2.9T end-of-preamble sync gap in burst A.)
                    honda_start_data(level, duration);
                } else {
                    honda_reset();
                }
                break;

            case HondaDecoderStepPostSync:
                // Count the ~89 alternating 1T pairs that follow the presync.
                if (DURATION_DIFF(duration, te_short) < te_delta) {
                    if (level != last_level) {
                        postsync_count++;
                        last_level = level;
                    } else {
                        honda_reset();
                    }
                } else if (postsync_count >= HONDA_POSTSYNC_MIN) {
                    // First non-1T after >=60 post-sync pairs → data starts here.
                    honda_start_data(level, duration);
                } else {
                    honda_reset();
                }
                break;

            case HondaDecoderStepData: {
                uint32_t symbols = (duration + te_short / 2) / te_short;
                if (symbols == 0) { honda_reset(); break; }
                uint32_t remaining = HONDA_BITS - decode_count_bit;
                if (symbols > remaining) symbols = remaining;
                honda_push_bits(level, symbols);
                if (decode_count_bit >= HONDA_BITS) {
                    data_count_bit = decode_count_bit;
                    if (callback) fire_callback();
                    honda_reset();
                }
                break;
            }

            default:
                honda_reset();
                break;
        }
    }

   private:
    uint16_t preamble_count  = 0;
    uint16_t postsync_count  = 0;
    bool     last_level      = false;
};
