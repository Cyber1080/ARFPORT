/*
 * Step 6b: Intertechno_V3, MegaCode, and BinRAW encoders.
 *
 * These don't fit the OOKTiming template:
 *   - Intertechno_V3: Manchester-style 4 pulses/bit + double-start sync
 *   - MegaCode: PWM with state-dependent LOW durations
 *   - BinRAW: packed bitstream, run-length-encoded to pulses
 *
 * Standalone functions; compose with Step 4 OOKTiming.
 */

#ifndef __FPROTO_ENCODER_EXTRA_HPP__
#define __FPROTO_ENCODER_EXTRA_HPP__

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

namespace ui::external_app::subghztx {
namespace fproto_enc {

// Intertechno_V3, 32-bit (or 36-bit dimming variant). 433.92 MHz typical.
// Frame:  -te_short*37  +te_short  -te_short*10  [bit-quads * N]  -te_short*11+
// Per bit: +te_s -X +te_s -te_s   where X = te_s (bit=0) or te_l (bit=1)
std::vector<int32_t> encode_intertechno_v3(uint64_t data,
                                           uint8_t  bit_count = 32,
                                           uint8_t  repeat = 5);

// MegaCode 24-bit PWM. Linear Corp garage-door / driveway-gate format.
// MUST have bit 23 set (the start bit is implicit and decoded as 1).
// Per-bit LOW depends on transition:
//   prev=1, cur=1 → 5*te_s   prev=1, cur=0 → 2*te_s
//   prev=0, cur=1 → 8*te_s   prev=0, cur=0 → 5*te_s
std::vector<int32_t> encode_megacode(uint32_t data, uint8_t repeat = 5);

// BinRAW: parse a hex-byte string as a packed bitstream and run-length
// encode to ±µs pulse durations. te_us is the duration of one bit slot.
//
// The bitstream is read MSB-first within each byte; '1' bits become HIGH
// runs, '0' bits become LOW runs.
//
// Example: bits=10, te=242, hex="80 00" → "1000 0000 00" → +242, -2178
std::vector<int32_t> encode_binraw(const std::string& hex_bytes,
                                   uint32_t bit_count,
                                   uint32_t te_us);

}  // namespace fproto_enc
}  // namespace ui::external_app::subghztx

#endif  // __FPROTO_ENCODER_EXTRA_HPP__
