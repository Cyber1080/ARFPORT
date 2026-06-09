/*
 * Generic OOK encoder — Step 4 + Step 6a.
 * Now parameterises start-bit length via start_bit_te_mult.
 */

#include "fproto_encoder_base.hpp"

namespace ui::external_app::subghztx {
namespace fproto_enc {

std::vector<int32_t> encode_ook(const OOKTiming& t, uint64_t data,
                                uint8_t bit_count, uint8_t repeat) {
    std::vector<int32_t> pulses;
    if (bit_count == 0 || repeat == 0) return pulses;

    const int32_t te_s = static_cast<int32_t>(t.te_short);
    const int32_t te_l = static_cast<int32_t>(t.te_long);
    const uint32_t start_mult = t.start_bit_te_mult ? t.start_bit_te_mult : 1;

    pulses.reserve(static_cast<size_t>(repeat) * (bit_count * 2 + 4));

    for (uint8_t r = 0; r < repeat; r++) {
        // Preamble LOW (coalesce with previous tail when r > 0).
        int32_t preamble_us = static_cast<int32_t>(te_s * t.preamble_te_mult);
        if (!pulses.empty() && pulses.back() < 0) {
            pulses.back() -= preamble_us;
        } else {
            pulses.push_back(-preamble_us);
        }

        // Optional start HIGH (length = te_short * start_bit_te_mult).
        if (t.has_start_bit) {
            pulses.push_back(static_cast<int32_t>(te_s * start_mult));
        }

        // Bit pulses, MSB-first.
        for (int8_t i = static_cast<int8_t>(bit_count) - 1; i >= 0; i--) {
            bool bit = ((data >> i) & 1ULL) != 0;
            int32_t a, b;
            if (!t.low_first_per_bit) {
                if (!bit) { a = +te_s; b = -te_l; }
                else      { a = +te_l; b = -te_s; }
            } else {
                if (!bit) { a = -te_s; b = +te_l; }
                else      { a = -te_l; b = +te_s; }
            }
            pulses.push_back(a);
            pulses.push_back(b);
        }

        // Princeton-style sync HIGH trailer (high-first protocols).
        if (!t.low_first_per_bit) {
            pulses.push_back(+te_s);
        }

        // Tail LOW.
        int32_t tail_us = static_cast<int32_t>(te_s * t.tail_te_mult);
        if (!pulses.empty() && pulses.back() < 0) {
            pulses.back() -= tail_us;
        } else {
            pulses.push_back(-tail_us);
        }
    }

    return pulses;
}

}  // namespace fproto_enc
}  // namespace ui::external_app::subghztx
