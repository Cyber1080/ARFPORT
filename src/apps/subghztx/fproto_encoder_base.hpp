/*
 * Sub-GHz protocol-aware TX encoder layer (Step 4 + 6a + 6b corrections).
 */

#ifndef __FPROTO_ENCODER_BASE_HPP__
#define __FPROTO_ENCODER_BASE_HPP__

#include <cstdint>
#include <cstddef>
#include <vector>

namespace ui::external_app::subghztx {
namespace fproto_enc {

struct OOKTiming {
    uint32_t te_short;
    uint32_t te_long;
    uint32_t preamble_te_mult;
    uint32_t tail_te_mult;
    bool     has_start_bit;
    bool     low_first_per_bit;
    uint8_t  default_repeat;
    uint32_t start_bit_te_mult;
};

std::vector<int32_t> encode_ook(const OOKTiming& t, uint64_t data,
                                uint8_t bit_count, uint8_t repeat);

inline std::vector<int32_t> encode_princeton(uint64_t data, uint8_t bits = 24, uint8_t repeat = 5) {
    OOKTiming t{ 390, 1170, 36, 36, false, false, 5, 1 };
    return encode_ook(t, data, bits, repeat);
}

inline std::vector<int32_t> encode_came(uint64_t data, uint8_t bits = 12, uint8_t repeat = 5) {
    OOKTiming t{ 320, 640, 56, 5, true, true, 5, 1 };
    return encode_ook(t, data, bits, repeat);
}

inline std::vector<int32_t> encode_holtek(uint64_t data, uint8_t repeat = 5) {
    OOKTiming t{ 430, 870, 36, 12, true, true, 5, 1 };
    return encode_ook(t, data, 40, repeat);
}

inline std::vector<int32_t> encode_holtek_ht12x(uint64_t data, uint8_t repeat = 5) {
    OOKTiming t{ 320, 640, 36, 12, true, true, 5, 1 };
    return encode_ook(t, data, 12, repeat);
}

inline std::vector<int32_t> encode_smc5326(uint64_t data, uint8_t bits = 25, uint8_t repeat = 5) {
    OOKTiming t{ 300, 900, 24, 24, false, false, 5, 1 };
    return encode_ook(t, data, bits, repeat);
}

inline std::vector<int32_t> encode_gate_tx(uint64_t data, uint8_t repeat = 5) {
    OOKTiming t{ 350, 700, 47, 12, true, true, 5, 2 };
    return encode_ook(t, data, 24, repeat);
}

// Step 6b: Ansonic now correctly modeled as CAME-style (low_first=true,
// has_start_bit=true), preamble*35 / tail*5.
inline std::vector<int32_t> encode_ansonic(uint64_t data, uint8_t bits = 12, uint8_t repeat = 5) {
    OOKTiming t{ 555, 1111, 35, 5, true, true, 5, 1 };
    return encode_ook(t, data, bits, repeat);
}

}  // namespace fproto_enc
}  // namespace ui::external_app::subghztx

#endif
