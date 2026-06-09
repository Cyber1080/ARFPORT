#include "fproto_enc_keeloq.hpp"
#include "keeloq_common.hpp"      // Mayhem's existing keeloq_encrypt / decrypt / normal_learning
#include "keeloq_common_ext.hpp"  // Step 3's secure / magic-xor / faac / magic-serial

namespace ui::external_app::subghztx {
namespace fproto_enc {

namespace {

constexpr int32_t TE_SHORT = 400;
constexpr int32_t TE_LONG  = 800;

uint64_t reverse_bits_64(uint64_t v, uint8_t n) {
    uint64_t r = 0;
    for (uint8_t i = 0; i < n; i++) if ((v >> i) & 1ULL) r |= 1ULL << (n - 1 - i);
    return r;
}

uint64_t derive_working_key(const KeeloqFrame& f, uint32_t fix) {
    switch (f.learning) {
        case KeeloqLearning::Simple:
            return f.mfr_key;
        case KeeloqLearning::Normal:
            return keeloq_normal_learning(fix, f.mfr_key);
        case KeeloqLearning::Secure:
            return keeloq_secure_learning(fix, f.seed, f.mfr_key);
        case KeeloqLearning::MagicXorType1:
            return keeloq_magic_xor_type1_learning(fix, f.mfr_key);
        case KeeloqLearning::Faac:
            return keeloq_faac_learning(f.seed, f.mfr_key);
        case KeeloqLearning::MagicSerialType1:
            return keeloq_magic_serial_type1_learning(fix, f.mfr_key);
        case KeeloqLearning::MagicSerialType2:
            return keeloq_magic_serial_type2_learning(fix, f.mfr_key);
        case KeeloqLearning::MagicSerialType3:
            return keeloq_magic_serial_type3_learning(fix & 0xFFFFFF, f.mfr_key);
    }
    return f.mfr_key;
}

}  // namespace

std::vector<int32_t> encode_keeloq(const KeeloqFrame& f, uint8_t repeat,
                                   uint64_t* out_logical_data) {
    std::vector<int32_t> p;
    if (repeat == 0) return p;

    // Build logical code:
    //   fix = (btn << 28) | serial(28-bit)
    //   hop_input = (btn << 28) | ((serial & 0x3FF) << 16) | counter
    //   encrypted = keeloq_encrypt(hop_input, working_key)
    //   code = (fix << 32) | encrypted
    uint32_t fix = (static_cast<uint32_t>(f.btn) << 28) | (f.serial & 0x0FFFFFFFu);
    uint32_t hop_input = (static_cast<uint32_t>(f.btn) << 28) |
                         ((f.serial & 0x3FFu) << 16) |
                         f.counter;
    uint64_t working_key = derive_working_key(f, fix);
    uint32_t encrypted = keeloq_encrypt(hop_input, working_key);
    uint64_t code = (static_cast<uint64_t>(fix) << 32) | encrypted;
    if (out_logical_data) *out_logical_data = code;

    uint64_t tx_bits = reverse_bits_64(code, 64);
    int32_t gap_us = TE_SHORT * (f.sommer_gap ? 29 : 40);

    p.reserve(static_cast<size_t>(repeat) * 160);

    for (uint8_t r = 0; r < repeat; r++) {
        // Coalesce inter-repeat gap from previous tail if present.
        if (r > 0) {
            // The previous repeat ended with -gap; nothing to do — that low
            // serves as the next-repeat preamble entry.
        }

        // 11× preamble pairs.
        for (int i = 0; i < 11; i++) {
            p.push_back(+TE_SHORT);
            p.push_back(-TE_SHORT);
        }
        // Sync: one more HIGH then long LOW.
        p.push_back(+TE_SHORT);
        p.push_back(-TE_SHORT * 10);

        // 64 data bits MSB-first.
        for (int i = 63; i >= 0; i--) {
            bool bit = ((tx_bits >> i) & 1ULL) != 0;
            if (bit) {
                p.push_back(+TE_SHORT);
                p.push_back(-TE_LONG);
            } else {
                p.push_back(+TE_LONG);
                p.push_back(-TE_SHORT);
            }
        }
        // 1 status bit (encoded as logical 1).
        p.push_back(+TE_SHORT);
        p.push_back(-TE_LONG);
        // End: one HIGH then gap.
        p.push_back(+TE_SHORT);
        p.push_back(-gap_us);
    }
    return p;
}

}  // namespace fproto_enc
}  // namespace ui::external_app::subghztx
