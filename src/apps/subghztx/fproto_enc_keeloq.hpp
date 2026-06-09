/*
 * KeeLoq frame encoder — Step 7.
 *
 * Composes:
 *   - Step 3's keeloq_encrypt + learning derivations
 *   - A new OOK frame encoder matching Mayhem's s-keeloq.hpp decoder
 *
 * Frame layout (verified vs s-keeloq.hpp):
 *   11 × (+te_short, -te_short) preamble pairs   [te_short = 400 µs]
 *   1  × (+te_short, -te_short*10)               sync
 *   64 × encoded bit pair, MSB-first              [bit=1: +te_s,-te_l ; bit=0: +te_l,-te_s]
 *   1  × (+te_short, -te_long)                    status bit (always 1)
 *   1  × (+te_short, -gap)                        gap = te_short*40 (or *29 for Sommer)
 *
 * Important: TX bitstream is the *bit-reversed* form of the logical
 * 64-bit code (per ARF reference). Caller passes logical fix + hop;
 * encoder handles the reverse internally.
 *
 * Counter management is the CALLER's responsibility — this encoder is
 * stateless. See Step 7's counter-store design doc for persistence.
 */

#ifndef __FPROTO_ENC_KEELOQ_HPP__
#define __FPROTO_ENC_KEELOQ_HPP__

#include <cstdint>
#include <cstddef>
#include <vector>

namespace ui::external_app::subghztx {
namespace fproto_enc {

enum class KeeloqLearning : uint8_t {
    Simple             = 1,
    Normal             = 2,
    Secure             = 3,
    MagicXorType1      = 4,
    Faac               = 5,
    MagicSerialType1   = 6,
    MagicSerialType2   = 7,
    MagicSerialType3   = 8,
};

struct KeeloqFrame {
    uint32_t serial;       // 28-bit
    uint8_t  btn;          // 4-bit
    uint16_t counter;      // 16-bit rolling counter (caller managed)
    uint64_t mfr_key;      // manufacturer key
    KeeloqLearning learning{KeeloqLearning::Normal};
    uint32_t seed{0};      // for Secure / FAAC learning; 0 for others
    bool sommer_gap{false};// gap = te_short*29 (Sommer) vs *40 (default)
};

/**
 * Encode a KeeLoq frame at the given fixed timing (te_short=400, te_long=800).
 * Returns ±µs pulse vector ready for proc_bint_stream_tx (caller appends sentinel).
 *
 * On success populates `out_logical_data` with the 64-bit logical code
 * (post-encrypt, pre-bit-reverse) so the caller can log/persist it.
 */
std::vector<int32_t> encode_keeloq(const KeeloqFrame& f,
                                   uint8_t repeat = 5,
                                   uint64_t* out_logical_data = nullptr);

}  // namespace fproto_enc
}  // namespace ui::external_app::subghztx

#endif  // __FPROT