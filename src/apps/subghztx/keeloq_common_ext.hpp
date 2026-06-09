/*
 * KeeLoq common — additional learning derivations from Flipper-ARF.
 *
 * Mayhem 2.4.0 ships only KEELOQ_SIMPLE_LEARNING (1) + KEELOQ_NORMAL_LEARNING (2)
 * in keeloq_common.{hpp,cpp}. ARF supports 7 + KingGates/Jarolift specials.
 * This header adds the missing five plus the two specials.
 *
 * All functions are pure algorithm ports — byte-for-byte from
 * `lib/subghz/protocols/keeloq_common.c` in Flipper-ARF, with `furi.h` and
 * `m-array.h` deps removed (they were unused).
 *
 * INTEGRATION:
 *   Option A — patch keeloq_common.{hpp,cpp} directly (see step3_*.patch).
 *   Option B — keep this file separate and #include it where needed.
 *
 * Deliberately uses C++ identifiers (no `xor` keyword as parameter name).
 */

#ifndef __KEELOQ_COMMON_EXT_HPP__
#define __KEELOQ_COMMON_EXT_HPP__

#include <cstdint>
#include "keeloq_common.hpp"  // for keeloq_encrypt/decrypt + KEELOQ_NLF + bit/g5 macros

// New learning-type constants (extend the existing KEELOQ_*_LEARNING set).
#define KEELOQ_LEARNING_UNKNOWN              0u
// 1 = KEELOQ_SIMPLE_LEARNING (already in keeloq_keystore.hpp)
// 2 = KEELOQ_NORMAL_LEARNING (already in keeloq_keystore.hpp)
#define KEELOQ_LEARNING_SECURE               3u
#define KEELOQ_LEARNING_MAGIC_XOR_TYPE_1     4u
#define KEELOQ_LEARNING_FAAC                 5u
#define KEELOQ_LEARNING_MAGIC_SERIAL_TYPE_1  6u
#define KEELOQ_LEARNING_MAGIC_SERIAL_TYPE_2  7u
#define KEELOQ_LEARNING_MAGIC_SERIAL_TYPE_3  8u
#define KEELOQ_LEARNING_SIMPLE_KINGGATES    10u
#define KEELOQ_LEARNING_NORMAL_JAROLIFT     11u

namespace ui::external_app::subghztx {

/** Secure Learning: 28-bit serial + 32-bit seed → 64-bit derived manufacture key. */
uint64_t keeloq_secure_learning(uint32_t data, uint32_t seed, uint64_t key);

/** Magic XOR Type 1: 28-bit serial XORed with manufacturer XOR-key. */
uint64_t keeloq_magic_xor_type1_learning(uint32_t data, uint64_t magic_xor);

/** FAAC SLH (Spa) Learning: 32-bit seed + manufacturer key → derived 64-bit. */
uint64_t keeloq_faac_learning(uint32_t seed, uint64_t key);

/** Magic Serial Type 1: 28-bit serial folded into top of 64-bit magic. */
uint64_t keeloq_magic_serial_type1_learning(uint32_t data, uint64_t magic);

/** Magic Serial Type 2: 32-bit btn+serial copies into high half of magic. */
uint64_t keeloq_magic_serial_type2_learning(uint32_t data, uint64_t magic);

/** Magic Serial Type 3: 24-bit serial fills the low 24 bits of magic. */
uint64_t keeloq_magic_serial_type3_learning(uint32_t data, uint64_t magic);

}  // namespace ui::external_app::subghztx

#endif  // __KEELOQ_COMMON_EXT_HPP__
