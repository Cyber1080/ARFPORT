#include "keeloq_common_ext.hpp"

namespace ui::external_app::subghztx {

uint64_t keeloq_secure_learning(uint32_t data, uint32_t seed, uint64_t key) {
    data &= 0x0FFFFFFF;
    uint32_t k1 = keeloq_decrypt(data, key);
    uint32_t k2 = keeloq_decrypt(seed, key);
    return ((uint64_t)k1 << 32) | k2;
}

uint64_t keeloq_magic_xor_type1_learning(uint32_t data, uint64_t magic_xor) {
    data &= 0x0FFFFFFF;
    return (((uint64_t)data << 32) | data) ^ magic_xor;
}

uint64_t keeloq_faac_learning(uint32_t seed, uint64_t key) {
    uint16_t hs = static_cast<uint16_t>(seed >> 16);
    constexpr uint16_t ending = 0x544D;
    uint32_t lsb = (static_cast<uint32_t>(hs) << 16) | ending;
    uint64_t man =
        (static_cast<uint64_t>(keeloq_encrypt(seed, key)) << 32) |
        keeloq_encrypt(lsb, key);
    return man;
}

uint64_t keeloq_magic_serial_type1_learning(uint32_t data, uint64_t magic) {
    return (magic & 0xFFFFFFFFULL) |
           (static_cast<uint64_t>(data) << 40) |
           (static_cast<uint64_t>(((data & 0xFF) + ((data >> 8) & 0xFF)) & 0xFF) << 32);
}

uint64_t keeloq_magic_serial_type2_learning(uint32_t data, uint64_t magic) {
    // Byte-swap data into high 4 bytes of magic, preserving low 4 bytes.
    // ARF used a uint8_t* cast which assumes little-endian; do it portably here.
    uint64_t high =
        (static_cast<uint64_t>((data >>  0) & 0xFF) << 56) |
        (static_cast<uint64_t>((data >>  8) & 0xFF) << 48) |
        (static_cast<uint64_t>((data >> 16) & 0xFF) << 40) |
        (static_cast<uint64_t>((data >> 24) & 0xFF) << 32);
    return (magic & 0x00000000FFFFFFFFULL) | high;
}

uint64_t keeloq_magic_serial_type3_learning(uint32_t data, uint64_t magic) {
    return (magic & 0xFFFFFFFFFF000000ULL) | (static_cast<uint64_t>(data) & 0xFFFFFFULL);
}

}  // namespace ui::external_app::subghztx
