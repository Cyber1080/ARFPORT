#include "sub_file_dispatch.hpp"
#include "fproto_encoder_extra.hpp"
#include <string>
#include <sstream>
#include <fstream>

namespace ui::external_app::subghztx {
namespace subfile {

namespace {
bool ieq(const std::string& a, const char* b) {
    size_t i = 0;
    for (; i < a.size() && b[i]; i++) {
        char ca = a[i], cb = b[i];
        if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca + ('a' - 'A'));
        if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb + ('a' - 'A'));
        if (ca != cb) return false;
    }
    return i == a.size() && b[i] == '\0';
}
}

std::optional<DispatchResult> dispatch(const SubFile& f, uint8_t repeat) {
    DispatchResult r;
    r.frequency_hz = f.frequency_hz;
    r.preset = f.preset;

    if (f.is_raw()) { r.pulses = f.raw_data; return r; }

    if (f.is_binraw()) {
        uint32_t te = f.te_us.value_or(0);
        if (te == 0 || f.binraw_bit_count == 0 || f.binraw_hex.empty())
            return std::nullopt;
        r.pulses = fproto_enc::encode_binraw(f.binraw_hex, f.binraw_bit_count, te);
        return r;
    }

    if (ieq(f.protocol, "Princeton")) {
        uint8_t bits = f.bit ? static_cast<uint8_t>(f.bit) : 24;
        r.pulses = fproto_enc::encode_princeton(f.key, bits, repeat);
        return r;
    }
    if (ieq(f.protocol, "CAME") || ieq(f.protocol, "Came") ||
        ieq(f.protocol, "Prastel") || ieq(f.protocol, "Airforce") ||
        ieq(f.protocol, "CAME TWEE")) {
        uint8_t bits = f.bit ? static_cast<uint8_t>(f.bit) : 12;
        r.pulses = fproto_enc::encode_came(f.key, bits, repeat);
        return r;
    }
    if (ieq(f.protocol, "Holtek")) {
        r.pulses = fproto_enc::encode_holtek(f.key, repeat);
        return r;
    }
    if (ieq(f.protocol, "Holtek_HT12X") || ieq(f.protocol, "HT12X") ||
        ieq(f.protocol, "Holtek HT12X")) {
        r.pulses = fproto_enc::encode_holtek_ht12x(f.key, repeat);
        return r;
    }
    if (ieq(f.protocol, "SMC5326") || ieq(f.protocol, "SMC-5326")) {
        uint8_t bits = f.bit ? static_cast<uint8_t>(f.bit) : 25;
        r.pulses = fproto_enc::encode_smc5326(f.key, bits, repeat);
        return r;
    }
    if (ieq(f.protocol, "GateTX") || ieq(f.protocol, "Gate TX") ||
        ieq(f.protocol, "Gate_TX")) {
        r.pulses = fproto_enc::encode_gate_tx(f.key, repeat);
        return r;
    }
    if (ieq(f.protocol, "Ansonic")) {
        uint8_t bits = f.bit ? static_cast<uint8_t>(f.bit) : 12;
        r.pulses = fproto_enc::encode_ansonic(f.key, bits, repeat);
        return r;
    }

    // Step 6b additions
    if (ieq(f.protocol, "Intertechno_V3") || ieq(f.protocol, "Intertechno V3")) {
        uint8_t bits = f.bit ? static_cast<uint8_t>(f.bit) : 32;
        r.pulses = fproto_enc::encode_intertechno_v3(f.key, bits, repeat);
        return r;
    }
    if (ieq(f.protocol, "MegaCode") || ieq(f.protocol, "Megacode")) {
        r.pulses = fproto_enc::encode_megacode(static_cast<uint32_t>(f.key), repeat);
        return r;
    }
    // BinRAW handled at parser level: SubFile already has raw_data populated
    // when it parses Bit_RAW + Data_RAW + TE. But Step 5 parser only handles
    // RAW (raw_data). BinRAW needs distinct handling — see SubFile::binraw_*.
    // For now, dispatch returns nullopt for BinRAW; the caller can use
    // encode_binraw() directly with the file's Data_RAW + Bit + TE fields.

    return std::nullopt;
}

}  // namespace subfile
}  // namespace ui::external_app::subghztx
