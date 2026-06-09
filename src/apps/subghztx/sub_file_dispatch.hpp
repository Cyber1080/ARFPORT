/*
 * Dispatcher: SubFile → OOK pulse stream.
 *
 * Maps the parsed Protocol name to the matching encoder in Step 4's
 * fproto_enc layer, or for `Protocol: RAW` returns the captured pulse
 * vector verbatim. Output is ready to feed to `proc_bint_stream_tx`
 * (caller appends the {42069, 613379} sentinel).
 *
 * Returns nullopt if the protocol isn't supported by the encoder layer
 * yet — caller can fall back to raw IQ replay or surface the gap to the
 * user.
 */

#ifndef __SUB_FILE_DISPATCH_HPP__
#define __SUB_FILE_DISPATCH_HPP__

#include <cstdint>
#include <vector>
#include <optional>
#include "sub_file_parser.hpp"
#include "fproto_encoder_base.hpp"

namespace ui::external_app::subghztx {
namespace subfile {

struct DispatchResult {
    std::vector<int32_t> pulses;
    uint64_t frequency_hz{0};
    Preset preset{Preset::Unknown};
};

std::optional<DispatchResult> dispatch(const SubFile& f, uint8_t repeat = 5);

}  // namespace subfile
}  // namespace ui::external_app::subghztx

#endif  // __SUB_FILE_DISPATCH_HPP__
