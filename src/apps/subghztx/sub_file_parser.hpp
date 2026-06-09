#ifndef __SUB_FILE_PARSER_HPP__
#define __SUB_FILE_PARSER_HPP__

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <iosfwd>

// Forward-declare Mayhem's custom path (defined in file.hpp, NOT <filesystem>).
namespace std { namespace filesystem { struct path; } }

namespace ui::external_app::subghztx {
namespace subfile {

enum class Preset : uint8_t {
    Unknown = 0, AM270, AM650, FM238, FM12K, FM476, Custom
};

struct SubFile {
    uint64_t frequency_hz{0};
    Preset preset{Preset::Unknown};
    std::string preset_raw{};
    std::string protocol{};
    uint16_t bit{0};
    uint64_t key{0};
    std::optional<uint32_t> te_us;
    std::vector<int32_t> raw_data;
    uint32_t binraw_bit_count{0};
    std::string binraw_hex{};

    bool is_raw() const { return protocol == "RAW"; }
    bool is_binraw() const { return protocol == "BinRAW"; }
};

std::optional<SubFile> parse(std::istream& in);
std::optional<SubFile> parse_file(const std::filesystem::path& path);
std::optional<SubFile> parse_string(const std::string& text);

}  // namespace subfile
}  // namespace ui::external_app::subghztx

#endif
