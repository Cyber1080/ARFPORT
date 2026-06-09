#include "sub_file_parser.hpp"
#include <sstream>
#include <istream>
#include <cstdlib>
#include "file.hpp"

namespace ui::external_app::subghztx {
namespace subfile {

namespace {
std::string trim(std::string s) {
    size_t a = 0;
    while (a < s.size() && (s[a]==' '||s[a]=='\t'||s[a]=='\r')) a++;
    size_t b = s.size();
    while (b > a && (s[b-1]==' '||s[b-1]=='\t'||s[b-1]=='\r'||s[b-1]=='\n')) b--;
    return s.substr(a, b - a);
}
bool starts_with(const std::string& s, const char* prefix) {
    size_t n = 0; while (prefix[n]) n++;
    return s.size() >= n && s.compare(0, n, prefix) == 0;
}
std::string after_colon(const std::string& line) {
    auto p = line.find(':');
    return (p == std::string::npos) ? "" : trim(line.substr(p + 1));
}
Preset preset_from_string(const std::string& s) {
    if (s.find("Ook270") != std::string::npos || s == "AM270") return Preset::AM270;
    if (s.find("Ook650") != std::string::npos || s == "AM650") return Preset::AM650;
    if (s.find("Dev238") != std::string::npos || s == "FM238") return Preset::FM238;
    if (s.find("Dev120") != std::string::npos || s == "FM12K" ||
        s.find("Dev12K") != std::string::npos)                 return Preset::FM12K;
    if (s.find("Dev476") != std::string::npos || s == "FM476") return Preset::FM476;
    if (s.empty()) return Preset::Unknown;
    return Preset::Custom;
}
uint64_t parse_hex_bytes(const std::string& s) {
    uint64_t v = 0;
    for (char c : s) {
        if (c == ' ' || c == '\t') continue;
        v <<= 4;
        if      (c >= '0' && c <= '9') v |= static_cast<uint64_t>(c - '0');
        else if (c >= 'A' && c <= 'F') v |= static_cast<uint64_t>(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') v |= static_cast<uint64_t>(c - 'a' + 10);
    }
    return v;
}
void parse_raw_data_line(const std::string& s, std::vector<int32_t>& out) {
    const char* p = s.c_str();
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        char* end = nullptr;
        long v = std::strtol(p, &end, 10);
        if (end == p) break;
        out.push_back(static_cast<int32_t>(v));
        p = end;
    }
}
}  // namespace

std::optional<SubFile> parse(std::istream& in) {
    SubFile f;
    bool saw_header = false;
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        if      (starts_with(line, "Filetype:"))   { saw_header = true; }
        else if (starts_with(line, "Version:"))    {}
        else if (starts_with(line, "Frequency:"))  { f.frequency_hz = std::strtoull(after_colon(line).c_str(), nullptr, 10); }
        else if (starts_with(line, "Preset:"))     { f.preset_raw = after_colon(line); f.preset = preset_from_string(f.preset_raw); }
        else if (starts_with(line, "Protocol:"))   { f.protocol = after_colon(line); }
        else if (starts_with(line, "Bit_RAW:"))    { f.binraw_bit_count = static_cast<uint32_t>(std::atoi(after_colon(line).c_str())); }
        else if (starts_with(line, "Bit:"))        { f.bit = static_cast<uint16_t>(std::atoi(after_colon(line).c_str())); }
        else if (starts_with(line, "Key:"))        { f.key = parse_hex_bytes(after_colon(line)); }
        else if (starts_with(line, "TE:"))         { f.te_us = static_cast<uint32_t>(std::atoi(after_colon(line).c_str())); }
        else if (starts_with(line, "RAW_Data:"))   {
            if (f.raw_data.size() < 8192)
                parse_raw_data_line(after_colon(line), f.raw_data);
        }
        else if (starts_with(line, "Data_RAW:"))   {
            if (!f.binraw_hex.empty()) f.binraw_hex.push_back(' ');
            f.binraw_hex += after_colon(line);
        }
    }
    if (!saw_header) return std::nullopt;
    if (f.frequency_hz == 0) return std::nullopt;
    if (f.protocol.empty()) return std::nullopt;
    return std::move(f);
}

// Zero-allocation prefix check on raw char buffer.
static bool buf_sw(const char* buf, uint8_t len, const char* prefix, uint8_t plen) {
    return len >= plen && __builtin_memcmp(buf, prefix, plen) == 0;
}
// Return pointer to char after first ':' in buf, skipping leading spaces.
static const char* buf_after_colon(const char* buf, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (buf[i] == ':') {
            i++;
            while (i < len && (buf[i] == ' ' || buf[i] == '\t')) i++;
            return buf + i;
        }
    }
    return buf + len;
}

std::optional<SubFile> parse_file(const std::filesystem::path& path) {
    File f;
    auto err = f.open(path);
    if (err) return std::nullopt;

    SubFile sf;
    bool saw_header = false;

    // Stack-only line buffer — no heap allocation in the parse loop.
    char line[256];
    uint8_t line_len = 0;

    // Number accumulator for inline RAW_Data parsing.
    char num[12];
    uint8_t num_len = 0;
    bool neg = false;

    auto flush_num = [&]() {
        if (num_len == 0) return;
        num[num_len] = '\0';
        int32_t v = static_cast<int32_t>(std::strtol(num, nullptr, 10));
        if (sf.raw_data.size() < 8192) sf.raw_data.push_back(neg ? -v : v);
        num_len = 0; neg = false;
    };

    // Read in 128-byte chunks to avoid 625K individual read() calls.
    uint8_t chunk[128];
    uint16_t chunk_pos = 0, chunk_len = 0;

    while (true) {
        if (chunk_pos >= chunk_len) {
            auto r = f.read(chunk, sizeof(chunk));
            if (!r || *r == 0) break;
            chunk_len = static_cast<uint16_t>(*r);
            chunk_pos = 0;
        }
        char ch = static_cast<char>(chunk[chunk_pos++]);

        if (ch == '\n') {
            // Trim trailing whitespace/CR.
            while (line_len > 0 && (line[line_len-1]=='\r'||line[line_len-1]==' '||line[line_len-1]=='\t')) line_len--;
            line[line_len] = '\0';

            if (line_len == 0 || line[0] == '#') { line_len = 0; continue; }

#define BSW(pfx) buf_sw(line, line_len, pfx, sizeof(pfx)-1)
            const char* val = buf_after_colon(line, line_len);
            uint8_t val_len = static_cast<uint8_t>(line_len - static_cast<uint8_t>(val - line));

            if      (BSW("Filetype"))  { saw_header = true; }
            else if (BSW("Frequency")){ sf.frequency_hz = std::strtoull(val, nullptr, 10); }
            else if (BSW("Preset"))   { sf.preset_raw = std::string(val, val_len);
                                        sf.preset = preset_from_string(sf.preset_raw); }
            else if (BSW("Protocol")) { sf.protocol = std::string(val, val_len); }
            else if (BSW("Bit_RAW"))  { sf.binraw_bit_count = static_cast<uint32_t>(std::atoi(val)); }
            else if (BSW("Bit:"))     { sf.bit = static_cast<uint16_t>(std::atoi(val)); }
            else if (BSW("Key"))      { sf.key = parse_hex_bytes(std::string(val, val_len)); }
            else if (BSW("TE:"))      { sf.te_us = static_cast<uint32_t>(std::atoi(val)); }
            else if (BSW("RAW_Data")){
                const char* p = val;
                while (*p && sf.raw_data.size() < 8192) {
                    while (*p == ' ' || *p == '\t') p++;
                    if (!*p) break;
                    char* e = nullptr;
                    long v = std::strtol(const_cast<char*>(p), &e, 10);
                    if (!e || e == p) break;
                    sf.raw_data.push_back(static_cast<int32_t>(v));
                    p = e;
                }
            }
            else if (BSW("Data_RAW")){
                if (sf.binraw_hex.size() < 16384) {
                    if (!sf.binraw_hex.empty()) sf.binraw_hex.push_back(' ');
                    sf.binraw_hex.append(val, val_len);
                }
            }
#undef BSW
            line_len = 0;
        } else if (ch != '\r') {
            if (line_len < 255) line[line_len++] = ch;
        }
    }

    if (!saw_header || sf.frequency_hz == 0 || sf.protocol.empty())
        return std::nullopt;
    return std::move(sf);
}

std::optional<SubFile> parse_string(const std::string& text) {
    std::istringstream in(text);
    return parse(in);
}

}  // namespace subfile
}  // namespace ui::external_app::subghztx
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               