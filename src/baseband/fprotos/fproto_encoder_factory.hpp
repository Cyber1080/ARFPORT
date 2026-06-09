#pragma once

#include "fproto_encoder_base.hpp"
#include "e-hormann.hpp"
#include "e-keeloq.hpp"
#include "e-starline.hpp"
#include "e-beninca.hpp"
#include "e-came.hpp"
#include "e-doorhan.hpp"
#include "e-clemsa.hpp"
#include "e-magicar.hpp"
#include "e-ford.hpp"
#include "e-subaru.hpp"
#include "e-honda.hpp"
#include "e-magicar-868.hpp"
#include "e-starline-868.hpp"
#include "e-beninca-868.hpp"
#include "e-volkswagen.hpp"
#include "e-bmw.hpp"
#include "e-mercedes.hpp"
#include "e-jeep.hpp"
#include "e-garage-door.hpp"
#include "e-wireless-sensor.hpp"
#include "e-linear.hpp"

#include <memory>
#include <cstring>

/**
 * Protocol type enumeration.
 * Matches SubGhzProtocol naming from Flipper/PANDORA.
 */
enum class FProtoProtocolType : uint8_t {
    UNKNOWN = 0,
    HORMANN = 1,      // Gate opener (static)
    KEELOQ = 2,       // Rolling code (cars, gates)
    STARLINE = 3,     // Car alarm (rolling code, FSK)
    BENINCA = 4,      // Gate opener (adaptive)
    CAME = 5,         // Gate opener (OOK)
    DOORHAN = 6,      // Gate opener
    MAGICAR = 7,      // Car alarm (433 MHz)
    HONDA = 8,        // Car-specific (315 MHz)
    FORD = 9,         // Car-specific (315 MHz)
    SUBARU = 10,      // Car-specific (315 MHz)
    CLEMSA = 11,      // Gate opener (Manchester)
    MAGICAR_868 = 12, // Car alarm (868 MHz variant)
    STARLINE_868 = 13,// Car alarm (868 MHz variant)
    BENINCA_868 = 14, // Gate opener (868 MHz variant)
    VOLKSWAGEN = 15,  // Car alarm (VW/Audi/Skoda)
    BMW = 16,         // Car alarm (BMW)
    MERCEDES = 17,    // Car alarm (Mercedes)
    JEEP = 18,        // Car alarm (Chrysler Group)
    GARAGE_DOOR = 19, // Generic garage door
    WIRELESS_SENSOR = 20, // Weather/sensor device
    LINEAR = 21,      // Liftmaster/Linear garage
    // ... extend as needed for remaining protocols (22-27+)
};

/**
 * Encoder factory: instantiate the correct encoder for a given protocol type.
 *
 * Usage:
 *   auto encoder = FProtoEncoderFactory::create(FProtoProtocolType::HORMANN);
 *   if (encoder) {
 *       encoder->build_packet(data, len, packet, &pkt_len);
 *       encoder->encode(packet, pkt_len, durations, levels, &out_len);
 *   }
 */
class FProtoEncoderFactory {
public:
    /**
     * Create an encoder instance for the specified protocol.
     *
     * @param protocol_type Protocol type (KEELOQ, HORMANN, STARLINE, etc.)
     * @return Unique pointer to encoder (nullptr if protocol not supported)
     */
    static std::unique_ptr<FProtoEncoderBase> create(FProtoProtocolType protocol_type) {
        switch (protocol_type) {
            case FProtoProtocolType::HORMANN:
                return std::make_unique<FProtoEncoderHormann>();

            case FProtoProtocolType::KEELOQ:
                return std::make_unique<FProtoEncoderKeeloq>();

            case FProtoProtocolType::STARLINE:
                return std::make_unique<FProtoEncoderStarline>();

            case FProtoProtocolType::BENINCA:
                return std::make_unique<FProtoEncoderBeninca>();

            case FProtoProtocolType::CAME:
                return std::make_unique<FProtoEncoderCame>();

            case FProtoProtocolType::DOORHAN:
                return std::make_unique<FProtoEncoderDoorhan>();

            case FProtoProtocolType::CLEMSA:
                return std::make_unique<FProtoEncoderClemsa>();

            case FProtoProtocolType::MAGICAR:
                return std::make_unique<FProtoEncoderMagicar>();

            case FProtoProtocolType::FORD:
                return std::make_unique<FProtoEncoderFord>();

            case FProtoProtocolType::SUBARU:
                return std::make_unique<FProtoEncoderSubaru>();

            case FProtoProtocolType::HONDA:
                return std::make_unique<FProtoEncoderHonda>();

            case FProtoProtocolType::MAGICAR_868:
                return std::make_unique<FProtoEncoderMagicar868>();

            case FProtoProtocolType::STARLINE_868:
                return std::make_unique<FProtoEncoderStarline868>();

            case FProtoProtocolType::BENINCA_868:
                return std::make_unique<FProtoEncoderBeninca868>();

            case FProtoProtocolType::VOLKSWAGEN:
                return std::make_unique<FProtoEncoderVolkswagen>();

            case FProtoProtocolType::BMW:
                return std::make_unique<FProtoEncoderBmw>();

            case FProtoProtocolType::MERCEDES:
                return std::make_unique<FProtoEncoderMercedes>();

            case FProtoProtocolType::JEEP:
                return std::make_unique<FProtoEncoderJeep>();

            case FProtoProtocolType::GARAGE_DOOR:
                return std::make_unique<FProtoEncoderGarageDoor>();

            case FProtoProtocolType::WIRELESS_SENSOR:
                return std::make_unique<FProtoEncoderWirelessSensor>();

            case FProtoProtocolType::LINEAR:
                return std::make_unique<FProtoEncoderLinear>();

            case FProtoProtocolType::UNKNOWN:
            default:
                return nullptr;
        }
    }

    /**
     * Get protocol type from name string.
     * Convenience method for parsing protocol names.
     *
     * @param name Protocol name (e.g., "HORMANN", "KEELOQ")
     * @return Protocol type enumeration
     */
    static FProtoProtocolType parse_protocol_name(const char* name) {
        if (!name) return FProtoProtocolType::UNKNOWN;

        if (strcmp(name, "HORMANN") == 0)      return FProtoProtocolType::HORMANN;
        if (strcmp(name, "KEELOQ") == 0)       return FProtoProtocolType::KEELOQ;
        if (strcmp(name, "STARLINE") == 0)     return FProtoProtocolType::STARLINE;
        if (strcmp(name, "BENINCA") == 0)      return FProtoProtocolType::BENINCA;
        if (strcmp(name, "CAME") == 0)         return FProtoProtocolType::CAME;
        if (strcmp(name, "DOORHAN") == 0)      return FProtoProtocolType::DOORHAN;
        if (strcmp(name, "MAGICAR") == 0)      return FProtoProtocolType::MAGICAR;
        if (strcmp(name, "HONDA") == 0)        return FProtoProtocolType::HONDA;
        if (strcmp(name, "FORD") == 0)         return FProtoProtocolType::FORD;
        if (strcmp(name, "SUBARU") == 0)       return FProtoProtocolType::SUBARU;
        if (strcmp(name, "CLEMSA") == 0)       return FProtoProtocolType::CLEMSA;
        if (strcmp(name, "MAGICAR-868") == 0)  return FProtoProtocolType::MAGICAR_868;
        if (strcmp(name, "STARLINE-868") == 0) return FProtoProtocolType::STARLINE_868;
        if (strcmp(name, "BENINCA-868") == 0)  return FProtoProtocolType::BENINCA_868;
        if (strcmp(name, "VOLKSWAGEN") == 0)   return FProtoProtocolType::VOLKSWAGEN;
        if (strcmp(name, "BMW") == 0)          return FProtoProtocolType::BMW;
        if (strcmp(name, "MERCEDES") == 0)     return FProtoProtocolType::MERCEDES;
        if (strcmp(name, "JEEP") == 0)         return FProtoProtocolType::JEEP;
        if (strcmp(name, "GARAGE-DOOR") == 0)  return FProtoProtocolType::GARAGE_DOOR;
        if (strcmp(name, "WIRELESS-SENSOR") == 0) return FProtoProtocolType::WIRELESS_SENSOR;
        if (strcmp(name, "LINEAR") == 0)       return FProtoProtocolType::LINEAR;

        return FProtoProtocolType::UNKNOWN;
    }

    /**
     * Get protocol name from type.
     * Reverse lookup for display/debugging.
     *
     * @param protocol_type Protocol type
     * @return String name of protocol
     */
    static const char* get_protocol_name(FProtoProtocolType protocol_type) {
        switch (protocol_type) {
            case FProtoProtocolType::HORMANN:    return "HORMANN";
            case FProtoProtocolType::KEELOQ:     return "KEELOQ";
            case FProtoProtocolType::STARLINE:   return "STARLINE";
            case FProtoProtocolType::BENINCA:    return "BENINCA";
            case FProtoProtocolType::CAME:       return "CAME";
            case FProtoProtocolType::DOORHAN:    return "DOORHAN";
            case FProtoProtocolType::MAGICAR:    return "MAGICAR";
            case FProtoProtocolType::HONDA:      return "HONDA";
            case FProtoProtocolType::FORD:       return "FORD";
            case FProtoProtocolType::SUBARU:     return "SUBARU";
            case FProtoProtocolType::CLEMSA:     return "CLEMSA";
            case FProtoProtocolType::MAGICAR_868: return "MAGICAR-868";
            case FProtoProtocolType::STARLINE_868: return "STARLINE-868";
            case FProtoProtocolType::BENINCA_868: return "BENINCA-868";
            case FProtoProtocolType::VOLKSWAGEN: return "VOLKSWAGEN";
            case FProtoProtocolType::BMW:        return "BMW";
            case FProtoProtocolType::MERCEDES:   return "MERCEDES";
            case FProtoProtocolType::JEEP:       return "JEEP";
            case FProtoProtocolType::GARAGE_DOOR: return "GARAGE-DOOR";
            case FProtoProtocolType::WIRELESS_SENSOR: return "WIRELESS-SENSOR";
            case FProtoProtocolType::LINEAR:     return "LINEAR";
            case FProtoProtocolType::UNKNOWN:
            default:
                return "UNKNOWN";
        }
    }
};
