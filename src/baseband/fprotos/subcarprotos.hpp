/*
This is the protocol list handler. It holds an instance of all known protocols.
So include here the .hpp, and add a new element to the protos vector in the constructor. That's all you need to do here if you wanna add a new proto.
    @htotoo
*/

#include <vector>
#include <memory>
#include "portapack_shared_memory.hpp"

#include "fprotolistgeneral.hpp"
#include "subcarbase.hpp"
#include "c-suzuki.hpp"
#include "c-vw.hpp"
#include "c-subaru.hpp"
#include "c-kia_v5.hpp"
#include "c-kia_v3v4.hpp"
#include "c-kia_v2.hpp"
#include "c-kia_v1.hpp"
#include "c-kia_v0.hpp"
#include "c-ford_v0.hpp"
#include "c-fiat_v0.hpp"
#include "c-bmw_v0.hpp"
#include "c-honda.hpp"
#include "c-ford_v1.hpp"   // Ford/Jaguar/Land Rover
#include "c-mazda_v0.hpp"
#include "c-mitsubishi_v0.hpp"
#include "c-kia_v7.hpp"
#include "c-fiat_v1.hpp"
#include "c-porsche_v0.hpp"
#include "c-chrysler_v0.hpp"
#include "c-bmw_cas4.hpp"
#include "c-hyundai_kia_v0.hpp"
#include "c-hyundai_santa_fe_v0.hpp"
// #include "c-kia_v6.hpp"
#include "c-ford_v2.hpp"
#include "c-ford_v3.hpp"
#include "c-land_rover_v0.hpp"   // Land Rover
// #include "c-psa.hpp"        // European PSA Group only
// #include "c-scher_khan.hpp" // Russian alarm only
// #include "c-star_line.hpp"  // Russian alarm only
// #include "c-honda_static.hpp"  // very old Honda fixed-code only
#include "c-toyota.hpp"   // Toyota KeeLoq (Corolla 433 / Tundra 315), decode-only

#ifndef __FPROTO_PROTOLISTCAR_H__
#define __FPROTO_PROTOLISTCAR_H__

class SubCarProtos : public FProtoListGeneral {
   public:
    SubCarProtos(const SubCarProtos&) = delete;
    SubCarProtos& operator=(const SubCarProtos&) = delete;
    SubCarProtos() {
        // Protocol objects are non-static members: no __cxa_guard, no GC-section elimination.
        // Addresses are always valid (offsets within the enclosing SubCarProcessor heap block).
        protos[FPC_SUZUKI]          = &obj_suzuki;
        protos[FPC_VW]              = &obj_vw;
        protos[FPC_SUBARU]          = &obj_subaru;
        protos[FPC_KIAV5]           = &obj_kiav5;
        protos[FPC_KIAV3V4]         = &obj_kiav3v4;
        protos[FPC_KIAV2]           = &obj_kiav2;
        protos[FPC_KIAV1]           = &obj_kiav1;
        protos[FPC_KIAV0]           = &obj_kiav0;
        protos[FPC_FORDV0]          = &obj_fordv0;
        protos[FPC_FIATV0]          = &obj_fiatv0;
        protos[FPC_BMWV0]           = &obj_bmwv0;
        protos[FPC_HONDA]           = &obj_honda;
        protos[FPC_FORDV1]          = &obj_fordv1;
        protos[FPC_MAZDAV0]         = &obj_mazdav0;
        protos[FPC_MITSUBISHIV0]    = &obj_mitsubishi;
        protos[FPC_KIAV7]           = &obj_kiav7;
        protos[FPC_FIATV1]          = &obj_fiatv1;
        protos[FPC_PORSCHEV0]       = &obj_porsche;
        protos[FPC_CHRYSLERV0]      = &obj_chrysler;
        protos[FPC_BMWCAS4]         = &obj_bmwcas4;
        protos[FPC_HYUNDAIKIA_V0]   = &obj_hyundaikia;
        protos[FPC_HYUNDAISANTAFEV0]= &obj_hyundaisantafe;
        protos[FPC_FORDV2]          = &obj_fordv2;
        protos[FPC_FORDV3]          = &obj_fordv3;
        protos[FPC_LAND_ROVER_V0]   = &obj_landrover;
        protos[FPC_TOYOTA]          = &obj_toyota;

        for (uint8_t i = 0; i < FPC_COUNT; ++i) {
            if (protos[i] != NULL) protos[i]->setCallback(callbackTarget);
        }
    }

    ~SubCarProtos() {}

    static void callbackTarget(FProtoSubCarBase* instance) {
        SubCarDataMessage packet_message{instance->sensorType, instance->data_count_bit, instance->decode_data, instance->decode_data2};
        shared_memory.application_queue.push(packet_message);
    }

    void feed(bool level, uint32_t duration) {
        obj_suzuki.feed(level, duration);
        obj_vw.feed(level, duration);
        obj_subaru.feed(level, duration);
        obj_kiav5.feed(level, duration);
        obj_kiav3v4.feed(level, duration);
        obj_kiav2.feed(level, duration);
        obj_kiav1.feed(level, duration);
        obj_kiav0.feed(level, duration);
        obj_fordv0.feed(level, duration);
        obj_fiatv0.feed(level, duration);
        obj_bmwv0.feed(level, duration);
        obj_honda.feed(level, duration);
        obj_mazdav0.feed(level, duration);
        obj_mitsubishi.feed(level, duration);
        obj_kiav7.feed(level, duration);
        obj_fiatv1.feed(level, duration);
        obj_porsche.feed(level, duration);
        obj_chrysler.feed(level, duration);
        obj_bmwcas4.feed(level, duration);
        obj_hyundaikia.feed(level, duration);
        obj_hyundaisantafe.feed(level, duration);
        obj_fordv2.feed(level, duration);
        obj_fordv3.feed(level, duration);
        obj_fordv1.feed(level, duration);
        obj_landrover.feed(level, duration);
        obj_toyota.feed(level, duration);
    }

   protected:
    FProtoSubCarBase* protos[FPC_COUNT] = {NULL};

    // Protocol decoders embedded as members — no static storage, no __cxa_guard.
    // Allocated as part of SubCarProcessor on the heap (via make_unique).
    FProtoSubCarSuzuki          obj_suzuki{};
    FProtoSubCarVW              obj_vw{};
    FProtoSubCarSubaru          obj_subaru{};
    FProtoSubCarKiaV5           obj_kiav5{};
    FProtoSubCarKiaV3V4         obj_kiav3v4{};
    FProtoSubCarKiaV2           obj_kiav2{};
    FProtoSubCarKiaV1           obj_kiav1{};
    FProtoSubCarKiaV0           obj_kiav0{};
    FProtoSubCarFordV0          obj_fordv0{};
    FProtoSubCarFiatV0          obj_fiatv0{};
    FProtoSubCarBMWV0           obj_bmwv0{};
    FProtoSubCarHonda           obj_honda{};         // 2FSK NRZ — FM mode required
    FProtoSubCarFordV1          obj_fordv1{};        // Ford/Jaguar/Land Rover
    FProtoSubCarMazdaV0         obj_mazdav0{};       // also Infiniti
    FProtoSubCarMitsubishiV0    obj_mitsubishi{};
    FProtoSubCarKiaV7           obj_kiav7{};
    FProtoSubCarFiatV1          obj_fiatv1{};
    FProtoSubCarPorscheV0       obj_porsche{};       // also VW Touareg
    FProtoSubCarChryslerV0      obj_chrysler{};      // Dodge/Jeep/PT Cruiser
    FProtoSubCarBmwCas4         obj_bmwcas4{};
    FProtoSubCarHyundaiKiaV0    obj_hyundaikia{};    // Accent/Rio/Elantra
    FProtoSubCarHyundaiSantaFeV0 obj_hyundaisantafe{};  // Santa Fe 2013-16
    FProtoSubCarFordV2          obj_fordv2{};
    FProtoSubCarFordV3          obj_fordv3{};
    FProtoSubCarLandRoverV0     obj_landrover{};   // Land Rover
    FProtoSubCarToyota          obj_toyota{};      // Toyota KeeLoq (Corolla/Tundra)
};

#endif
