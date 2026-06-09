/*
 * SubGhzTx external-app registration.
 * Modeled on firmware/application/external/flippertx/main.cpp.
 */

#include "ui.hpp"
#include "ui_navigation.hpp"
#include "ui_subghztx.hpp"
#include "external_app.hpp"

namespace ui::external_app::subghztx {

void initialize_app(ui::NavigationView& nav) {
    nav.push<SubGhzTxView>();
}

}  // namespace ui::external_app::subghztx

extern "C" {

__attribute__((section(".external_app.app_subghztx.application_information"), used))
application_information_t _application_information_subghztx = {
    /*.memory_location =*/ (uint8_t*)0x00000000,
    /*.externalAppEntry =*/ ui::external_app::subghztx::initialize_app,
    /*.header_version =*/ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,
    /*.app_name =*/ "SubGhzTx",
    /*.bitmap_data =*/ {
        0x20, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x00,
        0xE0, 0x07, 0xF0, 0x0F, 0x30, 0x0C, 0x30, 0x0C,
        0xF0, 0x0F, 0xF0, 0x0F, 0x70, 0x0D, 0xB0, 0x0E,
        0x70, 0x0D, 0xB0, 0x0E, 0xF0, 0x0F, 0xE0, 0x07,
    },
    /*.icon_color =*/ ui::Color::cyan().v,
    /*.menu_location =*/ app_location_t::TX,
    /*.desired_menu_position =*/ -1,
    /*.m4_app_tag = */ {'P', 'O', 'S', 'K'},
    /*.m4_app_offset =*/ 0x00000000,
};

}
