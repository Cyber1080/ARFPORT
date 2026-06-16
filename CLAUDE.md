Use as few tokens as possible.

# ARFPORT Build Workflow

## Source locations (edit these, NOT mayhem-firmware-2.4.0/)
- SubGhzTx (TX app): `E:\Claude\Projects\ARFPORT\src\apps\subghztx\`
- SubCarRx (RX app): `E:\Claude\Projects\ARFPORT\src\apps\subcarrx\`
- Baseband processor: `E:\Claude\Projects\ARFPORT\src\baseband\proc_subcar.cpp/.hpp`
- Protocol decoders: `E:\Claude\Projects\ARFPORT\src\baseband\fprotos\c-*.hpp` (and subcarprotos.hpp, subcarbase.hpp)

## Environment
- ARM toolchain: `/opt/gcc-arm-none-eabi-9-2020-q2-update/bin/` (already in WSL, native)
- Docker: BROKEN — volumes were deleted. Do NOT attempt to use Docker.
- WSL user: `cyber1080@DESKTOP-UT5SRSV`
- Build dir: `~/mayhem-firmware/build/`
- Releases: `E:\Claude\Projects\ARFPORT\releases\` (subcarrx.ppma, subghztx.ppma)

## ⚠️ DEPLOYMENT (CHANGED Jun 15 2026): SubCarRx is now a BUILT-IN app — deploy via FIRMWARE FLASH
- SubCarRx was converted from an external .ppma app to a BUILT-IN app to fix the SRAM-overflow crash
  (external packed UI+baseband into the 40KB m4_code region; all 25 decoders overflowed into
  shared_memory). Built-in M4 UI code runs from SPIFI flash (XIP), so only the 26KB baseband uses
  local_sram_1. Stock memory map now fits everything with room to spare.
- SubCarRx M4 UI: firmware/application/apps/ui_subcar.cpp (compiled into application.elf).
  Baseband launch: baseband::run_image(portapack::spi_flash::image_tag_subcar) where
  image_tag_subcar={'P','S','C','D'} (added to firmware/common/spi_image.hpp), loads PSCD from baseband.img.
- So changes to ui_subcar.*, proc_subcar.*, OR fprotos now require FLASHING THE FIRMWARE, not an SD copy.
  Flash file: releases/portapack-mayhem_dev.bin (USB: hackrf_spiflash -w, device in HackRF mode) OR
  releases/portapack-mayhem_OCI.ppfw.tar (on-device Utilities>Flash Utility; build.sh stages it on SD /FIRMWARE/).
- subghztx is STILL external: deploy via subghztx.ppma -> SD /APPS/ (build.sh does this), no flash.
- Firmware structural edits live in the WSL clone (re-apply if it resets): memory_map.hpp STOCK
  (m4_code 32_KiB / shared_memory 8_KiB); portapack_shared_memory.hpp app_local_queue_k=11;
  spi_image.hpp image_tag_subcar; ui_navigation.cpp (#include "ui_subcar.hpp" + appList RX entry);
  application CMakeLists.txt apps/ui_subcar.cpp; external.cmake has NO subcarrx.
- ⚠️ GOTCHA 1 (namespace = linker placement): external.ld has `*(*ui*external_app*subcarrx*)` -> 0xADF20000.
  The app namespace MUST NOT contain `external_app` or the built-in code gets banished to the external
  region (crash "Hard Fault", pc=0xADF2xxxx). Namespace is now `ui::subcarrx` (NOT ui::external_app::subcarrx).
- ⚠️ GOTCHA 2 (baseband must be in firmware): firmware/baseband/CMakeLists.txt has `set(add_to_firmware FALSE)`
  at ~line 506. DeclareTargets(PSCD subcar) sits below it, so it built an EXTERNAL chunk only -> run_image
  failed with "NoImg". Fix: wrap it `set(add_to_firmware TRUE)` / DeclareTargets(PSCD subcar) / `set(add_to_firmware FALSE)`
  so PSCD goes into baseband.img. Verify: `strings build/firmware/baseband/baseband.img | grep -c PSCD` >= 1.
- ⚠️ A CMake reconfigure (`cmake -GNinja -Bbuild .`) is REQUIRED after editing external.cmake / CMakeLists.txt /
  baseband CMakeLists — ninja alone won't pick up those, leaving stale external-app build rules.

## ⚠️ The "M0 Guru Meditation / Hard Fault" panic is an M4 UI fault, and the hint is HARDCODED
- firmware/application/debug.cpp:374 draws CORTEX_M0 + literal "Hard Fault" for M4 hard faults; it
  does NOT display the M0's m4_panic_msg. Never read the hint text to judge baseband freshness.
- The displayed registers (r0-r3,r12,lr,pc) are the M4 fault context. Decode with:
  arm-none-eabi-addr2line -e build/firmware/application/application.elf 0x<addr>.
  Observed crash: M4 paint cycle (Painter::paint_widget / Widget::style / View::paint + std::string
  build). Fix applied in ui_subcar.cpp pad_string_with_spaces (unsigned-underflow -> huge string).

## Build Steps (run in WSL terminal via clipboard paste)

1. Build (sync + compile + copy releases): `bash /mnt/e/Claude/Projects/ARFPORT/build.sh`
   - Syncs src/apps/subghztx → external/subghztx/ ; src/apps/subcarrx/ui_subcar.* → apps/ (built-in)
   - Syncs src/baseband/proc_subcar.* and fprotos → ~/mayhem-firmware/.../baseband/
   - Patches external.cmake (subghztx only) ; runs ninja
   - Copies subghztx.ppma + portapack-mayhem_dev.bin + .ppfw.tar → releases/ ; subghztx.ppma → SD /APPS/
   - If a clean rebuild is needed (memory_map or SharedMemory changes): `ninja -C build clean` first.

2. Flash firmware for SubCarRx (one of):
   - USB: device in HackRF mode, then `E:\...\releases\hackrf_spiflash.exe -w E:\...\releases\portapack-mayhem_dev.bin`
   - On-device: Utilities > Flash Utility, pick /FIRMWARE/portapack-mayhem_OCI.ppfw.tar (build.sh stages it on SD).
   - If CMakeCache is stale: `cd ~/mayhem-firmware && cmake -GNinja -Bbuild . && ninja -C build`

Note: /mnt/e/ DrvFs can go stale in WSL. If build.sh runs silently (no output), run `wsl --shutdown` from Windows CMD/PowerShell and reopen WSL.

## How to execute commands
- Write command to clipboard via Claude's write_clipboard tool
- User pastes in WSL terminal with Ctrl+Shift+V
- Terminal tier is "click" (no typing from Claude)

## SubCarRx status (RESOLVED Jun 15 2026)
- CRASH FIXED by converting SubCarRx from external app to BUILT-IN (see DEPLOYMENT section above).
  Root cause: external loader packed [26KB baseband][~10KB M4 UI] into the 40KB local_sram_1 region;
  with all 25 decoders the .ppma (~36.4KB) overflowed the 32KB m4_code boundary into shared_memory
  (0x10088000), and the running M0 baseband stomped the overflowed M4 code -> wild vtable -> "Hard Fault".
- Built-in build (verified): baseband_subcar.elf text=25988B fits 32KB (~6.8KB spare); SharedMemory=6712B
  fits stock 8KB; application.elf links ui::external_app::subcarrx::SubCarView; all 25 decoders active.
- The "Hard Fault" panic hint is HARDCODED (firmware/application/debug.cpp:374) — meaningless for diagnosis.
- Decoders restored: Ford_v1 + Land_Rover_v0 re-enabled (all 25 in subcarprotos.hpp feed()).
- pad_string_with_spaces underflow fixed in ui_subcar.cpp (real bug, not the crash).
- Title marker: ui_subcar.hpp title() may say "SubCar" (revert any debug marker). #pragma GCC optimize("Os")
  in ui_subcar.cpp is harmless (kept).
- SubGhzTx status: done_cb → TXProgressMessage working; rf_amp(true)+tx_gain(35) in start(); still external.
- Registered: subghztx in external.cmake; subcar in ui_navigation.cpp appList (RX) + apps/ in CMakeLists.txt.
