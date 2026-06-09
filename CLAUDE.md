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

## Build Steps (run in WSL terminal via clipboard paste)

1. Build (sync + compile + copy releases): `bash /mnt/e/Claude/Projects/ARFPORT/build.sh`
   - Syncs src/apps/subghztx → ~/mayhem-firmware/.../external/subghztx/
   - Syncs src/apps/subcarrx → ~/mayhem-firmware/.../external/subcarrx/
   - Syncs src/baseband/proc_subcar.* and fprotos → ~/mayhem-firmware/.../baseband/
   - Patches external.cmake if needed
   - Runs ninja (NOT make — project uses Ninja generator)
   - Copies subghztx.ppma + subcarrx.ppma → releases/

2. Copy to SD card (mount I:, copy ppma, unmount):
   `sudo mount -t drvfs I: /mnt/i -o uid=1000,gid=1000 && cp /mnt/e/Claude/Projects/ARFPORT/releases/subcarrx.ppma /mnt/i/APPS/ && sync && sudo umount /mnt/i`
   - If CMakeCache is stale: `cd ~/mayhem-firmware && cmake -GNinja -Bbuild . && ninja -C build`

Note: /mnt/e/ DrvFs can go stale in WSL. If build.sh runs silently (no output), run `wsl --shutdown` from Windows CMD/PowerShell and reopen WSL.

## How to execute commands
- Write command to clipboard via Claude's write_clipboard tool
- User pastes in WSL terminal with Ctrl+Shift+V
- Terminal tier is "click" (no typing from Claude)

## SubCarRx status (ACTIVE DEBUGGING as of Jun 8 2026)
- M0 crashes with Guru Meditation HardFault — vtable corruption, two known crashes:
  1. pc=0x10088548 (BMW V0 pointer bug — FIXED in src/baseband/fprotos/c-bmw_v0.hpp)
  2. pc=0x10008570, lr=0x10007600 — NEW crash after session fixes; cause TBD
- SUSPECT: `delete protos[i]` in subcarprotos.hpp destructor may be causing heap corruption
  - Previous code used `free(protos[i])` — consider reverting to diagnose
- M4-side Subaru bug: `(uint8_t*)entry_.data` in ui_subcar.cpp line 359 → should be `&entry_.data`
- SubGhzTx status: done_cb → TXProgressMessage working; rf_amp(true)+tx_gain(35) in start()
- subghztx + subcarrx registered in ~/mayhem-firmware/firmware/application/external/external.cmake
