#!/bin/bash
# ARFPORT build script
# SubCarRx is now a BUILT-IN app (UI in apps/, baseband via PSCD flash image).
# subghztx remains an external .ppma app.
set -e

REPO_ROOT="/mnt/e/Claude/Projects/ARFPORT"
SRC_SUBGHZTX="$REPO_ROOT/src/apps/subghztx"
SRC_SUBCARRX="$REPO_ROOT/src/apps/subcarrx"
SRC_BASEBAND="$REPO_ROOT/src/baseband"
APPDIR="$HOME/mayhem-firmware/firmware/application"
EXTDIR="$APPDIR/external"
BASEBANDDIR="$HOME/mayhem-firmware/firmware/baseband"
EXTCMAKE="$EXTDIR/external.cmake"
BUILDDIR="$HOME/mayhem-firmware/build"
OUTDIR="$REPO_ROOT/releases"

echo "=== Syncing subghztx (external) ==="
mkdir -p "$EXTDIR/subghztx"
cp "$SRC_SUBGHZTX"/* "$EXTDIR/subghztx/"
touch "$EXTDIR/subghztx/"* 2>/dev/null || true

echo "=== Syncing SubCarRx UI (built-in apps/) ==="
cp "$SRC_SUBCARRX/ui_subcar.cpp" "$SRC_SUBCARRX/ui_subcar.hpp" "$APPDIR/apps/"
touch "$APPDIR/apps/ui_subcar.cpp" "$APPDIR/apps/ui_subcar.hpp"
# Safety: ensure no stale external subcarrx app remains
rm -rf "$EXTDIR/subcarrx" 2>/dev/null || true

echo "=== Syncing baseband (proc_subcar + fprotos) ==="
cp "$SRC_BASEBAND/proc_subcar.cpp" "$SRC_BASEBAND/proc_subcar.hpp" \
   "$SRC_BASEBAND/proc_subcar_app.cpp" "$SRC_BASEBAND/proc_subcar_app.hpp" "$BASEBANDDIR/"
cp "$SRC_BASEBAND/fprotos/c-"*.hpp "$BASEBANDDIR/fprotos/"
cp "$SRC_BASEBAND/fprotos/subcarprotos.hpp" "$SRC_BASEBAND/fprotos/subcarbase.hpp" \
   "$SRC_BASEBAND/fprotos/subcartypes.hpp" "$BASEBANDDIR/fprotos/"
touch "$BASEBANDDIR/proc_subcar.cpp" "$BASEBANDDIR/proc_subcar.hpp" \
      "$BASEBANDDIR/proc_subcar_app.cpp" "$BASEBANDDIR/proc_subcar_app.hpp"
find "$BASEBANDDIR/fprotos/" -name "*.hpp" -exec touch {} \;
echo "  baseband synced + timestamps forced"

echo "=== Patching external.cmake (subghztx only) ==="
if ! grep -q "subghztx" "$EXTCMAKE"; then
    sed -i 's|external/keeloqtx/ui_keeloqtx.cpp|external/keeloqtx/ui_keeloqtx.cpp\n\n\t#subghztx\n\texternal/subghztx/main.cpp\n\texternal/subghztx/ui_subghztx.cpp\n\texternal/subghztx/sub_file_parser.cpp\n\texternal/subghztx/sub_file_dispatch.cpp\n\texternal/subghztx/fproto_encoder_base.cpp\n\texternal/subghztx/fproto_encoder_extra.cpp\n\texternal/subghztx/fproto_enc_keeloq.cpp\n\texternal/subghztx/keeloq_common_ext.cpp|' "$EXTCMAKE"
    sed -i 's/\tkeeloqtx$/\tkeeloqtx\n\tsubghztx/' "$EXTCMAKE"
    echo "  external.cmake patched"
else
    echo "  external.cmake already has subghztx"
fi

echo "=== Building ==="
cd "$HOME/mayhem-firmware"
ninja -C build 2>&1 | tail -30

echo "=== Copying releases ==="
mkdir -p "$OUTDIR"
# subghztx external app
PPMA=$(find "$BUILDDIR" -name "subghztx.ppma" 2>/dev/null | head -1)
[ -n "$PPMA" ] && cp "$PPMA" "$OUTDIR/subghztx.ppma" && echo "  subghztx.ppma -> releases/"
# firmware image (contains built-in SubCarRx)
FW=$(find "$BUILDDIR" -name "portapack-mayhem*.bin" 2>/dev/null | head -1)
[ -n "$FW" ] && cp "$FW" "$OUTDIR/$(basename "$FW")" && echo "  $(basename "$FW") -> releases/ (FLASH THIS for SubCarRx)"
FWTAR=$(find "$BUILDDIR" -name "portapack-mayhem*.ppfw.tar" 2>/dev/null | head -1)
[ -n "$FWTAR" ] && cp "$FWTAR" "$OUTDIR/$(basename "$FWTAR")" && echo "  $(basename "$FWTAR") -> releases/"

echo "=== Copying subghztx to SD card (I:) ==="
if sudo mount -t drvfs I: /mnt/i -o uid=1000,gid=1000 2>/dev/null; then
    cp "$OUTDIR/subghztx.ppma" /mnt/i/APPS/ 2>/dev/null && sync && echo "  subghztx.ppma -> SD OK"
    # remove stale external subcarrx.ppma if present (now built-in)
    rm -f /mnt/i/APPS/subcarrx.ppma 2>/dev/null && echo "  removed stale subcarrx.ppma from SD" || true
    sudo umount /mnt/i 2>/dev/null || true
else
    echo "  SD card (I:) not available, skipping"
fi
echo "=== NOTE: flash the firmware .bin for SubCarRx (built-in). ==="
