#!/bin/bash
# ARFPORT build script
# Syncs src/ -> WSL mayhem-firmware, then builds with ninja.
set -e

REPO_ROOT="/mnt/e/Claude/Projects/ARFPORT"
SRC_SUBGHZTX="$REPO_ROOT/src/apps/subghztx"
SRC_SUBCARRX="$REPO_ROOT/src/apps/subcarrx"
SRC_BASEBAND="$REPO_ROOT/src/baseband"
EXTDIR="$HOME/mayhem-firmware/firmware/application/external"
BASEBANDDIR="$HOME/mayhem-firmware/firmware/baseband"
EXTCMAKE="$EXTDIR/external.cmake"
BUILDDIR="$HOME/mayhem-firmware/build"
OUTDIR="$REPO_ROOT/releases"

echo "=== Syncing subghztx ==="
mkdir -p "$EXTDIR/subghztx"
cp "$SRC_SUBGHZTX"/* "$EXTDIR/subghztx/"
echo "  $(ls "$EXTDIR/subghztx" | wc -l) files"

echo "=== Syncing subcarrx ==="
mkdir -p "$EXTDIR/subcarrx"
cp "$SRC_SUBCARRX"/* "$EXTDIR/subcarrx/"
echo "  $(ls "$EXTDIR/subcarrx" | wc -l) files"

echo "=== Syncing baseband (proc_subcar + fprotos) ==="
cp "$SRC_BASEBAND/proc_subcar.cpp" "$SRC_BASEBAND/proc_subcar.hpp" "$BASEBANDDIR/"
cp "$SRC_BASEBAND/fprotos/c-"*.hpp "$BASEBANDDIR/fprotos/"
cp "$SRC_BASEBAND/fprotos/subcarprotos.hpp" "$SRC_BASEBAND/fprotos/subcarbase.hpp" \
   "$SRC_BASEBAND/fprotos/subcartypes.hpp" "$BASEBANDDIR/fprotos/"
echo "  fprotos synced"

echo "=== Patching external.cmake ==="
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
for app in subghztx subcarrx; do
    PPMA=$(find "$BUILDDIR" -name "${app}.ppma" 2>/dev/null | head -1)
    if [ -n "$PPMA" ]; then
        cp "$PPMA" "$OUTDIR/${app}.ppma"
        echo "  SUCCESS: ${app}.ppma -> releases/"
    else
        echo "  WARNING: ${app}.ppma not found in build"
    fi
done
