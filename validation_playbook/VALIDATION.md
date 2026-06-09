# ARFPORT on-device validation playbook

Sequenced from cheapest checks (does it build?) to most expensive (does
it actually transmit a key the receiver accepts?). Each step has clear
PASS/FAIL signals and a diagnostic for the most common failure mode.

## Phase 0 — Build prerequisites

```bash
git clone https://github.com/portapack-mayhem/mayhem-firmware portapack-mayhem
cd portapack-mayhem
git checkout v2.4.0    # or current main; rebase patches if needed
docker pull ghcr.io/portapack-mayhem/mayhem_dev_image:latest   # or whichever tag the repo specifies
```

Confirm baseline builds before adding any of our patches:

```bash
./dockerize.sh build
ls build/firmware/application/firmware_portarf.bin   # expect ~2 MB
```

**If baseline doesn't build:** issue is your toolchain, not our work. Stop and fix.

## Phase 1 — Apply patches

Each step's patch applies independently. Recommended order (least
risky first):

```bash
# Step 2 (PSA decrypt — pure algorithm, no Mayhem-API touch)
cp ARFPORT/step2_psa_xtea/{psa_decrypt.hpp,psa_decrypt.cpp} \
   firmware/application/

# Step 3 (KeeLoq common_ext + Key Manager)
cp ARFPORT/step3_keeloq_manager/{keeloq_common_ext.hpp,keeloq_common_ext.cpp} \
   firmware/application/
cp ARFPORT/step3_keeloq_manager/{ui_keeloq_manager.hpp,ui_keeloq_manager.cpp} \
   firmware/application/apps/
patch -p1 < ARFPORT/step3_keeloq_manager/step3_ui_subghzd.patch

# Step 9 (SubGhzTx external app)
mkdir -p firmware/application/external/subghztx
cp ARFPORT/step9_ui_subghztx/{*.hpp,*.cpp,CMakeLists.txt,main.cpp} \
   firmware/application/external/subghztx/
echo 'add_subdirectory(subghztx)' >> firmware/application/external/CMakeLists.txt

# Step 1 (Preset Hopping AM)
cp ARFPORT/step1_preset_hopping/{ui_subghzd_hopper.hpp,ui_subghzd_hopper.cpp} \
   firmware/application/apps/
patch -p1 < ARFPORT/step1_preset_hopping/step1_ui_subghzd.patch.txt

# Step 14 (FM RX) — biggest patch, do last
patch -p1 < ARFPORT/step14_fm_rx/step14_fm_rx.patch.txt
```

After each patch: `./dockerize.sh build` and confirm clean.

**Expected compile errors and fixes:**

| Error | Likely cause | Fix |
|---|---|---|
| `OptionsField` ctor mismatch | Mayhem 2.4.0+ changed signature | Add `app_settings::Mode` arg or remove extra |
| `Theme::getInstance()->fg_green` not found | Color name drift | Replace with `fg_yellow` or `fg_blue` |
| `MenuView::add_item` mismatch | Recent refactor | Check `ui_settings.cpp` line ~880 for current pattern |
| `set_bitstream_config` not declared | Mayhem internal name change | Search for actual declaration in `baseband_api.hpp` |
| `set_subghzd_config` arg count | `modulation` field new | Confirm Step 14 patch applied first |

## Phase 2 — Flash + smoke

```bash
# SD card route (preferred — no USB drivers needed)
cp build/firmware/portapack-mayhem*portarf*.ppfw.tar /Volumes/PORTAPACK/FIRMWARE/
# Eject, insert into PortaPack, power on → self-flashes
```

**PASS:** boot screen comes up, no panic.
**FAIL** (panic or hang): note the panic message — stack overflow in
`baseband` is usually the FM demod buffer too small; bump `fm_dst`
size or reduce decim_1 output count.

## Phase 3 — Step 1: Preset Hopping (AM tier)

1. Open SubGhzD.
2. Confirm new row: `Hop: [OFF] AM270`.
3. Set Hop to `-75 dBm`.
4. **Expected:** preset label flickers between `AM270` and `AM650`.
5. Press a known fob (any 433 MHz garage/gate remote — Princeton works).
6. **PASS:** preset label freezes (`AM270` or `AM650` whichever caught it)
   for ~5 seconds, recent-entries shows the decoded packet.
7. **FAIL — preset never freezes:** RSSI threshold too low; raise to -85.
8. **FAIL — never sees signal:** verify `field_frequency` matches fob.

## Phase 4 — Step 9: SubGhzTx — replay a static-code .sub

1. Drop a Princeton `.sub` on SD card under `/SUBGHZ/`.
2. Open SubGhzTx app → Browse → pick the file.
3. Status line shows `Protocol: Princeton @ 433920 kHz`.
4. Aim at a receiver wired to a relay/LED indicator at the same frequency.
5. Press Start.
6. **PASS:** receiver responds (relay clicks, light goes on).
7. **FAIL — no response:**
   - Check antenna polarization vs fob original (some are vertical-only).
   - Check TX gain (`tx_view` field).
   - Try a different known-good `.sub`.
   - Sniff with the Flipper Zero or another HackRF in RX to verify
     pulse train is on-air.

## Phase 5 — Step 14: FM RX (the new architectural piece)

This is the highest-risk validation. The patch went on theoretical
ground only.

### Quick smoke test (no specific signal needed):

1. Open SubGhzD → set new modulation field to `FM`.
2. Tune to 433.92 MHz.
3. **Expected:** RSSI bar moves with ambient RF; no panic.
4. **PASS:** UI responsive, RSSI plausible.
5. **FAIL — frozen UI / panic:** FM path is consuming CPU too hard;
   reduce `FSK_OFFSET_LEAK` to make slicing cheaper, or skip every
   N-th sample.

### Real signal test:

Best targets (in order of difficulty):

1. **Hormann BiSecure remote** (868.3 MHz, 2FSK ±12 kHz dev). If you
   own one — uncomment `s-hormannbisecure.hpp` in `subghzdprotos.hpp`
   first.
2. **KIA V5 fob** (433.92 MHz, FSK). If you have a KIA-equipped car.
3. **Honda Civic 2019** — your end goal. Won't decode without a
   Honda-specific fproto, but the pulse train should still appear in
   capture-mode.

For each:
- Tune to the right frequency.
- Press the fob.
- **PASS:** corresponding fproto decoder fires, recent-entries shows it.
- **FAIL — nothing visible:** capture wideband IQ first via
  `looking_glass_app`, confirm the signal is actually present at the
  expected frequency. If it is but SubGhzD doesn't decode:
  - Try inverting the slicer: change `level = v > fsk_offset` to
    `level = v < fsk_offset` in `proc_subghzd.cpp`.
  - Try widening `te_delta` in the relevant fproto decoder by 50%.
  - Try lowering `fm_deviation_hz` to 5000 (more sensitive slicing).

### Pulse-train inspection (deepest debug):

Add temporary logging in the FM branch of `proc_subghzd.cpp`:

```cpp
// After the slicer, before the run-length encoder:
static uint32_t debug_count = 0;
if (++debug_count < 100) {
    SubGhzDDebugMessage msg{level, currentDuration};
    shared_memory.application_queue.push(msg);
}
```

(Add `SubGhzDDebugMessage` to `message.hpp`, handle it on M0 by
appending to a debug log.) This shows whether the slicer is producing
plausible bit transitions vs uniform noise.

## Phase 6 — Step 7: KeeLoq TX (rolling code)

Only attempt with a fob you own and can re-pair if the receiver loses sync.

1. Capture a real fob press in SubGhzD (KeeLoq mode).
2. Note `Serial`, `Cnt`, `Btn`, `Manufacturer`.
3. In SubGhzTx (or the dedicated KeeLoq Manager UI when wired), select
   that capture.
4. Bump counter by +1 to +16 to stay within the receiver's resync window.
5. TX. Receiver should accept.
6. **FAIL — receiver doesn't accept:** counter outside window; press
   the real fob a few times, recapture, retry.

## Diagnostic flowchart

```
TX side broken?
├── No fproto decoder visible in SubGhzD on real signal
│   └── It's an RX issue (Step 14 FM, or fproto te_delta) — see Phase 5
├── SubGhzTx says "Protocol unsupported" on a .sub file
│   └── Either the encoder doesn't cover that protocol yet, OR
│       the .sub's `Protocol:` field uses a name we don't match.
│       Check sub_file_dispatch.cpp `ieq()` calls.
├── SubGhzTx transmits but receiver doesn't respond
│   ├── Counter wrong (rolling code) → see Phase 6
│   ├── Frequency wrong → check `field_frequency` value
│   ├── Modulation mismatch → AM file via FM TX or vice versa
│   └── Antenna / TX gain → check tx_view fields, antenna match
└── Patch applied but app doesn't appear in launcher
    └── External app registration: check
        firmware/application/external/CMakeLists.txt has add_subdirectory()
```

## Reporting back

If something fails on-device, capture:

1. The exact panic message (if any) — photo of screen
2. Last few lines of compile output (if a build error)
3. Wideband IQ capture via `looking_glass_app` of the failing-to-decode
   signal (replays for offline analysis)

These three artefacts let me iterate from sandbox without needing
hardware.

## Plan status

- ✅ Build process documented
- ✅ Per-feature smoke tests defined
- ✅ Diagnostic flowchart for common failures
- ⏳ Awaiting actual hardware run
