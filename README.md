# ARFPORT — Automotive RF for PortaPack

Automotive RF signal receiver and transmitter apps for [PortaPack Mayhem](https://github.com/portapack-mayhem/mayhem-firmware) (v2.4.0).  
Hardware: PortaPack H2 / PortaRF + HackRF One.

## Apps

### SubCarRx — Automotive Sub-GHz Receiver
Decodes 433/315 MHz car key fob signals in real time. Displays protocol, bit count, and decoded data. Supports both AM (OOK) and FM (FSK) modulation.

**Supported protocols (25+):**
BMW V0, BMW CAS4, Chrysler V0, Fiat V0/V1, Ford V0–V3, Honda, Honda Static,
Hyundai/Kia V0, Hyundai Santa Fe V0, Kia V0–V7, Land Rover V0, Mazda V0,
Mitsubishi V0, Porsche V0, PSA (Peugeot/Citroën), Scher-Khan, Star Line,
Subaru, Suzuki, VW.

### SubGhzTx — Sub-GHz File Transmitter
Replays Flipper Zero `.sub` capture files. Supports AM/FM, RAW, and BinRAW formats.

## Repository Structure

```
src/
  apps/
    subcarrx/       # M4 UI app (SubCarRx)
    subghztx/       # M4 UI app (SubGhzTx)
  baseband/
    proc_subcar.cpp/hpp   # M0 baseband processor (OOK/FSK demodulator)
    fprotos/              # Protocol decoder library (c-*.hpp files)
releases/           # Pre-built .ppma files (copy to SD card /APPS/)
build.sh            # Build + sync script (run in WSL)
```

## Building

### Prerequisites
1. Clone [Mayhem firmware v2.4.0](https://github.com/portapack-mayhem/mayhem-firmware/tree/v2.4.0) to `~/mayhem-firmware` in WSL
2. Set up the Mayhem build environment (ARM toolchain, Ninja) per the Mayhem README
3. Clone this repo to `E:\Claude\Projects\ARFPORT` (or update paths in `build.sh`)

### Build
```bash
# In WSL:
bash /mnt/e/Claude/Projects/ARFPORT/build.sh
```
This syncs `src/` → `~/mayhem-firmware/`, patches `external.cmake`, runs Ninja, and copies `.ppma` files to `releases/`.

### Install
Copy `releases/subcarrx.ppma` and/or `releases/subghztx.ppma` to the `/APPS/` folder on your PortaPack SD card.

## Contributing

The custom source lives entirely in `src/`. You do not need to modify the Mayhem firmware tree directly — `build.sh` handles the sync.

- Protocol decoders: `src/baseband/fprotos/c-<make>.hpp`
- To add a new car protocol, implement `FProtoSubCarBase`, add it to `src/baseband/fprotos/subcarprotos.hpp`, and add the type to `src/baseband/fprotos/subcartypes.hpp`
- The M0 baseband processor is `src/baseband/proc_subcar.cpp`

## License

GPLv2 (same as PortaPack Mayhem). Protocol decoders derived from [ProtoPirate](https://github.com/RocketGod-git/ProtoPirate) (Flipper Zero, also GPLv2).
