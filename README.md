# pico-ice Arduino-Pico CRAM Programmer (PlatformIO)

This project programs the iCE40UP5K FPGA on the pico-ice board using the Arduino-Pico core (Earle Philhower) and bit-banged SPI from the RP2040. It mirrors the pico-ice-sdk CRAM sequence and checks CDONE to confirm configuration. No post-load verification is performed; the sysCONFIG bus is released after configuration.

## Features
- Load FPGA bitstreams directly from flash (compiled headers)
- Runtime selection via Serial: blank, traffic light controller, and optional blink
- External 12 MHz clock output on GPIO24 to drive the FPGA user logic
- Clean, minimal Arduino-based implementation (no pico-sdk required)

## Hardware Pins (RP2040 ➜ FPGA)
- GPIO8  = PIN_ICE_SI  (ICE_SI, RP2040 ➜ FPGA)
- GPIO11 = PIN_ICE_SO  (ICE_SO, FPGA ➜ RP2040)
- GPIO10 = PIN_ICE_SCK (ICE_SCK)
- GPIO9  = PIN_ICE_SSN (sysCONFIG SS, active-low)
- GPIO27 = PIN_FPGA_CRESETN (CRESET_B, active-low)
- GPIO26 = PIN_FPGA_CDONE (CDONE)
- GPIO24 = PIN_CLOCK (12 MHz clock to FPGA)
- GPIO13/12/15 = LED_R/G/B (active-low)

## Prerequisites
- PlatformIO (VS Code extension or CLI)
- A pico-ice board connected via USB

## Quick Start
1. Place your bitstreams in the project root as binary files:
   - `traffic_light_controller.bin` (provided)
   - `blank_design.bin` (provided)
   - `blink.bin` (optional)
2. Build and upload with PlatformIO. A pre-build script converts `.bin` files into headers in `include/`.
3. Open the Serial Monitor at 115200 baud.
4. At boot, pick a bitstream or wait for the default:
   - `b` = blank
   - `t` = traffic
   - `l` = blink (if present)
5. You can press `b/t/l` any time to re-load a different bitstream.

## Notes
- The configuration sequence follows the iCE40UP datasheet and pico-ice-sdk: assert CRESETN, prepare bus, select CRAM, stream bitstream, deassert SS, dummy clocks, wait for CDONE, then release sysCONFIG pins.
- The external clock is defined as `FPGA_CLK_FREQ` (default 12 MHz) and is started automatically before and after configuration.
- No read-back/verify is done after loading; many designs repurpose sysCONFIG pins.

## Repository Layout
- `src/main.cpp` — Arduino sketch that drives CRAM programming
- `convert_binaries.py` — Converts `.bin` files to C headers in `include/`
- `scripts/preconvert.py` — PlatformIO pre-build hook to run the converter
- `platformio.ini` — PlatformIO environment (Arduino-Pico core)
- `pico-ice-sdk/` — SDK reference sources (not required to build this sketch)

## License
MIT
