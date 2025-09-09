# pico-ice / pico2-ice Arduino-Pico CRAM Programmer (PlatformIO)

This project programs the iCE40UP5K FPGA on the pico-ice (RP2040) or pico2-ice (RP2350) board using the Arduino-Pico core (Earle Philhower). It mirrors the pico-ice-sdk CRAM sequence and checks CDONE to confirm configuration. No post-load verification is performed; the sysCONFIG bus is released after configuration.

## Implementations (Branches)
- main: Bit-banged SPI implementation (simpler, CPU-driven). External clock via PWM.
- PIO: Uses RP2040 PIO for SPI and clock (lower CPU load, faster). Includes generated PIO headers so pioasm is optional.

## Features
- Load FPGA bitstreams directly from flash (compiled headers)
- Runtime selection via Serial: blank, traffic light controller, and optional blink
- FPGA clock on GPIO24
  - main: PWM at 12 MHz (configurable via `FPGA_CLK_FREQ`)
  - PIO: jitter-free PIO clock ≈ clk_sys/3 (e.g., ~44 MHz at 133 MHz clk_sys)
- SPI for CRAM
  - main: bit-banged
  - PIO: ~20 MHz SCK using PIO
- Clean, minimal Arduino-based implementation (no Pico SDK build needed)
- Supports both pico-ice (RP2040) and pico2-ice (RP2350) with pin adjustments

## Hardware Pins (RP2040/RP2350 ➜ FPGA)
For pico-ice (RP2040):
- GPIO8  = PIN_ICE_SI  (ICE_SI, RP2040 ➜ FPGA)
- GPIO11 = PIN_ICE_SO  (ICE_SO, FPGA ➜ RP2040)
- GPIO10 = PIN_ICE_SCK (ICE_SCK)
- GPIO9  = PIN_ICE_SSN (sysCONFIG SS, active-low)
- GPIO14 = PIN_RAM_SS  (External PSRAM SS)
- GPIO27 = PIN_FPGA_CRESETN (CRESET_B, active-low)
- GPIO26 = PIN_FPGA_CDONE (CDONE)
- GPIO24 = PIN_CLOCK (clock to FPGA)
- GPIO13/12/15 = LED_R/G/B (active-low)

For pico2-ice (RP2350):
- GPIO4  = PIN_ICE_SI  (ICE_SI, RP2350 ➜ FPGA)
- GPIO7  = PIN_ICE_SO  (ICE_SO, FPGA ➜ RP2350)
- GPIO6  = PIN_ICE_SCK (ICE_SCK)
- GPIO5  = PIN_ICE_SSN (sysCONFIG SS, active-low)
- GPIO8  = PIN_RAM_SS  (External PSRAM SS)
- GPIO31 = PIN_FPGA_CRESETN (CRESET_B, active-low)
- GPIO26 = PIN_FPGA_CDONE (CDONE)
- GPIO21 = PIN_CLOCK (clock to FPGA)
- GPIO1/0/9 = LED_R/G/B (active-low)

## Board Selection
To select the board, choose the appropriate PlatformIO environment:
- For pico-ice (RP2040): Use `env:pico`
- For pico2-ice (RP2350): Use `env:pico2`

The environment automatically sets the correct define (`PICO2_ICE` for pico2-ice) in the build flags, so no manual code changes are needed.

## Prerequisites
- PlatformIO (VS Code extension or CLI)
- A pico-ice or pico2-ice board connected via USB

## Quick Start
1. Select your board by choosing the appropriate PlatformIO environment:
   - For pico-ice: Select `env:pico`
   - For pico2-ice: Select `env:pico2`
2. Place your bitstreams in the project root as binary files:
   - `traffic_light_controller.bin` (provided)
   - `blank_design.bin` (provided)
   - `blink.bin` (optional)
3. Build and upload with PlatformIO. A pre-build script converts `.bin` files into headers in `include/`.
4. Open the Serial Monitor at 115200 baud.
5. At boot, pick a bitstream or wait for the default:
   - `b` = blank
   - `t` = traffic
   - `l` = blink (if present)
6. You can press `b/t/l` any time to re-load a different bitstream.

## Notes
- The configuration sequence follows the iCE40UP datasheet and pico-ice-sdk: assert CRESETN, prepare bus, select CRAM, stream bitstream, deassert SS, dummy clocks, wait for CDONE, then release sysCONFIG pins.
- In the PIO branch, the PIO program drives MOSI+SCK and a dedicated PIO SM generates the FPGA clock. SPI runs ~20 MHz by default (see `CRAM_SPI_HZ`).
- Generated PIO headers (`include/*.pio.h`) are tracked in the repo. Builds work even without `pioasm` installed.
- The Arduino-Pico core supports both RP2040 and RP2350 seamlessly.

## Repository Layout
- `src/main.cpp` — Arduino sketch that drives CRAM programming
- `src/pio/` — PIO assembly sources (PIO branch)
- `include/*.pio.h` — Generated PIO headers (checked-in)
- `convert_binaries.py` — Converts `.bin` files to C headers in `include/`
- `scripts/preconvert.py` — PlatformIO pre-build hook to run the converter
- `scripts/pioasm.py` — Assembles PIO sources and sanitizes headers (PIO branch)
- `platformio.ini` — PlatformIO environment (Arduino-Pico core)
- `pico-ice-sdk/` — SDK reference sources (not required to build this sketch)

## Branches
- `main`: bit-banged SPI, PWM clock.
- `PIO`: PIO SPI (~20 MHz) and PIO clock; PIO headers committed; build works without `pioasm`.

## License
MIT License
