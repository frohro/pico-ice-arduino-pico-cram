// Arduino-Pico port to program iCE40 UltraPlus CRAM on pico-ice (RP2040) or pico2-ice (RP2350) via bit-banged SPI
// Steps: hold CRESET low, prep pins, ensure external 12MHz clock, enter CRAM cfg, stream bitstream, close & check CDONE.
// The sysCONFIG bus is released after configuration; no post-load verification is performed.
// For pico2-ice, uncomment the next line: #define PICO2_ICE

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
// PIO header is generated at build time by scripts/pioasm.py
#if __has_include("syscfg_spi_tx.pio.h")
#include "syscfg_spi_tx.pio.h"
#define USE_PIO 1
#elif __has_include("syscfg_spi_tx.pio.h")
#define USE_PIO 1
extern const uint16_t syscfg_spi_tx_program_instructions[];
extern const struct pio_program syscfg_spi_tx_program;
extern const char syscfg_spi_tx_program_name[];
extern const int syscfg_spi_tx_program_length;
extern const int syscfg_spi_tx_program_origin;
#else
#define USE_PIO 0
#endif
#if __has_include("clk_out.pio.h")
#include "clk_out.pio.h"
#define USE_PIO_CLK 1
#elif __has_include("clk_out.pio.h")
#define USE_PIO_CLK 1
extern const uint16_t clk_out_program_instructions[];
extern const struct pio_program clk_out_program;
extern const char clk_out_program_name[];
extern const int clk_out_program_length;
extern const int clk_out_program_origin;
#else
#define USE_PIO_CLK 0
#endif

// Include both bitstreams; choose at runtime via serial prompt
#include "blank_design.h"
#include "traffic_light_controller.h"
#if __has_include("blink.h")
#include "blink.h"
#define HAVE_BLINK 1
#else
#define HAVE_BLINK 0
#endif

// Pin mapping for pico-ice (RP2040) and pico2-ice (RP2350)
// For pico2-ice, define PICO2_ICE before compiling
#ifdef PICO2_ICE
static constexpr int8_t PIN_ICE_SI     = 4;   // ICE_SI (MOSI from RP2350 to FPGA)
static constexpr int8_t PIN_ICE_SO     = 7;   // ICE_SO (MISO from FPGA to RP2350)
static constexpr int8_t PIN_ICE_SCK    = 6;   // ICE_SCK
static constexpr int8_t PIN_ICE_SSN    = 5;   // ICE_SSN (FPGA sysCONFIG SS, active low)
static constexpr int8_t PIN_RAM_SS     = -1;  // No external PSRAM on pico2-ice
static constexpr int8_t PIN_FPGA_CRESETN = 31; // FPGA CRESET_B (active low) - correct per pico-ice-sdk
static constexpr int8_t PIN_FPGA_CDONE  = 40; // FPGA CDONE - correct per pico-ice-sdk
static constexpr int8_t PIN_LED_R       = 1;  // Active-low
static constexpr int8_t PIN_LED_G       = 0;  // Active-low
static constexpr int8_t PIN_LED_B       = 9;  // Active-low
static constexpr int8_t PIN_CLOCK       = 21; // External clock to FPGA (ICE_CLK)
#else
static constexpr int8_t PIN_ICE_SI     = 8;   // ICE_SI (MOSI from RP2040 to FPGA)
static constexpr int8_t PIN_ICE_SO     = 11;  // ICE_SO (MISO from FPGA to RP2040)
static constexpr int8_t PIN_ICE_SCK    = 10;  // ICE_SCK
static constexpr int8_t PIN_ICE_SSN    = 9;   // ICE_SSN (FPGA sysCONFIG SS, active low)
static constexpr int8_t PIN_RAM_SS     = 14;  // External PSRAM SS (keep deasserted)
static constexpr int8_t PIN_FPGA_CRESETN = 27; // FPGA CRESET_B (active low)
static constexpr int8_t PIN_FPGA_CDONE  = 26; // FPGA CDONE
static constexpr int8_t PIN_LED_R       = 13; // Active-low
static constexpr int8_t PIN_LED_G       = 12; // Active-low
static constexpr int8_t PIN_LED_B       = 15; // Active-low
static constexpr int8_t PIN_CLOCK       = 24; // External clock to FPGA (ICE_CLK)
#endif

// External clock frequency for FPGA user logic
static constexpr uint32_t FPGA_CLK_FREQ = 12000000; // 12 MHz
// PIO clock runs at clk_sys/3 when clkdiv=1.0 in clk_out.pio (duty ~33/67)
// This avoids fractional divider jitter.
// static constexpr uint32_t FPGA_CLK_PIO_HZ = 48000000; // desired PIO clock to FPGA

// Target SPI SCK for CRAM programming (~20 MHz)
static constexpr uint32_t CRAM_SPI_HZ = 20000000;

// Helpers for active-low LEDs
static inline void ledOn(uint8_t pin) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
static inline void ledOff(uint8_t pin) { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }

// Bit-bang helpers for CRAM SPI
static inline void sck_low()  { digitalWrite(PIN_ICE_SCK, LOW); }
static inline void sck_high() { digitalWrite(PIN_ICE_SCK, HIGH); }
static inline void si_write(bool v) { digitalWrite(PIN_ICE_SI, v ? HIGH : LOW); }

// Manage the external FPGA clock. Safe to call repeatedly; re-applies settings.
static bool g_clk_running = false;
static void start_fpga_clock() {
  if (g_clk_running) {
    Serial.println("FPGA clock already running");
    return;
  }
  Serial.println("Starting FPGA clock...");
#if USE_PIO_CLK
  // Use PIO1 for the clock to avoid contention with PIO0 used for SPI
  PIO pio = pio1;
  int sm = pio_claim_unused_sm(pio, true);
  uint off = pio_add_program(pio, &clk_out_program);
  pio_gpio_init(pio, PIN_CLOCK);
  pio_sm_config c = clk_out_program_get_default_config(off);
  sm_config_set_sideset_pins(&c, PIN_CLOCK);
  // clkdiv = 1.0 => f_out = clk_sys / 3 (from 3 instructions in the loop)
  sm_config_set_clkdiv(&c, 1.0f);
  pio_sm_init(pio, sm, off, &c);
  pio_sm_set_consecutive_pindirs(pio, sm, PIN_CLOCK, 1, true);
  pio_sm_set_enabled(pio, sm, true);
  g_clk_running = true;
  Serial.println("FPGA clock started (PIO)");
#else
  pinMode(PIN_CLOCK, OUTPUT);
  analogWriteRange(2);
  analogWriteFreq(FPGA_CLK_FREQ);
  analogWrite(PIN_CLOCK, 1); // 50% duty (1/2)
  delayMicroseconds(10);
  g_clk_running = true;
  Serial.println("FPGA clock started (PWM)");
#endif
}

// Generate N dummy SCLK cycles with SCK while SS is in its current state
static void clocks(uint32_t count) {
  for (uint32_t i = 0; i < count; ++i) { sck_high(); sck_low(); }
}

#if USE_PIO
// PIO SPI TX engine for sysCONFIG (MOSI+SCK). SS/CRESETN handled in C++.
static PIO g_pio = pio0;
static int g_sm = -1;
static uint g_off = 0;

// Configure and start the PIO SPI TX on given pins and target SCK frequency
static void syscfg_pio_begin(uint pin_mosi, uint pin_sck, uint32_t sck_hz) {
  if (g_sm >= 0) return;
  g_off = pio_add_program(g_pio, &syscfg_spi_tx_program);
  g_sm = pio_claim_unused_sm(g_pio, true);

  // Route pins to PIO
  pio_gpio_init(g_pio, pin_mosi);
  pio_gpio_init(g_pio, pin_sck);

  pio_sm_config c = syscfg_spi_tx_program_get_default_config(g_off);
  sm_config_set_out_pins(&c, pin_mosi, 1);
  sm_config_set_sideset_pins(&c, pin_sck);
  // MSB-first: shift left so MSB is output first
  sm_config_set_out_shift(&c, /*shift_right=*/false, /*autopull=*/false, /*pull_thresh=*/8);

  // Make pins outputs
  pio_sm_set_consecutive_pindirs(g_pio, g_sm, pin_mosi, 1, true);
  pio_sm_set_consecutive_pindirs(g_pio, g_sm, pin_sck, 1, true);
  pio_sm_set_out_pins(g_pio, g_sm, pin_mosi, 1);
  pio_sm_set_sideset_pins(g_pio, g_sm, pin_sck);

  float div = (float)clock_get_hz(clk_sys) / (3.0f * (float)sck_hz);
  if (div < 1.0f) div = 1.0f;
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(g_pio, g_sm, g_off, &c);
  pio_sm_set_enabled(g_pio, g_sm, true);
}

// Send a byte buffer via PIO TX (blocking). Assumes SS already asserted.
static void syscfg_pio_tx(const uint8_t* data, size_t len) {
  Serial.print("SPI TX: sending ");
  Serial.print(len);
  Serial.println(" bytes");
  for (size_t i = 0; i < len; ++i) {
    // Align byte to MSB so MSB-first shifting outputs correct order
    pio_sm_put_blocking(g_pio, g_sm, ((uint32_t)data[i]) << 24);
    if (i < 10) { // Debug first 10 bytes
      Serial.print("Byte ");
      Serial.print(i);
      Serial.print(": 0x");
      Serial.println(data[i], HEX);
    }
  }
  // Wait for FIFO to drain
  while (!pio_sm_is_tx_fifo_empty(g_pio, g_sm)) { /* wait */ }
  // Small guard to let last edges complete
  delayMicroseconds(1);
}

static void syscfg_pio_end(uint pin_mosi, uint pin_sck) {
  if (g_sm >= 0) {
    pio_sm_set_enabled(g_pio, g_sm, false);
    pio_remove_program(g_pio, &syscfg_spi_tx_program, g_off);
    pio_sm_unclaim(g_pio, g_sm);
    g_sm = -1;
  }
  // Return pins to GPIO so we can bit-bang dummy clocks
  pinMode(pin_mosi, OUTPUT); digitalWrite(pin_mosi, LOW);
  pinMode(pin_sck, OUTPUT);  digitalWrite(pin_sck, LOW);
}
#endif

// Enter CRAM configuration mode and prepare to stream a bitstream
static bool cram_open() {
  Serial.println("CRAM open: starting FPGA configuration sequence");

  // Ensure pins configured
  pinMode(PIN_FPGA_CRESETN, OUTPUT);
  pinMode(PIN_FPGA_CDONE, INPUT);
  pinMode(PIN_ICE_SSN, OUTPUT);
  // Configure bit-bang pins
  pinMode(PIN_ICE_SCK, OUTPUT);
  pinMode(PIN_ICE_SI, OUTPUT); // drive FPGA SI via GPIO8
  // PSRAM SS is active-high via CMOS inverter; board has 10k pulldown on SRAM_SS
  // Tri-state with pulldown so the line stays low (deasserted) without active drive
  // Only configure PSRAM SS pin if it exists (not on pico2-ice)
  if (PIN_RAM_SS >= 0) {
    pinMode(PIN_RAM_SS, INPUT_PULLDOWN);
  }

  Serial.print("Pin states before config: CRESETN=");
  Serial.print(digitalRead(PIN_FPGA_CRESETN));
  Serial.print(", CDONE=");
  Serial.print(digitalRead(PIN_FPGA_CDONE));
  Serial.print(", SSN=");
  Serial.print(digitalRead(PIN_ICE_SSN));
  Serial.println();

  // Hold FPGA in reset (active low)
  Serial.println("Setting CRESETN LOW (FPGA in reset)");
  digitalWrite(PIN_FPGA_CRESETN, LOW);

  // Initialize clock idle low, data default low
  sck_low();
  si_write(0);

  // Ensure external FPGA clock is running before releasing reset
  if (!g_clk_running) start_fpga_clock();
  delayMicroseconds(10);

  // Select CRAM target (active low)
  Serial.println("Setting SSN LOW (select CRAM)");
  digitalWrite(PIN_ICE_SSN, LOW);

  // After at least 200ns, release reset
  delayMicroseconds(2);
  Serial.println("Setting CRESETN HIGH (FPGA out of reset)");
  digitalWrite(PIN_FPGA_CRESETN, HIGH);

  // Wait at least 1200us for internal config memory clear
  Serial.println("Waiting 1300us for FPGA internal memory clear");
  delayMicroseconds(1300);

  // Per datasheet: SS high for 8 SCLKs before bitstream
  Serial.println("Sending 8 dummy clocks with SSN HIGH");
  digitalWrite(PIN_ICE_SSN, HIGH);
  clocks(8);
  digitalWrite(PIN_ICE_SSN, LOW);

  Serial.println("CRAM open: ready for bitstream");
  return true;
}

#if USE_PIO
// Write a byte stream to FPGA CRAM via PIO SPI (mode 0, MSB first)
static bool cram_write(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  syscfg_pio_begin(PIN_ICE_SI, PIN_ICE_SCK, CRAM_SPI_HZ);
  syscfg_pio_tx(data, len);
  return true;
}
#else
// Fallback bit-banged write if PIO header is unavailable
static bool cram_write(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    for (int bit = 7; bit >= 0; --bit) {
      si_write((b >> bit) & 1);
      sck_high(); sck_low();
    }
  }
  return true;
}
#endif

// Finalize configuration and confirm CDONE goes high within spec
static bool cram_close() {
  Serial.println("CRAM close: starting CDONE detection");
  Serial.print("CDONE pin state before SS deassert: ");
  Serial.println(digitalRead(PIN_FPGA_CDONE));

  // Deassert SS and emulate SDK behavior: leave it pulled up and Hi-Z
  digitalWrite(PIN_ICE_SSN, HIGH);
  delayMicroseconds(1);
  pinMode(PIN_ICE_SSN, INPUT_PULLUP);
  pinMode(PIN_ICE_SSN, INPUT);

#if USE_PIO
  // Release PIO so we can drive dummy clocks via GPIO
  syscfg_pio_end(PIN_ICE_SI, PIN_ICE_SCK);
#endif

  // Output dummy clocks with SI=0 while SS is high. CDONE should go high within 100 SCLKs
  si_write(0);
  bool done = false;
  int clocks_until_done = -1;
  Serial.println("Sending dummy clocks, monitoring CDONE...");
  for (int i = 0; i < 13 && !done; ++i) {
    for (int bit = 0; bit < 8; ++bit) {
      sck_high(); sck_low();
      bool cdone_state = digitalRead(PIN_FPGA_CDONE);
      if (!done && cdone_state) {
        done = true;
        clocks_until_done = i * 8 + bit + 1;
        Serial.print("CDONE went HIGH at clock ");
        Serial.println(clocks_until_done);
        break;
      }
      if (bit == 0 && i < 5) { // Debug first few bytes
        Serial.print("Clock ");
        Serial.print(i * 8 + bit + 1);
        Serial.print(": CDONE=");
        Serial.println(cdone_state);
      }
    }
  }
  // At least another 49 SCLK cycles once CDONE goes high (use 56 like SDK for margin)
  clocks(56);
  if (!done) {
    Serial.println("CDONE did not assert within 100 SCLKs after SS high");
  } else {
    Serial.print("CDONE asserted after ");
    Serial.print(clocks_until_done);
    Serial.println(" SCLKs");
  }
  return done;
}

// Brief status indicator: on error, flash red; success: no indication (avoid contention with FPGA)
static void indicateStatus(bool ok) {
  if (ok) {
    return;
  } else {
    for (int i = 0; i < 6; ++i) {
      ledOn(PIN_LED_R); delay(100);
      ledOff(PIN_LED_R); delay(100);
    }
  }
}

struct BitstreamSel { const uint8_t* data; size_t size; const char* name; };

// Prompt over serial to choose which embedded bitstream to load
static BitstreamSel choose_bitstream(uint32_t timeout_ms = 5000) {
  uint32_t start = millis();
  char sel = 0;
  while ((millis() - start) < timeout_ms) {
    uint32_t remaining = (timeout_ms - (millis() - start) + 999) / 1000;
    Serial.print("Select bitstream: 'b' blank, 't' traffic");
#if HAVE_BLINK
    Serial.print(", 'l' blink");
#endif
    Serial.print(" (default traffic in ");
    Serial.print(remaining);
    Serial.println("s)");
    uint32_t tick = millis();
    while ((millis() - tick) < 1000) {
      if (Serial.available()) { sel = (char)Serial.read(); break; }
      delay(10);
    }
    if (sel) break;
  }
  if (sel == 'b' || sel == 'B') {
    return { blank_design_bin, blank_design_bin_size, "blank_design.bin" };
  }
#if HAVE_BLINK
  if (sel == 'l' || sel == 'L') {
    return { blink_bin, blink_bin_size, "blink.bin" };
  }
#endif
  return { traffic_light_controller_bin, traffic_light_controller_bin_size, "traffic_light_controller.bin" };
}

// Release LED pins so the FPGA design can drive them
static void release_led_pins() {
  pinMode(PIN_LED_R, INPUT);
  pinMode(PIN_LED_G, INPUT);
  pinMode(PIN_LED_B, INPUT);
}

// Full FPGA configuration sequence using the selected bitstream
static bool configure_fpga(BitstreamSel bs) {
  Serial.print("Bitstream: "); Serial.println(bs.name);
  Serial.println("FPGA CRAM: open");
  bool ok = cram_open();
  if (!ok) { Serial.println("open failed"); indicateStatus(false); return false; }
  // Lightweight checksum to confirm integrity
  uint32_t sum = 0;
  for (size_t i = 0; i < bs.size; ++i) sum += bs.data[i];
  Serial.print("Writing "); Serial.print(bs.name); Serial.print(" (bytes=");
  Serial.print(bs.size); Serial.print(", sum32=0x"); Serial.print(sum, HEX); Serial.println(")...");
  ok = cram_write(bs.data, bs.size);
  if (!ok) { Serial.println("write failed"); indicateStatus(false); return false; }
  Serial.println("FPGA CRAM: close");
  ok = cram_close();
  if (!ok) {
    Serial.println("close failed (CDONE not high)");
    indicateStatus(false);
    return false;
  }
  Serial.println("FPGA configured (CDONE=1)");
  release_led_pins();
  // Refresh the FPGA clock so user logic sees a clean start
  start_fpga_clock();
  Serial.println("FPGA clock: RESTART after configuration");
  return true;
}

// Arduino setup: bring up serial, start clock, select and configure bitstream
void setup() {
  Serial.begin(115200);

  // Start the FPGA clock ASAP so it's present during and after configuration
  start_fpga_clock();

  // Wait briefly for USB CDC to be ready so setup() prints are visible
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 4000) {
    delay(10);
  }

  // Ensure LEDs off
  ledOff(PIN_LED_R); ledOff(PIN_LED_G); ledOff(PIN_LED_B);

  // Choose bitstream and configure at boot
  BitstreamSel bs = choose_bitstream();
  (void)configure_fpga(bs);
}

// Arduino loop: handle serial commands for loading and status
void loop() {
  static uint32_t lastPrint = 0;
  static uint32_t lastHelp = 0;
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'b' || c == 'B') {
      (void)configure_fpga({ blank_design_bin, blank_design_bin_size, "blank_design.bin" });
    } else if (c == 't' || c == 'T') {
      (void)configure_fpga({ traffic_light_controller_bin, traffic_light_controller_bin_size, "traffic_light_controller.bin" });
#if HAVE_BLINK
    } else if (c == 'l' || c == 'L') {
      (void)configure_fpga({ blink_bin, blink_bin_size, "blink.bin" });
#endif
    } else if (c == 'p' || c == 'P') {
      Serial.println("Programming FPGA with current bitstream...");
      // Use the bitstream that was chosen at boot
      BitstreamSel current_bs = choose_bitstream();
      (void)configure_fpga(current_bs);
    } else if (c == 'r' || c == 'R') {
      Serial.println("Resetting FPGA...");
      // Simple reset by toggling CRESETN
      digitalWrite(PIN_FPGA_CRESETN, LOW);
      delayMicroseconds(10);
      digitalWrite(PIN_FPGA_CRESETN, HIGH);
    } else if (c == 's' || c == 'S') {
      Serial.println("FPGA status:");
      Serial.print("CDONE: ");
      Serial.println(digitalRead(PIN_FPGA_CDONE) ? "HIGH" : "LOW");
      Serial.print("CRESETN: ");
      Serial.println(digitalRead(PIN_FPGA_CRESETN) ? "HIGH" : "LOW");
      Serial.print("SSN: ");
      Serial.println(digitalRead(PIN_ICE_SSN) ? "HIGH" : "LOW");
    }
  }
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.print("HB CDONE=");
    Serial.print(digitalRead(PIN_FPGA_CDONE));
    Serial.print(", CRESETN=");
    Serial.print(digitalRead(PIN_FPGA_CRESETN));
    Serial.print(", SSN=");
    Serial.print(digitalRead(PIN_ICE_SSN));
    Serial.print(", SCK=");
    Serial.print(digitalRead(PIN_ICE_SCK));
    Serial.print(", SI=");
    Serial.print(digitalRead(PIN_ICE_SI));
    if (PIN_RAM_SS >= 0) {
      Serial.print(", RAM_SS=");
      Serial.print(digitalRead(PIN_RAM_SS));
    }
    Serial.println();
  }
  if (millis() - lastHelp > 10000) {
    lastHelp = millis();
    Serial.print("Keys: b/t=blank/traffic");
#if HAVE_BLINK
    Serial.print(", l=blink");
#endif
    Serial.println(", p=program, r=reset, s=status.");
  }
}