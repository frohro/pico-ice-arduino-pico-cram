// Arduino-Pico port to program iCE40 UltraPlus CRAM on pico-ice (RP2040) or pico2-ice (RP2350) via bit-banged SPI
// Steps: hold CRESET low, prep pins, ensure external 12MHz clock, enter CRAM cfg, stream bitstream, close & check CDONE.
// The sysCONFIG bus is released after configuration; no post-load verification is performed.
// For pico2-ice, uncomment the next line: #define PICO2_ICE

#include <Arduino.h>
#include <hardware/gpio.h>

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
static constexpr uint8_t PIN_ICE_SI     = 4;   // ICE_SI (MOSI from RP2350 to FPGA)
static constexpr uint8_t PIN_ICE_SO     = 7;   // ICE_SO (MISO from FPGA to RP2350)
static constexpr uint8_t PIN_ICE_SCK    = 6;   // ICE_SCK
static constexpr uint8_t PIN_ICE_SSN    = 5;   // ICE_SSN (FPGA sysCONFIG SS, active low)
static constexpr uint8_t PIN_RAM_SS     = -1;  // No external PSRAM on pico2-ice
static constexpr uint8_t PIN_FPGA_CRESETN = 31; // FPGA CRESET_B (active low) - correct per pico-ice-sdk
static constexpr uint8_t PIN_FPGA_CDONE  = 40; // FPGA CDONE - correct per pico-ice-sdk
static constexpr uint8_t PIN_LED_R       = 1;  // Active-low
static constexpr uint8_t PIN_LED_G       = 0;  // Active-low
static constexpr uint8_t PIN_LED_B       = 9;  // Active-low
static constexpr uint8_t PIN_CLOCK       = 21; // External clock to FPGA (ICE_CLK)
#else
static constexpr uint8_t PIN_ICE_SI     = 8;   // ICE_SI (MOSI from RP2040 to FPGA)
static constexpr uint8_t PIN_ICE_SO     = 11;  // ICE_SO (MISO from FPGA to RP2040)
static constexpr uint8_t PIN_ICE_SCK    = 10;  // ICE_SCK
static constexpr uint8_t PIN_ICE_SSN    = 9;   // ICE_SSN (FPGA sysCONFIG SS, active low)
static constexpr uint8_t PIN_RAM_SS     = 14;  // External PSRAM SS (keep deasserted)
static constexpr uint8_t PIN_FPGA_CRESETN = 27; // FPGA CRESET_B (active low)
static constexpr uint8_t PIN_FPGA_CDONE  = 26; // FPGA CDONE
static constexpr uint8_t PIN_LED_R       = 13; // Active-low
static constexpr uint8_t PIN_LED_G       = 12; // Active-low
static constexpr uint8_t PIN_LED_B       = 15; // Active-low
static constexpr uint8_t PIN_CLOCK       = 24; // External clock to FPGA (ICE_CLK)
#endif

// External clock frequency for FPGA user logic
static constexpr uint32_t FPGA_CLK_FREQ = 12000000; // 12 MHz

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
  pinMode(PIN_CLOCK, OUTPUT);
  analogWriteRange(2);
  analogWriteFreq(FPGA_CLK_FREQ);
  analogWrite(PIN_CLOCK, 1); // 50% duty (1/2)
  delayMicroseconds(10); // allow PWM to settle
  g_clk_running = true;
}

// Generate N dummy SCLK cycles with SCK while SS is in its current state
static void clocks(uint32_t count) {
  for (uint32_t i = 0; i < count; ++i) { sck_high(); sck_low(); }
}

// Write a byte stream to FPGA CRAM via bit-banged SPI (mode 0, MSB first)
static void bb_write(const uint8_t* data, size_t len) {
  for (size_t n = 0; n < len; ++n) {
    uint8_t b = data[n];
    for (int i = 7; i >= 0; --i) {
      si_write((b >> i) & 1);
      sck_high();
      sck_low();
    }
  }
}

// Enter CRAM configuration mode and prepare to stream a bitstream
static bool cram_open() {
  // Ensure pins configured
  pinMode(PIN_FPGA_CRESETN, OUTPUT);
  // Initialize CDONE pin properly like pico-ice-sdk (no pull resistors, high impedance)
  gpio_init(PIN_FPGA_CDONE);
  gpio_disable_pulls(PIN_FPGA_CDONE);
  gpio_put(PIN_FPGA_CDONE, false);
  gpio_set_dir(PIN_FPGA_CDONE, GPIO_IN);
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

  // Hold FPGA in reset (active low)
  digitalWrite(PIN_FPGA_CRESETN, LOW);

  // Initialize clock idle low, data default low
  sck_low();
  si_write(0);

  // Ensure external FPGA clock is running before releasing reset
  if (!g_clk_running) start_fpga_clock();
  delayMicroseconds(10);

  // Select CRAM target (active low)
  digitalWrite(PIN_ICE_SSN, LOW);

  // After at least 200ns, release reset
  delayMicroseconds(2);
  digitalWrite(PIN_FPGA_CRESETN, HIGH);

  // Wait at least 1200us for internal config memory clear
  delayMicroseconds(1300);

  // Per datasheet: SS high for 8 SCLKs before bitstream
  digitalWrite(PIN_ICE_SSN, HIGH);
  clocks(8);
  digitalWrite(PIN_ICE_SSN, LOW);

  return true;
}

// Stream bitstream bytes into CRAM
static bool cram_write(const uint8_t* data, size_t len) {
  if (!data || !len) return false;
  bb_write(data, len);
  return true;
}

// Finalize configuration and confirm CDONE goes high within spec
static bool cram_close() {
  // Deassert SS and emulate SDK behavior: leave it pulled up and Hi-Z
  digitalWrite(PIN_ICE_SSN, HIGH);
  delayMicroseconds(1);
  pinMode(PIN_ICE_SSN, INPUT_PULLUP);
  pinMode(PIN_ICE_SSN, INPUT);

  // Output dummy clocks with SI=0 while SS is high. CDONE should go high within 100 SCLKs
  si_write(0);
  bool done = false;
  int clocks_until_done = -1;
  for (int i = 0; i < 13 && !done; ++i) {
    for (int bit = 0; bit < 8; ++bit) {
      sck_high(); sck_low();
      if (!done && gpio_get(PIN_FPGA_CDONE)) {
        done = true;
        clocks_until_done = i * 8 + bit + 1;
        break;
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
    }
  }
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.print("HB CDONE=");
    Serial.println(gpio_get(PIN_FPGA_CDONE));
  }
  if (millis() - lastHelp > 10000) {
    lastHelp = millis();
    Serial.print("Keys: b/t=blank/traffic");
#if HAVE_BLINK
    Serial.print(", l=blink");
#endif
    Serial.println(".");
  }
}