/*
  ESP32-S3 DVS RECEIVER - DUAL TRAKTOR MK2 I2S DAC
  - Generates Traktor Scratch MK2 timecode for two decks in real time, speed
    and direction driven by each transmitter's RPM over ESP-NOW.
  - Onboard RGB LED status: RED = no transmitters, ORANGE = one, GREEN = both.

  Signal format (reverse-engineered from a rip of the real MK2 vinyl,
  timecode1.wav, verified bit-exact against 100k+ decoded bits):
    - 2500 Hz quadrature carrier at 33 1/3 rpm (R leads L by 90 deg forward).
      Constant amplitude - speed and direction only.
    - Absolute position: an additive NRZ serial bitstream on BOTH channels,
      1000 bit/s at 33 1/3, phase-locked to the carrier (1 bit = exactly 2.5
      carrier cycles). Data amplitude ~0.3x carrier.
    - Bit sequence: 23-tap LFSR, period 4,194,303 bits (2^22-1, ~70 min of
      groove), recurrence b[i] = b[i-1]^b[i-8]^b[i-9]^b[i-14]^b[i-16]
      ^b[i-22]^b[i-23].

  Only a TRAKTOR_LOOP_SECONDS window of the sequence is generated and looped
  (default 5 min = ~37.5 KB table; PSRAM used if enabled, plain heap
  otherwise), so the absolute position never reaches the record's end zone
  where Traktor disables vinyl control. RELATIVE MODE ONLY.
  Requires the Adafruit NeoPixel library.

  A one-way debug serial (DEBUG_SERIAL, any USB mode works) prints a boot
  banner plus periodic per-deck link status and free heap. DEBUG_SERIAL 0
  silences it.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <driver/i2s.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ===== I2S DAC pins (one stereo PCM5102A per deck) ====================
#define DAC_A_BCK_PIN   2
#define DAC_A_LRCK_PIN  1
#define DAC_A_DATA_PIN  42
#define DAC_B_BCK_PIN   14
#define DAC_B_LRCK_PIN  13
#define DAC_B_DATA_PIN  12

// ===== Onboard status LED (DevKitC-1 N16R8 WS2812 = GPIO 48) ===========
#define LED_PIN 48
#define NUM_LEDS 1
#define LED_BRIGHTNESS 40
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Radio / protocol (must match the transmitter) ==================
#define ESPNOW_CHANNEL 11
#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define MSG_EVENT 5   // transmitter status relayed to our serial (its own
                      // USB is unreachable while it spins on battery)
#define MSG_BUTTONS 6 // dicer button state (unit 3)
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 1000

// ===== Auto-pairing (receiver assigns decks by transmitter MAC) ========
// Power the receiver first: for PAIRING_WINDOW_MS an unknown puck may claim
// a free deck (first to arrive = deck 1, second = deck 2). A puck already
// in the table (dropped and powered back on) reclaims its own deck at any
// time, window or not. Table is RAM-only: rebooting the receiver, or a long
// press on the re-pair button, clears it and reopens the window.
#define PAIRING_WINDOW_MS 60000UL
#define REPAIR_BTN_PIN 4           // momentary button to GND (INPUT_PULLUP)
#define REPAIR_HOLD_MS 1500        // long-press duration to clear + re-open

// ===== Dicer (unit 3) -> USB-MIDI [TEST] ==============================
// The dicer sends its full button state as MSG_BUTTONS; we diff states
// into MIDI note on/off on the S3's NATIVE USB port. Requires Tools >
// USB Mode: "USB-OTG (TinyUSB)" - in CDC/JTAG mode MIDI compiles out
// (with a warning) and everything else still works.
// Note = MIDI_BASE_NOTE + mode*6 + button, so every mode page is a
// distinct set of notes for MIDI-learn.
#define DICER_ID 3
#define USB_MIDI_ENABLE 0         // production: MIDI/dicer output disabled.
                                  // The experimental dicer unit lives in
                                  // Experimental/; set to 1 + build in USB-OTG
                                  // (TinyUSB) mode to re-enable dicer->MIDI.
#define MIDI_CHANNEL 1
#define MIDI_BASE_NOTE 36
#if USB_MIDI_ENABLE && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE == 0
  #include "USB.h"
  #include "USBMIDI.h"
  #define USB_MIDI_ACTIVE 1
  USBMIDI MIDI;
#else
  #define USB_MIDI_ACTIVE 0
  #if USB_MIDI_ENABLE
    #warning "USB-MIDI disabled: set Tools > USB Mode to USB-OTG (TinyUSB)"
  #endif
#endif

#define LED_UPDATE_MS 50          // fast enough for clean low-batt blinking

// [TEST] Low-battery indication: a puck reporting LOW/CRITICAL makes the
// status LED blink (deck 1 = slow on/off, deck 2 = two fast pulses,
// both = three fast pulses). The state expires if the puck stops
// re-sending it (it repeats every 10 s while low).
#define BATT_LOW_HOLD_MS 30000

// [TEST] Link-loss failover: if a puck goes silent mid-performance, keep
// generating timecode at its last STABLE rpm (one that held within
// +-FAILOVER_STABLE_BAND_RPM for FAILOVER_STABLE_MS) instead of stopping
// the deck. Scratching never becomes the held value; a stopped platter
// holds 0, so "stop platter, then switch the puck off" still ends clean.
// The LED still drops to orange/red so the fallback is visible.
#define FAILOVER_ENABLE 1
#define FAILOVER_STABLE_BAND_RPM 0.75f
#define FAILOVER_STABLE_MS 2000

// ===== Serial debug (optional; not used to carry timecode) ============
#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
#define DEBUG_PRINT_INTERVAL_MS 500

// ===== Audio =========================================================
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 32            // small buffers = low output latency
#define DMA_BUF_COUNT 4

// ===== Timecode response ==============================================
#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.04f        // RPM (the transmitter's 0.2 dominates)
#define MAX_RPM_RATIO 8.0f        // 8x -> 20 kHz carrier, still < Nyquist
#define RPM_SMOOTHING 0.80f

// ===== Traktor MK2 generator (measured from timecode1.wav) ============
#define TRAKTOR_CARRIER_HZ 2500.0f      // carrier cycles/s at 33 1/3 rpm
#define TRAKTOR_BITS_PER_CYCLE 0.4f     // 1 data bit per 2.5 carrier cycles
#define TRAKTOR_SEQ_PERIOD 4194303UL    // 2^22-1 bits (~70 min of groove)
#define TRAKTOR_SEED 0x22F586UL         // 23-bit LFSR state at rip start
// Parity mask over the state register (LSB = newest bit): taps j-1 for
// j in {1,8,9,14,16,22,23}.
#define TRAKTOR_TAPS_MASK 0x60A181UL
// Loop a short window of the sequence instead of playing all ~70 min of it:
// near the record's end Traktor drops to internal "play" mode and disables
// vinyl control until the needle is repositioned. Wrapping early means the
// absolute position never gets there; relative mode treats each wrap as a
// needle skip and keeps playing. RELATIVE MODE ONLY - in absolute mode the
// deck would jump back every TRAKTOR_LOOP_SECONDS.
#define TRAKTOR_LOOP_SECONDS 300UL        // window length at 33 1/3 rpm
#define TRAKTOR_LOOP_START_SECONDS 0UL    // skip into the sequence first (raise
                                          // if the rip started too near lead-in)
#define TRAKTOR_LOOP_BITS (TRAKTOR_LOOP_SECONDS * 1000UL)
#define TRAKTOR_LOOP_START_BITS (TRAKTOR_LOOP_START_SECONDS * 1000UL)
#define TRAKTOR_PACKED_BYTES ((TRAKTOR_LOOP_BITS + 7) / 8)
// Bit boundaries sit at (carrier phase mod 2.5) ~ 0.089 cycles in the rip;
// as a data-phase offset that is 0.089/2.5 of a bit.
#define TRAKTOR_BIT_LOCK_OFFSET_BITS 0.0356f
// Rip measured data ~0.29x carrier after the phono/RIAA chain.
#define CARRIER_LEVEL 0.55f
#define DATA_LEVEL    0.165f
// One-pole lowpass on the data edges. 1.0 = hard one-sample edges: looks
// noisier on Traktor's scope/spectrum but measures HIGHER on its timecode
// quality meter (tested; 0.09 ~ vinyl-like 0.6 ms slew scored lower).
#define DATA_EDGE_SMOOTHING 1.0f
// Set to 1 if Traktor tracks speed but not position (flips NRZ polarity).
#define DATA_INVERT 0

// Known-answer test: first 8 packed table bytes, computed offline from the
// decoded rip (128/128 bits verified).
static const uint8_t traktorSelfTest[8] =
  { 0xA2, 0xD7, 0xB0, 0xF9, 0x87, 0x16, 0xF5, 0x65 };

// 32 fractional phase bits => frequency granularity 44100/2^32 ~ 1e-5 Hz.
#define PHASE_FRAC_BITS 32
#define PHASE_FRAC_ONE  4294967296.0f   // 2^32

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint8_t  version;
  uint8_t  deckId;
  int16_t  rpmCenti;
  int16_t  gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

typedef struct {
  bool     assigned;        // slot has an owner MAC (sticky for the session)
  bool     seen;
  int16_t  rpmCenti;
  uint32_t lastSeq;
  uint32_t lastSeenMillis;
  uint32_t lastPingMillis;
  uint32_t packetCount;
  uint16_t lostPackets;
  uint8_t  mac[6];          // owner MAC, set at assignment, matched thereafter
} deck_state;

typedef struct {
  uint8_t     deckId;
  i2s_port_t  port;
  int         bckPin, lrckPin, dataPin;
  const char *taskName;
  float       filteredRpm;
  float       dataLevel;         // lowpassed NRZ level (edge shaping)
  int64_t     carrierPhase;      // 1.0 (<<32) = one carrier cycle
  int64_t     dataPhase;         // 1.0 (<<32) = one data bit
} audio_deck_state;

deck_state deckStates[2];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Unknown pucks may claim a free deck only while now < this (absolute millis).
volatile uint32_t pairingWindowEndMs = 0;

static inline bool macEq(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

volatile bool hasPendingWelcome = false;
uint8_t pendingWelcomeMac[6];
uint8_t pendingWelcomeDeck = 0;

volatile bool hasPendingEvent = false;
uint8_t  pendingEventDeck = 0;
uint8_t  pendingEventCode = 0;
uint32_t pendingEventValue = 0;

volatile bool     dicerSeen = false;
volatile uint16_t dicerMask = 0;
volatile uint8_t  dicerMode = 0;
uint16_t dicerPrevMask = 0;
uint8_t  dicerActiveNote[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t *traktorPackedBits = NULL;   // 512 KB in PSRAM

uint32_t lastLedMillis = 0;
uint32_t lastLedPacked = 0xFFFFFFFFUL;
uint32_t deckBattLowUntil[2] = { 0, 0 };
uint32_t lastDebugPrintMillis = 0;

// Failover state: written only by each deck's own audio task; read by
// loop() for logging (transient races there are harmless).
float    failStableRpm[2]   = { 0.0f, 0.0f };
float    failCandRpm[2]     = { 0.0f, 0.0f };
uint32_t failCandSinceMs[2] = { 0, 0 };
volatile bool failHolding[2] = { false, false };

audio_deck_state audioDecks[2] = {
  { 1, I2S_NUM_0, DAC_A_BCK_PIN, DAC_A_LRCK_PIN, DAC_A_DATA_PIN, "audioDeckA",
    0.0f, 0.0f, 0, -(int64_t)(TRAKTOR_BIT_LOCK_OFFSET_BITS * PHASE_FRAC_ONE) },
  { 2, I2S_NUM_1, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckB",
    0.0f, 0.0f, 0, -(int64_t)(TRAKTOR_BIT_LOCK_OFFSET_BITS * PHASE_FRAC_ONE) }
};

// ===== Serial debug ==================================================
void setupDebugSerial() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("DVS dual I2S TRAKTOR MK2 receiver boot");
  Serial.printf("Debug baud: %lu\n", (unsigned long)DEBUG_BAUD);
  Serial.printf("ESP-NOW channel: %u\n", ESPNOW_CHANNEL);
#endif
}

void debugBootLog(const char *message) {
#if DEBUG_SERIAL
  Serial.println(message);
#else
  (void)message;
#endif
}

// Periodic per-deck status. Called from loop(), outside the ESP-NOW
// callback, so it never disturbs radio reception or the audio tasks.
void debugPrintStatus() {
#if DEBUG_SERIAL
  uint32_t nowMillis = millis();
  if (nowMillis - lastDebugPrintMillis < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMillis = nowMillis;

  deck_state copy[2];
  portENTER_CRITICAL(&stateMux);
  memcpy(copy, deckStates, sizeof(copy));
  portEXIT_CRITICAL(&stateMux);

  for (uint8_t i = 0; i < 2; i++) {
    bool online = copy[i].lastSeenMillis != 0 &&
                  nowMillis - copy[i].lastSeenMillis <= DECK_TIMEOUT_MS;
    float rpm = (float)copy[i].rpmCenti / 100.0f;
    Serial.printf("D%u %s rpm=%7.2f seq=%lu rx=%lu lost=%u age=%lums  ",
                  i + 1,
                  online ? "ON " : (failHolding[i] ? "HLD" : "OFF"),
                  rpm,
                  (unsigned long)copy[i].lastSeq,
                  (unsigned long)copy[i].packetCount,
                  copy[i].lostPackets,
                  copy[i].lastSeenMillis == 0 ? 0UL
                      : (unsigned long)(nowMillis - copy[i].lastSeenMillis));
  }
  Serial.printf("pair=%s heap=%lu\n",
                nowMillis < pairingWindowEndMs ? "OPEN" : "closed",
                (unsigned long)ESP.getFreeHeap());
#endif
}

// Handle transmitter status events from loop(), never from the radio
// callback. Battery LOW/CRITICAL is latched (with expiry) for the LED
// even when DEBUG_SERIAL is 0; printing is debug-only.
void serviceTransmitterEvents() {
  uint8_t deck = 0, code = 0; uint32_t val = 0; bool has = false;
  portENTER_CRITICAL(&stateMux);
  if (hasPendingEvent) {
    deck = pendingEventDeck; code = pendingEventCode; val = pendingEventValue;
    hasPendingEvent = false; has = true;
  }
  portEXIT_CRITICAL(&stateMux);
  if (!has) return;

  if (deck >= 1 && deck <= 2) {
    if (code == 6 || code == 7)      deckBattLowUntil[deck - 1] = millis() + BATT_LOW_HOLD_MS;
    else if (code == 8)              deckBattLowUntil[deck - 1] = 0;
  }

#if DEBUG_SERIAL
  float v = (float)val / 1000000.0f;
  switch (code) {
    case 1: Serial.printf("EVT deck %u: using stored trim=%.5f\n", deck, v); break;
    case 2: Serial.printf("EVT deck %u: NO stored trim - spin platter at 0%% pitch to calibrate\n", deck); break;
    case 3: Serial.printf("EVT deck %u: spin-cal reference %.2f settled, sampling ~10 s - don't touch\n", deck, v); break;
    case 4: Serial.printf("EVT deck %u: spin-cal LOCKED, trim=%.5f (saved)\n", deck, v); break;
    case 5: Serial.printf("EVT deck %u: battery %.2f V\n", deck, v); break;
    case 6: Serial.printf("EVT deck %u: battery LOW %.2f V - charge soon\n", deck, v); break;
    case 7: Serial.printf("EVT deck %u: battery CRITICAL %.2f V - charge now\n", deck, v); break;
    case 8: Serial.printf("EVT deck %u: battery recovered (%.2f V)\n", deck, v); break;
    default: Serial.printf("EVT deck %u: code %u value %.5f\n", deck, code, v); break;
  }
#endif
}

// Announce failover transitions from loop() (audio tasks must not print).
void serviceFailoverLog() {
#if DEBUG_SERIAL && FAILOVER_ENABLE
  static bool prev[2] = { false, false };
  for (uint8_t i = 0; i < 2; i++) {
    bool h = failHolding[i];
    if (h != prev[i]) {
      prev[i] = h;
      if (h) Serial.printf("FAILOVER deck %u: link lost - holding %.2f rpm\n",
                           i + 1, failStableRpm[i]);
      else   Serial.printf("FAILOVER deck %u: link restored - live rpm resumed\n", i + 1);
    }
  }
#endif
}

// Diff the dicer's button state into MIDI notes. State-based: a lost
// packet just delays the edge, it can never stick a note. Note-off uses
// the note that was sent at press time, so mode changes mid-hold stay
// consistent.
void serviceDicerMidi() {
  uint16_t mask; uint8_t mode;
  portENTER_CRITICAL(&stateMux);
  bool seen = dicerSeen;
  mask = dicerMask; mode = dicerMode;
  portEXIT_CRITICAL(&stateMux);
  if (!seen) return;
  uint16_t changed = mask ^ dicerPrevMask;
  if (!changed) return;
  for (uint8_t i = 0; i < 6; i++) {
    if (!(changed & (1u << i))) continue;
    if (mask & (1u << i)) {
      uint8_t note = MIDI_BASE_NOTE + mode * 6 + i;
      dicerActiveNote[i] = note;
#if USB_MIDI_ACTIVE
      MIDI.noteOn(note, 127, MIDI_CHANNEL);
#endif
#if DEBUG_SERIAL
      Serial.printf("DICER: mode %u btn %u DOWN -> note %u on\n", mode + 1, i + 1, note);
#endif
    } else if (dicerActiveNote[i]) {
#if USB_MIDI_ACTIVE
      MIDI.noteOff(dicerActiveNote[i], 0, MIDI_CHANNEL);
#endif
#if DEBUG_SERIAL
      Serial.printf("DICER: btn %u up -> note %u off\n", i + 1, dicerActiveNote[i]);
#endif
      dicerActiveNote[i] = 0;
    }
  }
  dicerPrevMask = mask;
}

// ===== RGB status LED =================================================
// Color = deck count (red none / orange one / green both). A low battery
// modulates it: deck 1 low = slow on/off blink, deck 2 low = two fast
// pulses per cycle, both low = three fast pulses per cycle.
static bool ledBlinkPhase(uint32_t now, bool low1, bool low2) {
  if (!low1 && !low2) return true;                 // solid
  if (low1 && !low2)  return (now % 1000) < 500;   // slow blink
  int pulses = (low1 && low2) ? 3 : 2;             // fast pulse burst
  uint32_t t = now % 1200;
  return t < (uint32_t)(pulses * 280) && (t % 280) < 140;
}

void serviceLed() {
  uint32_t now = millis();
  int onlineCount = onlineDeckCount();
  bool pairingOpen = now < pairingWindowEndMs;
  bool low1 = deckBattLowUntil[0] != 0 && now < deckBattLowUntil[0];
  bool low2 = deckBattLowUntil[1] != 0 && now < deckBattLowUntil[1];
  uint8_t r, g, b;
  if      (onlineCount >= 2) { r = 0;   g = 255; b = 0; }   // green
  else if (onlineCount == 1) { r = 255; g = 80;  b = 0; }   // orange
  else if (pairingOpen)      { r = 0;   g = 0;   b = 255; } // blue: waiting to pair
  else                       { r = 255; g = 0;   b = 0; }   // red
  bool on = ledBlinkPhase(now, low1, low2);
  if (pairingOpen) on = on && ((now % 800) < 400);          // blink while pairing
  if (!on) { r = 0; g = 0; b = 0; }
  uint32_t packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (packed == lastLedPacked) return;             // refresh only on change
  lastLedPacked = packed;
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// Fatal boot error: blink red forever, keep printing the message.
void haltWithError(const char *message) {
  while (true) {
#if DEBUG_SERIAL
    Serial.println(message);
#endif
    led.setPixelColor(0, led.Color(255, 0, 0)); led.show(); delay(300);
    led.setPixelColor(0, 0); led.show(); delay(300);
  }
}

// ===== Traktor MK2 bit sequence ======================================
static inline void setPackedBit(uint32_t i, uint8_t v) {
  uint32_t b = i >> 3; uint8_t m = (uint8_t)(1U << (i & 7));
  if (v) traktorPackedBits[b] |= m; else traktorPackedBits[b] &= (uint8_t)~m;
}
static inline uint8_t getPackedBit(uint32_t i) {
  return (traktorPackedBits[i >> 3] & (uint8_t)(1U << (i & 7))) != 0;
}

void buildTraktorBits() {
  // ~37.5 KB at the default 5-min loop: PSRAM if present, else heap.
  traktorPackedBits = (uint8_t *)(psramFound()
      ? ps_calloc(TRAKTOR_PACKED_BYTES, 1)
      : calloc(TRAKTOR_PACKED_BYTES, 1));
  if (!traktorPackedBits)
    haltWithError("ERR bit_table_alloc - loop too long for available RAM");

  // Run the LFSR from the rip's seed, keep only the loop window.
  // Sequence bits 0..22 are the seed itself (oldest at MSB); the state
  // register's LSB is the newest bit.
  uint32_t state = TRAKTOR_SEED;
  uint32_t stored = 0;
  for (uint32_t i = 0; stored < TRAKTOR_LOOP_BITS; i++) {
    uint8_t bit;
    if (i < 23) {
      bit = (uint8_t)((TRAKTOR_SEED >> (22 - i)) & 1U);
    } else {
      uint32_t nb = __builtin_parity(state & TRAKTOR_TAPS_MASK);
      state = ((state << 1) | nb) & 0x7FFFFFUL;
      bit = (uint8_t)nb;
    }
    if (i >= TRAKTOR_LOOP_START_BITS) setPackedBit(stored++, bit);
  }

#if TRAKTOR_LOOP_START_BITS == 0
  if (memcmp(traktorPackedBits, traktorSelfTest, sizeof(traktorSelfTest)) != 0)
    haltWithError("ERR traktor_selftest - generated sequence is wrong");
#endif
}

// ===== ESP-NOW =======================================================
bool ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6); p.channel = ESPNOW_CHANNEL; p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

void sendControlMessage(const uint8_t *mac, uint8_t deckId, uint8_t msgType) {
  if (!ensurePeer(mac)) return;
  dvs_packet r = {};
  r.msgType = msgType; r.version = PROTOCOL_VERSION; r.deckId = deckId;
  r.timestampMicros = micros();
  esp_now_send(mac, (uint8_t *)&r, sizeof(r));
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet packet; memcpy(&packet, data, sizeof(packet));
  // Transmitters now send deckId 0 (unassigned) and are routed by MAC; the
  // experimental dicer still identifies itself as deckId 3.
  if (packet.version != PROTOCOL_VERSION || packet.deckId > 3) return;

  const uint8_t *src = info->src_addr;

  // Dicer (unit 3): no deck_state - HELLO and EVENT reuse the shared
  // pending slots, button state goes to its own.
  if (packet.deckId == DICER_ID) {
    if (packet.msgType == MSG_HELLO) {
      portENTER_CRITICAL_ISR(&stateMux);
      memcpy(pendingWelcomeMac, src, 6);
      pendingWelcomeDeck = packet.deckId; hasPendingWelcome = true;
      portEXIT_CRITICAL_ISR(&stateMux);
    } else if (packet.msgType == MSG_BUTTONS) {
      portENTER_CRITICAL_ISR(&stateMux);
      dicerMask = (uint16_t)packet.rpmCenti;
      dicerMode = (uint8_t)packet.gyroRaw;
      dicerSeen = true;
      portEXIT_CRITICAL_ISR(&stateMux);
    } else if (packet.msgType == MSG_EVENT) {
      portENTER_CRITICAL_ISR(&stateMux);
      pendingEventDeck = packet.deckId;
      pendingEventCode = (uint8_t)packet.rpmCenti;
      pendingEventValue = packet.timestampMicros;
      hasPendingEvent = true;
      portEXIT_CRITICAL_ISR(&stateMux);
    }
    return;
  }

  // ---- Transmitter: find this MAC's deck, or assign a free one -------
  int slot = -1;
  portENTER_CRITICAL_ISR(&stateMux);
  for (uint8_t i = 0; i < 2; i++)
    if (deckStates[i].assigned && macEq(deckStates[i].mac, src)) { slot = (int)i; break; }
  if (slot < 0 && millis() < pairingWindowEndMs) {
    for (uint8_t i = 0; i < 2; i++)
      if (!deckStates[i].assigned) {
        deckStates[i].assigned = true;
        memcpy(deckStates[i].mac, src, 6);
        deckStates[i].seen = false;
        deckStates[i].lastSeq = 0;
        deckStates[i].lastPingMillis = 0;
        deckStates[i].packetCount = 0;
        slot = (int)i;
        break;
      }
  }
  if (slot >= 0) deckStates[slot].lastSeenMillis = millis();
  portEXIT_CRITICAL_ISR(&stateMux);
  if (slot < 0) return;    // unknown MAC, window closed or both decks taken

  deck_state *state = &deckStates[slot];
  uint8_t deckNum = (uint8_t)slot + 1;

  if (packet.msgType == MSG_HELLO) {
    portENTER_CRITICAL_ISR(&stateMux);
    memcpy(pendingWelcomeMac, src, 6);
    pendingWelcomeDeck = deckNum; hasPendingWelcome = true;
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }
  if (packet.msgType == MSG_EVENT) {
    portENTER_CRITICAL_ISR(&stateMux);
    pendingEventDeck = deckNum;
    pendingEventCode = (uint8_t)packet.rpmCenti;
    pendingEventValue = packet.timestampMicros;
    hasPendingEvent = true;
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }
  if (packet.msgType != MSG_DATA) return;

  uint16_t lost = 0;
  portENTER_CRITICAL_ISR(&stateMux);
  if (state->seen) { uint32_t d = packet.seq - state->lastSeq; if (d > 0) lost = (uint16_t)min(d - 1, 65535UL); }
  state->seen = true;
  state->rpmCenti = packet.rpmCenti;
  state->lastSeq = packet.seq;
  state->packetCount++;
  state->lostPackets = lost;
  portEXIT_CRITICAL_ISR(&stateMux);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if DEBUG_SERIAL
  // Paste this into the transmitter's receiverMAC[] to pair it.
  // esp_read_mac reads the factory STA MAC from eFuse - unlike
  // WiFi.macAddress() it works before the WiFi driver has started.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("  transmitter: uint8_t receiverMAC[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) haltWithError("ERR espnow_init");
  esp_now_register_recv_cb(OnDataRecv);
}

// ===== I2S + audio ===================================================
void setupI2S(audio_deck_state *deck) {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = { .bck_io_num = deck->bckPin, .ws_io_num = deck->lrckPin,
                            .data_out_num = deck->dataPin, .data_in_num = I2S_PIN_NO_CHANGE };
  i2s_driver_install(deck->port, &config, 0, NULL);
  i2s_set_pin(deck->port, &pins);
  i2s_zero_dma_buffer(deck->port);

#if DEBUG_SERIAL
  Serial.printf("I2S deck %u OK: port=%d BCK=%d LRCK=%d DATA=%d\n",
                deck->deckId, (int)deck->port,
                deck->bckPin, deck->lrckPin, deck->dataPin);
#endif
}

// rpm -> per-sample phase steps for carrier and data, computed once per
// DMA buffer. Data step = carrier step * 0.4 keeps the bitstream phase-
// locked to the carrier at every speed, exactly like the vinyl.
static void rpmToPhaseSteps(float rpm, int64_t *carrierStep, int64_t *dataStep) {
  if (fabsf(rpm) < DEADZONE_RPM) { *carrierStep = 0; *dataStep = 0; return; }
  float ratio = rpm / BASE_RPM;
  if (ratio >  MAX_RPM_RATIO) ratio =  MAX_RPM_RATIO;
  if (ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  float cyclesPerSample = ratio * (TRAKTOR_CARRIER_HZ / SAMPLE_RATE);
  *carrierStep = (int64_t)llroundf(cyclesPerSample * PHASE_FRAC_ONE);
  *dataStep    = (int64_t)llroundf(cyclesPerSample * TRAKTOR_BITS_PER_CYCLE * PHASE_FRAC_ONE);
}

// Quadrature carrier (L = -cos, R = sin; R leads L forward, matching the
// rip) plus the NRZ position bitstream added equally to both channels.
static inline void renderTraktorSample(audio_deck_state *deck,
                                       int64_t carrierStep, int64_t dataStep,
                                       int16_t *leftOut, int16_t *rightOut) {
  deck->carrierPhase += carrierStep;
  deck->dataPhase    += dataStep;

  float frac = (float)(uint32_t)deck->carrierPhase * (1.0f / PHASE_FRAC_ONE);
  float angle = frac * 6.28318530718f;
  float sine = sinf(angle), cosine = cosf(angle);

  int64_t bitCyc = deck->dataPhase >> PHASE_FRAC_BITS;
  uint32_t bitIndex = (uint32_t)(((bitCyc % (int64_t)TRAKTOR_LOOP_BITS)
                     + (int64_t)TRAKTOR_LOOP_BITS) % (int64_t)TRAKTOR_LOOP_BITS);
#if DATA_INVERT
  float target = getPackedBit(bitIndex) ? -DATA_LEVEL : DATA_LEVEL;
#else
  float target = getPackedBit(bitIndex) ? DATA_LEVEL : -DATA_LEVEL;
#endif
  deck->dataLevel += (target - deck->dataLevel) * DATA_EDGE_SMOOTHING;

  *leftOut  = (int16_t)((-cosine * CARRIER_LEVEL + deck->dataLevel) * 32767.0f);
  *rightOut = (int16_t)(( sine   * CARRIER_LEVEL + deck->dataLevel) * 32767.0f);
}

float readTargetRpm(uint8_t deckId) {
  uint8_t i = deckId <= 1 ? 0 : 1;
  int16_t rpmCenti; uint32_t lastSeen;
  portENTER_CRITICAL(&stateMux);
  rpmCenti = deckStates[i].rpmCenti; lastSeen = deckStates[i].lastSeenMillis;
  portEXIT_CRITICAL(&stateMux);
  uint32_t now = millis();
  bool online = lastSeen != 0 && now - lastSeen <= DECK_TIMEOUT_MS;
#if FAILOVER_ENABLE
  if (online) {
    failHolding[i] = false;
    float rpm = (float)rpmCenti / 100.0f;
    if (fabsf(rpm - failCandRpm[i]) > FAILOVER_STABLE_BAND_RPM) {
      failCandRpm[i] = rpm; failCandSinceMs[i] = now;  // new stability window
    } else if (now - failCandSinceMs[i] >= FAILOVER_STABLE_MS) {
      failStableRpm[i] = rpm;             // steady long enough -> hold value
    }
    return rpm;
  }
  if (lastSeen != 0) failHolding[i] = true;  // was alive -> hold, don't stop
  return failHolding[i] ? failStableRpm[i] : 0.0f;
#else
  if (!online) return 0.0f;
  return (float)rpmCenti / 100.0f;
#endif
}

void audioTask(void *param) {
  audio_deck_state *deck = (audio_deck_state *)param;
  int16_t buffer[DMA_BUF_LEN * 2];
  const int64_t dataSpan = (int64_t)TRAKTOR_LOOP_BITS << PHASE_FRAC_BITS;
  while (true) {
    float target = readTargetRpm(deck->deckId);
    deck->filteredRpm += (target - deck->filteredRpm) * RPM_SMOOTHING;
    int64_t carrierStep, dataStep;
    rpmToPhaseSteps(deck->filteredRpm, &carrierStep, &dataStep);
    deck->dataPhase %= dataSpan;       // fold so the accumulators never overflow
    deck->carrierPhase &= 0xFFFFFFFFLL;  // only the fractional cycle matters
    for (int i = 0; i < DMA_BUF_LEN; i++) {
      int16_t l = 0, r = 0;
      renderTraktorSample(deck, carrierStep, dataStep, &l, &r);
      buffer[i * 2] = l; buffer[i * 2 + 1] = r;
    }
    size_t written = 0;
    i2s_write(deck->port, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}

void serviceEspNowControl() {
  uint8_t mac[6]; uint8_t deck = 0; bool welcome = false;
  portENTER_CRITICAL(&stateMux);
  if (hasPendingWelcome) { memcpy(mac, pendingWelcomeMac, 6); deck = pendingWelcomeDeck; hasPendingWelcome = false; welcome = true; }
  portEXIT_CRITICAL(&stateMux);
  if (welcome) {
    sendControlMessage(mac, deck, MSG_WELCOME);
#if DEBUG_SERIAL
    Serial.printf("WELCOME deck %u -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                  deck, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  }

  uint32_t now = millis();
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t m[6]; bool ping = false;
    portENTER_CRITICAL(&stateMux);
    deck_state *s = &deckStates[i];
    if (s->assigned && s->lastSeenMillis && now - s->lastSeenMillis <= DECK_TIMEOUT_MS && now - s->lastPingMillis >= PING_INTERVAL_MS) {
      memcpy(m, s->mac, 6); s->lastPingMillis = now; ping = true;
    }
    portEXIT_CRITICAL(&stateMux);
    if (ping) sendControlMessage(m, i + 1, MSG_PING);
  }
}

// Long-press the re-pair button to drop both pucks and reopen the pairing
// window, so a replacement puck can be slotted without a power cycle.
void serviceRepairButton() {
  static uint32_t pressStart = 0;
  static bool fired = false;
  if (digitalRead(REPAIR_BTN_PIN) != LOW) { pressStart = 0; fired = false; return; }
  uint32_t now = millis();
  if (pressStart == 0) { pressStart = now; return; }
  if (fired || now - pressStart < REPAIR_HOLD_MS) return;
  fired = true;
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++) {
    deckStates[i].assigned = false;
    deckStates[i].seen = false;
    deckStates[i].lastSeenMillis = 0;
    memset(deckStates[i].mac, 0, 6);
  }
  pairingWindowEndMs = now + PAIRING_WINDOW_MS;
  portEXIT_CRITICAL(&stateMux);
#if FAILOVER_ENABLE
  failHolding[0] = failHolding[1] = false;   // stop any held-rpm output
#endif
  debugBootLog("RE-PAIR: pairings cleared, pairing window reopened");
}

int onlineDeckCount() {
  int n = 0; uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++)
    if (deckStates[i].lastSeenMillis && now - deckStates[i].lastSeenMillis <= DECK_TIMEOUT_MS) n++;
  portEXIT_CRITICAL(&stateMux);
  return n;
}

// ===== setup / loop ==================================================
void setup() {
  setupDebugSerial();
  pinMode(REPAIR_BTN_PIN, INPUT_PULLUP);
  led.begin();
  led.setBrightness(LED_BRIGHTNESS);
  serviceLed();                       // start red (no transmitters)

#if DEBUG_SERIAL
  Serial.printf("Building Traktor MK2 bit table (%lu s loop, %lu bytes)...\n",
                (unsigned long)TRAKTOR_LOOP_SECONDS,
                (unsigned long)TRAKTOR_PACKED_BYTES);
#endif
  uint32_t t0 = millis();
  buildTraktorBits();
#if DEBUG_SERIAL
  Serial.printf("Traktor bit table OK (self-test passed, %lu ms)\n",
                (unsigned long)(millis() - t0));
#endif

  debugBootLog("Starting I2S outputs...");
  setupI2S(&audioDecks[0]);
  setupI2S(&audioDecks[1]);
  debugBootLog("I2S outputs OK");

  debugBootLog("Starting ESP-NOW...");
  setupEspNow();
  pairingWindowEndMs = millis() + PAIRING_WINDOW_MS;   // open the pairing window
  debugBootLog("ESP-NOW OK - pairing window open (power the pucks now)");

  xTaskCreatePinnedToCore(audioTask, audioDecks[0].taskName, 8192, &audioDecks[0], 3, NULL, 1);
  xTaskCreatePinnedToCore(audioTask, audioDecks[1].taskName, 8192, &audioDecks[1], 3, NULL, 1);
  debugBootLog("Audio tasks OK");

#if USB_MIDI_ACTIVE
  MIDI.begin();
  USB.begin();
  debugBootLog("USB-MIDI up (native USB port)");
#endif
}

void loop() {
  serviceRepairButton();
  serviceEspNowControl();
  serviceTransmitterEvents();
  serviceFailoverLog();
  serviceDicerMidi();
  if (millis() - lastLedMillis >= LED_UPDATE_MS) {
    lastLedMillis = millis();
    serviceLed();
  }
  debugPrintStatus();
  delay(1);
}
