/*
  ESP32-S3 DVS RECEIVER - DUAL CV02 I2S DAC
  - Generates Serato-CV02 timecode for two decks in real time, speed and
    direction driven by each transmitter's RPM over ESP-NOW.
  - Onboard RGB LED status: RED = no transmitters, ORANGE = one, GREEN = both.

  No USB app/serial protocol. A one-way debug serial (DEBUG_SERIAL, any USB
  mode works) prints a boot banner plus periodic per-deck link status
  (rpm / seq / rx / lost / age) and free heap. Set DEBUG_SERIAL 0 to silence.
  Requires the Adafruit NeoPixel library.
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
// Set DEBUG_SERIAL 0 to silence all prints. It only reports whether the
// firmware booted and whether packets are arriving.
#define DEBUG_SERIAL 1
#define DEBUG_BAUD 115200
#define DEBUG_PRINT_INTERVAL_MS 500

// ===== Audio =========================================================
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 32            // small buffers = low output latency
#define DMA_BUF_COUNT 4

// ===== Timecode response ==============================================
#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.04f
#define MAX_RPM_RATIO 8.0f        // allow fast backspins
#define RPM_SMOOTHING 0.80f
// PCM5102A full scale is ~2.1 V RMS - far above consumer line level, and a
// massive overload for phono inputs (which also apply RIAA EQ; never feed
// them without an attenuator). 0.30 lands near normal line level; tune so
// Serato's calibration scope shows a round ring comfortably inside the
// view, then re-run Estimate on the noise threshold.
#define OUTPUT_GAIN 0.30f

// ===== CV02 generator (Serato/Bridge-compatible) ======================
#define CV02_RESOLUTION 1000
#define CV02_BITS 20
#define CV02_SEED 0x59017UL
#define CV02_TAPS 0x361e4UL
#define CV02_LENGTH 712000UL
#define CV02_PACKED_BYTES ((CV02_LENGTH + 7) / 8)

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
  bool     seen;
  int16_t  rpmCenti;
  uint32_t lastSeq;
  uint32_t lastSeenMillis;
  uint32_t lastPingMillis;
  uint32_t packetCount;
  uint16_t lostPackets;
  uint8_t  mac[6];
} deck_state;

typedef struct {
  uint8_t     deckId;
  i2s_port_t  port;
  int         bckPin, lrckPin, dataPin;
  const char *taskName;
  float       filteredRpm;
  int64_t     cv02Phase;         // 64-bit phase => clean reverse + full sequence
} audio_deck_state;

deck_state deckStates[2];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

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

uint8_t cv02PackedBits[CV02_PACKED_BYTES];

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
  { 1, I2S_NUM_0, DAC_A_BCK_PIN, DAC_A_LRCK_PIN, DAC_A_DATA_PIN, "audioDeckA", 0.0f, 0 },
  { 2, I2S_NUM_1, DAC_B_BCK_PIN, DAC_B_LRCK_PIN, DAC_B_DATA_PIN, "audioDeckB", 0.0f, 0 }
};

// ===== Serial debug ==================================================
// Brings up the debug serial with a boot banner. Not used to transport
// timecode; only to confirm the firmware started and packets arrive.
void setupDebugSerial() {
#if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(1000);
  Serial.println();
  Serial.println("DVS dual I2S CV02 receiver boot");
  Serial.printf("Debug baud: %lu\n", (unsigned long)DEBUG_BAUD);
  Serial.printf("ESP-NOW channel: %u\n", ESPNOW_CHANNEL);
#endif
}

// Simple boot log line, guarded so it compiles out when DEBUG_SERIAL 0.
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
  Serial.printf("heap=%lu\n", (unsigned long)ESP.getFreeHeap());
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
  bool low1 = deckBattLowUntil[0] != 0 && now < deckBattLowUntil[0];
  bool low2 = deckBattLowUntil[1] != 0 && now < deckBattLowUntil[1];
  uint8_t r, g, b;
  if      (onlineCount >= 2) { r = 0;   g = 255; b = 0; }   // green
  else if (onlineCount == 1) { r = 255; g = 80;  b = 0; }   // orange
  else                       { r = 255; g = 0;   b = 0; }   // red
  if (!ledBlinkPhase(now, low1, low2)) { r = 0; g = 0; b = 0; }
  uint32_t packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (packed == lastLedPacked) return;             // refresh only on change
  lastLedPacked = packed;
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ===== CV02 sequence =================================================
static uint8_t lfsrBit(uint32_t code, uint32_t taps) {
  uint32_t t = code & taps; uint8_t p = 0;
  while (t) { p ^= (uint8_t)(t & 1U); t >>= 1; }
  return p;
}
static uint32_t lfsrForward(uint32_t cur) {
  uint8_t bit = lfsrBit(cur, CV02_TAPS | 0x1U);
  return (cur >> 1) | ((uint32_t)bit << (CV02_BITS - 1));
}
static void setPackedBit(uint32_t i, uint8_t v) {
  uint32_t b = i >> 3; uint8_t m = (uint8_t)(1U << (i & 7));
  if (v) cv02PackedBits[b] |= m; else cv02PackedBits[b] &= (uint8_t)~m;
}
static inline uint8_t getPackedBit(uint32_t i) {
  return (cv02PackedBits[i >> 3] & (uint8_t)(1U << (i & 7))) != 0;
}
void buildCv02Bits() {
  memset(cv02PackedBits, 0, sizeof(cv02PackedBits));
  uint32_t code = CV02_SEED;
  for (uint32_t i = 0; i < CV02_LENGTH; i++) { setPackedBit(i, (uint8_t)(code & 1U)); code = lfsrForward(code); }
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
  if (packet.version != PROTOCOL_VERSION || packet.deckId < 1 || packet.deckId > 3) return;

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

  deck_state *state = &deckStates[packet.deckId - 1];

  portENTER_CRITICAL_ISR(&stateMux);
  memcpy(state->mac, src, 6);
  state->lastSeenMillis = millis();
  portEXIT_CRITICAL_ISR(&stateMux);

  if (packet.msgType == MSG_HELLO) {
    portENTER_CRITICAL_ISR(&stateMux);
    memcpy(pendingWelcomeMac, src, 6);
    pendingWelcomeDeck = packet.deckId; hasPendingWelcome = true;
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }
  if (packet.msgType == MSG_EVENT) {
    portENTER_CRITICAL_ISR(&stateMux);
    pendingEventDeck = packet.deckId;
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
  if (esp_now_init() != ESP_OK) while (true) delay(1000);
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

// 32 fractional phase bits => frequency granularity 44100/2^32 ~ 1e-5 Hz.
// (16 bits quantized pitch in ~0.07% steps, and truncation biased it low.)
#define CV02_FRAC_BITS 32
#define CV02_FRAC_ONE  4294967296.0f   // 2^32

// rpm -> per-sample phase step. Called once per DMA buffer (rpm only
// changes per buffer, so per-sample recomputation was pure overhead).
// Deadzone compares RPM: comparing the *ratio* against DEADZONE_RPM zeroed
// everything below 0.04*33.3 = 1.3 RPM and killed slow crawls; the
// transmitter's own 0.2 RPM deadzone is the intended floor.
static int64_t rpmToPhaseStep(float rpm) {
  if (fabsf(rpm) < DEADZONE_RPM) return 0;
  float ratio = rpm / BASE_RPM;
  if (ratio >  MAX_RPM_RATIO) ratio =  MAX_RPM_RATIO;
  if (ratio < -MAX_RPM_RATIO) ratio = -MAX_RPM_RATIO;
  return (int64_t)llroundf(ratio * ((float)CV02_RESOLUTION * (CV02_FRAC_ONE / SAMPLE_RATE)));
}

// L = -cos, R = sin (quadrature); bit stream amplitude-modulates the pair.
// int64 phase => glitch-free reverse/backspin and full sequence.
static inline void renderCv02Sample(audio_deck_state *deck, int64_t phaseStep,
                                    int16_t *leftOut, int16_t *rightOut) {
  deck->cv02Phase += phaseStep;

  int64_t cyc = deck->cv02Phase >> CV02_FRAC_BITS;
  uint32_t cycleIndex = (uint32_t)(((cyc % (int64_t)CV02_LENGTH) + (int64_t)CV02_LENGTH) % (int64_t)CV02_LENGTH);
  float frac = (float)(uint32_t)deck->cv02Phase * (1.0f / CV02_FRAC_ONE);

  float angle = frac * 6.28318530718f;
  float sine = sinf(angle), cosine = cosf(angle);
  uint8_t bit = getPackedBit(cycleIndex);
  float modulation = bit ? 1.0f : 1.0f - ((-cosine + 1.0f) * 0.25f);

  *leftOut  = (int16_t)((-cosine * modulation * OUTPUT_GAIN) * 32767.0f);
  *rightOut = (int16_t)(( sine   * modulation * OUTPUT_GAIN) * 32767.0f);
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
  const int64_t phaseSpan = (int64_t)CV02_LENGTH << CV02_FRAC_BITS;
  while (true) {
    float target = readTargetRpm(deck->deckId);
    deck->filteredRpm += (target - deck->filteredRpm) * RPM_SMOOTHING;
    int64_t phaseStep = rpmToPhaseStep(deck->filteredRpm);
    deck->cv02Phase %= phaseSpan;      // fold so the accumulator never overflows
    for (int i = 0; i < DMA_BUF_LEN; i++) {
      int16_t l = 0, r = 0;
      renderCv02Sample(deck, phaseStep, &l, &r);
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
    if (s->lastSeenMillis && now - s->lastSeenMillis <= DECK_TIMEOUT_MS && now - s->lastPingMillis >= PING_INTERVAL_MS) {
      memcpy(m, s->mac, 6); s->lastPingMillis = now; ping = true;
    }
    portEXIT_CRITICAL(&stateMux);
    if (ping) sendControlMessage(m, i + 1, MSG_PING);
  }
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
  led.begin();
  led.setBrightness(LED_BRIGHTNESS);
  serviceLed();                       // start red (no transmitters)

  debugBootLog("Building CV02 bit table...");
  buildCv02Bits();
  debugBootLog("CV02 bit table OK");

  debugBootLog("Starting I2S outputs...");
  setupI2S(&audioDecks[0]);
  setupI2S(&audioDecks[1]);
  debugBootLog("I2S outputs OK");

  debugBootLog("Starting ESP-NOW...");
  setupEspNow();
  debugBootLog("ESP-NOW OK");

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
