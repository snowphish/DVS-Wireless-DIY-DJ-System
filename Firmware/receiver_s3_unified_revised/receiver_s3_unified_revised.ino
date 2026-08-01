/*
  ESP32-S3 DVS RECEIVER - UNIFIED (Serato CV02 + Traktor) + SETTINGS PORTAL
  - Generates timecode for two decks in real time, speed and direction driven
    by each transmitter's RPM over ESP-NOW. The FORMAT is a runtime setting:
      Serato  = full CV02 timecode (position + carrier).
      Traktor = carrier only. Traktor tracks speed+direction in RELATIVE mode
                from the bare quadrature carrier; disable its timecode error
                messages and it works without the MK2 position bitstream. This
                avoids the end-zone/loop-wrap/drift problems of synthesizing
                the full MK2 signal.
  - Settings portal: long-press the settings button and the receiver raises a
    WiFi access point (SETTINGS_AP_SSID / SETTINGS_AP_PASS) with a phone-sized
    web page at http://192.168.4.1/ - output gain (live, watch the DJ
    software's scope while sliding), timecode format, LED brightness, pairing
    window length, re-pair action, per-deck link status. Values persist in
    NVS. Timecode keeps generating while the portal is up (the AP shares the
    ESP-NOW channel). Press again or 5 min idle to exit.
  - Onboard RGB LED: BLUE blink = pairing window open, RED = no transmitters,
    ORANGE = one, GREEN = both, PURPLE breathing = settings portal active.

  USB CDC also exposes the DVS Manager protocol. A Windows tray application can
  read live deck telemetry and edit the same NVS settings without enabling the
  WiFi access point. Human-readable debug lines remain available alongside the
  machine protocol and are ignored by the manager.

  Requires the Adafruit NeoPixel library.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_err.h>
#include <driver/i2s.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// newlib libm has sincosf (one call for both) but math.h only declares it
// under _GNU_SOURCE, so declare it ourselves.
extern "C" void sincosf(float x, float *s, float *c);

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
#define MSG_COMMAND 7 // receiver -> puck (rpmCenti = command, gyroRaw = arg)
#define CMD_SPINCAL 1 // re-arm the puck's spin-cal; arg = target rpm x100
#define PING_INTERVAL_MS 500
#define DECK_TIMEOUT_MS 1000

// ===== Auto-pairing (receiver assigns decks by transmitter MAC) ========
// Power the receiver first: for PAIRING_WINDOW_MS an unknown puck may claim
// a free deck (first to arrive = deck 1, second = deck 2); the window closes
// early as soon as both decks are claimed. A puck already
// in the table (dropped and powered back on) reclaims its own deck at any
// time, window or not. Table is RAM-only: rebooting the receiver, or a long
// press on the re-pair button, clears it and reopens the window.
#define PAIRING_WINDOW_MS 60000UL
#define REPAIR_BTN_PIN 4           // white button 1, momentary to GND
#define REPAIR_HOLD_MS 1500        // hold duration for either mapped action

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

// Low-battery indication: the receiver compares each puck's last reported
// battery voltage against gBattLowV (portal-adjustable) and flashes the
// status LED to identify the deck - 1 flash deck 1, 2 flashes deck 2,
// 3 flashes both. See serviceLed() / ledPulses().

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

// ===== DVS Manager over USB CDC =======================================
// Machine-readable lines always begin "@DVS ". Incoming commands use a
// compact key=value format so the receiver needs no JSON parser/library;
// outgoing replies and telemetry are JSON for easy desktop parsing.
#define USB_MANAGER_ENABLE 1
#define USB_MANAGER_PROTOCOL_VERSION 1
#define USB_MANAGER_TELEMETRY_MS 250
#define USB_MANAGER_RX_MAX 320
#define RECEIVER_FIRMWARE_VERSION "1.2.5"

// ===== Audio =========================================================
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 32            // small buffers = low output latency
#define DMA_BUF_COUNT 4

// ===== Timecode response ==============================================
#define BASE_RPM 33.333f
#define DEADZONE_RPM 0.04f
// Backspin headroom, per format - the two carriers are an octave and a bit
// apart, so a single limit cannot suit both.
//   Serato CV02: 1 kHz carrier at 33 1/3, so 8x = 8 kHz. No bandwidth concern.
//   Traktor:     2.5 kHz carrier, so 8x would be 20 kHz - 91% of Nyquist at
//                44.1 kHz. The PCM5102A's reconstruction filter is flat only
//                to ~19.8 kHz and the decoding input has its own anti-alias
//                filter with the same shape, so the carrier gets attenuated
//                twice (plus phase smear at the band edge) exactly when it is
//                changing fastest - it can fall under Traktor's detection
//                threshold and drop tracking. 6x = 15 kHz sits comfortably
//                inside the flat passband of both filters.
// 6x is 200 rpm, well beyond a realistic hand backspin (~100-170 rpm), so the
// cap is unreachable in practice. Past it the reported speed saturates rather
// than tracking - a graceful degradation, unlike losing the signal entirely.
#define MAX_RPM_RATIO 8.0f            // Serato CV02
#define MAX_RPM_RATIO_TRAKTOR 6.0f    // Traktor carrier-only
#define RPM_SMOOTHING 0.80f
// PCM5102A full scale is ~2.1 V RMS - far above consumer line level, and a
// massive overload for phono inputs (which also apply RIAA EQ; never feed
// them without an attenuator). 0.30 lands near normal line level; tune so
// Serato's calibration scope shows a round ring comfortably inside the
// view, then re-run Estimate on the noise threshold.
#define OUTPUT_GAIN 0.30f
#define GAIN_MIN 0.05f
#define GAIN_MAX 0.60f
// Gain is stored PER FORMAT. Serato peaks at `gain` while Traktor's carrier
// rides higher (see traktorLevelForGain), so one shared number meant a ~5 dB
// jump every time the format was switched and a fresh trim against the DJ
// software's scope each time. The wire/UI surface is unchanged: gGain is
// always the ACTIVE format's gain, which is the single value the manager app
// and the web portal read and write.

// ===== CV02 generator (Serato/Bridge-compatible) ======================
#define CV02_RESOLUTION 1000        // position bits per second at 33 1/3 rpm
#define CV02_BITS 20
#define CV02_SEED 0x59017UL
#define CV02_TAPS 0x361e4UL
#define CV02_LENGTH 712000UL        // full record: ~11.9 min of groove
// Loop a short window from the MIDDLE of the record instead of playing all of
// it. At the record's run-out Serato sees end-of-record and drops to INTERNAL;
// a backspin past position 0 also wraps (mod) into that same end zone. Staying
// in a mid-record window keeps the absolute position clear of both the lead-in
// and the run-out. RELATIVE MODE ONLY - each wrap is a harmless needle-skip in
// REL; in ABS mode the deck would jump every CV02_LOOP_SECONDS. (The original
// firmware avoided this by accident: a uint32 phase wrapped at bit 65536,
// ~65 s, so it never reached the end zone.)
#define CV02_LOOP_SECONDS 300UL
#define CV02_LOOP_START_SECONDS 30UL          // skip past the lead-in
#define CV02_LOOP_BITS (CV02_LOOP_SECONDS * CV02_RESOLUTION)
#define CV02_LOOP_START_BITS (CV02_LOOP_START_SECONDS * CV02_RESOLUTION)
#define CV02_PACKED_BYTES ((CV02_LOOP_BITS + 7) / 8)

// ===== Traktor generator - CARRIER ONLY ==============================
// Traktor tracks speed and direction in RELATIVE mode from the bare
// quadrature carrier alone; it does NOT need the MK2 position bitstream.
// So we emit just the carrier (no LFSR, no bit table, no loop, no wrap,
// no drift). Traktor flags "no timecode" - disable its timecode/calibration
// error messages (as PHASE and similar wireless systems instruct) and it
// tracks fine. This sidesteps every position-signal problem the MK2
// synthesis had (end-zone INT, loop-wrap skip, bit/carrier lock drift).
#define TRAKTOR_CARRIER_HZ 2500.0f      // carrier cycles/s at 33 1/3 rpm
#define CARRIER_LEVEL 0.55f             // carrier amplitude (rides OUTPUT_GAIN)
#define TRAKTOR_MAX_LEVEL 0.98f         // prevent int16 wrap at high portal gain

// ===== Settings portal =================================================
#define SETTINGS_BTN_PIN 5         // second PCB button to GND (INPUT_PULLUP).
                                   // ADJUST to match your board wiring.
#define SETTINGS_AP_SSID "DVS-Settings"
#define SETTINGS_AP_PASS "dvssetup"          // WPA2 needs >= 8 chars
#define SETTINGS_IDLE_TIMEOUT_MS 300000UL    // auto-exit after 5 min unused

#define FORMAT_SERATO  0
#define FORMAT_TRAKTOR 1

// Persisted physical-button actions. Keep these numeric values stable because
// they are stored in NVS and exchanged with DVS Manager.
#define BUTTON_ACTION_AP        0
#define BUTTON_ACTION_SWAP      1
#define BUTTON_ACTION_PAIR      2
#define BUTTON_ACTION_SPINCAL   3
#define BUTTON_ACTION_DISABLED  4
#define BUTTON_ACTION_MAX       BUTTON_ACTION_DISABLED

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
  // Link stats, accumulated per status-print interval, reset at each print:
  // DATA packets received, packets lost (seq gaps), RSSI sum + worst (dBm).
  uint16_t rxWindow;
  uint16_t lostWindow;
  int32_t  rssiSum;
  int8_t   rssiMin;         // 127 = nothing received this interval
  uint8_t  mac[6];          // owner MAC, set at assignment, matched thereafter
} deck_state;

typedef struct {
  uint8_t     deckId;
  i2s_port_t  port;
  int         bckPin, lrckPin, dataPin;
  const char *taskName;
  float       filteredRpm;
  int64_t     cv02Phase;         // CV02: 64-bit phase, clean reverse/backspin
  int64_t     carrierPhase;      // Traktor: 1.0 (<<32) = one carrier cycle
} audio_deck_state;

deck_state deckStates[2];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Unknown pucks may claim a free deck only while now < this (absolute millis).
volatile uint32_t pairingWindowEndMs = 0;

// Signed deadline comparison is safe when millis() wraps (all deadlines here
// are far shorter than 2^31 ms).
static inline bool timePending(uint32_t now, uint32_t deadline) {
  return (int32_t)(deadline - now) > 0;
}

static inline bool macEq(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

// One pending slot per device prevents simultaneous puck HELLO/EVENT packets
// from overwriting each other before loop() services them. Bits 0..2 map to
// deck 1, deck 2, and the optional dicer.
volatile uint8_t pendingWelcomeMask = 0;
uint8_t pendingWelcomeMac[3][6] = {};
volatile uint8_t pendingEventMask = 0;
uint8_t  pendingEventCode[3] = {};
uint32_t pendingEventValue[3] = {};

volatile bool     dicerSeen = false;
volatile uint16_t dicerMask = 0;
volatile uint8_t  dicerMode = 0;
uint16_t dicerPrevMask = 0;
uint8_t  dicerActiveNote[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t cv02PackedBits[CV02_PACKED_BYTES];

// ===== Runtime settings (NVS-backed, editable via the portal) =========
// Written from loop()/web handlers, read by the audio tasks - single-word
// volatile reads/writes are atomic on Xtensa, no locking needed.
Preferences settingsPrefs;
volatile float   gGain      = OUTPUT_GAIN;      // ACTIVE format's output gain
// The per-format pair behind gGain. Only touched from loop()/web/manager
// context; the audio tasks read gGain alone, exactly as before.
float            gGainSerato  = OUTPUT_GAIN;
float            gGainTraktor = OUTPUT_GAIN;
volatile uint8_t gFormat    = FORMAT_SERATO;    // FORMAT_SERATO / FORMAT_TRAKTOR
uint8_t          gBright    = LED_BRIGHTNESS;   // LED brightness
uint32_t         gPairWinMs = PAIRING_WINDOW_MS;
// Platter base speed (33.333 or 45) - CALIBRATION REFERENCE ONLY: it tells
// the pucks which quartz speed to trust when the portal's calibrate button
// re-arms their spin-cal. It deliberately does NOT rebase the timecode
// ratio math: the receiver mimics real vinyl (platter at 45 -> 1.35x tone),
// and the DJ software's own 33/45 selector does the normalising. Rebasing
// here too would double-compensate.
volatile float   gBaseRpm   = 33.3333f;
// Low-battery LED threshold (volts), receiver-side + portal-adjustable. A
// deck flashes its low pattern when its last reported battery voltage is
// below this. (The puck reports voltage over MSG_EVENT.)
volatile float   gBattLowV  = 3.50f;
// Keep battery telemetry/events active but allow the operator to suppress the
// receiver's repeating low-battery LED pulse pattern.
bool             gLowBatteryLedAlert = true;
// Keep the AP as an emergency recovery path, but it is never needed while the
// Windows manager is available over USB. The manager can disable it entirely.
bool             gApFallback = true;
uint8_t          gButton1Action = BUTTON_ACTION_PAIR; // white: open pairing
uint8_t          gButton2Action = BUTTON_ACTION_AP;   // black: settings AP

bool      settingsActive    = false;
uint32_t  settingsLastHitMs = 0;
WebServer settingsServer(80);
DNSServer settingsDns;

// Last per-deck link stats computed by debugPrintStatus, for /api/status
// (loss -1 = no packets that interval).
float webLoss[2] = { -1.0f, -1.0f };
int   webRssi[2] = { 0, 0 };

// Pending puck-command repeats: commands are unicast one-shots and the
// link runs 10-25% loss, so each command is sent 5x spaced 150 ms apart.
uint8_t  cmdRepeatsLeft[2] = { 0, 0 };
uint32_t cmdNextMs[2]      = { 0, 0 };
int16_t  cmdArgCenti[2]    = { 0, 0 };

// Last calibration-related event per deck, shown on the portal.
char webCalMsg[2][48] = { "", "" };

uint32_t lastLedMillis = 0;
uint32_t lastLedPacked = 0xFFFFFFFFUL;
float    deckBattV[2] = { 0.0f, 0.0f };   // last reported battery voltage/deck
uint32_t lastDebugPrintMillis = 0;
uint32_t lastManagerTelemetryMillis = 0;
char     managerRxLine[USB_MANAGER_RX_MAX];
uint16_t managerRxLength = 0;

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

// ===== USB serial: DVS Manager + optional human debug ================
// Brings up USB CDC for the manager even when human debug is disabled.
// It never carries timecode and is serviced only from loop().
void setupDebugSerial() {
#if DEBUG_SERIAL || USB_MANAGER_ENABLE
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // A complete two-deck status frame is larger than the native USB CDC
  // default TX ring (256 bytes). Without a larger ring, setTxTimeoutMs(0)
  // correctly avoids blocking the audio/radio loop but truncates every frame.
  // Allocate enough space for several complete manager frames before begin().
  Serial.setTxBufferSize(2048);
#endif
  Serial.begin(DEBUG_BAUD);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // Never let writes stall loop() when a host is attached but not draining
  // the port (manager or serial monitor closed mid-session).
  Serial.setTxTimeoutMs(0);
#endif
  delay(1000);
#endif
#if DEBUG_SERIAL
  Serial.println();
  Serial.println("DVS unified receiver boot (Serato CV02 + Traktor carrier)");
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

// Arduino normally generates prototypes for .ino functions, but keeping the
// manager's later dependencies explicit also makes this file friendly to
// conventional C++ tooling.
void clearPairingsAndReopen();
bool swapDeckAssignments();
void saveSettings();
void applyGainAndFormat(float newGain, uint8_t newFormat);

#if USB_MANAGER_ENABLE
static void managerJsonEscape(const char *src, char *dst, size_t dstSize) {
  if (dstSize == 0) return;
  size_t out = 0;
  while (*src && out + 1 < dstSize) {
    unsigned char c = (unsigned char)*src++;
    if ((c == '"' || c == '\\') && out + 2 < dstSize) {
      dst[out++] = '\\';
      dst[out++] = (char)c;
    } else if (c >= 0x20) {
      dst[out++] = (char)c;
    }
  }
  dst[out] = '\0';
}

static void managerSendJson(const char *json) {
  Serial.print("@DVS ");
  Serial.println(json);
}

static void managerSendResponse(uint32_t id, bool ok, const char *message) {
  char escaped[96], json[180];
  managerJsonEscape(message, escaped, sizeof(escaped));
  snprintf(json, sizeof(json),
           "{\"type\":\"response\",\"id\":%lu,\"ok\":%s,\"message\":\"%s\"}",
           (unsigned long)id, ok ? "true" : "false", escaped);
  managerSendJson(json);
}

static void managerSendEvent(const char *name, uint8_t deck, const char *detail) {
  char escapedName[48], escapedDetail[96], json[230];
  managerJsonEscape(name, escapedName, sizeof(escapedName));
  managerJsonEscape(detail, escapedDetail, sizeof(escapedDetail));
  snprintf(json, sizeof(json),
           "{\"type\":\"event\",\"name\":\"%s\",\"deck\":%u,\"detail\":\"%s\",\"at\":%lu}",
           escapedName, (unsigned)deck, escapedDetail, (unsigned long)millis());
  managerSendJson(json);
}

static void managerSendHello(uint32_t id) {
  uint64_t chip = ESP.getEfuseMac();
  char json[240];
  snprintf(json, sizeof(json),
           "{\"type\":\"hello\",\"id\":%lu,\"protocol\":%u,"
           "\"firmware\":\"%s\",\"device\":\"DVS Receiver\","
           "\"serial\":\"%04X%08X\"}",
           (unsigned long)id, (unsigned)USB_MANAGER_PROTOCOL_VERSION,
           RECEIVER_FIRMWARE_VERSION, (unsigned)((chip >> 32) & 0xFFFF),
           (unsigned)(chip & 0xFFFFFFFFUL));
  managerSendJson(json);
}

static void managerSendConfig(uint32_t id) {
  char json[360];
  snprintf(json, sizeof(json),
           "{\"type\":\"config\",\"id\":%lu,\"gain\":%.3f,"
           "\"gains\":%.3f,\"gaint\":%.3f,\"format\":%u,"
           "\"brightness\":%u,\"pairWindow\":%lu,\"baseRpm\":%.4f,"
           "\"batteryLow\":%.2f,\"lowBatteryLedAlert\":%s,\"apFallback\":%s,"
           "\"button1Action\":%u,\"button2Action\":%u}",
           (unsigned long)id, (float)gGain, gGainSerato, gGainTraktor,
           (unsigned)gFormat,
           (unsigned)gBright, (unsigned long)(gPairWinMs / 1000UL),
           (float)gBaseRpm, (float)gBattLowV,
           gLowBatteryLedAlert ? "true" : "false",
           gApFallback ? "true" : "false",
           (unsigned)gButton1Action, (unsigned)gButton2Action);
  managerSendJson(json);
}

static void managerDeckJson(uint8_t i, char *out, size_t outSize) {
  deck_state copy;
  portENTER_CRITICAL(&stateMux);
  memcpy(&copy, &deckStates[i], sizeof(copy));
  portEXIT_CRITICAL(&stateMux);

  uint32_t now = millis();
  bool online = copy.lastSeenMillis != 0 &&
                now - copy.lastSeenMillis <= DECK_TIMEOUT_MS;
  const char *state = online ? "live" : (failHolding[i] ? "holding" : "offline");
  char loss[16], rssi[16], mac[20], cal[80];
  if (webLoss[i] >= 0.0f) snprintf(loss, sizeof(loss), "%.1f", webLoss[i]);
  else snprintf(loss, sizeof(loss), "null");
  if (webRssi[i] != 0) snprintf(rssi, sizeof(rssi), "%d", webRssi[i]);
  else snprintf(rssi, sizeof(rssi), "null");
  if (copy.assigned)
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             copy.mac[0], copy.mac[1], copy.mac[2],
             copy.mac[3], copy.mac[4], copy.mac[5]);
  else
    snprintf(mac, sizeof(mac), "--");
  managerJsonEscape(webCalMsg[i], cal, sizeof(cal));
  uint32_t age = copy.lastSeenMillis ? now - copy.lastSeenMillis : 0;
  snprintf(out, outSize,
           "{\"deck\":%u,\"assigned\":%s,\"state\":\"%s\",\"rpm\":%.2f,"
           "\"loss\":%s,\"rssi\":%s,\"battery\":%.2f,\"lowBattery\":%s,"
           "\"age\":%lu,\"cal\":\"%s\",\"mac\":\"%s\"}",
           (unsigned)(i + 1), copy.assigned ? "true" : "false", state,
           copy.rpmCenti / 100.0f, loss, rssi, deckBattV[i],
           deckBattV[i] > 0.1f && deckBattV[i] < gBattLowV ? "true" : "false",
           (unsigned long)age, cal, mac);
}

static void managerSendStatus(uint32_t id) {
  char d1[300], d2[300], json[780];
  managerDeckJson(0, d1, sizeof(d1));
  managerDeckJson(1, d2, sizeof(d2));
  uint32_t now = millis();
  bool pairOpen = timePending(now, pairingWindowEndMs);
  uint32_t remaining = pairOpen ? (pairingWindowEndMs - now + 999UL) / 1000UL : 0;
  snprintf(json, sizeof(json),
           "{\"type\":\"status\",\"id\":%lu,\"uptime\":%lu,\"format\":%u,"
           "\"gain\":%.3f,\"pairOpen\":%s,\"pairRemaining\":%lu,"
           "\"portalActive\":%s,\"decks\":[%s,%s]}",
           (unsigned long)id, (unsigned long)now, (unsigned)gFormat,
           (float)gGain, pairOpen ? "true" : "false",
           (unsigned long)remaining, settingsActive ? "true" : "false", d1, d2);
  managerSendJson(json);
}

static const char *managerArg(char *const *tokens, uint8_t count, const char *name) {
  size_t nameLength = strlen(name);
  for (uint8_t i = 0; i < count; i++)
    if (strncmp(tokens[i], name, nameLength) == 0 && tokens[i][nameLength] == '=')
      return tokens[i] + nameLength + 1;
  return nullptr;
}

static bool managerParseLong(const char *text, long *value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  long parsed = strtol(text, &end, 10);
  if (!end || *end != '\0') return false;
  *value = parsed;
  return true;
}

static bool managerParseFloat(const char *text, float *value) {
  if (!text || !*text) return false;
  char *end = nullptr;
  float parsed = strtof(text, &end);
  if (!end || *end != '\0' || !isfinite(parsed)) return false;
  *value = parsed;
  return true;
}

static uint8_t managerArmCalibration() {
  int16_t arg = (int16_t)lroundf((float)gBaseRpm * 100.0f);
  uint32_t now = millis();
  uint8_t armed = 0;
  for (uint8_t i = 0; i < 2; i++) {
    bool assigned;
    portENTER_CRITICAL(&stateMux);
    assigned = deckStates[i].assigned;
    portEXIT_CRITICAL(&stateMux);
    if (!assigned) continue;
    armed++;
    cmdArgCenti[i] = arg;
    cmdRepeatsLeft[i] = 5;
    cmdNextMs[i] = now;
    snprintf(webCalMsg[i], sizeof(webCalMsg[0]), "cal: command sent...");
  }
  return armed;
}

static void managerProcessCommand(char *line) {
  char *save = nullptr;
  char *prefix = strtok_r(line, " ", &save);
  char *action = strtok_r(nullptr, " ", &save);
  if (!prefix || strcmp(prefix, "@DVS") != 0 || !action) return;

  char *tokens[18];
  uint8_t tokenCount = 0;
  while (tokenCount < 18) {
    char *token = strtok_r(nullptr, " ", &save);
    if (!token) break;
    tokens[tokenCount++] = token;
  }

  long parsedId = 0;
  managerParseLong(managerArg(tokens, tokenCount, "id"), &parsedId);
  uint32_t id = parsedId > 0 ? (uint32_t)parsedId : 0;

  if (strcasecmp(action, "HELLO") == 0) {
    managerSendHello(id);
    return;
  }
  if (strcasecmp(action, "GET_CONFIG") == 0) {
    managerSendConfig(id);
    return;
  }
  if (strcasecmp(action, "GET_STATUS") == 0) {
    managerSendStatus(id);
    return;
  }
  if (strcasecmp(action, "PING") == 0) {
    managerSendResponse(id, true, "pong");
    return;
  }
  if (strcasecmp(action, "SET_CONFIG") == 0) {
    float newGain = gGain, newBase = gBaseRpm, newBatteryLow = gBattLowV;
    float newGainSerato = gGainSerato, newGainTraktor = gGainTraktor;
    long newFormat = gFormat, newBrightness = gBright;
    long newPairWindow = (long)(gPairWinMs / 1000UL), newAp = gApFallback ? 1 : 0;
    long newLowBatteryLed = gLowBatteryLedAlert ? 1 : 0;
    long newButton1 = gButton1Action, newButton2 = gButton2Action;
    const char *gainSeratoArg = managerArg(tokens, tokenCount, "gains");
    const char *gainTraktorArg = managerArg(tokens, tokenCount, "gaint");
    const char *lowBatteryLedArg = managerArg(tokens, tokenCount, "lbled");
    const char *button1Arg = managerArg(tokens, tokenCount, "btn1");
    const char *button2Arg = managerArg(tokens, tokenCount, "btn2");
    bool hasExplicitGains = gainSeratoArg || gainTraktorArg;
    bool valid =
      managerParseFloat(managerArg(tokens, tokenCount, "gain"), &newGain) &&
      managerParseLong(managerArg(tokens, tokenCount, "fmt"), &newFormat) &&
      managerParseLong(managerArg(tokens, tokenCount, "bri"), &newBrightness) &&
      managerParseLong(managerArg(tokens, tokenCount, "pwin"), &newPairWindow) &&
      managerParseFloat(managerArg(tokens, tokenCount, "base"), &newBase) &&
      managerParseFloat(managerArg(tokens, tokenCount, "blow"), &newBatteryLow) &&
      managerParseLong(managerArg(tokens, tokenCount, "ap"), &newAp);
    if (lowBatteryLedArg)
      valid = managerParseLong(lowBatteryLedArg, &newLowBatteryLed) && valid;
    if (button1Arg)
      valid = managerParseLong(button1Arg, &newButton1) && valid;
    if (button2Arg)
      valid = managerParseLong(button2Arg, &newButton2) && valid;
    if (gainSeratoArg)
      valid = managerParseFloat(gainSeratoArg, &newGainSerato) && valid;
    if (gainTraktorArg)
      valid = managerParseFloat(gainTraktorArg, &newGainTraktor) && valid;
    valid = valid &&
      newGain >= GAIN_MIN && newGain <= GAIN_MAX &&
      newGainSerato >= GAIN_MIN && newGainSerato <= GAIN_MAX &&
      newGainTraktor >= GAIN_MIN && newGainTraktor <= GAIN_MAX &&
      (newFormat == FORMAT_SERATO || newFormat == FORMAT_TRAKTOR) &&
      newBrightness >= 5 && newBrightness <= 255 &&
      newPairWindow >= 10 && newPairWindow <= 300 &&
      newBase >= 30.0f && newBase <= 50.0f &&
      newBatteryLow >= 3.0f && newBatteryLow <= 4.0f &&
      (newLowBatteryLed == 0 || newLowBatteryLed == 1) &&
      newButton1 >= 0 && newButton1 <= BUTTON_ACTION_MAX &&
      newButton2 >= 0 && newButton2 <= BUTTON_ACTION_MAX &&
      (newAp == 0 || newAp == 1);
    if (!valid) {
      managerSendResponse(id, false, "invalid settings");
      return;
    }
    // Legacy clients know only "gain": preserve its outgoing/active-format
    // meaning so a format switch cannot overwrite the destination trim.
    // Explicit gains/gaint keys are unambiguous and take precedence together.
    if (hasExplicitGains) {
      gGainSerato = newGainSerato;
      gGainTraktor = newGainTraktor;
      gFormat = (uint8_t)newFormat;
      gGain = gFormat == FORMAT_TRAKTOR ? gGainTraktor : gGainSerato;
    } else {
      applyGainAndFormat(newGain, (uint8_t)newFormat);
    }
    gBright = (uint8_t)newBrightness;
    gPairWinMs = (uint32_t)newPairWindow * 1000UL;
    gBaseRpm = newBase > 40.0f ? 45.0f : 33.3333f;
    gBattLowV = newBatteryLow;
    gLowBatteryLedAlert = newLowBatteryLed != 0;
    gApFallback = newAp != 0;
    gButton1Action = (uint8_t)newButton1;
    gButton2Action = (uint8_t)newButton2;
    led.setBrightness(gBright);
    lastLedPacked = 0xFFFFFFFFUL;
    saveSettings();
    managerSendResponse(id, true, "settings saved");
    managerSendConfig(0);
    return;
  }
  if (strcasecmp(action, "CALIBRATE") == 0) {
    uint8_t armed = managerArmCalibration();
    managerSendResponse(id, armed > 0, armed > 0
      ? "calibration armed"
      : "no paired pucks available");
    return;
  }
  if (strcasecmp(action, "PAIR") == 0) {
    long seconds = (long)(gPairWinMs / 1000UL);
    const char *value = managerArg(tokens, tokenCount, "seconds");
    if (value && !managerParseLong(value, &seconds)) seconds = -1;
    if (seconds < 10 || seconds > 300) {
      managerSendResponse(id, false, "invalid pairing window");
      return;
    }
    pairingWindowEndMs = millis() + (uint32_t)seconds * 1000UL;
    managerSendResponse(id, true, "pairing window opened");
    return;
  }
  if (strcasecmp(action, "SWAP_DECKS") == 0) {
    bool swapped = swapDeckAssignments();
    managerSendResponse(id, swapped, swapped
      ? "deck assignments swapped"
      : "two paired pucks are required to swap decks");
    return;
  }
  if (strcasecmp(action, "REPAIR") == 0) {
    clearPairingsAndReopen();
    managerSendResponse(id, true, "pairings cleared");
    return;
  }

  managerSendResponse(id, false, "unknown command");
}

void serviceManagerSerial() {
  uint8_t budget = 96; // bound USB work per loop so radio/audio service wins
  while (budget-- && Serial.available() > 0) {
    int raw = Serial.read();
    if (raw < 0) break;
    char c = (char)raw;
    if (c == '\r') continue;
    if (c == '\n') {
      if (managerRxLength > 0) {
        managerRxLine[managerRxLength] = '\0';
        managerProcessCommand(managerRxLine);
        managerRxLength = 0;
      }
      continue;
    }
    if (managerRxLength + 1 < sizeof(managerRxLine)) {
      managerRxLine[managerRxLength++] = c;
    } else {
      managerRxLength = 0;
      managerSendResponse(0, false, "command too long");
    }
  }
}

void serviceManagerTelemetry() {
  uint32_t now = millis();
  if (now - lastManagerTelemetryMillis < USB_MANAGER_TELEMETRY_MS) return;
  lastManagerTelemetryMillis = now;
  managerSendStatus(0);
}
#else
void serviceManagerSerial() {}
void serviceManagerTelemetry() {}
#endif

// Periodic per-deck status. Called from loop(), outside the ESP-NOW
// callback, so it never disturbs radio reception or the audio tasks.
void debugPrintStatus() {
  uint32_t nowMillis = millis();
  if (nowMillis - lastDebugPrintMillis < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMillis = nowMillis;

  deck_state copy[2];
  portENTER_CRITICAL(&stateMux);
  memcpy(copy, deckStates, sizeof(copy));
  for (uint8_t i = 0; i < 2; i++) {          // stats windows restart now
    deckStates[i].rxWindow = 0;
    deckStates[i].lostWindow = 0;
    deckStates[i].rssiSum = 0;
    deckStates[i].rssiMin = 127;
  }
  portEXIT_CRITICAL(&stateMux);

  for (uint8_t i = 0; i < 2; i++) {
    bool online = copy[i].lastSeenMillis != 0 &&
                  nowMillis - copy[i].lastSeenMillis <= DECK_TIMEOUT_MS;
    float rpm = (float)copy[i].rpmCenti / 100.0f;
    uint32_t got = copy[i].rxWindow, lost = copy[i].lostWindow;
    // Feed the settings portal even when serial debug is compiled out.
    webLoss[i] = (got + lost > 0) ? 100.0f * (float)lost / (float)(got + lost) : -1.0f;
    webRssi[i] = got > 0 ? (int)(copy[i].rssiSum / (int32_t)got) : 0;
#if DEBUG_SERIAL
    // loss = share of the puck's sent packets that never arrived this
    // interval; rssi = interval average / worst, in dBm.
    char lossBuf[8] = "   --", rssiBuf[12] = "  --";
    if (got + lost > 0)
      snprintf(lossBuf, sizeof(lossBuf), "%4.1f%%",
               100.0f * (float)lost / (float)(got + lost));
    if (got > 0)
      snprintf(rssiBuf, sizeof(rssiBuf), "%d/%d",
               (int)(copy[i].rssiSum / (int32_t)got), (int)copy[i].rssiMin);
    Serial.printf("D%u %s rpm=%7.2f loss=%s rssi=%s age=%lums  ",
                  i + 1,
                  online ? "ON " : (failHolding[i] ? "HLD" : "OFF"),
                  rpm, lossBuf, rssiBuf,
                  copy[i].lastSeenMillis == 0 ? 0UL
                      : (unsigned long)(nowMillis - copy[i].lastSeenMillis));
#else
    (void)online;
    (void)rpm;
#endif
  }
#if DEBUG_SERIAL
  Serial.printf("pair=%s\n", timePending(nowMillis, pairingWindowEndMs) ? "OPEN" : "closed");
#endif
}

// Handle transmitter status events from loop(), never from the radio
// callback. Battery voltage is stored per deck for the LED + portal even
// when DEBUG_SERIAL is 0; printing is debug-only.
void serviceTransmitterEvents() {
  for (uint8_t device = 0; device < 3; device++) {
    uint8_t code = 0; uint32_t val = 0; bool has = false;
    portENTER_CRITICAL(&stateMux);
    uint8_t bit = (uint8_t)(1U << device);
    if (pendingEventMask & bit) {
      code = pendingEventCode[device];
      val = pendingEventValue[device];
      pendingEventMask &= (uint8_t)~bit;
      has = true;
    }
    portEXIT_CRITICAL(&stateMux);
    if (!has) continue;

    uint8_t deck = device + 1;
    if (deck <= 2) {
      float ev = (float)val / 1000000.0f;
      // Codes 1-9 only ever carry positive quantities (trim, volts, target
      // rpm). The zero-cal codes added later can legitimately be negative -
      // a gyro bias is as often negative as positive - so they decode the
      // same payload as signed. Codes 1-9 are untouched by this.
      float evSigned = (float)(int32_t)val / 1000000.0f;
      // Battery events (5 periodic, 6 LOW, 7 CRIT, 8 recovered) all carry the
      // voltage - store it; the receiver compares against gBattLowV for the LED.
      if (code >= 5 && code <= 8) deckBattV[deck - 1] = ev;
      // Mirror calibration progress onto the settings portal.
      switch (code) {
        case 1: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "trim loaded: %.5f", ev); break;
        case 2: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "no trim stored - calibrate"); break;
        case 3: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "cal: sampling ~10 s, don't touch"); break;
        case 4: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "cal: LOCKED trim=%.5f", ev); break;
        case 9: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "cal: armed (%.2f rpm) - spin at 0%% pitch", ev); break;
        case 10: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "STOP PLATTER: zeroing (%.1f rpm)", evSigned); break;
        case 11: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "zero ok: bias %.2f dps", evSigned); break;
        case 12: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "IMU FAULT %d - check wiring", (int)ev); break;
        case 13: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "INT1 drdy unusable - free-run"); break;
        case 14: snprintf(webCalMsg[deck - 1], sizeof(webCalMsg[0]), "zero kept: stored bias mismatch %.1f dps", ev); break;
      }
#if USB_MANAGER_ENABLE
      char detail[72];
      switch (code) {
        case 4:
          snprintf(detail, sizeof(detail), "calibration locked at trim %.5f", ev);
          managerSendEvent("calibration_complete", deck, detail);
          break;
        case 6:
          snprintf(detail, sizeof(detail), "battery low at %.2f V", ev);
          managerSendEvent("battery_low", deck, detail);
          break;
        case 7:
          snprintf(detail, sizeof(detail), "battery critical at %.2f V", ev);
          managerSendEvent("battery_critical", deck, detail);
          break;
        case 8:
          snprintf(detail, sizeof(detail), "battery recovered to %.2f V", ev);
          managerSendEvent("battery_recovered", deck, detail);
          break;
        case 10:
          snprintf(detail, sizeof(detail),
                   "gyro zero refused, platter turning at %.2f rpm - stop it", evSigned);
          managerSendEvent("zero_cal_blocked", deck, detail);
          break;
        case 11:
          snprintf(detail, sizeof(detail), "gyro zero accepted, bias %.2f dps", evSigned);
          managerSendEvent("zero_cal_complete", deck, detail);
          break;
        case 12:
          snprintf(detail, sizeof(detail),
                   "IMU fault code %d - puck is paired but sending no motion data", (int)ev);
          managerSendEvent("imu_fault", deck, detail);
          break;
        case 13:
          snprintf(detail, sizeof(detail),
                   "INT1 data-ready unusable, puck reverted to free-running sampling");
          managerSendEvent("drdy_fallback", deck, detail);
          break;
        case 14:
          snprintf(detail, sizeof(detail),
                   "gyro zero rejected: candidate bias differs from stored value by %.1f dps", ev);
          managerSendEvent("zero_bias_mismatch", deck, detail);
          break;
      }
#endif
    }

#if DEBUG_SERIAL
    float v = (float)val / 1000000.0f;
    switch (code) {
      case 1: Serial.printf("EVT deck %u: using stored trim=%.5f\n", deck, v); break;
      case 2: Serial.printf("EVT deck %u: NO stored trim - use the settings portal's Calibrate button\n", deck); break;
      case 3: Serial.printf("EVT deck %u: spin-cal reference %.2f settled, sampling ~10 s - don't touch\n", deck, v); break;
      case 4: Serial.printf("EVT deck %u: spin-cal LOCKED, trim=%.5f (saved)\n", deck, v); break;
      case 5: Serial.printf("EVT deck %u: battery %.2f V\n", deck, v); break;
      case 6: Serial.printf("EVT deck %u: battery LOW %.2f V - charge soon\n", deck, v); break;
      case 7: Serial.printf("EVT deck %u: battery CRITICAL %.2f V - charge now\n", deck, v); break;
      case 8: Serial.printf("EVT deck %u: battery recovered (%.2f V)\n", deck, v); break;
      case 9: Serial.printf("EVT deck %u: spin-cal ARMED, target %.2f rpm\n", deck, v); break;
      case 10: Serial.printf("EVT deck %u: zero-cal BLOCKED - platter turning at %.2f rpm, STOP IT\n",
                             deck, (float)(int32_t)val / 1000000.0f); break;
      case 11: Serial.printf("EVT deck %u: zero-cal accepted, bias %.2f dps\n",
                             deck, (float)(int32_t)val / 1000000.0f); break;
      case 12: Serial.printf("EVT deck %u: IMU FAULT code %d - puck paired but sending no data\n",
                             deck, (int)v); break;
      case 13: Serial.printf("EVT deck %u: INT1 data-ready unusable, free-running sampling\n", deck); break;
      case 14: Serial.printf("EVT deck %u: zero-cal bias mismatch %.2f dps - keeping stored bias\n",
                             deck, v); break;
      default: Serial.printf("EVT deck %u: code %u value %.5f\n", deck, code, v); break;
    }
#endif
  }
}

// Announce failover transitions from loop() (audio tasks must not print).
void serviceFailoverLog() {
#if FAILOVER_ENABLE
  static bool prev[2] = { false, false };
  for (uint8_t i = 0; i < 2; i++) {
    bool h = failHolding[i];
    if (h != prev[i]) {
      prev[i] = h;
#if DEBUG_SERIAL
      if (h) Serial.printf("FAILOVER deck %u: link lost - holding %.2f rpm\n",
                           i + 1, failStableRpm[i]);
      else   Serial.printf("FAILOVER deck %u: link restored - live rpm resumed\n", i + 1);
#endif
#if USB_MANAGER_ENABLE
      char detail[72];
      if (h) snprintf(detail, sizeof(detail), "holding last stable RPM %.2f", failStableRpm[i]);
      else snprintf(detail, sizeof(detail), "live RPM resumed");
      managerSendEvent(h ? "link_lost" : "link_restored", i + 1, detail);
#endif
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
// Color = deck count (red none / orange one / green both). Low battery
// modulates it with a counted pulse burst identifying the deck:
//   deck 1 low = 1 flash,  deck 2 low = 2 flashes,  both low = 3 flashes,
// then a gap, repeating. Returns whether the LED is lit this instant.
static bool ledPulses(uint32_t now, int n) {
  if (n <= 0) return true;                          // solid (no low battery)
  uint32_t t = now % 1600;                          // burst + gap cycle
  if (t >= (uint32_t)(n * 280)) return false;       // gap after the flashes
  return (t % 280) < 140;                           // on/off within the burst
}

// Is this deck (0/1) currently receiving packets?
static bool deckOnline(uint8_t i) {
  uint32_t ls;
  portENTER_CRITICAL(&stateMux);
  ls = deckStates[i].lastSeenMillis;
  portEXIT_CRITICAL(&stateMux);
  return ls != 0 && millis() - ls <= DECK_TIMEOUT_MS;
}

void serviceLed() {
  uint32_t now = millis();
  if (settingsActive) {
    // Purple breathing while the settings portal is up - overrides all
    // other states so the modal mode is unmistakable.
    float breath = 0.25f + 0.375f * (1.0f + sinf((float)now * 0.004f));
    uint8_t r = (uint8_t)(180.0f * breath), b = (uint8_t)(255.0f * breath);
    uint32_t packed = ((uint32_t)r << 16) | b;
    if (packed == lastLedPacked) return;
    lastLedPacked = packed;
    led.setPixelColor(0, led.Color(r, 0, b));
    led.show();
    return;
  }
  int onlineCount = onlineDeckCount();
  bool pairingOpen = timePending(now, pairingWindowEndMs);
  // Low = deck online and its last reported voltage is under the threshold.
  bool low1 = deckOnline(0) && deckBattV[0] > 0.1f && deckBattV[0] < gBattLowV;
  bool low2 = deckOnline(1) && deckBattV[1] > 0.1f && deckBattV[1] < gBattLowV;
  int battN = gLowBatteryLedAlert
                ? ((low1 && low2) ? 3 : (low2 ? 2 : (low1 ? 1 : 0)))
                : 0;
  uint8_t r, g, b;
  if      (onlineCount >= 2) { r = 0;   g = 255; b = 0; }   // green
  else if (onlineCount == 1) { r = 255; g = 80;  b = 0; }   // orange
  else if (pairingOpen)      { r = 0;   g = 0;   b = 255; } // blue: waiting to pair
  else                       { r = 255; g = 0;   b = 0; }   // red
  bool on = ledPulses(now, battN);
  if (pairingOpen) on = on && ((now % 800) < 400);          // blink while pairing
  if (!on) { r = 0; g = 0; b = 0; }
  uint32_t packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (packed == lastLedPacked) return;             // refresh only on change
  lastLedPacked = packed;
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

// ===== Fatal init failure ============================================
// These used to be bare `while (true) delay(1000)` - the receiver just sat
// there with a stale LED and no way to tell a dead board from a dead cable.
// Now it blinks a counted red code, keeps answering the manager so the fault
// is visible in the desktop app, and keeps repeating itself on serial.
#define FATAL_I2S        1     // 1 red flash  - I2S / DAC bring-up failed
#define FATAL_ESPNOW     2     // 2 red flashes - ESP-NOW init failed
#define FATAL_AUDIO_TASK 3     // 3 red flashes - audio task creation failed

void serviceManagerSerial();   // defined with the manager block above

static void fatalError(uint8_t code, const char *message) {
#if DEBUG_SERIAL
  Serial.printf("FATAL (%u): %s\n", code, message);
#endif
  uint32_t lastShout = 0;
  while (true) {
    uint32_t now = millis();
    // Counted red burst, then a gap - same shape as the low-battery pattern
    // so the number of flashes is the diagnosis.
    bool on = ledPulses(now, code);
    uint32_t packed = on ? 0xFF0000UL : 0;
    if (packed != lastLedPacked) {
      lastLedPacked = packed;
      led.setPixelColor(0, led.Color(on ? 255 : 0, 0, 0));
      led.show();
    }
    // Keep the USB side alive: HELLO still gets answered, so the manager can
    // still discover the receiver and show why it is not producing timecode.
    serviceManagerSerial();
    if (now - lastShout >= 2000) {
      lastShout = now;
#if DEBUG_SERIAL
      Serial.printf("FATAL (%u): %s\n", code, message);
#endif
#if USB_MANAGER_ENABLE
      managerSendEvent("fatal_error", 0, message);
#endif
    }
    delay(10);
  }
}

// ===== CV02 sequence =================================================
static uint8_t lfsrBit(uint32_t code, uint32_t taps) {
  return (uint8_t)__builtin_parity(code & taps);
}
static uint32_t lfsrForward(uint32_t cur) {
  uint8_t bit = lfsrBit(cur, CV02_TAPS | 0x1U);
  return (cur >> 1) | ((uint32_t)bit << (CV02_BITS - 1));
}
static void cv02SetBit(uint32_t i, uint8_t v) {
  uint32_t b = i >> 3; uint8_t m = (uint8_t)(1U << (i & 7));
  if (v) cv02PackedBits[b] |= m; else cv02PackedBits[b] &= (uint8_t)~m;
}
static inline uint8_t cv02GetBit(uint32_t i) {
  return (cv02PackedBits[i >> 3] & (uint8_t)(1U << (i & 7))) != 0;
}
void buildCv02Bits() {
  memset(cv02PackedBits, 0, sizeof(cv02PackedBits));
  uint32_t code = CV02_SEED;
  for (uint32_t i = 0; i < CV02_LOOP_START_BITS; i++) code = lfsrForward(code);   // skip lead-in
  for (uint32_t i = 0; i < CV02_LOOP_BITS; i++) { cv02SetBit(i, (uint8_t)(code & 1U)); code = lfsrForward(code); }
}

// ===== ESP-NOW =======================================================
bool ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6); p.channel = ESPNOW_CHANNEL; p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) return false;
  // Match the pucks' 6 Mbps legacy-OFDM rate for our unicast replies. The
  // stock rate is 1 Mbps 802.11b, which is why WELCOME/PING/COMMAND needed
  // the 5x command burst to punch through - this is the other half of that
  // fix and costs nothing. A failure here is not fatal: the peer still works
  // at the default rate.
  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_11G;
  rate.rate = WIFI_PHY_RATE_6M;
  esp_err_t rateErr = esp_now_set_peer_rate_config(mac, &rate);
#if DEBUG_SERIAL
  if (rateErr != ESP_OK)
    Serial.printf("WARN peer rate config failed (%s) - using stock rate\n",
                  esp_err_to_name(rateErr));
#else
  (void)rateErr;
#endif
  return true;
}

void sendControlMessage(const uint8_t *mac, uint8_t deckId, uint8_t msgType) {
  if (!ensurePeer(mac)) return;
  dvs_packet r = {};
  r.msgType = msgType; r.version = PROTOCOL_VERSION; r.deckId = deckId;
  r.timestampMicros = micros();
  esp_now_send(mac, (uint8_t *)&r, sizeof(r));
}

void sendCommandMessage(const uint8_t *mac, uint8_t deckId, int16_t cmd, int16_t arg) {
  if (!ensurePeer(mac)) return;
  dvs_packet r = {};
  r.msgType = MSG_COMMAND; r.version = PROTOCOL_VERSION; r.deckId = deckId;
  r.rpmCenti = cmd; r.gyroRaw = arg;
  r.timestampMicros = micros();
  esp_now_send(mac, (uint8_t *)&r, sizeof(r));
}

// Send queued command repeats from loop() (never from web handlers' many
// invocations or the radio callback).
void serviceCommandRepeats() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < 2; i++) {
    if (!cmdRepeatsLeft[i] || (int32_t)(now - cmdNextMs[i]) < 0) continue;
    uint8_t mac[6]; bool assigned = false;
    portENTER_CRITICAL(&stateMux);
    if (deckStates[i].assigned) { memcpy(mac, deckStates[i].mac, 6); assigned = true; }
    portEXIT_CRITICAL(&stateMux);
    if (assigned) sendCommandMessage(mac, i + 1, CMD_SPINCAL, cmdArgCenti[i]);
    cmdRepeatsLeft[i]--;
    cmdNextMs[i] = now + 150;
  }
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
      memcpy(pendingWelcomeMac[DICER_ID - 1], src, 6);
      pendingWelcomeMask |= (uint8_t)(1U << (DICER_ID - 1));
      portEXIT_CRITICAL_ISR(&stateMux);
    } else if (packet.msgType == MSG_BUTTONS) {
      portENTER_CRITICAL_ISR(&stateMux);
      dicerMask = (uint16_t)packet.rpmCenti;
      dicerMode = (uint8_t)packet.gyroRaw;
      dicerSeen = true;
      portEXIT_CRITICAL_ISR(&stateMux);
    } else if (packet.msgType == MSG_EVENT) {
      portENTER_CRITICAL_ISR(&stateMux);
      pendingEventCode[DICER_ID - 1] = (uint8_t)packet.rpmCenti;
      pendingEventValue[DICER_ID - 1] = packet.timestampMicros;
      pendingEventMask |= (uint8_t)(1U << (DICER_ID - 1));
      portEXIT_CRITICAL_ISR(&stateMux);
    }
    return;
  }

  // ---- Transmitter: find this MAC's deck, or assign a free one -------
  int slot = -1;
  portENTER_CRITICAL_ISR(&stateMux);
  for (uint8_t i = 0; i < 2; i++)
    if (deckStates[i].assigned && macEq(deckStates[i].mac, src)) { slot = (int)i; break; }
  if (slot < 0 && timePending(millis(), pairingWindowEndMs)) {
    for (uint8_t i = 0; i < 2; i++)
      if (!deckStates[i].assigned) {
        deckStates[i].assigned = true;
        memcpy(deckStates[i].mac, src, 6);
        deckStates[i].seen = false;
        deckStates[i].lastSeq = 0;
        deckStates[i].lastPingMillis = 0;
        deckStates[i].rxWindow = 0;
        deckStates[i].lostWindow = 0;
        deckStates[i].rssiSum = 0;
        deckStates[i].rssiMin = 127;
        slot = (int)i;
        break;
      }
    // Both decks claimed -> close the window right away (LED stops its
    // pairing blink and no third unit can sneak in).
    if (deckStates[0].assigned && deckStates[1].assigned)
      pairingWindowEndMs = millis();
  }
  portEXIT_CRITICAL_ISR(&stateMux);
  if (slot < 0) return;    // unknown MAC, window closed or both decks taken

  deck_state *state = &deckStates[slot];
  uint8_t deckNum = (uint8_t)slot + 1;

  if (packet.msgType == MSG_HELLO) {
    portENTER_CRITICAL_ISR(&stateMux);
    memcpy(pendingWelcomeMac[slot], src, 6);
    pendingWelcomeMask |= (uint8_t)(1U << slot);
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }
  if (packet.msgType == MSG_EVENT) {
    portENTER_CRITICAL_ISR(&stateMux);
    pendingEventCode[slot] = (uint8_t)packet.rpmCenti;
    pendingEventValue[slot] = packet.timestampMicros;
    pendingEventMask |= (uint8_t)(1U << slot);
    portEXIT_CRITICAL_ISR(&stateMux);
    return;
  }
  if (packet.msgType != MSG_DATA) return;

  int8_t rssi = (int8_t)(info->rx_ctrl ? info->rx_ctrl->rssi : 0);
  portENTER_CRITICAL_ISR(&stateMux);
  // Only motion DATA makes a deck live. HELLO, battery and fault events prove
  // the radio is reachable but must not interrupt hold-last-RPM failover.
  state->lastSeenMillis = millis();
  if (state->seen) {
    uint32_t d = packet.seq - state->lastSeq;
    // d==1 is the normal step. A huge jump means the puck rebooted (seq
    // reset), not real loss - don't count those.
    if (d > 1 && d < 10000)
      state->lostWindow = (uint16_t)min((uint32_t)state->lostWindow + (d - 1), 65535UL);
  }
  state->seen = true;
  state->rpmCenti = packet.rpmCenti;
  state->lastSeq = packet.seq;
  state->rxWindow++;
  state->rssiSum += rssi;
  if (state->rssiMin == 127 || rssi < state->rssiMin) state->rssiMin = rssi;
  portEXIT_CRITICAL_ISR(&stateMux);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if DEBUG_SERIAL
  // Informational only - pucks auto-pair by their own MAC, nothing to copy.
  // esp_read_mac reads the factory STA MAC from eFuse - unlike
  // WiFi.macAddress() it works before the WiFi driver has started.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Max radio TX power so WELCOME/PING replies reach a distant puck
  // (units 0.25 dBm; the driver clamps to chip limit).
  esp_wifi_set_max_tx_power(84);
  if (esp_now_init() != ESP_OK)
    fatalError(FATAL_ESPNOW, "ESP-NOW init failed - receiver cannot hear the pucks");
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
  esp_err_t err = i2s_driver_install(deck->port, &config, 0, NULL);
  if (err == ESP_OK) err = i2s_set_pin(deck->port, &pins);
  if (err == ESP_OK) err = i2s_zero_dma_buffer(deck->port);
  if (err != ESP_OK) {
#if DEBUG_SERIAL
    Serial.printf("ERR I2S deck %u setup failed: %s\n", deck->deckId, esp_err_to_name(err));
#endif
    char detail[80];
    snprintf(detail, sizeof(detail), "I2S deck %u setup failed: %s",
             deck->deckId, esp_err_to_name(err));
    fatalError(FATAL_I2S, detail);
  }

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
// int64 phase => glitch-free reverse/backspin across the loop window.
static inline void renderCv02Sample(audio_deck_state *deck, int64_t phaseStep,
                                    float gain,
                                    int16_t *leftOut, int16_t *rightOut) {
  deck->cv02Phase += phaseStep;

  int64_t cyc = deck->cv02Phase >> CV02_FRAC_BITS;
  uint32_t cycleIndex = (uint32_t)(((cyc % (int64_t)CV02_LOOP_BITS) + (int64_t)CV02_LOOP_BITS) % (int64_t)CV02_LOOP_BITS);
  float frac = (float)(uint32_t)deck->cv02Phase * (1.0f / CV02_FRAC_ONE);

  float angle = frac * 6.28318530718f;
  float sine, cosine;
  sincosf(angle, &sine, &cosine);   // one libm call for both = cheaper
  uint8_t bit = cv02GetBit(cycleIndex);
  float modulation = bit ? 1.0f : 1.0f - ((-cosine + 1.0f) * 0.25f);

  *leftOut  = (int16_t)((-cosine * modulation * gain) * 32767.0f);
  *rightOut = (int16_t)(( sine   * modulation * gain) * 32767.0f);
}

// rpm -> per-sample carrier phase step, computed once per DMA buffer.
// Uses the Traktor-specific ratio cap so the carrier stays inside the flat
// passband of the DAC reconstruction and soundcard anti-alias filters.
static int64_t rpmToCarrierStep(float rpm) {
  if (fabsf(rpm) < DEADZONE_RPM) return 0;
  float ratio = rpm / BASE_RPM;
  if (ratio >  MAX_RPM_RATIO_TRAKTOR) ratio =  MAX_RPM_RATIO_TRAKTOR;
  if (ratio < -MAX_RPM_RATIO_TRAKTOR) ratio = -MAX_RPM_RATIO_TRAKTOR;
  float cyclesPerSample = ratio * (TRAKTOR_CARRIER_HZ / SAMPLE_RATE);
  return (int64_t)llroundf(cyclesPerSample * CV02_FRAC_ONE);
}

// Portal/manager gain -> Traktor carrier amplitude.
//
// The old map was `min(CARRIER_LEVEL * gain / OUTPUT_GAIN, TRAKTOR_MAX_LEVEL)`,
// which saturated at gain 0.535 - so the top ~12% of the slider did nothing
// while the readout kept climbing. Two segments instead: unchanged below the
// default (existing Traktor trims stay valid to the digit), then the rest of
// the travel stretched to reach TRAKTOR_MAX_LEVEL at full slider. Monotonic
// and continuous, and still clamped below full scale so the int16 conversion
// cannot wrap.
static inline float traktorLevelForGain(float gain) {
  if (gain <= OUTPUT_GAIN)
    return CARRIER_LEVEL * gain * (1.0f / OUTPUT_GAIN);
  float t = (gain - OUTPUT_GAIN) / (GAIN_MAX - OUTPUT_GAIN);
  if (t > 1.0f) t = 1.0f;
  return CARRIER_LEVEL + t * (TRAKTOR_MAX_LEVEL - CARRIER_LEVEL);
}

// Pure quadrature carrier: L = -cos, R = sin (R leads L by 90 forward).
// No position bitstream - Traktor tracks speed+direction from this in REL.
static inline void renderTraktorSample(audio_deck_state *deck, int64_t carrierStep,
                                       float level,
                                       int16_t *leftOut, int16_t *rightOut) {
  deck->carrierPhase += carrierStep;
  float frac = (float)(uint32_t)deck->carrierPhase * (1.0f / CV02_FRAC_ONE);
  float sine, cosine;
  sincosf(frac * 6.28318530718f, &sine, &cosine);
  *leftOut  = (int16_t)((-cosine * level) * 32767.0f);
  *rightOut = (int16_t)(( sine   * level) * 32767.0f);
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
  const int64_t cv02Span = (int64_t)CV02_LOOP_BITS << CV02_FRAC_BITS;
  while (true) {
    float target = readTargetRpm(deck->deckId);
    deck->filteredRpm += (target - deck->filteredRpm) * RPM_SMOOTHING;
    // Sample the runtime settings once per buffer; format can flip live.
    uint8_t fmt  = gFormat;
    float   gain = gGain;

    if (fmt == FORMAT_SERATO) {
      int64_t phaseStep = rpmToPhaseStep(deck->filteredRpm);
      deck->cv02Phase %= cv02Span;     // fold so the accumulator never overflows
      for (int i = 0; i < DMA_BUF_LEN; i++) {
        int16_t l = 0, r = 0;
        renderCv02Sample(deck, phaseStep, gain, &l, &r);
        buffer[i * 2] = l; buffer[i * 2 + 1] = r;
      }
    } else {
      // Traktor: pure carrier, driven by this format's own stored gain.
      float level = traktorLevelForGain(gain);
      int64_t carrierStep = rpmToCarrierStep(deck->filteredRpm);
      deck->carrierPhase &= 0xFFFFFFFFLL;  // keep only the fractional cycle
      for (int i = 0; i < DMA_BUF_LEN; i++) {
        int16_t l = 0, r = 0;
        renderTraktorSample(deck, carrierStep, level, &l, &r);
        buffer[i * 2] = l; buffer[i * 2 + 1] = r;
      }
    }
    size_t written = 0;
    i2s_write(deck->port, buffer, sizeof(buffer), &written, portMAX_DELAY);
  }
}

void serviceEspNowControl() {
  for (uint8_t device = 0; device < 3; device++) {
    uint8_t mac[6]; bool welcome = false;
    portENTER_CRITICAL(&stateMux);
    uint8_t bit = (uint8_t)(1U << device);
    if (pendingWelcomeMask & bit) {
      memcpy(mac, pendingWelcomeMac[device], 6);
      pendingWelcomeMask &= (uint8_t)~bit;
      welcome = true;
    }
    portEXIT_CRITICAL(&stateMux);
    if (!welcome) continue;
    uint8_t deck = device + 1;
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

// Drop both pucks and reopen the pairing window, so a replacement puck can
// be slotted without a power cycle. Called by the button and the portal.
void clearPairingsAndReopen() {
  // Drop the ESP-NOW peer entries too. They used to leak: the deck table was
  // wiped but the peers stayed, so repeated re-pairs with different pucks
  // would eventually exhaust the peer list and ensurePeer() would start
  // silently failing - WELCOME/PING/COMMAND would just stop working.
  uint8_t oldMacs[2][6];
  bool hadPeer[2] = { false, false };
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++) {
    if (deckStates[i].assigned) {
      memcpy(oldMacs[i], deckStates[i].mac, 6);
      hadPeer[i] = true;
    }
  }
  portEXIT_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++)
    if (hadPeer[i] && esp_now_is_peer_exist(oldMacs[i]))
      esp_now_del_peer(oldMacs[i]);

  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++) {
    deckStates[i].assigned = false;
    deckStates[i].seen = false;
    deckStates[i].rpmCenti = 0;
    deckStates[i].lastSeenMillis = 0;
    deckStates[i].lastPingMillis = 0;
    deckStates[i].rxWindow = 0;
    deckStates[i].lostWindow = 0;
    deckStates[i].rssiSum = 0;
    deckStates[i].rssiMin = 127;
    memset(deckStates[i].mac, 0, 6);
  }
  pendingWelcomeMask &= (uint8_t)~0x03U;
  pendingEventMask &= (uint8_t)~0x03U;
  pairingWindowEndMs = millis() + gPairWinMs;
  portEXIT_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++) {
    deckBattV[i] = 0.0f;
    webLoss[i] = -1.0f;
    webRssi[i] = 0;
    webCalMsg[i][0] = '\0';
    cmdRepeatsLeft[i] = 0;
    failStableRpm[i] = failCandRpm[i] = 0.0f;
    failCandSinceMs[i] = 0;
  }
#if FAILOVER_ENABLE
  failHolding[0] = failHolding[1] = false;   // stop any held-rpm output
#endif
  debugBootLog("RE-PAIR: pairings cleared, pairing window reopened");
}

// Exchange the two live puck assignments without clearing/re-pairing. The
// deck table is the routing source of truth, so swapping complete records
// changes both audio outputs atomically. Puck-specific UI telemetry follows
// its MAC; transient command/event state is cleared to avoid delivering an
// already queued action to the newly assigned deck.
bool swapDeckAssignments() {
  uint8_t newMacs[2][6];
  portENTER_CRITICAL(&stateMux);
  if (!deckStates[0].assigned || !deckStates[1].assigned) {
    portEXIT_CRITICAL(&stateMux);
    return false;
  }
  deck_state state = deckStates[0];
  deckStates[0] = deckStates[1];
  deckStates[1] = state;
  memcpy(newMacs[0], deckStates[0].mac, 6);
  memcpy(newMacs[1], deckStates[1].mac, 6);
  pendingWelcomeMask &= (uint8_t)~0x03U;
  pendingEventMask &= (uint8_t)~0x03U;
  portEXIT_CRITICAL(&stateMux);

  float floatValue = deckBattV[0];
  deckBattV[0] = deckBattV[1];
  deckBattV[1] = floatValue;
  floatValue = webLoss[0];
  webLoss[0] = webLoss[1];
  webLoss[1] = floatValue;
  int intValue = webRssi[0];
  webRssi[0] = webRssi[1];
  webRssi[1] = intValue;
  char cal[sizeof(webCalMsg[0])];
  memcpy(cal, webCalMsg[0], sizeof(cal));
  memcpy(webCalMsg[0], webCalMsg[1], sizeof(webCalMsg[0]));
  memcpy(webCalMsg[1], cal, sizeof(webCalMsg[1]));

  cmdRepeatsLeft[0] = cmdRepeatsLeft[1] = 0;
  failStableRpm[0] = failStableRpm[1] = 0.0f;
  failCandRpm[0] = failCandRpm[1] = 0.0f;
  failCandSinceMs[0] = failCandSinceMs[1] = millis();
  failHolding[0] = failHolding[1] = false;
  lastLedPacked = 0xFFFFFFFFUL;

  // Refresh each puck's displayed deck assignment immediately.
  sendControlMessage(newMacs[0], 1, MSG_WELCOME);
  sendControlMessage(newMacs[1], 2, MSG_WELCOME);
  debugBootLog("DECKS: assignments swapped");
  return true;
}

int onlineDeckCount() {
  int n = 0; uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i = 0; i < 2; i++)
    if (deckStates[i].lastSeenMillis && now - deckStates[i].lastSeenMillis <= DECK_TIMEOUT_MS) n++;
  portEXIT_CRITICAL(&stateMux);
  return n;
}

// ===== Settings: NVS + WiFi portal ===================================
void loadSettings() {
  settingsPrefs.begin("dvsrx", false);
  float gain = settingsPrefs.getFloat("gain", OUTPUT_GAIN);
  uint8_t fmt = settingsPrefs.getUChar("fmt", FORMAT_SERATO);
  uint8_t bright = settingsPrefs.getUChar("bri", LED_BRIGHTNESS);
  uint32_t pairWin = settingsPrefs.getUInt("pwin", PAIRING_WINDOW_MS);
  float base = settingsPrefs.getFloat("base", 33.3333f);
  float battLow = settingsPrefs.getFloat("blow", 3.50f);
  bool lowBatteryLedAlert = settingsPrefs.getBool("lbled", true);
  bool apFallback = settingsPrefs.getBool("apfb", true);
  uint8_t button1Action = settingsPrefs.getUChar("btn1", BUTTON_ACTION_PAIR);
  uint8_t button2Action = settingsPrefs.getUChar("btn2", BUTTON_ACTION_AP);

  // "gain" keeps its original meaning (Serato) so an existing NVS blob loads
  // unchanged; "gaint" is new and defaults to the same value, so the first
  // boot after this update behaves exactly as before until Traktor is trimmed.
  float gainTraktor = settingsPrefs.getFloat("gaint", gain);

  gGainSerato = isfinite(gain) && gain >= GAIN_MIN && gain <= GAIN_MAX
                    ? gain : OUTPUT_GAIN;
  gGainTraktor = isfinite(gainTraktor) && gainTraktor >= GAIN_MIN && gainTraktor <= GAIN_MAX
                    ? gainTraktor : OUTPUT_GAIN;
  gFormat = fmt <= FORMAT_TRAKTOR ? fmt : FORMAT_SERATO;
  gGain = gFormat == FORMAT_TRAKTOR ? gGainTraktor : gGainSerato;
  gBright = bright >= 5 ? bright : LED_BRIGHTNESS;
  gPairWinMs = pairWin >= 10000UL && pairWin <= 300000UL ? pairWin : PAIRING_WINDOW_MS;
  gBaseRpm = isfinite(base) && base >= 30.0f && base <= 50.0f
                 ? (base > 40.0f ? 45.0f : 33.3333f)
                 : 33.3333f;
  gBattLowV = isfinite(battLow) && battLow >= 3.0f && battLow <= 4.0f ? battLow : 3.50f;
  gLowBatteryLedAlert = lowBatteryLedAlert;
  gApFallback = apFallback;
  gButton1Action = button1Action <= BUTTON_ACTION_MAX ? button1Action : BUTTON_ACTION_PAIR;
  gButton2Action = button2Action <= BUTTON_ACTION_MAX ? button2Action : BUTTON_ACTION_AP;
}

// Commit a gain/format change. `newGain` belongs to the format that is active
// RIGHT NOW - that is what the manager slider and the portal slider have been
// displaying - so it is filed against the outgoing format before the switch.
// The incoming format's own stored gain then becomes live, which is what both
// UIs pick up on their next config/telemetry read.
void applyGainAndFormat(float newGain, uint8_t newFormat) {
  if (gFormat == FORMAT_TRAKTOR) gGainTraktor = newGain;
  else                           gGainSerato  = newGain;
  gFormat = newFormat;
  gGain = newFormat == FORMAT_TRAKTOR ? gGainTraktor : gGainSerato;
}

void saveSettings() {
  // Keep the pair consistent with the live value whatever path got us here.
  if (gFormat == FORMAT_TRAKTOR) gGainTraktor = gGain; else gGainSerato = gGain;
  settingsPrefs.putFloat("gain",  gGainSerato);
  settingsPrefs.putFloat("gaint", gGainTraktor);
  settingsPrefs.putUChar("fmt",  gFormat);
  settingsPrefs.putUChar("bri",  gBright);
  settingsPrefs.putUInt("pwin",  gPairWinMs);
  settingsPrefs.putFloat("base", gBaseRpm);
  settingsPrefs.putFloat("blow", gBattLowV);
  settingsPrefs.putBool("lbled", gLowBatteryLedAlert);
  settingsPrefs.putBool("apfb", gApFallback);
  settingsPrefs.putUChar("btn1", gButton1Action);
  settingsPrefs.putUChar("btn2", gButton2Action);
}

// Single self-contained page: no external assets, so it works inside captive
// portal sheets with no internet access.
static const char SETTINGS_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#080b12"><title>DVS Control</title>
<style>
:root{color-scheme:dark;--bg:#080b12;--panel:#111722;--panel2:#171f2d;--line:#263146;--text:#f4f7fb;--muted:#91a0b7;--cyan:#55d8ff;--violet:#9278ff;--green:#55e6a5;--amber:#ffca68;--red:#ff6b7d}
*{box-sizing:border-box}html{background:var(--bg)}body{margin:0;min-height:100vh;font:15px/1.45 system-ui,-apple-system,Segoe UI,sans-serif;color:var(--text);background:radial-gradient(900px 420px at 50% -120px,#243365 0,transparent 65%),var(--bg)}
main{width:min(760px,100%);margin:auto;padding:24px 16px calc(32px + env(safe-area-inset-bottom))}
header{display:flex;align-items:center;justify-content:space-between;gap:16px;margin:4px 2px 22px}.brand{display:flex;align-items:center;gap:12px}.mark{width:42px;height:42px;border-radius:13px;display:grid;place-items:center;background:linear-gradient(145deg,var(--cyan),var(--violet));color:#07111c;font-weight:900;box-shadow:0 10px 30px #557cff33}.eyebrow{color:var(--cyan);font-size:11px;font-weight:800;letter-spacing:.14em;text-transform:uppercase}h1{font-size:22px;line-height:1.1;margin:3px 0 0}.connect{display:flex;align-items:center;gap:7px;color:var(--muted);font-size:12px}.dot{width:8px;height:8px;border-radius:50%;background:var(--amber);box-shadow:0 0 0 4px #ffca6818}.connect.live .dot{background:var(--green);box-shadow:0 0 0 4px #55e6a51c}
.section-title{display:flex;justify-content:space-between;align-items:end;margin:18px 2px 9px}.section-title h2{font-size:13px;text-transform:uppercase;letter-spacing:.12em;margin:0;color:#c8d3e4}.section-title span{font-size:12px;color:var(--muted)}
.decks{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}.card{background:linear-gradient(155deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:18px;box-shadow:0 14px 38px #0004}.deck{padding:16px;min-width:0}.deck-head{display:flex;justify-content:space-between;align-items:center}.deck-name{font-weight:800}.badge{font-size:10px;font-weight:900;letter-spacing:.08em;text-transform:uppercase;padding:5px 8px;border-radius:99px;background:#77849b20;color:var(--muted)}.badge.online{color:var(--green);background:#55e6a518}.badge.hold{color:var(--amber);background:#ffca6818}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:14px}.metric{background:#080c1480;border:1px solid #243047;border-radius:10px;padding:9px;min-width:0}.metric b{display:block;font-size:15px;font-variant-numeric:tabular-nums;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.metric span{color:var(--muted);font-size:10px;text-transform:uppercase}.battery{margin-top:11px;background:#080c1480;border:1px solid #243047;border-radius:11px;padding:10px}.battery-head,.battery-scale{display:flex;align-items:center;justify-content:space-between}.battery-head span{color:var(--muted);font-size:10px;text-transform:uppercase}.battery-head b{font-size:14px;font-variant-numeric:tabular-nums}.battery-track{height:10px;margin-top:8px;border-radius:99px;background:#273044;overflow:hidden;box-shadow:inset 0 1px 3px #0008}.battery-fill{display:block;width:0;height:100%;border-radius:inherit;transition:width .35s ease,background .2s}.battery-fill.good{background:linear-gradient(90deg,#34c98a,var(--green));box-shadow:0 0 12px #55e6a548}.battery-fill.warn{background:linear-gradient(90deg,#ff9d52,var(--amber));box-shadow:0 0 12px #ffca6840}.battery-fill.low{background:linear-gradient(90deg,#dc4158,var(--red));box-shadow:0 0 12px #ff6b7d48}.battery-scale{margin-top:4px;color:#637087;font-size:9px}.calmsg{min-height:18px;margin-top:9px;color:var(--cyan);font-size:11px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.settings{padding:18px}.field{margin:0 0 18px}.field-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}.field label{font-weight:650}.value{color:var(--cyan);font-variant-numeric:tabular-nums;font-size:12px;background:#55d8ff12;border:1px solid #55d8ff26;border-radius:8px;padding:3px 7px}select{width:100%;appearance:none;background:var(--bg);border:1px solid var(--line);border-radius:11px;color:var(--text);font:inherit;padding:12px;outline:none}select:focus{border-color:var(--cyan);box-shadow:0 0 0 3px #55d8ff18}input[type=range]{width:100%;height:22px;margin:0;accent-color:var(--cyan)}.hint{font-size:11px;color:var(--muted);margin-top:5px}
.actions{display:grid;grid-template-columns:1fr 1fr;gap:10px}.btn{border:1px solid var(--line);border-radius:11px;padding:12px 14px;background:#1a2332;color:var(--text);font:700 14px system-ui;cursor:pointer}.btn:active{transform:translateY(1px)}.btn:disabled{opacity:.45}.primary{grid-column:1/-1;background:linear-gradient(135deg,#227fa1,#6757c6);border:0}.danger{color:#ff9ba7;border-color:#69303a;background:#30171e}.pairbar{margin-top:12px;display:flex;justify-content:space-between;color:var(--muted);font-size:12px}
#toast{position:fixed;left:50%;bottom:calc(20px + env(safe-area-inset-bottom));transform:translate(-50%,18px);max-width:calc(100% - 32px);padding:11px 15px;border-radius:11px;background:#e9f8ff;color:#07111c;font-weight:750;box-shadow:0 15px 50px #0008;opacity:0;pointer-events:none;transition:.2s}#toast.show{opacity:1;transform:translate(-50%,0)}#toast.error{background:#ffd9de;color:#3a0710}
@media(max-width:560px){main{padding-top:18px}.decks{grid-template-columns:1fr}.actions{grid-template-columns:1fr}.primary{grid-column:auto}}@media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;transition:none!important}}
</style></head><body><main>
<header><div class="brand"><div class="mark">DVS</div><div><div class="eyebrow">Wireless timecode</div><h1>Receiver Control</h1></div></div><div class="connect" id="connection"><i class="dot"></i><span>Connecting</span></div></header>
<div class="section-title"><h2>Live decks</h2><span id="formatSummary">—</span></div>
<section class="decks" aria-label="Deck status">
<article class="card deck" id="deck1"><div class="deck-head"><span class="deck-name">Deck 1</span><span class="badge" data-role="state">Offline</span></div><div class="metrics"><div class="metric"><b data-role="rpm">0.00</b><span>RPM</span></div><div class="metric"><b data-role="signal">—</b><span>Signal</span></div><div class="metric"><b data-role="loss">—</b><span>Loss</span></div></div><div class="battery"><div class="battery-head"><span>Battery voltage</span><b data-role="battery">—</b></div><div class="battery-track" data-role="batteryTrack" role="progressbar" aria-label="Deck 1 battery voltage" aria-valuemin="3" aria-valuemax="4.2"><i class="battery-fill" data-role="batteryFill"></i></div><div class="battery-scale"><span>3.0 V</span><span>4.2 V</span></div></div><div class="calmsg" data-role="cal"></div></article>
<article class="card deck" id="deck2"><div class="deck-head"><span class="deck-name">Deck 2</span><span class="badge" data-role="state">Offline</span></div><div class="metrics"><div class="metric"><b data-role="rpm">0.00</b><span>RPM</span></div><div class="metric"><b data-role="signal">—</b><span>Signal</span></div><div class="metric"><b data-role="loss">—</b><span>Loss</span></div></div><div class="battery"><div class="battery-head"><span>Battery voltage</span><b data-role="battery">—</b></div><div class="battery-track" data-role="batteryTrack" role="progressbar" aria-label="Deck 2 battery voltage" aria-valuemin="3" aria-valuemax="4.2"><i class="battery-fill" data-role="batteryFill"></i></div><div class="battery-scale"><span>3.0 V</span><span>4.2 V</span></div></div><div class="calmsg" data-role="cal"></div></article>
</section>
<div class="section-title"><h2>Output & hardware</h2><span>Saved on receiver</span></div>
<form class="card settings" id="settingsForm">
<div class="field"><div class="field-head"><label for="fmt">Timecode format</label></div><select id="fmt"><option value="0">Serato CV02 · relative mode</option><option value="1">Traktor · carrier only</option></select></div>
<div class="field"><div class="field-head"><label for="gain">Output gain</label><output class="value" id="gv">0.30</output></div><input type="range" id="gain" min="0.05" max="0.60" step="0.01"><div class="hint">Stored separately per timecode format. Tune against the calibration scope; high levels may overload phono inputs.</div></div>
<div class="field"><div class="field-head"><label for="bri">Status LED brightness</label><output class="value" id="bv">40</output></div><input type="range" id="bri" min="5" max="255" step="5"></div>
<div class="field"><div class="field-head"><label for="pwin">Pairing window</label><output class="value" id="wv">60 s</output></div><input type="range" id="pwin" min="10" max="300" step="10"></div>
<div class="field"><div class="field-head"><label for="base">Calibration reference</label></div><select id="base"><option value="33.33">33⅓ RPM</option><option value="45">45 RPM</option></select><div class="hint">Calibrate with pitch at 0% and the platter fully settled.</div></div>
<div class="field"><div class="field-head"><label for="blow">Low-battery threshold</label><output class="value" id="blv">3.50 V</output></div><input type="range" id="blow" min="3.0" max="4.0" step="0.05"></div>
<div class="actions"><button class="btn primary" id="saveBtn" type="submit">Save settings</button><button class="btn" id="calBtn" type="button">Calibrate pucks</button><button class="btn danger" id="repairBtn" type="button">Clear pairings</button></div>
<div class="pairbar"><span>Pairing</span><strong id="pairState">—</strong></div>
</form></main><div id="toast" role="status" aria-live="polite"></div>
<script>
const $=id=>document.getElementById(id), fields=['gain','bri','pwin','blow'];
let toastTimer;
function toast(message,error=false){const t=$('toast');t.textContent=message;t.className=error?'show error':'show';clearTimeout(toastTimer);toastTimer=setTimeout(()=>t.className='',3200)}
async function api(path,options={}){const r=await fetch(path,{cache:'no-store',...options});const type=r.headers.get('content-type')||'';const body=type.includes('json')?await r.json():await r.text();if(!r.ok)throw new Error(body.error||body||('Request failed '+r.status));return body}
function syncValues(){$('gv').textContent=Number($('gain').value).toFixed(2);$('bv').textContent=$('bri').value;$('wv').textContent=$('pwin').value+' s';$('blv').textContent=Number($('blow').value).toFixed(2)+' V'}
fields.forEach(id=>$(id).addEventListener('input',syncValues));
async function loadConfig(){try{const c=await api('/api/config');$('gain').value=c.gain;$('fmt').value=c.fmt;$('bri').value=c.bri;$('pwin').value=c.pwin;$('base').value=c.base>40?45:33.33;$('blow').value=c.blow;syncValues()}catch(e){toast('Could not load settings: '+e.message,true)}}
function deckView(d,index){const card=$('deck'+index),get=r=>card.querySelector('[data-role='+r+']');const state=d.state||'offline',badge=get('state'),volts=Number(d.battery),known=Number.isFinite(volts)&&volts>0,fill=get('batteryFill'),track=get('batteryTrack');badge.textContent=state==='online'?'Online':state==='hold'?'Holding':'Offline';badge.className='badge '+state;get('rpm').textContent=Number(d.rpm||0).toFixed(2);get('signal').textContent=Number.isFinite(d.rssi)?d.rssi+' dBm':'—';get('loss').textContent=Number.isFinite(d.loss)?Number(d.loss).toFixed(1)+'%':'—';get('battery').textContent=known?(volts.toFixed(2)+' V'+(d.lowBattery?' · LOW':'')):'No reading';const level=known?Math.max(0,Math.min(100,(volts-3)/1.2*100)):0;fill.style.width=level+'%';fill.className='battery-fill '+(!known?'':d.lowBattery?'low':volts<3.65?'warn':'good');track.setAttribute('aria-valuenow',known?volts.toFixed(2):'3');track.setAttribute('aria-valuetext',known?volts.toFixed(2)+' volts':'No battery reading');get('cal').textContent=d.cal||(!d.assigned?'Waiting for a puck':'')}
async function poll(){try{const s=await api('/api/status');deckView(s.decks[0],1);deckView(s.decks[1],2);$('formatSummary').textContent=(s.fmt?'Traktor carrier':'Serato CV02')+' · gain '+Number(s.gain).toFixed(2);$('pairState').textContent=s.pairOpen?('Open · '+s.pairRemaining+' s left'):'Closed';$('connection').className='connect live';$('connection').querySelector('span').textContent='Live'}catch(e){$('connection').className='connect';$('connection').querySelector('span').textContent='Reconnecting'}setTimeout(poll,1000)}
$('settingsForm').addEventListener('submit',async e=>{e.preventDefault();const b=$('saveBtn');b.disabled=true;const q=new URLSearchParams({gain:$('gain').value,fmt:$('fmt').value,bri:$('bri').value,pwin:$('pwin').value,base:$('base').value,blow:$('blow').value});try{await api('/api/config?'+q,{method:'POST'});toast('Settings saved')}catch(x){toast(x.message,true)}finally{b.disabled=false}});
$('calBtn').addEventListener('click',async()=>{if(!confirm('Start calibration now? Both platters must be fully settled at the selected speed with pitch at 0%.'))return;try{await api('/api/cal',{method:'POST'});toast('Calibration armed — keep the platters steady')}catch(e){toast(e.message,true)}});
$('repairBtn').addEventListener('click',async()=>{if(!confirm('Clear both deck assignments and reopen pairing? Audio links will briefly drop.'))return;try{await api('/api/repair',{method:'POST'});toast('Pairings cleared — power the pucks now')}catch(e){toast(e.message,true)}});
loadConfig();poll();
</script></body></html>)HTML";

static void webDeckJson(uint8_t i, char *out, size_t n) {
  int16_t rpmCenti; uint32_t lastSeen; bool assigned;
  portENTER_CRITICAL(&stateMux);
  rpmCenti = deckStates[i].rpmCenti;
  lastSeen = deckStates[i].lastSeenMillis;
  assigned = deckStates[i].assigned;
  portEXIT_CRITICAL(&stateMux);
  uint32_t now = millis();
  bool online = lastSeen != 0 && now - lastSeen <= DECK_TIMEOUT_MS;
  const char *state = online ? "online" : (failHolding[i] ? "hold" : "offline");
  char loss[16], rssi[12];
  if (webLoss[i] >= 0.0f) snprintf(loss, sizeof(loss), "%.1f", webLoss[i]);
  else snprintf(loss, sizeof(loss), "null");
  if (webRssi[i] != 0) snprintf(rssi, sizeof(rssi), "%d", webRssi[i]);
  else snprintf(rssi, sizeof(rssi), "null");
  uint32_t age = lastSeen ? now - lastSeen : 0;
  snprintf(out, n,
           "{\"assigned\":%s,\"state\":\"%s\",\"rpm\":%.2f,\"loss\":%s,"
           "\"rssi\":%s,\"battery\":%.2f,\"lowBattery\":%s,\"age\":%lu,\"cal\":\"%s\"}",
           assigned ? "true" : "false", state, rpmCenti / 100.0f, loss, rssi,
           deckBattV[i], deckBattV[i] > 0.1f && deckBattV[i] < gBattLowV ? "true" : "false",
           (unsigned long)age, webCalMsg[i]);
}

// Serve the portal page itself (200) for every OS connectivity probe.
// Android wants a 204, Apple wants a page containing "Success", Windows
// wants a fixed string - answering ALL of them with our HTML page (none of
// those) is what makes the phone pop the "Sign in to network" sheet. This
// is more reliable across phones than a 302 redirect.
static void serveCaptivePortal() {
  settingsLastHitMs = millis();
  settingsServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  settingsServer.send_P(200, "text/html", SETTINGS_PAGE);
}

void setupSettingsServer() {
  settingsServer.on("/", HTTP_GET, []() {
    settingsLastHitMs = millis();
    settingsServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    settingsServer.send_P(200, "text/html", SETTINGS_PAGE);
  });
  settingsServer.on("/api/config", HTTP_GET, []() {
    settingsLastHitMs = millis();
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"gain\":%.2f,\"gains\":%.2f,\"gaint\":%.2f,"
             "\"fmt\":%u,\"bri\":%u,\"pwin\":%lu,\"base\":%.2f,\"blow\":%.2f}",
             (float)gGain, gGainSerato, gGainTraktor,
             (unsigned)gFormat, (unsigned)gBright,
             (unsigned long)(gPairWinMs / 1000UL), (float)gBaseRpm, (float)gBattLowV);
    settingsServer.send(200, "application/json", buf);
  });
  settingsServer.on("/api/config", HTTP_POST, []() {
    settingsLastHitMs = millis();
    bool valid = true;
    float newGain = gGain, newBase = gBaseRpm, newBattLow = gBattLowV;
    float newGainSerato = gGainSerato, newGainTraktor = gGainTraktor;
    uint8_t newFormat = gFormat, newBright = gBright;
    uint32_t newPairWin = gPairWinMs;
    bool hasExplicitGains = settingsServer.hasArg("gains") || settingsServer.hasArg("gaint");
    if (settingsServer.hasArg("gain")) {
      float v = settingsServer.arg("gain").toFloat();
      if (isfinite(v) && v >= GAIN_MIN && v <= GAIN_MAX) newGain = v; else valid = false;
    }
    if (settingsServer.hasArg("gains")) {
      float v = settingsServer.arg("gains").toFloat();
      if (isfinite(v) && v >= GAIN_MIN && v <= GAIN_MAX) newGainSerato = v; else valid = false;
    }
    if (settingsServer.hasArg("gaint")) {
      float v = settingsServer.arg("gaint").toFloat();
      if (isfinite(v) && v >= GAIN_MIN && v <= GAIN_MAX) newGainTraktor = v; else valid = false;
    }
    if (settingsServer.hasArg("fmt")) {
      int v = settingsServer.arg("fmt").toInt();
      if (v == FORMAT_SERATO || v == FORMAT_TRAKTOR) newFormat = (uint8_t)v; else valid = false;
    }
    if (settingsServer.hasArg("bri")) {
      int v = settingsServer.arg("bri").toInt();
      if (v >= 5 && v <= 255) newBright = (uint8_t)v; else valid = false;
    }
    if (settingsServer.hasArg("pwin")) {
      long v = settingsServer.arg("pwin").toInt();
      if (v >= 10 && v <= 300) newPairWin = (uint32_t)v * 1000UL; else valid = false;
    }
    if (settingsServer.hasArg("base")) {
      float v = settingsServer.arg("base").toFloat();
      if (isfinite(v) && v >= 30.0f && v <= 50.0f)
        newBase = v > 40.0f ? 45.0f : 33.3333f;
      else valid = false;
    }
    if (settingsServer.hasArg("blow")) {
      float v = settingsServer.arg("blow").toFloat();
      if (isfinite(v) && v >= 3.0f && v <= 4.0f) newBattLow = v; else valid = false;
    }
    if (!valid) {
      settingsServer.send(400, "application/json", "{\"error\":\"One or more settings are invalid\"}");
      return;
    }
    if (hasExplicitGains) {
      gGainSerato = newGainSerato;
      gGainTraktor = newGainTraktor;
      gFormat = newFormat;
      gGain = gFormat == FORMAT_TRAKTOR ? gGainTraktor : gGainSerato;
    } else {
      applyGainAndFormat(newGain, newFormat);
    }
    gBright = newBright;
    gPairWinMs = newPairWin;
    gBaseRpm = newBase;
    gBattLowV = newBattLow;
    led.setBrightness(gBright);
    lastLedPacked = 0xFFFFFFFFUL;
    saveSettings();
    settingsServer.send(200, "application/json", "{\"ok\":true}");
  });
  settingsServer.on("/api/status", HTTP_GET, []() {
    settingsLastHitMs = millis();
    char d1[260], d2[260], buf[640];
    webDeckJson(0, d1, sizeof(d1));
    webDeckJson(1, d2, sizeof(d2));
    uint32_t now = millis();
    bool pairOpen = timePending(now, pairingWindowEndMs);
    uint32_t pairRemaining = pairOpen ? (pairingWindowEndMs - now + 999UL) / 1000UL : 0;
    snprintf(buf, sizeof(buf),
             "{\"fmt\":%u,\"gain\":%.2f,\"pairOpen\":%s,\"pairRemaining\":%lu,"
             "\"decks\":[%s,%s]}",
             (unsigned)gFormat, (float)gGain, pairOpen ? "true" : "false",
             (unsigned long)pairRemaining, d1, d2);
    settingsServer.send(200, "application/json", buf);
  });
  settingsServer.on("/api/repair", HTTP_POST, []() {
    settingsLastHitMs = millis();
    clearPairingsAndReopen();
    settingsServer.send(200, "application/json", "{\"ok\":true}");
  });
  settingsServer.on("/api/cal", HTTP_POST, []() {
    settingsLastHitMs = millis();
    int16_t arg = (int16_t)lroundf((float)gBaseRpm * 100.0f);
    uint32_t now = millis();
    uint8_t armed = 0;
    for (uint8_t i = 0; i < 2; i++) {
      bool assigned;
      portENTER_CRITICAL(&stateMux);
      assigned = deckStates[i].assigned;
      portEXIT_CRITICAL(&stateMux);
      if (!assigned) continue;
      armed++;
      cmdArgCenti[i] = arg;
      cmdRepeatsLeft[i] = 5;      // burst; loop() spaces the sends out
      cmdNextMs[i] = now;
      snprintf(webCalMsg[i], sizeof(webCalMsg[0]), "cal: command sent...");
    }
    if (armed == 0)
      settingsServer.send(409, "application/json", "{\"error\":\"No paired pucks are available to calibrate\"}");
    else
      settingsServer.send(200, "application/json", "{\"ok\":true}");
  });
  // ---- Captive portal ------------------------------------------------
  // Explicit OS connectivity-probe URLs, plus a catch-all: all serve the
  // portal page with 200 (see serveCaptivePortal). Combined with the
  // catch-all DNS, that triggers the phone's captive-portal sheet.
  for (const char *probe : {
         "/generate_204", "/gen_204",                    // Android
         "/hotspot-detect.html", "/library/test/success.html",
         "/success.html", "/canonical.html",             // Apple
         "/connecttest.txt", "/ncsi.txt", "/fwlink",     // Windows
         "/redirect", "/chat" })                         // misc probes
    settingsServer.on(probe, HTTP_GET, serveCaptivePortal);
  settingsServer.onNotFound(serveCaptivePortal);
}

void startSettingsMode() {
  // AP must share the ESP-NOW channel so the pucks keep streaming.
  WiFi.mode(WIFI_AP_STA);
  // Pin the AP IP = gateway so DHCP hands the phone the ESP as its DNS
  // server; without this the phone may never send us the probe lookups.
  IPAddress apIP(192, 168, 4, 1);
  if (!WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)) ||
      !WiFi.softAP(SETTINGS_AP_SSID, SETTINGS_AP_PASS, ESPNOW_CHANNEL)) {
    debugBootLog("ERR SETTINGS: access point failed to start");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return;
  }
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Catch-all DNS: every hostname resolves to us (default settings, matching
  // known-good captive portals like WLED), so the phone's connectivity probe
  // reaches our web server and gets the 200 page above.
  if (!settingsDns.start(53, "*", WiFi.softAPIP())) {
    debugBootLog("ERR SETTINGS: captive DNS failed to start");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    return;
  }
  settingsServer.begin();
  settingsActive = true;
  settingsLastHitMs = millis();
#if DEBUG_SERIAL
  Serial.printf("SETTINGS: portal up - join \"%s\" (pass \"%s\"); the phone should\n"
                "  offer a sign-in page automatically, else open http://%s/\n",
                SETTINGS_AP_SSID, SETTINGS_AP_PASS,
                WiFi.softAPIP().toString().c_str());
#endif
}

void stopSettingsMode() {
  settingsServer.stop();
  settingsDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  settingsActive = false;
  lastLedPacked = 0xFFFFFFFFUL;        // leave the purple breathing cleanly
  debugBootLog("SETTINGS: portal closed");
}

void executeReceiverButtonAction(uint8_t action, uint8_t button) {
  char detail[96];
  switch (action) {
    case BUTTON_ACTION_DISABLED:
      return;
    case BUTTON_ACTION_AP:
      if (settingsActive) {
        stopSettingsMode();
        snprintf(detail, sizeof(detail), "button %u closed settings AP", button);
      } else if (gApFallback) {
        startSettingsMode();
        snprintf(detail, sizeof(detail), "button %u opened settings AP", button);
      } else {
        snprintf(detail, sizeof(detail), "button %u AP action blocked; enable emergency AP", button);
        debugBootLog(detail);
#if USB_MANAGER_ENABLE
        managerSendEvent("ap_disabled", 0, detail);
#endif
        return;
      }
      break;
    case BUTTON_ACTION_SWAP:
      snprintf(detail, sizeof(detail), swapDeckAssignments()
        ? "button %u swapped deck assignments"
        : "button %u swap requires two paired pucks", button);
      break;
    case BUTTON_ACTION_PAIR:
      pairingWindowEndMs = millis() + gPairWinMs;
      snprintf(detail, sizeof(detail), "button %u opened pairing window", button);
      break;
    case BUTTON_ACTION_SPINCAL: {
      uint8_t armed = managerArmCalibration();
      snprintf(detail, sizeof(detail), armed
        ? "button %u armed spin calibration at %.2f RPM"
        : "button %u calibration requires a paired puck", button, (double)gBaseRpm);
      break;
    }
    default:
      return;
  }
  debugBootLog(detail);
#if USB_MANAGER_ENABLE
  managerSendEvent("receiver_button", 0, detail);
#endif
}

void serviceMappedButton(uint8_t pin, uint8_t action, uint8_t button,
                         uint32_t &pressStart, bool &fired) {
  if (digitalRead(pin) != LOW) { pressStart = 0; fired = false; return; }
  uint32_t now = millis();
  if (pressStart == 0) { pressStart = now; return; }
  if (fired || now - pressStart < REPAIR_HOLD_MS) return;
  fired = true;
  executeReceiverButtonAction(action, button);
}

void serviceReceiverButtons() {
  static uint32_t button1Start = 0, button2Start = 0;
  static bool button1Fired = false, button2Fired = false;
  serviceMappedButton(REPAIR_BTN_PIN, gButton1Action, 1, button1Start, button1Fired);
  serviceMappedButton(SETTINGS_BTN_PIN, gButton2Action, 2, button2Start, button2Fired);
}

// ===== setup / loop ==================================================
void setup() {
  setupDebugSerial();
  loadSettings();
#if USB_MANAGER_ENABLE
  managerSendHello(0);
  managerSendConfig(0);
#endif
  pinMode(REPAIR_BTN_PIN, INPUT_PULLUP);
  pinMode(SETTINGS_BTN_PIN, INPUT_PULLUP);
  led.begin();
  led.setBrightness(gBright);
  serviceLed();                       // start red (no transmitters)
#if DEBUG_SERIAL
  Serial.printf("Format: %s  gain: %.2f (change via settings portal)\n",
                gFormat == FORMAT_TRAKTOR ? "Traktor carrier" : "Serato CV02",
                (float)gGain);
#endif

  debugBootLog("Building CV02 bit table...");
  buildCv02Bits();
  debugBootLog("CV02 bit table OK (Traktor is carrier-only, no table)");

  debugBootLog("Starting I2S outputs...");
  setupI2S(&audioDecks[0]);
  setupI2S(&audioDecks[1]);
  debugBootLog("I2S outputs OK");

  debugBootLog("Starting ESP-NOW...");
  setupEspNow();
  pairingWindowEndMs = millis() + gPairWinMs;   // open the pairing window
  debugBootLog("ESP-NOW OK - pairing window open (power the pucks now)");

  setupSettingsServer();              // routes only; AP starts on button press

  BaseType_t taskA = xTaskCreatePinnedToCore(audioTask, audioDecks[0].taskName,
                                             8192, &audioDecks[0], 3, NULL, 1);
  BaseType_t taskB = xTaskCreatePinnedToCore(audioTask, audioDecks[1].taskName,
                                             8192, &audioDecks[1], 3, NULL, 1);
  if (taskA != pdPASS || taskB != pdPASS) {
    debugBootLog("ERR audio task creation failed");
    fatalError(FATAL_AUDIO_TASK, "audio task creation failed - out of memory?");
  }
  debugBootLog("Audio tasks OK");

#if USB_MIDI_ACTIVE
  MIDI.begin();
  USB.begin();
  debugBootLog("USB-MIDI up (native USB port)");
#endif
}

void loop() {
  serviceManagerSerial();
  serviceReceiverButtons();
  serviceCommandRepeats();
  if (settingsActive) {
    settingsDns.processNextRequest();
    settingsServer.handleClient();
    if (millis() - settingsLastHitMs > SETTINGS_IDLE_TIMEOUT_MS)
      stopSettingsMode();             // nobody using it - drop the AP
  }
  serviceEspNowControl();
  serviceTransmitterEvents();
  serviceFailoverLog();
  serviceDicerMidi();
  if (millis() - lastLedMillis >= LED_UPDATE_MS) {
    lastLedMillis = millis();
    serviceLed();
  }
  debugPrintStatus();
  serviceManagerTelemetry();
  delay(1);
}
