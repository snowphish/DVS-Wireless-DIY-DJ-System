# DIY DVS revision notes — USB Manager release

## Web flasher and distribution refresh - 2026-07-31

- Restored the GitHub Pages ESP Web Tools flasher for the current three-build
  firmware layout: unified receiver, MPU6050 puck, and BMI160/BMI120 puck.
- Recompiled all three merged flash images with Arduino CLI 1.5.1, ESP32
  Arduino core 3.3.10, and Adafruit NeoPixel 1.15.5.
- Receiver build uses ESP32-S3 hardware CDC with USB CDC on boot; transmitter
  builds use ESP32-C3 with USB CDC on boot.
- Espressif image inspection confirmed valid chip IDs, checksums, and validation
  hashes for all three application images.
- Added a complete repository landing-page README and removed the obsolete
  original Manager executable from the current release tree.

## Firmware audit fixes - 2026-07-23

- Receiver firmware version advanced to 1.2.1.
- Deck LIVE/HOLDING freshness now advances only on motion DATA packets.
  Battery, handshake, calibration and IMU-fault events can no longer interrupt
  hold-last-stable-RPM failover.
- Both transmitter variants enter reported IMU fault code 3 after 50
  consecutive gyro-read failures, including failures during boot zero-cal.
- BMI160 PMU startup failure is now fatal and reported instead of continuing
  with a suspended gyro.
- BMI160 INT1 uses a pull-down so an older puck without the interrupt wire
  deterministically enters the bounded free-running fallback.
- Stored-bias cross-check rejection has its own event code and reports delta
  dps instead of mislabeling the value as platter RPM.
- Legacy `gain=` behavior is retained. Optional `gains=` and `gaint=` keys can
  explicitly update the Serato and Traktor gains; both values are emitted in
  manager and portal config JSON for future clients.
- ESP-NOW packet layout, protocol version and channel are unchanged.
- Windows DVS Manager source and executable are unchanged.
- Non-compiling structural, protocol, portal-JavaScript and semantic checks
  passed. Arduino compilation was intentionally not run.

## Windows DVS Manager 1.0

- Native Windows x64 WPF application matching the approved dashboard preview.
- Automatic receiver discovery using a protocol handshake instead of a fixed
  COM-port number.
- Automatic recovery after receiver reboot, USB removal, and COM renumbering.
- System-tray operation, optional start with Windows, and clean explicit exit.
- Prominent 3.0–4.2 V battery gauges plus RPM, RSSI, packet loss, packet age,
  pairing state, calibration state, and receiver firmware information.
- USB controls for timecode format, gain, base RPM, LED brightness, pairing
  duration, low-battery threshold, calibration, pairing, assignment reset, and
  emergency AP availability.
- Windows notifications for holding/restored links, low/critical/recovered
  battery, and completed calibration.
- Session activity log with CSV export.
- Receiver remains the source of truth for settings; closing the manager has no
  effect on timecode generation or failover.
- Single-file, self-contained Windows x64 build.

## Receiver USB protocol

- Versioned protocol over ESP32-S3 USB CDC at 115200 baud.
- Outgoing machine messages use line-delimited JSON with an `@DVS ` prefix.
- Incoming commands use bounded key/value tokens and require no JSON library on
  the microcontroller.
- Fixed 320-byte RX buffer with overlength rejection.
- USB input work is capped per `loop()` iteration.
- Telemetry is emitted every 250 ms from `loop()`, never from ESP-NOW callbacks
  or audio tasks.
- Human debug lines may remain enabled; the manager ignores unprefixed lines.
- Command IDs provide request/reply correlation.
- Settings are range-checked atomically before NVS writes.
- Emergency AP availability is now NVS-backed and manager-controlled.
- Link, battery, calibration, and AP-disabled events feed desktop notifications.

## Existing transmitter and receiver hardening retained

- Non-blocking ESP-NOW reconnect on both transmitter variants.
- BMI160 write verification and MPU6050 identity/configuration checks.
- Immediate low/critical battery transition reporting.
- Independent pending control/event slots for simultaneous pucks.
- Rollover-safe pairing and calibration deadlines.
- Traktor output clipping protection.
- Responsive offline settings portal with battery meters.
- Intentional indefinite hold-last-stable-RPM failover.

## Validation performed

- DVS Manager Release build: zero warnings and zero errors.
- Self-contained Windows x64 publish: successful.
- Executable startup smoke test: successful.
- Embedded USB JSON parser self-test: successful.
- Receiver/transmitter delimiter and preprocessor balance: successful.
- Embedded portal JavaScript syntax: successful.
- Exact portal HTML extraction: successful.
- Unified receiver Arduino compile: successful.
- MPU6050 and BMI160/BMI120 transmitter Arduino compiles: successful.
- Web-flasher manifests, asset paths, JavaScript, and image headers: successful.

## Required bench test

1. Confirm USB CDC enumerates and DVS Manager detects the receiver.
2. Confirm config round-trips and persists across receiver reboot.
3. Confirm both pucks pair, report battery, and calibrate.
4. Confirm live RPM direction and audio output on both decks.
5. Interrupt each radio link and verify HOLDING plus restoration notifications.
6. Verify the DJ software's Internal → reconnect → Relative recovery workflow.
7. Verify AP fallback enabled and disabled behavior.
8. Test Serato and Traktor at conservative and intended output levels.
