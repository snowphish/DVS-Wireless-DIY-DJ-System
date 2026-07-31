# DIY DVS complete package

This package contains the two-deck ESP-NOW DVS firmware plus the new Windows
DVS Manager. The manager uses the receiver's existing USB connection, so normal
configuration and monitoring no longer require Wi-Fi AP mode.

## Package contents

| Folder | Purpose |
|---|---|
| `Windows_Manager` | Self-contained Windows x64 application and USB protocol |
| `Firmware/receiver_s3_unified_revised` | ESP32-S3 receiver with USB Manager support |
| `Firmware/transmitter_bmi160_revised` | ESP32-C3 puck with Bosch BMI160 |
| `Firmware/transmitter_c3_mpu6050_revised` | ESP32-C3 puck with MPU6050 |
| `Source/DVSManager` | Complete .NET 8 WPF application source |
| `docs` | GitHub Pages web flasher, manifests, compiled images, and revision notes |

Use only the transmitter sketch matching the IMU fitted to that puck.

## Recommended setup order

1. Use the web flasher linked from `README.md`, or flash
   `receiver_s3_unified_revised.ino` in Arduino IDE.
2. Flash the BMI160/BMI120 or MPU6050 build to each matching puck.
3. Power the receiver first, then the pucks one at a time.
4. Connect the receiver's native USB data port to the Windows PC.
5. Run `Windows_Manager/DVSManager.exe`.
6. Calibrate both pucks from the manager after the platters settle at the
   selected speed and 0% pitch.

## Receiver Arduino settings

- Board: `ESP32S3 Dev Module`, adjusted for the exact S3 module and flash size.
- ESP32 Arduino core: a recent 3.x release.
- **USB CDC On Boot: Enabled** so the receiver appears as a Windows COM port.
- USB MIDI remains disabled in this production receiver.
- Library: `Adafruit NeoPixel`.

Retain the known-good partition, flash, PSRAM, and upload settings for the
physical receiver board.

## Transmitter Arduino settings

- Board: `ESP32C3 Dev Module`, adjusted for the exact puck board.
- ESP32 Arduino core: the same recent 3.x release used by the receiver.
- Power each puck with its platter stopped so gyro-zero calibration is valid.

The receiver and transmitters must all retain ESP-NOW channel `11` and radio
protocol version `1`.

## DVS Manager

`DVSManager.exe` is self-contained for 64-bit Windows. It does not require an
installer or a separate .NET runtime.

The app automatically:

- Finds the receiver on a COM port.
- Reconnects after USB removal, board resets, or COM-number changes.
- Shows both battery gauges and live radio/link diagnostics.
- Stores settings directly in receiver NVS.
- Runs in the system tray and optionally starts with Windows.
- Notifies on link loss/restoration and low/critical battery.

The executable is not code-signed, so Windows SmartScreen may display an
unrecognized-app warning on first launch. Verify `SHA256SUMS.txt` before using
**More info → Run anyway**.

## Intentional failover behavior

Link loss continues to hold the last stable RPM indefinitely. When DVS Manager
shows **HOLDING**:

1. Switch the affected deck to Internal.
2. Reconnect the puck.
3. Wait for **link restored**.
4. Return the deck to Relative.

## Emergency AP

The original web portal remains an optional recovery path. DVS Manager can
disable the receiver button's AP fallback entirely. AP mode is never needed for
normal Windows management.

## Calibration

1. Select 33⅓ or 45 RPM in DVS Manager.
2. Set both turntables to that speed and pitch to exactly 0%.
3. Wait for both platters to settle fully.
4. Press **Calibrate** and confirm.
5. Do not touch the platters during the approximately 10-second sampling period.

Each puck stores its learned gyro trim independently.

## Electrical warning

PCM5102A output can be much hotter than a phono cartridge. Use the intended
line-level input or suitable attenuation. Do not feed an unattenuated DAC
output into a phono preamp unless the hardware was designed for it.

## Before performance use

- Verify forward/reverse direction and stopped-platter stability on both decks.
- Verify Serato and Traktor at conservative gain first.
- Test USB unplug/reconnect and manager auto-reconnection.
- Test intentional failover and the Internal → reconnect → Relative workflow.
- Test low-battery indication, pairing reset, and calibration.
- Keep the emergency AP enabled until the USB workflow has been bench-tested.
