# Wireless DIY DVS

## Lineage and AI authorship disclosure

This is my own spin on the original project from
[FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System),
which established the original transmitter and receiver system. This fork
includes some added functionality like unified receiver control, spin
calibration, automatic pairing, failover, telemetry, the Windows Manager, and
web flashing.

**All programming in this fork has been done by AI.** The project owner has
provided the requirements, hardware design decisions, physical builds, testing,
diagnosis feedback, and final approval, while AI systems generated and modified
the firmware, desktop application, web flasher, build tooling, and documentation.

Wireless DIY DVS turns two ordinary turntables into a two-deck digital vinyl
control system without needles or timecode records. Battery-powered ESP32-C3
gyro pucks measure platter movement over ESP-NOW, while one ESP32-S3 receiver
generates line-level Serato CV02 timecode or a Traktor-compatible quadrature
carrier signal for both decks.

> **[Flash the receiver and pucks in Chrome or Edge](https://snowphish.github.io/DVS-Wireless-DIY-DJ-System/)**

The package also includes a self-contained Windows x64 manager for monitoring,
pairing, calibration, deck swapping, and receiver settings over USB.

## System overview

```text
Turntable 1 + gyro puck --\                         /--> DAC A --> audio interface
                            ESP-NOW --> S3 receiver
Turntable 2 + gyro puck --/                         \--> DAC B --> audio interface
                                         |
                                         +--> USB --> DVS Manager for Windows
```

One receiver handles both decks. Both pucks run the same firmware for their
sensor type; deck 1 and deck 2 are assigned automatically during pairing and
can be swapped later from DVS Manager.

## Highlights

- Full Serato CV02 output and carrier-only Traktor output from one unified
  receiver build.
- Two simultaneous decks over low-latency ESP-NOW on channel 11.
- MPU6050 and BMI160/BMI120 transmitter variants.
- Automatic MAC-based pairing and reconnect to the puck's previous deck.
- One-time platter-speed calibration stored independently by each puck.
- Hold-last-stable-RPM failover if a wireless link drops.
- Live battery voltage, RPM, 7-second signal/loss averages, and packet age.
- USB configuration with automatic COM-port discovery and reconnection.
- Adjustable output gain, base speed, LED brightness, battery threshold, and
  pairing window, including an option to suppress low-battery LED flashing.
- Browser-based flashing with no Arduino installation required.

## Quick start

1. Open the **[web flasher](https://snowphish.github.io/DVS-Wireless-DIY-DJ-System/)**
   in desktop Chrome or Edge.
2. Flash **Unified receiver** to the ESP32-S3 receiver.
3. Flash the transmitter build matching each puck's gyro:
   - `MPU6050` for MPU6050 pucks.
   - `BMI160 / BMI120` for either Bosch chip ID `0xD1` or `0xD3`.
4. Power the receiver first, then switch on the two pucks one at a time. The
   first new puck becomes deck 1 and the second becomes deck 2.
5. Connect the receiver's native USB data port to Windows and run
   [`Windows_Manager/DVSManager.exe`](Windows_Manager/DVSManager.exe).
6. Select Serato CV02 or Traktor carrier in **Settings**, choose the platter
   speed in **Quick controls**, and calibrate with the platters settled at 0%
   pitch.
7. Connect both PCM5102A outputs to the audio interface's **line inputs**, set
   up the corresponding input mode in the DJ software, and use **Relative
   mode**.

### Important Traktor limitation

The current Traktor mode outputs only the bare quadrature carrier used for
speed and direction tracking. It does **not** generate the complete Traktor
Scratch MK2 position/authenticity bitstream, so Traktor will not detect it as
genuine timecode for now.

Use **Relative mode** and disable Traktor's timecode error messages/warnings.
With those errors disabled, Traktor can follow platter speed and direction from
the carrier, but genuine-timecode detection and absolute position are not
available. Serato mode is unaffected and continues to output full CV02
timecode.

The Windows executable is not code-signed, so SmartScreen may show an
unrecognized-app warning. Verify it against [`SHA256SUMS.txt`](SHA256SUMS.txt)
before choosing **More info > Run anyway**.

## Pairing and deck assignment

- The receiver opens its pairing window at boot.
- The first unassigned puck becomes deck 1; the second becomes deck 2.
- A previously paired puck that reconnects reclaims its own deck.
- **Open pairing** in DVS Manager allows a new puck to claim a free deck.
- **Swap decks** exchanges the two assignments without reflashing either puck.
- Holding the receiver's GPIO 4 pairing button for about 1.5 seconds clears
  both assignments and reopens pairing.

## Calibration

Power each puck on while its platter is stopped so boot-time gyro zeroing is
valid. For the one-time speed calibration:

1. Set both turntables to 33 1/3 or 45 RPM with pitch at exactly 0%.
2. Let the platters settle completely.
3. Select the same base speed under **Quick controls**.
4. Click **Calibrate pucks at selected speed**.
5. Do not touch either platter during the roughly 10-second sample.

The learned gyro scale is saved in each puck. Normal power-on zeroing still
happens every time because gyro bias changes with temperature.

## DVS Manager

`DVSManager.exe` is a portable, self-contained .NET 8 application for 64-bit
Windows. No installer or separate .NET runtime is required. It automatically
finds the receiver, reconnects after USB resets or COM-number changes, and
keeps receiver settings in the receiver's NVS rather than on the PC.

Closing the window sends it to the system tray. System balloon notifications
are intentionally disabled so disconnects do not produce notification sounds;
status changes remain visible in the app's Activity view.

If a deck enters **HOLDING**, switch that deck to Internal mode, restore the
puck connection, wait for live RPM to return, then switch back to Relative.

## Repository layout

| Path | Contents |
|---|---|
| [`Firmware/receiver_s3_unified_revised`](Firmware/receiver_s3_unified_revised) | ESP32-S3 two-deck receiver firmware |
| [`Firmware/transmitter_c3_mpu6050_revised`](Firmware/transmitter_c3_mpu6050_revised) | ESP32-C3 MPU6050 puck firmware |
| [`Firmware/transmitter_bmi160_revised`](Firmware/transmitter_bmi160_revised) | ESP32-C3 BMI160/BMI120 puck firmware |
| [`Windows_Manager`](Windows_Manager) | Ready-to-run Windows Manager and USB protocol |
| [`Source/DVSManager`](Source/DVSManager) | Complete .NET 8 WPF source |
| [`docs`](docs) | GitHub Pages web flasher, manifests, and merged binaries |
| [`Old`](Old) | Printable enclosure files, Fusion 360 sources, and original build photos |
| [`licenses`](licenses) | Included third-party license notices |

## CAD cases and original build photos

[`Old/`](Old) preserves the original non-custom-PCB enclosure
designs and build photos. It includes print-ready `.3mf` receiver, transmitter,
and carry-box parts plus the editable Fusion 360 `.f3d` sources. Read the
folder's assembly notes before printing or modifying the designs.

## Firmware build settings

The checked-in web-flasher images were built with Arduino CLI 1.5.1, ESP32
Arduino core 3.3.10, a 4 MB default partition layout, and Adafruit NeoPixel
1.15.5 for the receiver.

| Firmware | Board/options |
|---|---|
| Receiver | `ESP32S3 Dev Module`, Hardware CDC/JTAG, USB CDC on boot, PSRAM disabled |
| Transmitters | `ESP32C3 Dev Module`, USB CDC on boot |

All receiver and transmitter builds must use ESP-NOW channel 11 and protocol
version 1. If either is changed, every board must be rebuilt together.

## Important electrical notes

- PCM5102A output is line level and can be much hotter than a phono cartridge.
  Use line inputs or suitable attenuation; do not feed an unattenuated output
  into a phono preamp.
- The puck battery monitor expects a 2:1 divider: switched BAT+ through 100k to
  GPIO 3, then 100k to ground, with 100 nF from GPIO 3 to ground.
- Current transmitter I2C pins are SDA GPIO 8 and SCL GPIO 9.
- The receiver DAC pins and hardware assumptions are documented at the top of
  the receiver sketch; confirm them against your board before powering it.

Bench-test direction, stop stability, calibration, both DAC outputs, link-loss
recovery, battery readings, and the selected Serato/Traktor mode before live use.

## Trademarks

Serato and Traktor are trademarks of their respective owners. This project is
not affiliated with or endorsed by Serato or Native Instruments.
