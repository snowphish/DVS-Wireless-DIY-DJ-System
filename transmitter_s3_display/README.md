# transmitter_s3_display — Waveshare ESP32-S3-LCD-1.69 puck (WIP)

All-in-one transmitter on the **Waveshare ESP32-S3-LCD-1.69**: ESP32-S3R8
with a built-in 1.69" 240×280 IPS LCD, QMI8658 IMU, battery charger, touch,
buzzer, RTC — replacing the bare C3 + separate gyro + separate display.

## Confirmed hardware

| Part | Chip | Bus |
|---|---|---|
| MCU | ESP32-S3R8 (8 MB PSRAM, 16 MB flash) | — |
| Display | ST7789V2, 240×280 IPS | SPI |
| IMU (gyro) | **QMI8658** 6-axis | I²C |
| Touch (touch version) | CST816T | I²C |
| RTC | PCF85063 | I²C |
| Charger | ETA6098 (charge only — no fuel gauge) | — |
| Battery sense | voltage divider → **GPIO1** (ADC) | — |
| Buzzer | **GPIO33** (pull LOW when idle or it heats!) | — |

**Pins below are from Waveshare's demo — VERIFY against the wiki/demo before
trusting** (the display/touch pins especially). Wrong pins just mean no
display/touch, nothing dangerous.

```
LCD (ST7789 SPI):  MOSI 7  SCLK 6  CS 5  DC 4  RST 8  BL 15
I²C bus (QMI8658 + CST816T + PCF85063):  SDA 11  SCL 10
Touch CST816T:  INT 14  RST 13   (addr 0x15)
QMI8658 addr:  0x6B (or 0x6A) — boot WHO_AM_I check auto-detects
Battery ADC:  GPIO1     Buzzer: GPIO33     BOOT button: GPIO0
```

## Staged build (one verified layer at a time)

**Stage 1 — core (this sketch now):** QMI8658 gyro → RPM, ESP-NOW at 500 Hz,
spin-cal learn-once, battery on GPIO1, boot diagnostics. Compiles with **no
extra libraries** (Wire + ESP-NOW only). Display and touch are stubbed.
Goal: confirm the QMI8658 reads and RPM tracks on the new board before
adding UI. Watch the WHO_AM_I result at boot first.

**Stage 2 — display (ST7789) — DONE (needs on-board visual check):**
Status screen via **LovyanGFX** (flicker-free through a 240x280 PSRAM
sprite): deck ID, battery voltage + colour bar, link status, calibration
hint. Drawn **only when the platter is stopped** (`|rpm| < DISP_MAX_RPM`) so
SPI never disturbs the 500 Hz send loop during play; also shows boot
progress ("gyro init", "waiting for receiver", "zeroing gyro").
- Verify on the board: if the image is shifted, tweak `offset_y` (try 0/20);
  if colours are inverted/wrong, flip `invert` or `rgb_order` in the `LGFX`
  class. Set `DISPLAY_ENABLE 0` to build without the screen.

**Stage 3 — touch cue points — DONE (untested on hardware):** the face is
a **2×2 cue pad** (fixed quadrants TL/TR/BL/BR = cues 1-4), drawn on screen
with a status strip on top. Touch (CST816T) → `MSG_BUTTONS` unicast +
heartbeat → receiver → USB-MIDI, **note = 60 + (deck-1)*4 + quadrant**
(60-63 deck 1, 64-67 deck 2; distinct from the dicer's 36-53). The puck is
now a transmitter *and* a controller in one.
- The tiles are static, so they only redraw on a press (a brief SPI blip),
  which is why they can stay shown while spinning. If cue taps cause
  audible glitches during play, the next step is moving the sprite push to
  core 0 with a Wire mutex.
- **This feature's receiver changes are self-contained here:** the deck-cue
  versions of both receivers live in `receiver_s3/` and
  `receiver_s3_traktor/` *inside this folder*. The main `test/receiver_s3*`
  are kept clean (dicer + failover + battery only) so this uncertain feature
  stays isolated. To try the cue MIDI, flash the receivers from *this*
  folder; otherwise the mainline ones are untouched.
- **Verify on hardware:** touch INT/RST pins (14/13) and addr 0x15 are from
  the Waveshare demo; if taps don't register, that's the first check. Also
  confirm the quadrant→cue mapping feels right (rotate the board 180° and
  the physical quadrants swap — expected).

## QMI8658 notes (stage 1)

- WHO_AM_I (0x00) = 0x05. Gyro Z data at 0x3F/0x40, little-endian.
- Config used: ±1024 dps (`CTRL3=0x63`), ~896 Hz ODR, gyro-only enable
  (`CTRL7=0x02`). Sensitivity 32768/1024 = **32.0 LSB/dps**.
- Mount so the QMI8658's Z axis is vertical (board flat on platter). RPM
  sign may need flipping depending on which way up the board sits — the
  `-(dps/6)` in `rawGyroToNominalRPM` is the knob.
- Spin-cal learn-once + battery EVT relay carried over from the puck
  firmware unchanged.
