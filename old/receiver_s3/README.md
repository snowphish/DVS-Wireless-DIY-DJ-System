# receiver_s3 — dual timecode generator, compatible with Serato CV02

> **Lineage:** based on [FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System)
> by Felipe Alme, who started this project.

ESP32-S3 receiver: takes RPM from up to two transmitter pucks over ESP-NOW
and generates timecode **compatible with Serato CV02** into two stereo I2S
DACs in real time — speed, direction, backspins and all. Use with Serato,
Mixxx, or anything else that decodes Serato 2nd Edition vinyl.

## Hardware

Board: ESP32-S3 DevKitC-1 (tested on N16R8). Two PCM5102A I2S DAC boards,
one per deck:

| DAC | BCK | LRCK | DATA |
|---|---|---|---|
| A (deck 1) | GPIO 2 | GPIO 1 | GPIO 42 |
| B (deck 2) | GPIO 14 | GPIO 13 | GPIO 12 |

Onboard WS2812 RGB LED on **GPIO 48** (change `LED_PIN` if your board
differs): **RED** = no transmitters, **ORANGE** = one online, **GREEN** =
both online.

DAC line outputs go to the sound card inputs your DJ software expects
control vinyl on. In the software select **Serato 2nd Ed. / CV02** vinyl,
**relative mode** (absolute position is synthetic — there is no needle).

## Flashing

- ESP32 Arduino core 3.x, **Adafruit NeoPixel** library.
- `ESPNOW_CHANNEL` (11) and the `dvs_packet` struct must match the
  transmitters.
- At boot the Serial Monitor (115200) prints the board's MAC in
  copy-paste form for the transmitter's `receiverMAC[]`.

## Timecode

Generates a control signal compatible with **Serato CV02** vinyl. Per deck,
a high-resolution phase accumulator advances by a per-buffer step derived
from the smoothed RPM, giving effectively continuous pitch and bit-exact,
glitch-free reverse and backspin.

Audio runs at 44.1 kHz / 16-bit, one FreeRTOS task per deck pinned to core
1, DMA 4 × 32 samples ≈ **2.9 ms output latency**. A deck with no packets
for `DECK_TIMEOUT_MS` (1 s) glides to silence via the RPM smoothing.

## Tuning constants

| Constant | Default | Meaning |
|---|---|---|
| `DEADZONE_RPM` | 0.04 | RPM below which output freezes (transmitter's own 0.2 dominates) |
| `MAX_RPM_RATIO` | 8.0 | Speed clamp (8× = 266 RPM, fast backspins) |
| `RPM_SMOOTHING` | 0.80 | Per-buffer smoothing toward target RPM (~2 ms settle) |
| `OUTPUT_GAIN` | 0.70 | Output level |
| `DEBUG_SERIAL` | 1 | 0 = compile out every serial print |

## Serial debug output

Boot: banner, per-stage progress, per-DAC pin report, receiver MAC.
Runtime (every 500 ms):

```
D1 ON  rpm=  33.33 seq=123456 rx=123401 lost=0 age=2ms  D2 OFF rpm=   0.00 ... heap=298400
```

`lost` is packets missed since the previous one (from `seq` gaps) — a
persistently nonzero value means radio congestion or range problems.
`WELCOME deck N -> MAC` lines mark transmitter handshakes.

Transmitter status events (`MSG_EVENT`) are relayed here too, since the
puck's own USB is unreachable while it spins on battery — this is where you
watch the spin-calibration ritual:

```
EVT deck 1: NO stored trim - spin platter at 0% pitch to calibrate
EVT deck 1: spin-cal reference 33.33 settled, sampling ~10 s - don't touch
EVT deck 1: spin-cal LOCKED, trim=0.99810 (saved)
EVT deck 1: using stored trim=0.99810        (normal boots)
```

All printing runs in the low-priority `loop()` task and copies state under
a critical section, so it can't disturb the radio callback or audio.

## Notes

- Both audio tasks are priority 3 on core 1; WiFi/ESP-NOW stays on core 0.
- ESP-NOW control replies (WELCOME/PING) are sent from `loop()`, never from
  the receive callback.
- `use_apll` is a no-op on the S3 (no APLL); the fractional clock divider
  still hits 44.1 kHz within ~0.002%, identically for both DACs.
