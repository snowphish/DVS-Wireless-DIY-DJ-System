# Wireless Gyro DVS

Most of this project was written by Claude Code / Fable 5

## Lineage

Based on [FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System) —
Felipe Alme started this project and built the original transmitter/receiver
firmware this system grew from. This fork reworks it substantially: precision
and latency fixes, self-learning calibration, MAC-based auto-pairing, per-deck
link-loss failover, low-battery indication, and a redone Serato/Traktor
timecode engine that stays out of the end-of-record zone that used to trigger
INTERNAL mode.

A DIY digital vinyl system with **no needle and no timecode record**. A
battery-powered gyro puck sits on the turntable platter and streams its
rotation speed over ESP-NOW to a receiver, which generates timecode
**compatible with Serato CV02 or Traktor Scratch MK2** into the DJ software's
audio interface, depending on which receiver sketch you flash.

> ⚡ **Flash your board in-browser — no install:**
> **<https://snowphish.github.io/DVS-Wireless-DIY-DJ-System/>**
> (Chrome or Edge on desktop; pick your Transmitter gyro variant and Serato or Traktor receiver.)

```
 platter                       2.4 GHz                        line out
┌─────────────────┐   ESP-NOW broadcast, 500 Hz   ┌──────────────────────┐
│ transmitter_c3_ │ ────────────────────────────► │ receiver_s3_cv02     │ ──► deck A
│ mpu6050 or _bmi │                               │   or                 │ ──► deck B
│ ESP32-C3        │   (second puck = deck 2       │ receiver_s3_traktor  │
│ + gyro          │    just by turning it on)     │                      │
│ + battery       │ ◄──────────────────────────── │ ESP32-S3 + 2x        │
└─────────────────┘      WELCOME / PING           │ PCM5102A I2S DACs    │
                                                  └──────────────────────┘
```

## Repository layout

| Path | What it is |
|---|---|
| [transmitter_c3_mpu6050/](transmitter_c3_mpu6050/) | Platter puck: ESP32-C3 + MPU6050 gyro, 500 Hz RPM stream |
| [transmitter_bmi160/](transmitter_bmi160/) | Same puck with a Bosch BMI160 gyro (newer PCBs use this) |
| [receiver_s3_cv02/](receiver_s3_cv02/) | Receiver generating timecode compatible with **Serato CV02** (Serato, Mixxx) |
| [receiver_s3_traktor/](receiver_s3_traktor/) | Receiver generating timecode compatible with **Traktor Scratch MK2** |
| [docs/](docs/) | Web flasher (GitHub Pages: manifests + merged binaries) |
| [Experimental/](Experimental/) | Non-shipping sketches: S3-display puck, wireless dicer, CYD |
| [old/](old/) | Pre-restructure sketches, kept for reference |
| [CAD Cases/](CAD%20Cases/) | 3D-printed enclosures (Fusion 360 sources + .3mf) and build photos |

One receiver drives **two decks** (two DACs); each deck needs its own
transmitter puck. Pucks are interchangeable — none of them carry a deck
number or the receiver's MAC. All boards run the same ESP-NOW protocol, so
transmitters don't care which receiver sketch is flashed.

## Quick start

1. **Flash the receiver** (`receiver_s3_cv02` for Serato/Mixxx, or
   `receiver_s3_traktor` for Traktor). ESP32 Arduino core 3.x, needs the
   **Adafruit NeoPixel** library.
2. **Flash the same puck sketch** to both platter units — pick the one
   matching your gyro chip (`transmitter_c3_mpu6050` or `transmitter_bmi160`).
   No per-unit edits, no MAC to paste. All pucks flash identical.
3. **Pair by power order** (see below).
4. **Calibrate** (once per puck, ever): with the platter **stopped**, power
   the puck (gyro zero at boot). Then start the platter at **0% pitch** and
   leave it alone ~15 s — the puck settles, averages 10 s against the quartz
   lock, and stores its gyro scale trim in flash. Watch it on the
   **receiver's** Serial Monitor (`EVT deck 1: spin-cal LOCKED, trim=...`);
   the puck's own USB is unreachable while it spins.
   See [README-features.md](README-features.md).
5. Connect the receiver's DAC outputs to the sound card **line-level** inputs
   your DJ software expects timecode on (**not phono** — see below), select
   the matching control vinyl type (Serato 2nd Ed. / Traktor Vinyl MK2),
   **relative mode**, and play.

## Pairing: power the receiver first, then the pucks

- Power the receiver. Its LED **blinks blue** — the pairing window is open
  for ~60 s.
- Power your first puck: LED goes **orange**, receiver serial prints
  `WELCOME deck 1`, and that puck is now deck 1 for the session.
- Power your second puck: LED goes **solid green** and the window closes
  immediately — no third unit can slip in.

The receiver holds a `MAC → deck` table in RAM. That means:

- **A puck that drops (battery dies, switch bump, wanders out of range) and
  comes back reclaims its own deck automatically** — window or no window.
  So a mid-set battery swap does not require re-pairing.
- To swap pucks or redo pairing without a power cycle, **long-press the
  re-pair button (GPIO 4) for ~1.5 s** — the table clears, the window
  reopens, and the LED goes back to blue blinking. Rebooting the receiver
  does the same thing.

## Everyday use

- Power the puck on with the platter **stopped** — it re-zeroes its gyro at
  every boot (temperature drift). Power-cycle = re-zero.
- Receiver LED: **BLUE blinking** = pairing window open, **RED** = no
  transmitters and window closed, **ORANGE** = one deck up, **GREEN** = both
  decks up. Low-battery from a puck modulates the LED (blink patterns —
  see [README-features.md](README-features.md)).
- Both receivers print per-deck link health on the Serial Monitor every 500
  ms — the `loss` percentage and `rssi` (average / worst, dBm) tell you how
  clean the radio is at the current puck position. See "Serial output" below.
- Line inputs, **not phono**. The DAC output is line-level (~1 Vrms at the
  0.30 output gain shipping default); phono inputs expect ~5 mVrms and add
  RIAA EQ, so plugging in there clips and warps the timecode. Serato in
  particular will drop to internal mode intermittently on a clipped signal.
- **Relative mode only** in the DJ software. There is no needle, so absolute
  position is synthetic — meaningless at best. Both receivers loop a short
  mid-record window on purpose to stay clear of the run-out zone that would
  otherwise trigger INTERNAL mode.

## Serial output (receiver debug)

Every 500 ms, per deck:

```
D1 ON  rpm=  33.33 loss= 2.1% rssi=-56/-64 age=1ms  D2 ON  rpm=... pair=closed
```

| Field | Meaning |
|---|---|
| `ON` / `OFF` / `HLD` | Deck live / silent / failover-holding last stable rpm |
| `rpm` | Platter speed (negative = reverse) |
| `loss` | % of the puck's packets that didn't arrive **this 500 ms window** |
| `rssi` | Signal strength this window, dBm, as **average / worst** |
| `age` | Milliseconds since the last packet |
| `pair` | Pairing window state (`OPEN` blinks blue, `closed` = normal) |

Rules of thumb for `rssi`: −50s = strong, −60s = comfortable, −70s = marginal.
If a session shows steady loss > a few percent, the fixes in order are: raise
the puck off the metal platter (a few mm makes a big difference), change
`ESPNOW_CHANNEL` on all four sketches together, then move the receiver
closer.

## Design decisions (and why)

- **MAC-based auto-pairing** → pucks are interchangeable and carry zero
  configuration. Same binary flashes to every puck; the receiver assigns
  decks by MAC on a first-come basis and remembers them in RAM.
- **Broadcast ESP-NOW DATA, no ACK/retry** → constant, low radio jitter.
- **6 Mbps OFDM instead of the 1 Mbps default** (`ESPNOW_FAST_RATE`) →
  ~3× less airtime per packet than the 1 Mbps default, ~15 dB more receive
  sensitivity than the 24 Mbps rate the fork briefly ran on. Two decks at
  500 Hz coexist comfortably even in noisy rooms.
- **Max TX power both directions** → the puck→receiver signal has a metal
  platter and turntable motor sitting right under the antenna; every dB of
  margin helps. Bench-measured 20 dBm gave the best loss numbers.
- **int64 phase accumulators with 32 fractional bits, rounded steps** →
  pitch granularity ~10 µHz, glitch-free reverse and backspin.
- **Small I2S DMA buffers** (4 × 32 samples) → ~2.9 ms output latency.
- **Gyro DLPF at 188 Hz** (MPU6050) / **BW ≈ 230 Hz** (BMI160), gyro range
  ±1000 dps, send rate 500 Hz → end-to-end control latency ≈ 8 ms worst
  case (gyro filter ~2 + packet interval 2 + receiver smoothing 1 + DMA 2.9).
- **Two-point gyro calibration**: zero-offset from the stopped platter at
  every boot (bias drifts with temperature), scale factor learned once from
  the quartz-locked platter and persisted (silicon constant). No per-unit
  firmware edits.
- **CV02 & Traktor loop a mid-record window, not the full sequence** → the
  full 12-minute CV02 sequence and the ~70-minute Traktor sequence both
  include lead-in and run-out zones that make DJ software drop to
  INTERNAL. Both receivers now generate a 5-minute window that starts 30 s
  past the lead-in and wraps, invisible in relative mode.
- **Link-loss failover with stable-value hold**: if a puck goes silent
  mid-set the receiver keeps generating timecode at the puck's last
  *stable* RPM — a scratch flick will never be held, but a steady groove
  speed or a stopped platter will. See [README-features.md](README-features.md).

## Troubleshooting

| Symptom | Check |
|---|---|
| Receiver LED stays red (never blue) | Boot line at 115200 present? `LED_PIN`/NeoPixel wiring? |
| Receiver blue, but pucks don't pair | All four sketches on the same `ESPNOW_CHANNEL` and `dvs_packet` layout? Pucks powered *after* the receiver? |
| Serato randomly flips to INTERNAL | Line inputs (not phono)? Threshold re-Estimated after installing the new binaries? Signal ring round and inside the scope view? |
| Pitch reads slightly off at 0% | Redo spin-cal at 0% pitch (`SPIN_CAL_FORGET 1`, flash once, flash back, redo the ritual) |
| Speed wrong by a large factor | Gyro range / LSB mismatch — the pair (`GYR_RANGE_SEL` + `GYRO_LSB_PER_DPS`, or MPU6050 equivalents) must be changed together |
| Hiccups on hard scratch flicks | Gyro may clip at ±1000 dps (166 rpm). Raise the range one step |
| High `loss` (>5 %) in the receiver serial | Raise the puck a few mm off the metal platter first. Then try a different `ESPNOW_CHANNEL` (**change all four sketches**). Move the receiver away from PC cases / access points |
| Traktor drops to internal play mode | Shouldn't happen — if it does, raise `TRAKTOR_LOOP_START_SECONDS`, see receiver_s3_traktor README |
