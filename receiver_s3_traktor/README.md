# receiver_s3_traktor — dual Traktor Scratch MK2 timecode generator

> **Lineage:** based on [FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System)
> by Felipe Alme, who started this project.

Same hardware and radio protocol as [receiver_s3](../receiver_s3/), but
generates timecode **compatible with Traktor Scratch MK2** instead of
Serato CV02. Flash whichever matches your DJ software; the transmitter
pucks don't care.

**Relative mode only** — enforced by design (see "End-of-record loop"
below). Traktor detects it as *Vinyl MK2* and tracks speed, direction,
scratching and backspins.

## Hardware

Identical to receiver_s3: ESP32-S3 DevKitC-1, two PCM5102A DACs
(A: BCK 2 / LRCK 1 / DATA 42, B: BCK 14 / LRCK 13 / DATA 12), WS2812 LED on
GPIO 48. PSRAM is used if enabled but **not required** at the default loop
length (the bit table is ~37.5 KB).

## End-of-record loop — why relative mode only

Real MK2 vinyl has an end zone where Traktor drops to internal "play" mode
and disables vinyl control until the needle is repositioned — with no
needle, that would be a dead end. So this sketch only ever plays a
**`TRAKTOR_LOOP_SECONDS` window** (default 300 s) of the position sequence
and wraps. Traktor sees each wrap as a needle skip, which relative mode
absorbs by design (a sub-second quality-meter blip at most).

- Longer window = rarer wraps: `900` = 15 min, table grows to ~113 KB.
- `TRAKTOR_LOOP_START_SECONDS` slides the window deeper into the record —
  raise it if Traktor ever complains immediately at boot (would mean the rip
  started too near the lead-in/end zone).
- In absolute mode the deck would jump back every wrap — don't use it.

## Output-level constants

A few compile-time constants tune the generated signal if a given setup
needs it. Defaults work out of the box; only touch these if the timecode
quality meter in Traktor isn't happy.

| Constant | Default | Meaning |
|---|---|---|
| `CARRIER_LEVEL` | 0.55 | Carrier output level |
| `DATA_LEVEL` | 0.165 | Position-signal output level |
| `DATA_EDGE_SMOOTHING` | 1.0 | Edge shaping; 1.0 = crisp edges (highest quality-meter score in testing) |
| `DATA_INVERT` | 0 | Try 1 only if Traktor tracks speed but not position |
| `DEADZONE_RPM` / `MAX_RPM_RATIO` / `RPM_SMOOTHING` | 0.04 / 8.0 / 0.80 | As in receiver_s3 |

Same phase machinery as receiver_s3 (high-resolution accumulators, per-deck
audio task on core 1, 4 × 32-sample DMA ≈ 2.9 ms).

## Setup in Traktor

1. Route DAC A/B into the timecode inputs, input mode = phono/timecode as
   appropriate for your interface.
2. Scope view should show a stable ring.
3. Calibrate with the platter stopped, then confirm the deck tracks.
4. **Relative mode.** Quality meter should sit high and steady; a brief dip
   every `TRAKTOR_LOOP_SECONDS` is the loop wrap and is normal.

## Serial debug

Same output as receiver_s3: boot stages, receiver MAC for pairing, WELCOME
events, 500 ms per-deck status, and relayed transmitter events (`EVT deck
N: ...` — including the spin-calibration progress, which is only visible
here since the puck spins on battery).

`DEBUG_SERIAL 0` compiles all of it out.
