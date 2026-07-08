# Wireless Gyro DVS

Most of this project was written by Claude Code / Fable 5

## Lineage

Based on [FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System) —
Felipe Alme started this project and built the original transmitter/receiver
firmware this system grew from. This fork reworks it substantially:
precision/latency fixes, self-learning calibration, and fast ESP-NOW rate.

A DIY digital vinyl system with **no needle and no timecode record**. A
battery-powered gyro puck sits on the turntable platter and streams its
rotation speed over ESP-NOW to a receiver, which generates timecode
**compatible with Serato CV02 or Traktor Scratch MK2** into the DJ
software's audio interface, depending on which receiver sketch you flash.

```
 platter                        2.4 GHz                       line out
┌─────────────────┐   ESP-NOW broadcast, 500 Hz   ┌──────────────────────┐
│ transmitter_c3  │ ────────────────────────────► │ receiver_s3 (CV02)   │ ──► deck A
│ ESP32-C3        │                               │   or                 │ ──► deck B
│ + MPU6050 gyro  │   (second puck = deck 2)      │ receiver_s3_traktor  │
│ + battery       │ ◄──────────────────────────── │ ESP32-S3 + 2x        │
└─────────────────┘      WELCOME / PING           │ PCM5102A I2S DACs    │
                                                  └──────────────────────┘
```

## Repository layout

| Path | What it is |
|---|---|
| [transmitter_c3/](transmitter_c3/) | Platter puck: ESP32-C3 + MPU6050, sends RPM at 500 Hz |
| [receiver_s3/](receiver_s3/) | Receiver generating **Serato CV02** timecode (Serato, Mixxx) |
| [receiver_s3_traktor/](receiver_s3_traktor/) | Receiver generating **Traktor Scratch MK2** timecode |
| [CAD Cases/](CAD%20Cases/) | 3D-printed enclosures (Fusion 360 sources + .3mf) and build photos |

One receiver drives **two decks** (two DACs); each deck needs its own
transmitter puck (`DECK_ID 1` / `DECK_ID 2`). All boards run the same
ESP-NOW protocol, so transmitters don't care which receiver sketch is
flashed.

## Quick start

1. **Flash the receiver** (`receiver_s3` for Serato/Mixxx, or
   `receiver_s3_traktor` for Traktor). ESP32 Arduino core 3.x, needs the
   **Adafruit NeoPixel** library.
2. Open the Serial Monitor at 115200 and note the boot line:
   `transmitter: uint8_t receiverMAC[] = { 0x.., ... };`
3. **Paste that MAC** into `transmitter_c3.ino`, set `DECK_ID`, and flash
   the puck(s).
4. **Calibrate** (once per puck, ever): power the puck with the platter
   **stopped** (gyro zero), then start the platter at **0% pitch** and leave
   it alone ~15 s — the puck settles, averages 10 s against the quartz lock,
   and stores its gyro scale trim in flash. Watch it on the **receiver's**
   Serial Monitor (`EVT deck 1: spin-cal LOCKED, trim=...`); the puck's own
   USB is unreachable while it spins.
   See [transmitter_c3/README.md](transmitter_c3/README.md).
5. Connect the receiver's DAC outputs to the sound card inputs your DJ
   software expects timecode on, select the matching control vinyl type
   (Serato 2nd Ed. / Traktor Vinyl MK2), **relative mode**, and play.

## Everyday use

- Power the puck on with the platter **stopped** — it re-zeroes its gyro at
  every boot (temperature drift). Power-cycle = re-zero.
- Receiver LED: **RED** = no transmitters, **ORANGE** = one, **GREEN** = both.
- Both receivers print per-deck link health (rpm / packets / loss / age) on
  the Serial Monitor every 500 ms if you want to watch it.
- **Relative mode only** in the DJ software. There is no needle, so absolute
  position is synthetic — meaningless at best, and the Traktor sketch loops
  a short position window on purpose.

## Design decisions (and why)

- **Broadcast ESP-NOW DATA, no ACK/retry** → constant, low radio jitter.
- **24 Mbps OFDM instead of the 1 Mbps default** (`ESPNOW_FAST_RATE`) →
  ~6× less airtime per packet. At 500 packets/s per deck this is the
  difference between two decks constantly colliding (~33% channel use each)
  and coexisting cleanly (~5% each). Trade-off: ~10 dB less range — fine in
  a booth, disable if the link gets lossy at distance.
- **int64 phase accumulators with 32 fractional bits, rounded steps** →
  pitch granularity ~10 µHz, glitch-free reverse and backspin.
- **Small I2S DMA buffers** (4 × 32 samples) → ~2.9 ms output latency.
- **Gyro DLPF at 188 Hz**, gyro range ±1000 dps, send rate 500 Hz →
  end-to-end control latency ≈ 8 ms worst case (gyro filter 1.9 + packet
  interval 2 + receiver smoothing 1 + DMA 2.9).
- **Two-point gyro calibration**: zero-offset from the stopped platter at
  every boot (bias drifts with temperature), scale factor learned once from
  the quartz-locked platter and persisted (silicon constant). No per-unit
  firmware edits.

## Troubleshooting

| Symptom | Check |
|---|---|
| Receiver LED stays RED | Transmitter powered? Same `ESPNOW_CHANNEL` (11) and `dvs_packet` layout on both boards? Correct `receiverMAC` in the transmitter (HELLO/WELCOME handshake is unicast)? |
| Pitch reads slightly off at 0% | Redo spin-cal at 0% pitch, see transmitter README |
| Speed wrong by a large factor | `GYRO_FS_SEL` / `GYRO_LSB_PER_DPS` mismatch — they must be changed as a pair |
| Hiccups on hard scratch flicks | Gyro may clip at ±1000 dps (166 RPM). Switch pair to `0x18` / `16.4f` (±2000 dps) |
| Packet loss at distance | Set `ESPNOW_FAST_RATE 0` (or use `WIFI_PHY_RATE_12M`) on the transmitter |
| Traktor drops to internal play mode | Shouldn't happen (looped window) — if it does, raise `TRAKTOR_LOOP_START_SECONDS`; see receiver_s3_traktor README |

