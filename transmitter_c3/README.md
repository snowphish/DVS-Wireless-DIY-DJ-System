# transmitter_c3 — platter gyro puck

> **Lineage:** based on [FelipeAlme/DVS-Wireless-DIY-DJ-System](https://github.com/FelipeAlme/DVS-Wireless-DIY-DJ-System)
> by Felipe Alme, who started this project.

ESP32-C3 + MPU6050, battery powered, sits on the turntable platter. Reads
platter angular velocity from the gyro's Z axis and broadcasts it as RPM
over ESP-NOW **500 times per second**. One puck per deck.

I'm using a very common small SPDT Micro Switch for on/off, Size: 8.7 x 3.7 x 11.7mm 

## Hardware

| Signal | Pin |
|---|---|
| MPU6050 SDA | GPIO 8 |
| MPU6050 SCL | GPIO 9 |
| MPU6050 address | 0x68 (AD0 low) |

I2C runs at 400 kHz. No LED, no buttons — all feedback is on the Serial
Monitor (115200, any USB mode).

Mount the puck so the gyro's **Z axis is vertical** (board flat on the
platter). Centering is *not* critical — angular velocity is the same
everywhere on a rigid platter — but keep it from sliding.

## Flashing / pairing

- ESP32 Arduino core 3.x. No special USB mode.
- Set `DECK_ID` (1 or 2 — the receiver maps it to DAC A/B).
- Set `receiverMAC[]` to your receiver's MAC. The receiver prints a
  copy-paste-ready line on its Serial Monitor at every boot:
  `transmitter: uint8_t receiverMAC[] = { 0x.., ... };`
  DATA packets are broadcast, but the HELLO→WELCOME handshake is unicast,
  so a wrong MAC means the puck never starts streaming.

## Calibration — two different things

### 1. Zero offset (bias) — automatic, every boot

Gyro bias drifts with temperature, so it is re-measured at **every
power-up**: the firmware waits for a stable-still window (motion restarts
the window, so it can never zero while spinning) and averages it.

**Ritual: power the puck on with the platter stopped.** Power-cycling the
puck = re-zeroing.

### 2. Scale factor (spin-cal) — automatic, once per unit

The MPU6050's rate scale is only trimmed to ±3% from the factory, which
shows up directly as a pitch error (e.g. 33.45 instead of 33.33 = +0.35%).
The fix uses the turntable itself as the reference: a quartz-locked platter
at 0% pitch is accurate to ~0.01%.

Within the **first 2 minutes after boot**, once the reading sits near 33.33
(or 45.00) RPM it first **settles for 4 s** (a turntable creeps through its
last ~0.1% of spin-up — sampling that ramp would bias the trim), then
**averages 10 s** of readings, computes `trim = target / measured`, and
**stores it in flash** (NVS). Every later boot loads it automatically.

**Ritual, once per puck ever: boot with platter stopped → start the platter
at 0% pitch → leave it alone for ~15 s.**

Watch the progress on the **receiver's** Serial Monitor — the puck spins on
battery, so its own USB is unreachable. It relays its calibration status
over ESP-NOW (`MSG_EVENT`) and the receiver prints:

```
EVT deck 1: NO stored trim - spin platter at 0% pitch to calibrate
EVT deck 1: spin-cal reference 33.33 settled, sampling ~10 s - don't touch
EVT deck 1: spin-cal LOCKED, trim=0.99810 (saved)
```

On later boots it reports `EVT deck 1: using stored trim=0.99810` instead.
(The same lines also appear on the puck's own serial if you ever have it on
USB at the bench.)

**The trim survives reflashing.** Uploading a sketch only rewrites the
application partition; the trim lives in the NVS *data* partition, which
normal uploads never touch. Flash new firmware as often as you like — every
boot will keep printing `spin-cal: using stored trim=...`. You do NOT need
to recalibrate after reflashing. (NVS is only wiped by Tools → *Erase All
Flash Before Sketch Upload: Enabled*, or a partition-scheme change.)

Safeguards:
- **Once learned, spin-cal never arms again** — later boots load the trim
  and lock it. Re-learning per boot turned out to be a bug: each spin
  measurement folds that moment's thermal *bias* drift into the *scale*
  trim (the two are inseparable in a single measurement), slowly
  corrupting it. `SPIN_CAL_FORGET` is the only way to re-learn.
- When it does arm (no stored trim), only for the first `SPIN_CAL_ARM_MS`
  (2 min) after boot.
- Reading must stay within `SPIN_CAL_STABLE_RPM` (0.15) of the running mean
  for the full window; wobble restarts both the settle and the sampling.
- Acceptance window ±5% (full gyro tolerance); sanity clamp 0.90–1.10.

⚠️ Do the first spin-up **at 0% pitch** — it is the one measurement that
becomes the permanent trim. Later boots can't corrupt it (spin-cal is
disarmed), but a wrong first calibration persists until erased.

### Erasing a bad calibration (`SPIN_CAL_FORGET`)

Because the trim survives reflashing, a *wrong* trim (e.g. the first
spin-up was accidentally done at +2% pitch) also survives — and since
spin-cal never re-arms once a trim exists, it will not self-correct.
Reflashing alone will not fix it; you have to erase the stored trim
explicitly:

1. Set `SPIN_CAL_FORGET 1`, flash, and let it boot once — serial prints
   `spin-cal: stored trim erased`.
2. Set `SPIN_CAL_FORGET 0` and flash again. This step is required: a
   FORGET build erases at *every* boot, so any trim it learns is wiped
   again at the next power-up.
3. Redo the ritual (boot stopped → spin at 0% pitch) — the puck learns and
   saves a fresh trim.

Shortcut without editing code: do one upload with Tools → *Erase All Flash
Before Sketch Upload: Enabled* (then set it back to Disabled) — that wipes
all of NVS, trim included, with the same end result.

You will likely never need any of this — it exists only for the
"taught it garbage on day one" scenario.

## Radio

- `ESPNOW_CHANNEL 11` — must match the receiver.
- DATA is **broadcast** (`USE_BROADCAST 1`): no ACK/retry → low jitter.
- `ESPNOW_FAST_RATE 1` sends at **24 Mbps OFDM** instead of the 1 Mbps
  default: ~6× less airtime (~5% channel use per deck instead of ~33%),
  which matters a lot with two pucks transmitting simultaneously.
  Trade-off: ~10 dB less link margin. If you see loss at long range, set it
  to 0 or use `WIFI_PHY_RATE_12M`.
- Link management: puck HELLOs until the receiver WELCOMEs, then streams;
  receiver PINGs every 500 ms; 2 s without a reply → back to HELLO mode.

## Tuning constants

| Constant | Default | Meaning |
|---|---|---|
| `DEADZONE_RPM` | 0.20 | Below this the output snaps to 0 (stops drift/noise when idle). Lower it if very slow crawls matter |
| `RPM_MULTIPLIER` | 1.00 | Manual scale trim, multiplied on top of the learned spin-cal trim. Normally leave at 1.0 |
| `SMOOTHING` | 1.00 | Output filter; 1.0 = pass-through (lowest latency) |
| `GYRO_FS_SEL` / `GYRO_LSB_PER_DPS` | 0x10 / 32.8 | Range ±1000 dps (166 RPM ceiling). For very hard scratch flicks use 0x18 / 16.4 (±2000 dps). **Change as a pair** |
| `GYRO_DLPF_CFG` | 1 | 188 Hz filter, ~1.9 ms delay. Higher values = smoother but slower |
| `SEND_RATE_HZ` | 500 | Packet rate |

## Packet format

```c
typedef struct __attribute__((packed)) {
  uint8_t  msgType;          // 1 HELLO, 2 WELCOME, 3 DATA, 4 PING, 5 EVENT
  uint8_t  version;          // PROTOCOL_VERSION 1
  uint8_t  deckId;           // 1 or 2
  int16_t  rpmCenti;         // signed RPM x100 (33.33 -> 3333)
  int16_t  gyroRaw;          // raw gyro sample (diagnostics)
  uint32_t seq;              // for loss counting on the receiver
  uint32_t timestampMicros;
} dvs_packet;                // must match the receiver exactly
```
