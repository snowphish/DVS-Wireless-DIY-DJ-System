# test — low-battery, link-loss failover, wireless dicer (experimental, local only)

Forked copies of the sketches plus a new **dicer** button unit. The
top-level sketches are untouched; flash these to try the features. Not
committed to git until proven.

`transmitter_bmi160/` is the same test transmitter (low-batt, spin-cal
learn-once, no auto-rezero) with the gyro driver ported to the Bosch
BMI160 per the PCB schematic: SDA→GPIO8, SCL→GPIO9, CS→3V3 (I2C),
SAO→GND (0x68), INT1→GPIO4 reserved. Untested until boards arrive.

## Hardware change (transmitter puck)

One divider + one cap, wired **after the power switch** so it draws nothing
when off (~20 µA when on):

```
BAT+ (switched) ── 100k ──┬── 100k ── GND
                          ├── GPIO3 (ADC)
                          └── 100 nF ── GND
```

`BATT_DIVIDER 2.00` assumes equal resistors — if you measure the real pair,
trim the constant: `divider = (R_top + R_bottom) / R_bottom`.

## Behavior

- Puck checks the battery every 5 s (8-sample average, eFuse-calibrated
  ADC). Thresholds: **LOW < 3.50 V**, **CRITICAL < 3.20 V**, recovery above
  3.60 V (hysteresis).
- Reports over the existing `MSG_EVENT` channel: voltage at boot + every
  60 s, LOW/CRITICAL re-sent every 10 s while it persists.
- Receiver serial: `EVT deck 1: battery LOW 3.42 V - charge soon` etc.
- **Receiver status LED** (color still = deck count):
  - deck 1 low → slow on/off blink (500 ms / 500 ms)
  - deck 2 low → **two fast pulses** per 1.2 s cycle
  - both low → three fast pulses per cycle
  - LED state expires 30 s after the last LOW report (puck died/went off).

## Spin-cal fix + the drift verdict (transmitter)

- **Spin-cal now truly runs once.** It used to re-arm on every boot and
  "refine" the trim — which silently folded whatever thermal bias drift
  existed at that moment into the permanent scale constant (trim drifting
  from 0.99872 to 1.00022 = exactly the 0.05 rpm error observed). It is
  now learned once and locked. `SPIN_CAL_FORGET 1` (one flash) re-learns.
- **The multi-hour drift at 33 rpm is the gyro's scale-temperature
  coefficient** — established experimentally: cooling the gyro reverses
  the drift direction, and a reboot re-zero (bias recal at current temp)
  does not fix it. No zero-offset scheme can help; a stationary
  auto-rezero was tried and removed as pointless for this failure mode.
  Fixes if it ever matters: temperature-compensate scale via the IMU's
  die-temp sensor (two-point thermal cal), or a better IMU — a BMI160
  swap is planned for evaluation.

**Recovery after the polluted trim:** flash once with `SPIN_CAL_FORGET 1`
(or one upload with Erase All Flash enabled), flash back, redo the spin
ritual once at 0% pitch. That trim is then final.

## Link-loss failover (receivers)

If a puck goes silent mid-performance (dead battery, switch bump, radio
dropout), the receiver no longer stops the deck after 1 s. Instead it keeps
generating timecode at the puck's **last stable RPM** — a value that held
within ±0.75 RPM for 2 s. Scratch moves never qualify as "stable", so a
puck dying mid-scratch holds the last steady groove speed, not the flail.

- Serial: `FAILOVER deck 1: link lost - holding 33.33 rpm`, and the status
  line shows `HLD` instead of `OFF`. Link back → `link restored`, live RPM
  resumes through the normal smoothing.
- The LED still drops to orange/red (deck count), so the fallback is
  visible at a glance while the music keeps playing.
- A stopped platter is itself a stable 0 — so the clean shutdown ritual is
  **stop the platter first, then switch the puck off**. Killing the puck
  while spinning = deck keeps playing (that's the point).
- Set `FAILOVER_ENABLE 0` to get the old stop-on-timeout behavior.

Bench test: play a track, flip the puck's power switch mid-play — the deck
should keep rolling at pitch and the receiver should log the failover.

## Wireless dicer (`dicer_c3`, unit 3)

Novation-Dicer-style wireless button pad: ESP32-C3 with **6 performance
buttons + 1 mode button + 1 mode LED**. Route: dicer → ESP-NOW (unicast,
ACKed) → receiver → **USB-MIDI** → MIDI-learn in Traktor/Serato.

Wiring (buttons pin→GND, internal pull-ups, diagonal legs on 6 mm tacts):

| Function | Pin |
|---|---|
| Performance buttons 1–6 | GPIO 0, 1, 4, 5, 6, 7 |
| Mode button | GPIO 10 |
| Mode LED (→ ~330R → GND) | GPIO 21 |

- **Modes are pages** (3 by default, `NUM_MODES`): mode button cycles,
  LED blinks (mode+1) pulses every 3 s. Each page emits a distinct note
  range: `note = 36 + mode*6 + button` → pages MIDI-map independently.
- State-based protocol (`MSG_BUTTONS` bitmask + heartbeat every 500 ms):
  lost packets can't stick notes; note-off always matches the note sent
  at press time even if the mode changed mid-hold.
- **Receiver needs Tools → USB Mode: "USB-OTG (TinyUSB)"** and the laptop
  on the S3's *native* USB port — it then shows up as a MIDI device (plus
  CDC serial for the debug output, same cable). In CDC/JTAG mode MIDI
  compiles out with a warning; timecode is unaffected.
- Receiver serial shows `DICER: mode 1 btn 3 DOWN -> note 38 on` lines
  even without MIDI, so the radio path can be tested before the MIDI setup.
- No battery monitor on the dicer (deliberate — a dead one is obvious,
  just swap the cell). GPIO 3 stays free.
- A second dicer would be unit 4 — receiver support for that comes after
  unit 3 is proven.

## Bench test without draining a battery

Power the puck from USB with the divider input on a bench supply (or just
touch GPIO3's divider to 3.3 V / GND through the resistors): below 1.75 V
at the pin = LOW, below 1.6 V = CRITICAL. Or temporarily raise
`BATT_LOW_MV` above the current battery voltage to force the indication.
