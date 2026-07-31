# DVS Manager USB protocol

## Transport

- ESP32-S3 USB CDC serial
- 115200 baud
- UTF-8 / ASCII-compatible text
- One message per line (`\n`; receiver also accepts `\r\n`)
- Protocol version: `1`

Human debug text may share the port. Machine messages are always prefixed with
`@DVS `, so clients must ignore every other line.

## Receiver to manager

The content after `@DVS ` is JSON.

```text
@DVS {"type":"hello","id":1,"protocol":1,"firmware":"1.2.1","device":"DVS Receiver","serial":"A1B2C3D4"}
@DVS {"type":"config","id":2,"gain":0.300,"gains":0.300,"gaint":0.550,"format":0,"brightness":40,"pairWindow":60,"baseRpm":33.3333,"batteryLow":3.50,"apFallback":true}
@DVS {"type":"response","id":3,"ok":true,"message":"settings saved"}
```

Status telemetry is sent every 250 ms:

```json
{
  "type": "status",
  "id": 0,
  "uptime": 12000,
  "format": 0,
  "gain": 0.300,
  "pairOpen": false,
  "pairRemaining": 0,
  "portalActive": false,
  "decks": [
    {
      "deck": 1,
      "assigned": true,
      "state": "live",
      "rpm": 33.33,
      "loss": 0.4,
      "rssi": -56,
      "battery": 3.91,
      "lowBattery": false,
      "age": 8,
      "cal": "trim loaded: 0.99870",
      "mac": "AA:BB:CC:DD:EE:01"
    }
  ]
}
```

Deck state is `live`, `holding`, or `offline`. Unknown loss/RSSI values are
JSON `null`.

Asynchronous events include:

- `link_lost`
- `link_restored`
- `battery_low`
- `battery_critical`
- `battery_recovered`
- `calibration_complete`
- `zero_cal_blocked`
- `zero_cal_complete`
- `zero_bias_mismatch`
- `imu_fault`
- `drdy_fallback`
- `ap_disabled`

## Manager to receiver

Commands use space-separated ASCII tokens to keep the microcontroller parser
fixed-size and dependency-free. Every command should include a positive `id`;
the receiver echoes it in the corresponding reply.

```text
@DVS HELLO id=1
@DVS GET_CONFIG id=2
@DVS GET_STATUS id=3
@DVS PING id=4
@DVS CALIBRATE id=5
@DVS PAIR seconds=60 id=6
@DVS REPAIR id=7
@DVS SWAP_DECKS id=8
@DVS SET_CONFIG gain=0.300 fmt=0 bri=40 pwin=60 base=33.3333 blow=3.50 ap=1 id=9
@DVS SET_CONFIG gain=0.300 gains=0.300 gaint=0.550 fmt=1 bri=40 pwin=60 base=33.3333 blow=3.50 ap=1 id=10
```

Ranges:

- `gain`: 0.05–0.60
- `fmt`: 0 Serato CV02, 1 Traktor carrier
- `bri`: 5–255
- `pwin`: 10–300 seconds
- `base`: 30–50; normalized to 33.3333 or 45
- `blow`: 3.0–4.0 volts
- `ap`: 0 disabled, 1 available by receiver-button hold

Optional `gains` and `gaint` keys explicitly update the stored Serato and
Traktor gains, respectively, using the same range as `gain`.

For backward compatibility, bare `gain` updates the format that is active
before `fmt` is applied. If either explicit `gains` or `gaint` is present,
the supplied explicit values take precedence and bare `gain` is ignored.
The receiver then activates `fmt` and loads that format's stored gain.

The receiver has a 320-byte fixed input buffer and rejects overlong commands.
`SWAP_DECKS` atomically exchanges two existing puck assignments and fails when
fewer than two pucks are paired.
USB parsing and telemetry run only from `loop()` with bounded work; neither the
ESP-NOW callback nor audio tasks perform serial I/O.
