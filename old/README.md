# Legacy firmware for original and non-custom-PCB builds

This directory preserves the earlier firmware that was present in the project
before the unified receiver and Windows Manager release. It is intended for
people building around the original wiring, development boards, or their own
non-custom-PCB hardware.

| Folder | Historical purpose |
|---|---|
| [`receiver_s3`](receiver_s3) | Two-deck ESP32-S3 receiver with Serato CV02 output |
| [`receiver_s3_traktor`](receiver_s3_traktor) | Separate historical ESP32-S3 Traktor receiver |
| [`transmitter_c3`](transmitter_c3) | ESP32-C3 + MPU6050 puck using manual deck/MAC configuration |

These sketches are preserved as legacy reference builds and are not the
firmware served by the current web flasher. They predate automatic pairing, the
unified runtime format selection, USB Manager protocol, deck swapping, and the
other features in [`../Firmware`](../Firmware).

Read each subfolder's README and verify every pin against your physical build
before flashing. In particular, the legacy transmitter requires manual
`DECK_ID` and `receiverMAC[]` configuration, and all boards must use matching
ESP-NOW channel and packet definitions.
