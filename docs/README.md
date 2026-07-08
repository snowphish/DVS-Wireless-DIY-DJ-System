# DVS Kit web flasher

A static site that flashes the kit boards straight from Chrome/Edge using
[ESP Web Tools](https://esphome.github.io/esp-web-tools/) — no software for
the buyer to install. A dropdown picks the target:

- **Transmitter puck** — ESP32-C3
- **Receiver, Traktor MK2** — ESP32-S3
- **Receiver, Serato CV02** — ESP32-S3

ESP Web Tools checks the connected chip against each manifest's
`chipFamily`, so a buyer can't flash S3 firmware onto a C3 puck (or vice
versa) — it errors instead.

## Files

```
flasher/
  index.html              the page (target selector + install button)
  manifest-tx.json        -> firmware/transmitter-c3.bin
  manifest-rx-traktor.json-> firmware/receiver-traktor-s3.bin
  manifest-rx-serato.json -> firmware/receiver-serato-s3.bin
  firmware/               the merged .bin files (you build these)
```

## Building the merged .bin files

ESP Web Tools flashes one **merged** image per target at offset 0
(bootloader + partition table + app in a single file). Two ways:

### From Arduino IDE / arduino-cli
1. Sketch → Export Compiled Binary (or `arduino-cli compile --export-binaries`).
   That gives you separate `.bootloader.bin`, `.partitions.bin`, `.bin`.
2. Merge them with esptool:

```bash
# Transmitter (ESP32-C3)
esptool.py --chip esp32c3 merge_bin -o firmware/transmitter-c3.bin \
  0x0    transmitter_c3.ino.bootloader.bin \
  0x8000 transmitter_c3.ino.partitions.bin \
  0xe000 boot_app0.bin \
  0x10000 transmitter_c3.ino.bin

# Receiver, Traktor (ESP32-S3)
esptool.py --chip esp32s3 merge_bin -o firmware/receiver-traktor-s3.bin \
  0x0    receiver_s3_traktor.ino.bootloader.bin \
  0x8000 receiver_s3_traktor.ino.partitions.bin \
  0xe000 boot_app0.bin \
  0x10000 receiver_s3_traktor.ino.bin

# Receiver, Serato (ESP32-S3) — same, with the receiver_s3 build
```

- `boot_app0.bin` lives in the ESP32 core install
  (`.../packages/esp32/hardware/esp32/<ver>/tools/partitions/`).
- **Receiver S3 note:** build it with the USB mode the kit ships in. For
  USB-MIDI (dicer) you need **USB-OTG (TinyUSB)**; otherwise the default is
  fine. The merged bin captures whatever you compiled.
- Bump each manifest's `version` when you rebuild so buyers can tell.

## Serving it

Web Serial needs a **secure context** — HTTPS, or `localhost` for testing.

- **Test locally:** `python -m http.server 8000` in this folder, open
  `http://localhost:8000` (localhost counts as secure).
- **Publish:** any static host with HTTPS — GitHub Pages, Netlify, Cloudflare
  Pages. Just upload the `flasher/` contents. No backend needed.

## Requirements for the buyer

- Desktop **Chrome** or **Edge** (Web Serial isn't in Safari/Firefox).
- A **data** USB-C cable (charge-only cables enumerate nothing).
- Battery pucks: switch **OFF** before flashing (flash over USB only).

## Notes

- The page pins `esp-web-tools@10` from unpkg. To avoid a CDN dependency,
  vendor `install-button.js` locally and point the `<script>` at it.
- `new_install_prompt_erase: true` offers a full-erase on first install —
  handy for wiping a stale spin-cal trim (NVS) during production flashing.
