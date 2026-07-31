# DVS web flasher

This directory is published by GitHub Pages at:

<https://snowphish.github.io/DVS-Wireless-DIY-DJ-System/>

It uses ESP Web Tools to flash three complete merged images at offset `0x0`:

- Unified ESP32-S3 receiver
- ESP32-C3 transmitter with MPU6050
- ESP32-C3 transmitter with BMI160 or BMI120

The binaries are compiled from the matching sketches under `Firmware/` with
ESP32 Arduino core 3.3.10. The JSON manifests select the correct chip family and
must remain next to `index.html`; binary paths are relative to this directory.
