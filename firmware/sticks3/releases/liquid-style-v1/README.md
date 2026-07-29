# Liquid Style v1

Verified StickS3 application image for the independent fine-sand/liquid
hourglass variant.

- File: `vibestick_hourglass_liquid.bin`
- Size: `655008` bytes
- SHA-256:
  `d8a277334a9208c0dbabc2f2dec34679f4a48c2cf5d564f1404919d32dcc71a7`
- App slot address: `0x20000`
- VibeStick-Codex remains in the other OTA slot
- Includes duration-scaled final-grain synchronization and the non-blocking
  three-note completion chime
- Restarts a completed cycle by resetting both the timer and particle pool

Flash only this application image:

```sh
python -m esptool \
  --chip esp32s3 -p <PORT> -b 460800 \
  --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x20000 vibestick_hourglass_liquid.bin
```

Do not rewrite the partition table or OTA data when switching only between the
archived hourglass visual variants.
