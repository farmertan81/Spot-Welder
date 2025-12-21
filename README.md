<<<<<<< HEAD
# ESP32 Spot Welder Firmware (C++ / PlatformIO)

This folder contains the ESP32-S3 weld controller firmware currently running on the welder.

- Platform: ESP32-S3 (PlatformIO project)
- WebSocket client for the Raspberry Pi weld server
- Commands: SET_PULSE, STATUS, CHARGE_ON/OFF, etc.
- Sends `HELLO,ESP32` on connect and `FIRED,<ms>` after each weld
- Debounced active-low pedal input with one weld per press

Copied from `~/weldctl_arduino` on the Raspberry Pi.
=======
# Spot Welder Control
>>>>>>> 3e372d4f544fae64a89fa5d752ddeb21ddfe6f4e
