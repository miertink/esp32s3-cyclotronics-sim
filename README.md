# Cyclotronics Simulator (FTMS + CSC)

Simulates a Bluetooth smart trainer/power meter (FTMS power service +
CSC cadence service) so the `Cyclo_130_S3` bridge project can be
developed and tested without a real bike/trainer attached. Advertises as
`Cyclo_130_SIMv1`.

- **FTMS (`0x1826`):** notifies simulated speed/power on the Indoor Bike
  Data characteristic, and implements the Control Point so a real
  training app can set ERG target power or simulate an incline/grade —
  the simulator just logs and reflects whichever mode is active.
- **CSC (`0x1816`):** notifies cadence as cumulative wheel/crank
  revolutions + event time, generated from a simulated cadence value with
  jitter-free timing (fractional revolutions are tracked internally so
  the reported event time matches when a full revolution actually
  completed).

## Hardware

- ESP32-S3 DevKit N16R8 (16MB flash, 8MB PSRAM — see `platformio.ini` for
  the PSRAM/flash config).
- No other hardware required; this is a pure BLE simulator.

## Environment

- **Toolchain:** PlatformIO (`platform = espressif32`, board
  `esp32-s3-devkitc-1`, `framework = arduino`).
- **Libs:** NimBLE-Arduino, Adafruit NeoPixel.
- **Flashing:** UART, COM5 (see `platformio.ini`).
- Two build environments: `dev` (debug logging on) and `release`
  (optimized, default).

## Build / run

```sh
pio run -e dev                  # compile, debug build
pio run -e release              # compile, release build (default env)
pio run -t upload               # flash over UART (COM5)
pio device monitor              # serial log (COM5, 115200)
```
