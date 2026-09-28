# FRIENDS-2 Firmware (STM32WB5)

Bare-metal C firmware for a custom sensor board built on the STM32WB5MMG
dual-core wireless module (Cortex-M4 application core + Cortex-M0+ BLE core).

This repository shares the application source code only. Hardware design files,
schematics, and the CubeMX/CubeIDE project configuration are not included.

## Highlights

- **Dual magnetometer acquisition** - two LIS2MDL sensors over SPI at 20 Hz,
  paired by data-ready in a free-running main loop (no fixed delay), with
  per-sensor magnitude, a differential vector that cancels the Earth's field,
  and boot-time baseline zeroing (`mag_zero.c`).
- **Event detection with timer capture/compare** - TIM2 in PWM-input mode
  (slave reset on each rising edge) starts an event on the first capture and
  measures its duration by summing captured periods (15.6 ns resolution). A
  CH3 output compare fires when no edge arrives within a timeout and ends the
  event. The ISR drives the status LED and pushes events into a lock-free
  single-producer/single-consumer queue; SD and USB work stays in the main loop.
- **SD card logging** - FatFs over SPI with a custom disk I/O layer
  (`sd_spi_diskio.c`), paired CSV run files, periodic sync, and safe close.
- **USB composite behaviour at runtime** - boots as a CDC virtual COM port for
  commands and streaming, then switches to USB Mass Storage on command so the
  host can read the SD card, and back again on eject (`usb_mode.c`).
- **BLE streaming** - STM32_WPAN stack on CPU2, one GATT notification per new
  sensor sample pair, ATT MTU exchange on connect for full-size packets.
- **USB DFU from software** - a `boot` command closes files, detaches USB, sets a
  flag in an RTC backup register and resets; the next boot jumps to the ST
  system bootloader before any clock or peripheral init (`boot_dfu.c`).
- **Dual-core clock ownership fix** - USB 48 MHz clock is claimed through a
  hardware semaphore so the BLE core cannot power it down, with a runtime guard.

## Code layout

| Path | Contents |
|------|----------|
| `Core/Src/main.c` | Init, main loop, sensor reads, event detection, SD logging, CDC commands |
| `Core/Src/usb_mode.c` | CDC / MSC runtime class switching |
| `Core/Src/boot_dfu.c` | Software entry into the USB DFU bootloader |
| `Core/Src/mag_zero.c` | Magnetometer baseline zeroing |
| `Core/Src/sd_spi_diskio.c`, `diskio.c` | SD card driver and FatFs glue |
| `STM32_WPAN/App/` | BLE application and GATT notification service |
| `USB_Device/` | USB CDC and MSC class interfaces |

## Serial commands (USB CDC)

`d` start logging, `s` stop logging, `e` switch USB to mass storage,
`boot` enter USB DFU, `p`/`m`/`v` start streaming, `x` stop streaming,
`z` re-zero the magnetometer difference.

## Tools

STM32CubeIDE, STM32 HAL, STM32_WPAN BLE stack, ST USB Device library, FatFs.

## Note

ST HAL/CMSIS drivers, middleware and startup files are omitted; they are
available from ST. The project will not build from this repository alone.
