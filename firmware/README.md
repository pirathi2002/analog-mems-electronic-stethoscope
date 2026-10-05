# Firmware

Embedded firmware for the Analog MEMS Electronic Stethoscope, targeting an STM32 microcontroller.

**Owner:** Pirathi (branch `firmware/pirathi`)

## Scope

- Audio acquisition from the analog front-end (ADC sampling)
- Digital filtering of the acquired signal
- BLE communication (if applicable)
- Device control and configuration

## Layout

| Folder | Contents |
|---|---|
| `src/` | C source files |
| `include/` | Header files |
| `config/` | STM32CubeMX `.ioc`, linker scripts, and other configuration |

## Build

_To be documented once the toolchain and project setup are finalized (e.g. STM32CubeIDE, or CMake with arm-none-eabi-gcc in VS Code)._
