# BoilerRoomCntrl — STM32 Boiler Room Controller

## Overview

Embedded HVAC/heat management system for monitoring and controlling a heating system. Reads multiple temperature, humidity, and pressure sensors, then controls pumps, valves, and a heating boiler. Features a touchscreen ILI9341 LCD UI for viewing sensor data and configuring setpoints.

## Hardware

- **MCU:** STM32F103CBT6 (Cortex-M3, 128KB Flash, 20KB RAM, 72 MHz)
- **Display:** ILI9341 2.4" TFT LCD (240x320) with XPT2046 touch controller
- **Sensors:** DHT22 (temperature/humidity), NTC thermistors, pressure sensors, water flow meter
- **Communication:** RS-485 Modbus RTU, I2C EEPROM

## Build (Linux)

### Prerequisites

Install ARM GCC cross-compiler toolchain:

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

### Build Commands

```bash
make          # Build the project
make clean    # Remove build artifacts
```

### Output Files

| File | Description |
|------|-------------|
| `BoilerRoomCntrl.elf` | ELF binary for debugging |
| `BoilerRoomCntrl.bin` | Raw binary for flashing |
| `BoilerRoomCntrl.hex` | Intel HEX format |
| `BoilerRoomCntrl.map` | Linker map file |

## Architecture

### FreeRTOS Tasks

| Task | Priority | Purpose |
|------|----------|---------|
| `defaultTask` | Normal | I2C EEPROM communication, Modbus init |
| `sensReadTask` | Normal | Read all sensors (DHT22, thermistors, pressure) |
| `dispTask` | Normal | Display rendering, menu, button handling |

### Project Structure

```
Core/
  Inc/                    # Headers
  Src/                    # Main source (main.c, freertos.c, ISRs)
  Startup/                # Vector table & startup code
  ili9341/                # ILI9341 LCD + XPT2046 touch drivers
  modbus/                 # SimpleModbusSlave (RTU, functions 3 & 16)
  my_file/                # Custom application code
    mylib.h/c             # Utilities, pressure management logic
    my_sensors.h/c        # Thermistor lookup tables
    mymenu.h/c            # Menu definitions (Russian, CP1251)
    my_disp_lib.h/c       # Display menu rendering library
    mytime.h/c            # DWT timing (delay_us, millis)
Middlewares/FreeRTOS/     # FreeRTOS V10.0.1 + CMSIS-RTOS
Drivers/                  # STM32 HAL + CMSIS
```

## Branches

- `main` — main development branch
- `linux-build` — branch for testing Linux build compatibility
