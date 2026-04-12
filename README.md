# BoilerRoomCntrl — STM32 Boiler Room Controller

## Overview

Embedded HVAC/heat management system for monitoring and controlling a heating system. Reads multiple temperature, humidity, and pressure sensors, then controls pumps, valves, and a heating boiler. Features a touchscreen ILI9341 LCD UI for viewing sensor data and configuring setpoints.

**Language:** C++17 (converted from C)

## Hardware

- **MCU:** STM32F103CBT6 (Cortex-M3, 128KB Flash, 20KB RAM, 72 MHz)
- **Display:** ILI9341 2.4" TFT LCD (240x320) with XPT2046 touch controller
- **Sensors:** DHT22 (temperature/humidity), NTC thermistors, pressure sensors, water flow meter
- **Communication:** RS-485 Modbus RTU, I2C EEPROM

## Build (Linux)

### Prerequisites

Install ARM GCC cross-compiler toolchain:

```bash
sudo apt install gcc-arm-none-eabi g++-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

### Build Commands

```bash
make          # Build the project (Debug by default)
make BUILD_TYPE=Release  # Build Release version
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

### User Interface

The display is split into two halves (left = Data, right = Setpoints). Navigate with the three physical buttons:

- **Left/Right buttons** — move cursor up/down
- **Enter button** on title bar — switch between Data and Setpages
- **Enter button** on an item — toggle relay (ON/OFF) or enter edit mode

#### Time Editing

Six time-editing items are available in the Setpoints menu (Часы, Мин., Сек., День, Месяц, Год):

1. Navigate to "Часы" (Hours) and press **Enter** to enter time-edit mode (loads current RTC values)
2. Use **Left/Right** buttons to increment/decrement the highlighted value
3. Press **Enter** on any time item to adjust it
4. Press **Enter** on "Часы" again to apply the new time to the RTC

### Project Structure

```
Core/
  Inc/                    # Headers (C++ with extern "C" for C compatibility)
  Src/                    # Main source (main.cpp, freertos.c, ISRs)
  Startup/                # Vector table & startup code
  ili9341/                # ILI9341 LCD + XPT2046 touch drivers (C with extern "C")
  modbus/                 # SimpleModbusSlave (C with extern "C")
  app/                    # Custom application code (C++17)
    hal_utils.h/cpp       # ADC averaging, pressure management classes
    lcd_ui.h/cpp          # Display menu rendering (C++ classes)
    menu_strings.h/cpp    # Menu definitions (Russian, CP1251)
    thermistor_table.h    # Thermistor lookup tables (constexpr)
    dwt_timer.h/cpp       # DWT timing (singleton class)
  DHT.h/cpp               # DHT22 sensor interface (C++ class)
Middlewares/FreeRTOS/     # FreeRTOS V10.0.1 + CMSIS-RTOS (C)
Drivers/                  # STM32 HAL + CMSIS (C)
```

## C++ Conversion Notes

This project has been converted from C to C++17 for improved type safety and modern C++ features:

- **Constants:** Replaced `#define` with `constexpr` in namespaces (e.g., `Pressure::`, `ADC::`, `Menu::`)
- **Types:** Replaced integer enums with `enum class` for type safety
- **Classes:** Converted structs with functions to proper C++ classes with constructors and methods
- **Backward compatibility:** C library code (HAL, FreeRTOS, ILI9341, Modbus) wrapped with `extern "C"`
- **No RTTI/Exceptions:** Disabled `-fno-rtti -fno-exceptions` for embedded systems
- **Zero overhead:** All C++ features used have zero runtime overhead compared to C

### Build Statistics

- **Flash usage:** ~63.7 KB / 128 KB (48.61%)
- **RAM usage:** ~14.9 KB / 20 KB (72.93%)
- **Compiler warnings:** 0 warnings, 0 errors

## Branches

- `main` — main development branch
- `cpp-conversion` — C++17 conversion branch
