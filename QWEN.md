# BoilerRoomCntrl — STM32 Boiler Room Controller

## Project Overview

Embedded HVAC/heat management system for monitoring and controlling a heating system. Reads multiple temperature, humidity, and pressure sensors, then controls pumps, valves, and a heating boiler. Features a touchscreen ILI9341 LCD UI for viewing sensor data and configuring setpoints.

## Hardware

- **MCU:** STM32F103CBT6 (Cortex-M3, 128KB Flash, 20KB RAM, 72 MHz)
- **Package:** LQFP48
- **IDE:** STM32CubeIDE (Eclipse-based, ARM GCC)
- **Firmware:** STM32Cube FW_F1 V1.8.3

## Key Peripherals

| Peripheral | Pins | Purpose |
|------------|------|---------|
| ADC1 | PA1, PA2, PA3, PA4, PB0, PB1 | 4 thermistors + 2 pressure sensors |
| SPI2 | PB13(SCK), PB15(MOSI), PB12(CS) | ILI9341 display (DMA-accelerated) |
| USART3 | PB10(TX), PB11(RX), PB2(RE/DE) | RS-485 Modbus RTU (57600 baud) |
| I2C1 | PB6(SCL), PB7(SDA) | External EEPROM (24C-series) |
| TIM1 | — | FreeRTOS timebase |
| TIM2 | — | Button debouncing |
| TIM3 | PA6 | PWM output (boiler modulation) |
| TIM4 | — | Modbus inter-character/frame timing |
| RTC | — | Real-time clock (LSE 32.768 kHz) |
| EXTI | PA7, PA10, PA11, PA12 | Water flow meter + 3 navigation buttons |

### Pin Mapping

| Pin | Label | Function |
|-----|-------|----------|
| PA0 | DHT22_1 | DHT22 sensor 1 |
| PA1 | TR_1 | ADC thermistor 1 |
| PA2 | TR_2 | ADC thermistor 2 |
| PA3 | TR_4 | ADC thermistor 4 |
| PA4 | TR_3 | ADC thermistor 3 |
| PA5 | wtr_hm_in | Water heating home relay |
| PA6 | Emul_PhotoRez | TIM3 PWM output |
| PA7 | Wtr_flow_met | Water flow meter (EXTI) |
| PA8 | DISP_RES | Display reset |
| PA9 | DISP_BLK | Display backlight |
| PA10 | ER11_LINE1 | Left button (EXTI) |
| PA11 | ER11_LINE2 | Right button (EXTI) |
| PA12 | ER11_BUTTON | Enter button (EXTI) |
| PA15 | WtoHS | Pump heating system relay |
| PB0 | PM_2 | ADC pressure sensor 2 |
| PB1 | PM_1 | ADC pressure sensor 1 |
| PB2 | RS485_RE | RS-485 transceiver enable |
| PB4 | CPW_HEAT_HOME | Circulation pump heating home relay |
| PB5 | WATER_VALVE | Water valve relay |
| PB6 | I2C1_SCL | I2C clock |
| PB7 | I2C1_SDA | I2C data |
| PB8 | CP_HOT_WATER | Circulation pump hot water relay |
| PB9 | Water_Heat_Home | Water heating home relay |
| PB10 | RS485_TX | USART3 TX |
| PB11 | RS485_RX | USART3 RX |
| PB12 | DISP_CS | Display chip select |
| PB13 | SPI2_SCK | Display SPI clock |
| PB14 | DISP_DC | Display data/command |
| PB15 | SPI2_MOSI | Display SPI data |
| PC13 | DHT22_2 | DHT22 sensor 2 |

## Architecture

### FreeRTOS Tasks

| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| `defaultTask` | Normal (2) | 128 words | I2C EEPROM communication, Modbus init |
| `sensReadTask` | Normal (2) | 128 words | Read all sensors (DHT22, thermistors, pressure) |
| `dispTask` | Normal (2) | 400 words | Display rendering, menu, button handling |

### Project Structure

```
Core/
  Inc/                  # Headers
  Src/                  # Main source (main.c, freertos.c, ISRs)
  Startup/              # Vector table & startup code
  ili9341/              # ILI9341 LCD + XPT2046 touch drivers
  modbus/               # SimpleModbusSlave (RTU, functions 3 & 16)
  my_file/              # Custom application code → renamed to `app/`
    mylib.h/c           # → hal_utils.h/c
    my_sensors.h/c      # → thermistor_table.h/c
    mymenu.h/c          # → menu_strings.h/c
    my_disp_lib.h/c     # → lcd_ui.h/c
    mytime.h/c          # → dwt_timer.h/c
Middlewares/FreeRTOS/   # FreeRTOS V10.0.1 + CMSIS-RTOS
Drivers/                # STM32 HAL + CMSIS
```

## Sensors

| Sensor | Interface | Details |
|--------|-----------|---------|
| DHT22 (x2) | Bit-banged single-wire (PA0, PC13) | Temp + humidity, read alternately every ~72s |
| NTC Thermistors (x4) | ADC (PA1-PA4) | 1000-sample avg, 151-entry lookup table for linearization |
| Pressure sensors (x2) | ADC (PB0, PB1) | Voltage → bar conversion with pump control logic |
| Water flow meter | EXTI7 (PA7) | Pulse counter in ISR |

## Communication

- **RS-485 Modbus RTU:** USART3, 57600 8N1, slave ID=1, functions 3 (read) and 16 (write)
- **I2C EEPROM:** I2C1, 100 kHz, device address 0x50 (24C32/24C64)

## Display

- ILI9341 2.4" TFT LCD (240x320, 16-bit color)
- SPI2 with DMA for transfers
- XPT2046 touch controller (defined but not fully integrated)
- Menu system with data display (14 items) and setpoints (19 items — 13 original + 6 time editing)
- All UI text in Russian (CP1251 encoding)

### Time Editing UI

Six new setpoints items were added for RTC time configuration:
- **Часы** (Hours, 0–23) — also acts as enter/exit edit mode toggle
- **Мин.** (Minutes, 0–59)
- **Сек.** (Seconds, 0–59)
- **День** (Day, 1–31)
- **Месяц** (Month, 1–12)
- **Год** (Year, 0–99)

Editing flow: Press Enter on "Часы" to load current RTC time into temp vars, then use Left/Right buttons to increment/decrement any value, and press Enter on "Часы" again to write to RTC. A new `ITTIME` type distinguishes the entry-point item from regular `ITINT` items.

## Build (Linux)

### Prerequisites

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

### Build Commands

```bash
make                    # Build Debug (default)
make BUILD_TYPE=Debug   # Build Debug
make BUILD_TYPE=Release # Build Release
make clean              # Remove current build artifacts
make clean_all          # Remove both Debug/ and Release/
```

### Output Files

| File | Description |
|------|-------------|
| `BoilerRoomCntrl.elf` | ELF binary for debugging |
| `BoilerRoomCntrl.bin` | Raw binary for flashing |
| `BoilerRoomCntrl.hex` | Intel HEX format |
| `BoilerRoomCntrl.map` | Linker map file |

## Linux Build Fixes

The following changes were made to ensure compatibility with modern ARM GCC (13.x) on Linux:

1. **`sysmem.c`**: Replaced deprecated `caddr_t` with `intptr_t`
2. **Headers**: Added `extern` keyword to global variable declarations to prevent multiple definition errors:
   - `main.h` — all global variables
   - `DHT.h` — `data[]`, `_pin`, `_type`, `_count`, `_lastreadtime`, `firstreading`
   - `ili9341.h` — `ILI9341_SPI_PORT`
   - `mylib.h` — `enChannelsTr`, `enChannelsPm`
   - `mymenu.h` — menu string arrays
   - `my_disp_lib.h` — `dm` struct
3. **`main.c`**: Added actual variable definitions for all `extern` declarations
4. **Makefile**: Created standalone Makefile (no longer depends on STM32CubeIDE-generated files)
5. **Linker flags**: Removed `syscalls.c` and `sysmem.c` from build, using `libnosys` stubs instead

## Known Issues / Notes

1. ~~**No mutex/semaphore for shared data:**~~ **FIXED** — `sensorDataMutexHandle` protects `DHT22Temp`, `DHT22Hum`, `travg[]`, and `pmavg[]` with `osMutexWait`/`osMutexRelease` in both `sensReadTask` and `dispTask`.
2. ~~**`millis()` returns DWT cycle count, not milliseconds:**~~ **FIXED** — `millis()` now returns actual milliseconds by dividing DWT cycle count by `SystemCoreClock / 1000`. All callers using the `72000` multiplier have been corrected.
3. **Two display drivers:** Both `ili9341.c` and `disp_spi.c` (ST7789VW) exist. Active code uses `ili9341.c`.
4. ~~**Duplicate pin assignment:**~~ **NOT A BUG** — `Water_Heat_Home` is PB9 and `wtr_hm_in` is PA5. They are different pins. The `rel_manage[]` array is correct.
5. **Touch panel not fully configured:** SPI1 for XPT2046 is referenced but not configured in `.ioc`.
6. **ITFLOAT editing stub:** The value increment/decrement logic for ITFLOAT items is still a placeholder (empty bodies in the edit loop). ITINT items for time editing are fully implemented.

## Branches

- `main` — main development branch
- `linux-build` — branch for testing Linux build compatibility

## Qwen Added Memories
- Before committing changes, always edit README.md and QWEN.md files to reflect the changes. All comments and documentation must be written in English.
- After applying fixes, always run debug and verify the program on the controller does not crash for at least 20 seconds.
- Always run debug sessions through ./debug.sh instead of manually running openocd/gdb commands. Use ./debug.sh check for stability verification, ./debug.sh interactive for interactive debugging.
- All git commit messages and comments must be written in English.
