# Standalone Makefile for BoilerRoomCntrl
# Build on Linux with arm-none-eabi-gcc

TARGET   := BoilerRoomCntrl.elf
CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size

# MCU flags
CPU      := -mcpu=cortex-m3
FPU      :=
FLOAT-ABI := -mfloat-abi=soft
THUMB    := -mthumb
CSTD     := -std=gnu11

CFLAGS   := $(CPU) $(CSTD) -g3 $(FLOAT-ABI) $(THUMB)
CFLAGS   += -DUSE_HAL_DRIVER -DSTM32F103xB
CFLAGS   += -ffunction-sections -fdata-sections
CFLAGS   += -Wall -fstack-usage
CFLAGS   += -MMD -MP

# Include paths
INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
  -IDrivers/CMSIS/Include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3

# Linker flags
LDSCRIPT := STM32F103CBTX_FLASH.ld
LDFLAGS  := $(CPU) $(FLOAT-ABI) $(THUMB)
LDFLAGS  += -T$(LDSCRIPT)
LDFLAGS  += -Wl,--gc-sections
LDFLAGS  += -Wl,--print-memory-usage
LDFLAGS  += -Wl,-Map=$(TARGET:.elf=.map)
LDFLAGS  += --specs=nano.specs -lc -lnosys -lm

# Source files
C_SRCS := \
  Core/Src/main.c \
  Core/Src/stm32f1xx_hal_msp.c \
  Core/Src/stm32f1xx_hal_timebase_tim.c \
  Core/Src/stm32f1xx_it.c \
  Core/Src/freertos.c \
  Core/Src/DHT.c \
  Core/Src/disp_spi.c \
  Core/Src/system_stm32f1xx.c \
  Core/ili9341/ili9341.c \
  Core/ili9341/ili9341_touch.c \
  Core/ili9341/fonts.c \
  Core/modbus/SimpleModbusSlave.c \
  Core/my_file/my_disp_lib.c \
  Core/my_file/my_sensors.c \
  Core/my_file/mylib.c \
  Core/my_file/mymenu.c \
  Core/my_file/mytime.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/port.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c

ASM_SRCS := \
  Core/Startup/startup_stm32f103cbtx.s

# Object files
OBJS := $(C_SRCS:.c=.o) $(ASM_SRCS:.s=.o)
DEPS := $(OBJS:.o=.d)

# Default target
all: $(TARGET) $(TARGET:.elf=.bin) $(TARGET:.elf=.hex) size

# Link
$(TARGET): $(OBJS)
	@echo "Linking $@"
	$(CC) $(LDFLAGS) $^ -o $@

# Generate .bin
%.bin: %.elf
	@echo "Generating $@"
	$(OBJCOPY) -O binary $< $@

# Generate .hex
%.hex: %.elf
	@echo "Generating $@"
	$(OBJCOPY) -O ihex $< $@

# Compile C
%.o: %.c
	@echo "Compiling $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Assemble
%.o: %.s
	@echo "Assembling $<"
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Memory usage
size: $(TARGET)
	@echo ""
	$(SIZE) $(TARGET)

# Clean
clean:
	rm -rf $(OBJS) $(DEPS) $(TARGET) $(TARGET:.elf=.map) $(TARGET:.elf=.bin) $(TARGET:.elf=.hex)

# Dependencies
-include $(DEPS)

.PHONY: all clean size
