# Makefile

# Output file name
TARGET = OS

# Linker script file
LINKER_FILE = linker_script.ld

# MCU specification
MCU_SPEC = cortex-m3

# Toolchain definitions (ARM bare metal defaults)
TOOLCHAIN = /usr
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
CXX = $(TOOLCHAIN)/bin/arm-none-eabi-g++
OC = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
OS = $(TOOLCHAIN)/bin/arm-none-eabi-size

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -g

# CPP flags
CPPFLAGS = -DSTM32F103xB \
		   -Ivendor/CMSIS/Device/ST/STM32F1/Include \
		   -Ivendor/CMSIS/CMSIS/Core/Include
CPPFLAGS += --specs=nano.specs
CPPFLAGS += -fmessage-length=0 -fno-common
CPPFLAGS += -ffunction-sections -fdata-sections
CPPFLAGS += -fno-exceptions
CPPFLAGS += -fno-rtti
CPPFLAGS += -std=c++11
CPPFLAGS += -g

# Linker directives
LSCRIPT = ./$(LINKER_FILE) 
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -T$(LSCRIPT)

SRC_DIR = src
C_SRC = $(wildcard $(SRC_DIR)/*.c)
CPP_SRC = $(wildcard $(SRC_DIR)/*.cpp)

# Object files
OBJS = $(C_SRC:.c=.o)
OBJS += $(CPP_SRC:.cpp=.o)

.PHONY: all

all: $(TARGET).bin

system_stm32f1xx.o: vendor/CMSIS/Device/ST/STM32F1/Source/Templates/system_stm32f1xx.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

:%.o: %.c
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CXX) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -O binary $< $@
	$(OS) $< 
	rm -f src/*.o

clean:
	rm -f *.o *.elf


reb:
	rm -f *.o *.elf
	rm -f src/*.o

PROGRAMMER = openocd
PROGRAMMER_FLAGS = -f interface/stlink.cfg -f target/stm32f1x.cfg

.PHONY: flash
flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8000000

