MCU      := atmega328p
F_CPU_HZ := 16000000UL
CC       := avr-gcc
OBJCOPY  := avr-objcopy
SIZE     := avr-size
AVRDUDE  := avrdude

TARGET   := hovercraft
SRC_DIR  := src
INC_DIR  := include
SRCS     := $(wildcard $(SRC_DIR)/*.c)
OBJS     := $(SRCS:.c=.o)
PROGRAMMER := arduino
PORT       := /dev/ttyUSB0
BAUD       := 115200

CFLAGS   := -mmcu=$(MCU) -DF_CPU=$(F_CPU_HZ) -I. -I$(INC_DIR) -Os -std=gnu11 -Wall -Wextra
LDFLAGS  := -mmcu=$(MCU)

all: $(TARGET).hex size

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

size: $(TARGET).elf
	$(SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf

clean:
	rm -f $(SRC_DIR)/*.o $(TARGET).elf $(TARGET).hex

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p m328p -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex
	@echo "Example: avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 115200 -U flash:w:$(TARGET).hex"

.PHONY: all clean flash size
