CC=avr-g++
OBJCOPY=avr-objcopy
SIZE=avr-size
AVRDUDE=avrdude

MCU=attiny13
LFUSE=0x7B
HFUSE=0xFF
PROGRAMMER=usbasp


F_CPU=128000L
TARGET=beeper


SRCS=main.cpp
OBJECTS=$(SRCS:.cpp=.o)

CFLAGS=-Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -fno-tree-loop-optimize -DF_CPU=$(F_CPU)
LDFLAGS=-Wl,--gc-sections


all: hex size

hex: $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -R .eeprom -R .fuse -R .lock -R .signature -O ihex $(TARGET).elf  "$(TARGET).hex"

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(TARGET).elf

.cpp.o:
	$(CC) $(CFLAGS) -mmcu=$(MCU) $< -c -o $@

size: $(TARGET).elf
	$(SIZE) --format=berkeley -t $(TARGET).elf

fuse:
	$(AVRDUDE) -p$(MCU) -c$(PROGRAMMER) -U -lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

flash:
	$(AVRDUDE) -p$(MCU) -c$(PROGRAMMER) -Uflash:w:$(TARGET).hex:a

clean:
	rm *.o
	rm *.elf
	rm *.hex
