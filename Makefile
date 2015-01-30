# Linux,Mac,Windows用avr-gcc用
# 最後に出力される text + data がプログラムメモリーの使用量です．

# デバイスと周波数[Hz]
DEVICE = atmega328p
F_CPU  = 8000000UL

# FUSES
LFUSE  = 0xf7
HFUSE  = 0xd9
EFUSE  = 0x07

# ソースコードと出力ファイル
CSOURCES = main.c micomfs.c micomfs_dev.c mpu9150.c ak8975.c i2c.c sd.c spi.c lps25h.c
SSOURCES =
TARGET   = main

# オプション
CFLAGS  = -Os -fshort-enums -g -Wall -mmcu=$(DEVICE) -DF_CPU=$(F_CPU)
LDFLAGS = -g -mmcu=$(DEVICE)
LINK	=
INCLUDE =

# 環境依存定数
MAKE    = make -r
AVRCC   = avr-gcc
OBJDUMP = avr-objdump
OBJCOPY = avr-objcopy
SIZE    = avr-size
AVRDUDE = avrdude
SIM     = ../../usr/bin/simulavr
ifeq ($(OS),Windows_NT)
    SUDO = 
else
    SUDO = sudo
endif

# 偽ルールとオブジェクトファイル保持
.PRECIOUS : $(CSOURCES:.c=.o)
.PRECIOUS : $(SSOURCES:.s=.o)
.PHONY    : $(TARGET) eeprom install fuse prompt rebuild sim clean run

# ルール
all : $(TARGET)

$(TARGET) : $(TARGET).hex

%.o : %.s
	$(AVRCC) $(CFLAGS) $(INCLUDE) -c $<

%.o : %.c
	$(AVRCC) $(CFLAGS) $(INCLUDE) -c $<

%.s : %.c
	$(AVRCC) $(CFLAGS) $(INCLUDE) -S $<

%.elf : $(CSOURCES:.c=.o) $(SSOURCES:.s=.o)
	$(AVRCC) $(LDFLAGS) -Wl,-Map,$(@:.elf=.map) $(LINK) -o $@ $+
	$(OBJDUMP) -h -S $@ > $(@:.elf=.lst)

%.hex : %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	$(SIZE) $<

eeprom : $(TARGET).elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $(TARGET).elf $(TARGET)_eeprom.hex

install : $(TARGET)
	$(SUDO) $(AVRDUDE) -P usb -c avrisp2 -p $(DEVICE) -U flash:w:$(TARGET).hex

fuse :
	$(SUDO) $(AVRDUDE) -P usb -c avrisp2 -p $(DEVICE) -u -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

prompt :
	$(SUDO) $(AVRDUDE) -P usb -c avrisp2 -p $(DEVICE) -t

sim : $(TARGET).elf
	$(SIM) -d $(DEVICE) -f $+ -g

clean :
	-rm $(CSOURCES:.c=.o)
	-rm $(SSOURCES:.s=.o)
	-rm *.lst
	-rm *.map
	-rm *.elf
	-rm *.hex

rebuild :
	$(MAKE) -B
	
run : $(TARGET)
	$(MAKE) install
	cu -l /dev/ttyUSB0 -s 4800


