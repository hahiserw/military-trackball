DEVICE  = atmega8
FUSE_L  = 0x9f
FUSE_H  = 0xc9
F_CPU   = 16000000

#CONF_FILE = /home/hahi/.avrdude.conf

# In order to use /dev/parport0 you need to do modprobe ppdev
#DUDE    = avrdude -p $(DEVICE) -P /dev/parport0 -C $(CONF_FILE) -c avrsp-lpt-hahi-novcc
DUDE    = avrdude -p $(DEVICE) -c usbasp

C       = avr-gcc
CFLAGS  = -Wall -Os -DF_CPU=$(F_CPU) -Iusbdrv -I. -DDEBUG_LEVEL=0 -mmcu=$(DEVICE)
# OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbrdv/oddebug.o main.o
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o main.o
# OBJECTS = main.o


military-trackball: main.hex
	@#


help:
	@echo "Compile code to hex file by default"
	@echo "make fuses    flash fuses in your device"
	@echo "make install  flash device with hex file"

fuses:
	@[ "$(FUSE_L)" != "" -a "$(FUSE_H)" != "" ] || \
		{ echo "Fuses in Makefile not set"; exit 1; }
	$(DUDE) -U lfuse:w:$(FUSE_L):m -U hfuse:w:$(FUSE_H):m

install: main.hex
	$(DUDE) -U flash:w:$<:i

backup:
	$(DUDE) -U flash:r:backup-`date +%F-%T`:i

clean:
	rm -f *.o usbdrv/*.o usbdrv/usbdrv.s usbdrv/oddebug.s
	rm -f main.s main.lst main.obj main.cof main.list main.map
	rm -f main.eep.hex main.elf main.hex

.c.o:
	$(C) $(CFLAGS) -c $< -o $@

.S.o:
	$(C) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(C) $(CFLAGS) -S $< -o $@


main.elf: usbdrv $(OBJECTS)
	$(C) $(CFLAGS) -o main.elf $(OBJECTS)

main.hex: main.elf usbconfig.h
	rm -f main.hex main.eep.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size main.hex

