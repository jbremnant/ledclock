# In order to use internal RC to run at 8MHz (max internal clock speed)
#   avrdude -F -p m168 -P usb -c avrispmkII -U lfuse:w:0xE2:m
#
# To read the fuse settings:
#   avrdude -F -p m168 -P usb -c avrispmkII -U lfuse:r:-:h -U hfuse:r:-:h
#
# Good source for this info:
# 
# lfuse: 0xe2 
# 			 0xe6  for 16MHz external crystal
# hfuse: 0xdf  0b1001 1011
#
# To change clock setting on the programmer:
# jbkim@homelnx:/nas/dev/c/embedded/nerdkit/projects/clockdisplay$ avrdude -V -F -P usb -c avrispmkII -p m8 -u -t -s
#
# avrdude: AVR device initialized and ready to accept instructions
#
# Reading | ################################################## | 100% 0.05s
#
# avrdude: Device signature = 0x1e940b
# avrdude: Expected signature for ATMEGA8 is 1E 93 07
# avrdude> parms
# >>> parms
# Vtarget         : 0.0 V
# SCK period      : 32.12 us
# avrdude> sck 10
# >>> sck 10
# avrdude> parms
# >>> parms
# Vtarget         : 0.0 V
# SCK period      : 10.37 us
# avrdude> quit
# >>> quit
#
# 	http://www.sparkfun.com/commerce/tutorial_info.php?tutorials_id=95

TARGET=clockdisplay
PORT=/dev/ttyUSB0

MCU = atmega168
# CPU_FREQ = 16000000
# CPU_FREQ = 14745600
CPU_FREQ = 8000000
TWI_FREQ = 400000 
UPLOAD_RATE = 115200  # 57600
AVRDUDE_PROGRAMMER = stk500

GCCFLAGS=-g -Os -Wall -mmcu=$(MCU) -DCPU_FREQ=$(CPU_FREQ) -DF_CPU=$(CPU_FREQ) -DTWI_FREQ=$(TWI_FREQ)
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm

AVRDUDE = avrdude
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_FLAGS=-V -F -c avr109 -p m168 -P $(PORT) -b $(UPLOAD_RATE) $(AVRDUDE_WRITE_FLASH)
AVRDUDE_FLAGS_ISP=-V -F -P usb -c avrispmkII -p m168 $(AVRDUDE_WRITE_FLASH)
# -c $(AVRDUDE_PROGRAMMER) -b $(UPLOAD_RATE)

all:	$(TARGET).hex
upload: $(TARGET)-upload 
uploadisp: $(TARGET)-uploadisp
clean:
	rm *.o *.hex

ht1632.o: ht1632.c
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o ht1632.o ht1632.c

uart.o: uart.c
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -c -o uart.o uart.c

$(TARGET).hex: $(TARGET).c ht1632.o uart.o
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o $(TARGET).o ht1632.o uart.o $(TARGET).c
	avr-objcopy -j .text -j .data -O ihex $(TARGET).o $(TARGET).hex
	
$(TARGET).ass:	$(TARGET).hex
	avr-objdump -S -d $(TARGET).o > $(TARGET).ass
	
$(TARGET)-upload:	$(TARGET).hex
	./pulsedtr.py $(PORT)
	$(AVRDUDE) $(AVRDUDE_FLAGS)

$(TARGET)-uploadisp:	$(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS_ISP)
