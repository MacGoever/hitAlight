#!/bin/bash
rm hitAlight.cof  hitAlight.eep.hex  hitAlight.hex  hitAlight.obj
avra hitAlight.asm
#avr-objcopy -O ihex -R .eeprom hitAlight.elf hitAlight.hex
avrdude -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -p m16 -P /dev/ttyUSB0 -U flash:w:hitAlight.hex:i -c stk500
#avrdude -U lfuse:w:0xe2:m -U hfuse:w:0x19:m -p m16 -P /dev/ttyUSB0 -U flash:w:hitAlight.hex:i -c stk500
cp hitAlight.asm ~/blub.asm
