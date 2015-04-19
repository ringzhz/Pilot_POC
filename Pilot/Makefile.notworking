bin = $(BUILDROOT)\$(PROJECT)\bin\
obj = $(BUILDROOT)\$(PROJECT)\obj\

# can we get these from environment?
arduino = "C:\Program Files (x86)\Arduino\"
includes = -I$(PROJECT) -I$(Arduino)hardware\arduino\avr\cores\arduino -I$(Arduino)hardware\arduino\avr\variants\standard -I$(Arduino)hardware\arduino\avr\libraries\Wire 

hex = $(O)Pilot.hex

sources = Pilot.ino MPU6050.cpp I2Cdev.cpp core.a

# pick and choose only what you need!
# objs = I2Cdev.cpp.o MPU6050.cpp.o Pilot.ino.o \
#		HardwareSerial0.cpp.o HardwareSerial1.cpp.o HardwareSerial2.cpp.o HardwareSerial3.cpp.o HID.cpp.o IPAddress.cpp.o main.cpp.o new.cpp.o \
#		Print.cpp.o Stream.cpp.o Tone.cpp.o USBCore.cpp.o WMath.cpp.o WString.cpp.o core.cpp.o \
#		Wire.cpp.o abi.cpp.o CDC.cpp.o \
#		twi.c.o ArduinoJson.cpp.o hooks.c.o WInterrupts.c.o  wiring.c.o wiring_analog.c.o wiring_digital.c.o wiring_pulse.c.o \
#		wiring_shift.c.o 

objs = I2Cdev.cpp.o MPU6050.cpp.o Pilot.ino.o Wire.cpp.o twi.c.o ArduinoJson.cpp.o core.a 

cc = $(Arduino)hardware/tools/avr/bin/avr-g++ -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics \
		-MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10601 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR $(Includes) 

all: $(hex)

.cpp.o:
	$(cc) $*.cpp -o $(t)$*.cpp.o

.c.o:
	$(cc) $*.c -o $(t)$*.c.o

.ino.o:
	$(cc) $*.ino -o $(t)$*.ino.o

.hex:
		
\\\\\\\\\\\\\\\\

:\Program Files (x86)\Arduino/hardware/tools/avr/bin/avr-ar rcs C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/core.a \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\hooks.c.o 
C:\Program Files (x86)\Arduino/hardware/tools/avr/bin/avr-ar rcs \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/core.a

C:\Program Files (x86)\Arduino/hardware/tools/avr/bin/avr-gcc -w -Os -Wl,--gc-sections -mmcu=atmega328p \
		-o C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/Pilot.cpp.elf \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\I2Cdev.cpp.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\MPU6050.cpp.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\Pilot.cpp.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\Wire\Wire.cpp.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\Wire\utility\twi.c.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp\ArduinoJson\ArduinoJson.cpp.o \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/core.a \
		-LC:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp -lm 

C:\Program Files (x86)\Arduino/hardware/tools/avr/bin/avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/Pilot.cpp.elf \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/Pilot.cpp.eep 

C:\Program Files (x86)\Arduino/hardware/tools/avr/bin/avr-objcopy -O ihex -R .eeprom C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/Pilot.cpp.elf \
		C:\Users\mikep\AppData\Local\Temp\build8110850230209665533.tmp/Pilot.cpp.hex 