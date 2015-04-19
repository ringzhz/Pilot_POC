"C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude" "-CC:avrdude.conf" -v -patmega328p -carduino -PCOM4 -b115200 -D "-Uflash:w:ArduinoPilot.hex:i"

pause