port=com4
srcPath = C:\Users\mikep\Documents\MWPRobotics\Pilot\Pilot
bldPath = $(srcPath)\build
arduinoCmd = "C:\Program Files (x86)\Arduino\arduino.exe"
arduinoFlags = --upload --board arduino:avr:uno --port $(port) --preserve-temp-files --preferences-file $(srcPath)\pilotPrefs.txt
prefs = --pref sketchbook.path=$(srcPath) --pref build.path=$(bldPath)

deps = $(srcPath)\Pilot.ino $(srcPath)\commands.h $(srcPath)\publish.h $(srcPath)\motor.h 

pilot.hex : $(deps)
#	type $(srcPath)\Pilot.ino
	$(arduinoCmd) $(arduinoFlags) $(prefs) $(srcPath)\Pilot.ino
# problem arduino exits long before it finishes :|
	copy $(bldPath)\Pilot.cpp.hex $(srcPath)\Pilot.hex