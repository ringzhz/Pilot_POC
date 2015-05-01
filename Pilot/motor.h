//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define NOLIMIT 0x7fffffff

float Kp1 = 4, Ki1 = .05, Kd1 = 10;		// per motor regulator
float Kp2, Ki2, Kd2;					// synchronizing (pilot) regulator

volatile long rawTacho[2];		// interrupt 0 & 1 tachometers

extern Geometry Geom;

// a motor is generally not accessed directly, but by the pilot regulator who also controls heading
// re-write 5th attempt :|

class PilotMotor
{
public:
	char motorName[3];
	char reserved1 = '\0';

	// pin level
	byte pwmPin;
	byte dirPin;
	byte feedBackPin;
	byte interruptIndex;
	bool reversed;

	// used by pose
	long lastTacho;	

	// set by regulator via tick
	long tacho;

	// regulator variables
	// speed/velocity are ticks per second

	bool moving;
	bool checkLimit;
	
	unsigned long baseTime;
	unsigned long lastTickTime;
	long limit;
	float power;
	float previousError, previousIntegral, previousDerivative;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

public:
	PilotMotor(const char *name, unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	void SetSpeed(int speed, int acceleration, long limit);
	void Stop(bool sudden);
	void Tick();

private:
	long GetRawTacho() { return reversed ? -rawTacho[interruptIndex] : rawTacho[interruptIndex]; }
	void EndMove(bool stalled);

protected:
	void PinPower(int power);
	friend void PilotRegulatorTick();
};

extern PilotMotor M1, M2;

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
		b ? rawTacho[0]++ : rawTacho[0]--;
	else
		b ? rawTacho[0]-- : rawTacho[0]++;
	}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
		b ? rawTacho[1]++ : rawTacho[1]--;
	else
		b ? rawTacho[1]-- : rawTacho[1]++;
}

PilotMotor::PilotMotor(const char *name, unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd)
{
	strncpy(motorName, name, sizeof(motorName));
	pwmPin = pwm;
	dirPin = dir;
	feedBackPin = fb;
	interruptIndex = idx;
	reversed = revrsd;

	if (escEnabled)
	{
		pinMode(pwm, OUTPUT);
		pinMode(dir, OUTPUT);
	}
	Reset();
}

void PilotMotor::Reset()
{
	SetSpeed(0, 0, NOLIMIT);	
	rawTacho[interruptIndex] = lastTacho = 0L;
	previousError = previousDerivative = previousIntegral = 0;
	baseTime = millis();
}

void PilotMotor::PinPower(int p)
{
	unsigned short newDir = LOW;
	int realPower = 0;

#if 0
	Serial.print("//PinPower p="); Serial.println(p);
	Serial.print("// power="); Serial.println(power);
#endif
	newDir = (p >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
	realPower = map(abs(p), 0, 100, 0, 255);
	digitalWrite(dirPin, newDir);
	analogWrite(pwmPin, realPower);
}

// limit actually sets the direction, use +NOLIMIT/-NOLIMIT for continuous
void PilotMotor::SetSpeed(int setSpeed, int setAccel, long setLimit)
{
	Serial.print("//SetSpeed\n");
	// 185 RPM motor, 30 ticks per rotation
	tgtVelocity = (setSpeed / 100.0) * (185.0 * 30.0 / 60.0);	// speed as % times max ticks per sec speed
	limit = setLimit;
	checkLimit = abs(setLimit) != NOLIMIT;
	moving = tgtVelocity != 0;
	baseTime = millis();

	Serial.print("// setLimit="); Serial.println(setLimit);
	Serial.print("// tgtVelocity="); Serial.println(tgtVelocity);
}

void PilotMotor::Stop(bool immediate)
{
}

void PilotMotor::EndMove(bool stalled)
{
	moving = false;
	// +++ publish event?
}

void PilotMotor::Tick()
{
	unsigned long now = millis();
	unsigned long moveElapsedTime = now - baseTime;
	unsigned long tickElapsedTime = now - lastTickTime;
	int error;
	tacho = GetRawTacho();

	if (moving)
	{
		Serial.println("//Tick moving");
		velocity = tacho - lastTacho / tickElapsedTime / 1000;

		Serial.print("// tgtVelocity="); Serial.println(tgtVelocity);
		Serial.print("// velocity="); Serial.println(velocity);
		Serial.print("// tacho="); Serial.println(tacho);

		// PID
		power += constrain(Pid(tgtVelocity, velocity, Kp1, Ki1, Kd1, previousError, previousIntegral, previousDerivative, tickElapsedTime / 1000), -100, 100);
		PinPower(power);

		lastTickTime = now;
	}
}

//----------------------------------------------------------------------------

void MotorInit()
{
	rawTacho[0] = rawTacho[1] = 0L;

	// !! hardcoded interrupt handlers !!
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);
	pinMode(8, INPUT_PULLUP);
	pinMode(9, INPUT_PULLUP);

	attachInterrupt(PCINT0, MotorISR1, CHANGE);	// pin 2
	attachInterrupt(PCINT1, MotorISR2, CHANGE); // pin 3

	if (escEnabled)
	{
		pinMode(M1_PWM, OUTPUT);
		digitalWrite(M1_PWM, 0);
		pinMode(M2_PWM, OUTPUT);
		digitalWrite(M2_PWM, 0);
		pinMode(M1_DIR, OUTPUT);
		digitalWrite(M1_DIR, 0);
		pinMode(M2_DIR, OUTPUT);
		digitalWrite(M2_DIR, 0);
		M1.Reset();
		M2.Reset();
	}

	escEnabled = false;
}

void PilotRegulatorTick()
{
	// +++ regulation needed now! but waiting on hardware :|
	/*
	Zieglerï¿½Nichols method
		Control Type	K_p		K_i				K_d
		P			0.50{K_u}	--
		PI			0.45{K_u}	1.2{K_p} / P_u	-
		PID			0.60{K_u}	2{K_p} / P_u	{ K_p }{P_u} / 8
	*/

	M1.Tick();
	//M2.Tick();
}


