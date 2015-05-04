//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define NOLIMIT 0x7fffffff

float Kp1 = .4, Ki1 = 0, Kd1 = .01;		// motor velocity regulator
float Kp2 = .1, Ki2 = 0.01, Kd2 = 1;	// pilot regulator

// pilot regulator
float lastHeadaing = 0, tgtHeading;
float travelX = 0, travelY = 0;
unsigned long lastTickTime;
float previousError, previousIntegral, previousDerivative;
bool traveling = false;

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
	long lastPoseTacho;

	// used by regulator via tick
	long tickTacho, lastTickTacho;

	// regulator variables
	// speed/velocity are ticks per second

	bool moving;
	bool checkLimit;
	
	//unsigned long baseTime;
	long limit;
	float power;
	float previousIntegral, previousDerivative;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

public:
	PilotMotor(const char *name, unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	void SetSpeed(int speed, int acceleration, long limit);
	void Stop(bool sudden);
	void Tick(unsigned int elapsed);

private:
	float ZdomainXferPid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& iState, float& dState);
	void EndMove(bool stalled);

protected:
	friend void PilotRegulatorTick();
	friend bool CalcPose();
	long GetRawTacho() { return reversed ? -rawTacho[interruptIndex] : rawTacho[interruptIndex]; }
	void PinPower(int power);
};

extern PilotMotor M1, M2;

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
	{
		b ? rawTacho[0]-- : rawTacho[0]++;
	}
	else
	{
		b ? rawTacho[0]++ : rawTacho[0]--;
	}
}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
	{
		b ? rawTacho[1]-- : rawTacho[1]++;
	}
	else
	{
		b ? rawTacho[1]++ : rawTacho[1]--;
	}
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
		Reset();
	}
}

void PilotMotor::Reset()
{
	SetSpeed(0, 0, NOLIMIT);	
	rawTacho[interruptIndex] = lastTickTacho = lastPoseTacho = 0L;
	previousDerivative = previousIntegral = 0;	
	lastTickTime = millis();
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
	//Serial.println("//SetSpeed");
	// 185 RPM motor, 30 ticks per rotation
	tgtVelocity = setSpeed / 100.0 * 450;	// speed as % times max ticks per ???? speed
	limit = setLimit;
	checkLimit = abs(setLimit) != NOLIMIT;
	moving = tgtVelocity != 0;
	previousDerivative = previousIntegral = 0;

	//Serial.print("// setLimit="); Serial.println(setLimit);
	//Serial.print("// tgtVelocity="); Serial.println(tgtVelocity);

	if (setSpeed > 0 && velocity == 0)
		PinPower(setSpeed >= 0 ? 40 : -40);		// minimum kick to get motor moving
}

void PilotMotor::Stop(bool immediate)
{
	SetSpeed(0, 0, NOLIMIT);
}

void PilotMotor::EndMove(bool stalled)
{
	moving = false;
	// +++ publish event?
}

#define iMax 1.1
#define iMin .001

float PilotMotor::ZdomainXferPid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& iState, float& dState)
{
	// at the moment this is just pid, ignoring interval
	// but will become; http://www.wescottdesign.com/articles/zTransform/z-transforms.html
	float error = setPoint - presentValue;
	double pTerm, dTerm, iTerm;
	pTerm = Kp * error;
	iState = constrain(iState + error, iMin, iMax);
	iTerm = Ki * iState;
	dTerm = Kd * (dState - error);
	dState = error;
	return pTerm + dTerm + iTerm;
}

void PilotMotor::Tick(unsigned int eleapsed)
{
	velocity = (tickTacho - lastTickTacho) * 1000 / eleapsed;	// TPS

	if (moving)
	{
		//Serial.println("//moving");
		//Serial.print("// tgtVelocity="); Serial.println(tgtVelocity);
		//Serial.print("// velocity="); Serial.println(velocity);

		float pid = ZdomainXferPid(tgtVelocity, velocity, Kp1, Ki1, Kd1, previousIntegral, previousDerivative);
		//Serial.print("// pid="); Serial.println(pid);

		power = constrain(power + pid, -100, 100);
		//Serial.print("// power="); Serial.println(power);
	}
	lastTickTacho = tickTacho;
}

//----------------------------------------------------------------------------

float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& previousIntegral, float& previousDerivative, float dt)
{
	if (dt <= 0)
		return 0;
	float error = setPoint - presentValue;
	float integral = previousIntegral + error * dt;
	float derivative = (error - previousError) / dt;
	float output = Kp * error + Ki * integral + Kd * derivative;
	previousIntegral = integral;
	previousDerivative = derivative;
	previousError = error;
	return output;
}

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

	lastHeadaing = tgtHeading = travelX = travelY = 0;
	previousError = previousIntegral = previousDerivative = 0;

	escEnabled = false;
}

float Distance(float x1, float y1, float x2, float y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2* y1));
}

void Travel(float x, float y, int speed)
{
	travelX = x;
	travelY = y;
	previousError = previousIntegral = previousDerivative = 0;
	traveling = true;
	M1.SetSpeed(speed, 0, +NOLIMIT);
	M2.SetSpeed(speed, 0, +NOLIMIT);
}

void Travel(float distance, int speed)
{
	Travel(X + (distance * cos(H)), Y - (distance * sin(Y)), speed);
}

void PilotRegulatorTick()
{
	unsigned long now = millis();
	unsigned int tickElapsedTime = now - lastTickTime;

	float adjustment = 0;

	// +++ pilot regulation not tested in any way, waiting motors

	M1.tickTacho = M1.GetRawTacho();
	M2.tickTacho = M2.GetRawTacho();

	M1.Tick(tickElapsedTime);
	M2.Tick(tickElapsedTime);

	if (traveling)
	{
		float headingTo = atan2(travelY - Y, travelX - X);
		
		adjustment = Pid(headingTo, H, Kp2, Ki2, Kd2, previousError, previousIntegral, previousDerivative, tickElapsedTime);

		// +++ sanity check?

		if (Distance(travelX, X, travelY, Y) < .1)
		{
			traveling = false;
			adjustment = M1.power = M2.power = 0;
		}
	}

	// +++ ignoring heading for now
#if 0
	if (adjustment >= 0)
	{
		M1.PinPower = constrain(M1.power + abs(adjustment, -100, 100);
		M2.PinPower = constrain(M2.power - abs(adjustment), -100, 100);
	}
	else
	{
		M1.PinPower = constrain(M1.power - abs(adjustment, -100, 100);
		M2.PinPower = constrain(M2.power + abs(adjustment), -100, 100);
	}
#endif

	M1.PinPower(M1.power);
	M2.PinPower(M2.power);
	lastTickTime = now;
}


