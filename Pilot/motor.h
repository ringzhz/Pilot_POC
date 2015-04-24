//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define STARTUP_POWER 40	// what it takes to move a motor
#define ACCELERATION 6000
#define NOLIMIT 0x7fffffff

float Kp1, Ki1, Kd1;		// per motor regulator
float Kp2, Ki2, Kd2;		// synchronizing regulator

volatile long tacho[2];		// interrupt 0 & 1 tachometers

extern Geometry Geom;

// a motor is generally not accessed directly, but by the regulator who also controls direction
// re-write 5th attempt :|

class PilotMotor
{
public:
	char motorName[3];
	char reserved1 = '\0';

	// pin level
	unsigned short	pwmPin;
	unsigned short	dirPin;
	unsigned short	feedBackPin;
	unsigned short	interruptIndex;
	bool			reversed;

	long lastTacho;	// used by pose

	// regulator variables
	// speed/velocity are ticks per second
	// 'new' variables are for pending move

	bool pending;
	bool moving;
	bool checkLimit;
	unsigned long baseTime;
	float baseVelocity, acceleration;
	unsigned long lastTickTime;
	unsigned long accelTime; // time at which an accel or de-accel will be completed
	long baseTacho, accTacho, limit;
	float power, basePower;
	float err1, err2;
	float previousIntegral;
	float previousDerivative;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

	float newAccel;
	long  newLimit;
	float newSpeed;

public:
	PilotMotor(const char *name, unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	long GetTacho() { return reversed ? -tacho[interruptIndex] : tacho[interruptIndex]; }
	void SetSpeed(unsigned short speed, unsigned int accelration, long limit);
	void Stop(bool sudden);
	float Pid1(float setPoint, float presentValue, float elasped);
	void Tick();

private:
	float CalcPower(float error, float Kp, float Ki, float Kd, float elapsed);
	void PinPower(int power);
	void NewMove(float speed, float accel, int limit);
	void SubMove(float speed, float accel, int limit);
	void EndMove(bool stalled);
};

extern PilotMotor M1, M2;

ISR(MotorISR1)
{
	int b = digitalReadFast(8);
	if (digitalReadFast(2))
		b ? tacho[0]++ : tacho[0]--;
	else
		b ? tacho[0]-- : tacho[0]++;
}

ISR(MotorISR2)
{
	int b = digitalReadFast(9);
	if (digitalReadFast(3))
		b ? tacho[1]++ : tacho[1]--;
	else
		b ? tacho[1]-- : tacho[1]++;
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
	SetSpeed(0, 1000, NOLIMIT);
	tacho[interruptIndex] = 0L;
	baseTacho = 0L;
	baseTime = millis();
}

void PilotMotor::PinPower(int p)
{
	unsigned short newDir = LOW;
	int realPower = 0;
#if 0
	Serial.print("// SetPwr p("); Serial.print(p); 
	Serial.print(") currentP("); Serial.print(power);
	Serial.print(")\n");
#endif

	if (power != p)
	{
		newDir = (p >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
		realPower = map(abs(p), 0, 100, 0, 255);
		digitalWrite(dirPin, newDir);
		analogWrite(pwmPin, realPower);
#if 0
		Serial.print("//  ");
		Serial.print(" pwi("); Serial.print(power); Serial.print(")");
		Serial.print(" sp("); Serial.print(spTPM); Serial.print(")");
		Serial.print(" pv("); Serial.print(spTPM); Serial.print(")");
		Serial.print(" pe("); Serial.print(previousError); Serial.print("),");
		Serial.print(" dir("); Serial.print(newDir); Serial.print(")");
		Serial.print(" pwo("); Serial.print(realPower); Serial.print(")\n");
#endif
		power = p;
	}
}

// limit actually sets the direction, use +NOLIMIT/-NOLIMIT for continuous
void PilotMotor::SetSpeed(unsigned short setSpeed, unsigned int setAccel, long setLimit)
{
	// +++spd = calculate 
	int spd = setSpeed;
	if (tgtVelocity != 0)
		SubMove(spd, setAccel, setLimit);
	if (pending)
		spd = setSpeed;
}

void PilotMotor::Stop(bool immediate)
{
	NewMove(0, immediate ? 0 : 1000, NOLIMIT);
}

float PilotMotor::CalcPower(float error, float Kp, float Ki, float Kd, float elapsed)
{
	err1 = 0.375f * err1 + 0.625f * error;	// fast smoothing
	err2 = 0.75f * err2 + 0.25f * error;	// slow smoothing
	float newPower = basePower + Kp * err1 + Kd * (err1 - err2) / elapsed;
	basePower = basePower + Ki * (newPower - basePower) * elapsed;
	basePower = constrain(basePower, -100, 100);
	return newPower;
}

void PilotMotor::NewMove(float moveSpeed, float moveAccel, int moveLimit)
{

}

void PilotMotor::SubMove(float moveSpeed, float moveAccel, int moveLimit)
{
	float absAcc = abs(moveAccel);
	checkLimit = abs(moveLimit) != NOLIMIT;
	baseTime = millis();
	if (!moving && abs(moveLimit - GetTacho()) < 3.0)
		tgtVelocity = 0;
	else
		tgtVelocity = (moveLimit - GetTacho()) >= 0 ? moveSpeed : -moveSpeed;

	acceleration = tgtVelocity - velocity >= 0 ? absAcc : -absAcc;
	accelTime = ((velocity - tgtVelocity) / acceleration) * 1000;
	accTacho = (velocity + tgtVelocity) * accelTime / (2 * 1000);
	baseTacho = GetTacho();
	baseVelocity = velocity;
	limit = moveLimit;
	moving = tgtVelocity != 0 || baseVelocity != 0;
}

void PilotMotor::EndMove(bool stalled)
{
	moving = pending;
	//++ stalled check
	if (pending)
	{
		pending = false;
		SubMove(newSpeed, newAccel, newLimit);
	}
}

void PilotMotor::Tick()
{
	unsigned long now = millis();
	unsigned long moveElapsed = now - baseTime;
	unsigned long tickElapsed = now - lastTickTime;
	float error;
	long expectedTacho;

	if (moving)
	{
		if (moveElapsed < accelTime)
		{
			velocity = (baseVelocity + (accTacho * moveElapsed)) / 1000;
			expectedTacho = (velocity + baseTacho) * moveElapsed / (2 * 1000);
			error = expectedTacho - GetTacho();
		}
		else
		{
			velocity = tgtVelocity;
			expectedTacho = baseTacho + accTacho + (velocity * (moveElapsed - accelTime)) / 1000;
			error = expectedTacho - GetTacho();
			// is move complete?
			if (tgtVelocity == 0 && (pending ||
					(abs(error) < 12 && moveElapsed > accelTime + 100) ||
					moveElapsed > accelTime + 500))
				EndMove(false);
		}

		// +++ stalled?

		// PID
		power = CalcPower(error, Kp1, Ki1, Kd1, (float)tickElapsed / 1000);
		PinPower(power);	
	}

	lastTickTime = now;
}

//----------------------------------------------------------------------------

void MotorInit()
{
	tacho[0] = tacho[1] = 0L;

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
}

bool moveInProgress = false;

void StartMove(float moveGoalX, float moveGoalY)
{
	// move does not block, and goalHeading is adjusted each tick
	// when move is complete an event is fired
}

void StartRotate(float rotateGoal)
{
	// rotate is blocking (and often not really needed, use goto)	
}



