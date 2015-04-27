//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define NOLIMIT 0x7fffffff

float Kp1 = 4, Ki1 = 0.04, Kd1 = 10;		// per motor regulator
float Kp2, Ki2, Kd2;							// synchronizing (pilot) regulator

volatile long tacho[2];		// interrupt 0 & 1 tachometers

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

	// regulator variables
	// speed/velocity are ticks per second
	// 'new' variables are for pending move
	// 'base' is what something was when the move started
	bool pending;
	bool moving;
	bool checkLimit;
	
	unsigned long baseTime;
	float baseVelocity, acceleration;
	unsigned long lastTickTime;
	long accelTime;				// time it will take for an accel/deaccel
	long baseTacho, accTacho;	// how many tachos to complete accel/deaccel
	long limit;
	float power, basePower;
	float err1, err2;
	float previousIntegral, previousDerivative;
	float tgtVelocity;		// is what we asked for
	float velocity;			// is what we have 

	float newAccel;
	long  newLimit;
	float newSpeed;

public:
	PilotMotor(const char *name, unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
	void Reset();
	long GetTacho() { return reversed ? -tacho[interruptIndex] : tacho[interruptIndex]; }
	void SetSpeed(int speed, int acceleration, long limit);
	void Stop(bool sudden);
	void Tick();

private:
	void CalcPower(float error, float Kp, float Ki, float Kd, float elapsed);
	void NewMove(float speed, float accel, long limit);
	void SubMove(float speed, float accel, long limit);
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
	pending = false;
	SetSpeed(0, 0, NOLIMIT);	
	tacho[interruptIndex] = lastTacho = 0L;
	baseVelocity = err1 = err2 = previousDerivative = previousIntegral = 0;
}

void PilotMotor::PinPower(int p)
{
	unsigned short newDir = LOW;
	int realPower = 0;

#if 1
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
	float tgtVelocity = (setSpeed / 100.0) * (185.0 * 30.0 / 60.0);	// speed as % times max ticks per sec speed
	NewMove(tgtVelocity, setAccel, setLimit);
}

void PilotMotor::Stop(bool immediate)
{
	NewMove(0, immediate ? 0 : 1000, NOLIMIT);
}

void PilotMotor::NewMove(float moveSpeed, float moveAccel, long moveLimit)
{
	pending = false;
	// +++ stalled = false
	if (moveSpeed == 0)
		SubMove(0, moveAccel, NOLIMIT);
	else if (!moving)
		SubMove(moveSpeed, moveAccel, moveLimit);
	else
	{
		// already moving, modify current move if possible
		float moveLen = moveLimit - GetTacho();
		float accel = (velocity * velocity) / (2 * moveLen);
		if (moveLen * velocity >= 0 && abs(accel) <= moveAccel)
			SubMove(moveSpeed, moveAccel, moveLimit);
		else
		{
			newSpeed = moveSpeed;
			newAccel = moveAccel;
			newLimit = moveLimit;
			pending = true;
			SubMove(0, moveAccel, NOLIMIT);
		}
	}
}

void PilotMotor::SubMove(float moveSpeed, float moveAccel, long moveLimit)
{
	float absAcc = abs(moveAccel);
	checkLimit = abs(moveLimit) != NOLIMIT;
	baseTime = millis();
	long currentTacho = GetTacho();

	if (!moving && abs(moveLimit - currentTacho) < 1)
		tgtVelocity = 0;
	else
		tgtVelocity = (moveLimit - currentTacho) >= 0 ? moveSpeed : -moveSpeed;

	acceleration = tgtVelocity - velocity >= 0 ? absAcc : -absAcc;
	accelTime = ((tgtVelocity - velocity) / acceleration) * 1000;
	accTacho = (velocity + tgtVelocity) * accelTime / (2 * 1000);
	baseTacho = currentTacho;
	baseVelocity = velocity;
	limit = moveLimit;
	moving = tgtVelocity != 0 || baseVelocity != 0;

	Serial.print("// moveLimit="); Serial.println(moveLimit);
	Serial.print("// currentTacho="); Serial.println(currentTacho);
	Serial.print("// tgtVelocity="); Serial.println(tgtVelocity);
	Serial.print("// velocity="); Serial.println(velocity);
	Serial.print("// baseTacho="); Serial.println(baseTacho);
	Serial.print("// acceleration="); Serial.println(acceleration);
	Serial.print("// accelTime="); Serial.println(accelTime);
	Serial.print("// accTacho="); Serial.println(accTacho);
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
	// +++ publish event?
}

void PilotMotor::Tick()
{
	unsigned long now = millis();
	unsigned long moveElapsedTime = now - baseTime;
	unsigned long tickElapsedTime = now - lastTickTime;
	float error;
	long expectedTacho;

	if (moving)
	{
		if (moveElapsedTime < accelTime)
		{
			Serial.println("//accelerating");
			velocity = baseVelocity + accTacho * moveElapsedTime / 1000;
			expectedTacho = (baseVelocity + velocity) * moveElapsedTime / (2 * 1000);
			error = (float)(expectedTacho - GetTacho());
		}
		else
		{
			Serial.println("//moving");
			velocity = tgtVelocity;
			expectedTacho = baseTacho + accTacho + velocity * (moveElapsedTime - accelTime) / 1000;
			error = (float)(expectedTacho - GetTacho());
			// is move complete?
			if (tgtVelocity == 0 &&
					(pending ||
					(abs(error) < 2 && moveElapsedTime > accelTime + 100) ||
					moveElapsedTime > accelTime + 500) )
				EndMove(false);
		}

		// +++ stalled?

		// PID
		CalcPower(error, Kp1, Ki1, Kd1, (float) tickElapsedTime / 1000);
	}

	lastTickTime = now;
}

void PilotMotor::CalcPower(float error, float Kp, float Ki, float Kd, float time)
{
	Serial.print("//CalcPower error="); Serial.println(error);

	err1 = 0.375f * err1 + 0.625f * error;	// fast smoothing
	err2 = 0.75f * err2 + 0.25f * error;	// slow smoothing
	float newPower = basePower + Kp * err1 + Kd * (err1 - err2) / time;
	basePower = basePower + Ki * (newPower - basePower) * time;
	basePower = constrain(basePower, -100, 100);
	power = constrain(newPower, -100, 100);

	//Serial.print("// err1="); Serial.println(err1);
	//Serial.print("// err2="); Serial.println(err2);
	Serial.print("// power="); Serial.println(power);
	//Serial.print("// basePower="); Serial.println(basePower);
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

	M1.Tick();
	//M2.Tick();

	M1.PinPower(M1.power);
	//M2.PinPower(M1.power);
}


