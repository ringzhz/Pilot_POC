//* S3 Pilot, Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

#define STARTUP_POWER 40	// what it takes to move a motor
#define ACCELERATION 6000
#define NOLIMIT 0x7fffffff

uint8_t MotorMax = 100;
float Kp1, Ki1, Kd1;		// per motor regulator
float Kp2, Ki2, Kd2;		// synchronizing regulator

volatile uint32_t tacho[2];		// interrupt 0 & 1 tachometers

extern Geometry Geom;

// a motor is generally not accessed directly, but by the regulator who also controls direction

class PilotMotor
{
public:
	char motorName[3];
	char reserved1 = '\0';
	uint8_t pwmPin;
	uint8_t dirPin;
	uint8_t feedBackPin;
	uint8_t interruptIndex;
	bool	reversed;

	uint32_t baseTime;
	uint32_t lastTickTime;

	uint32_t lastTacho;
	bool moving;
	uint32_t accelTime;		// time at which an accel or de-accel will be completed
	
	// regulator variables
	float spTPM, newSpeed;			// target ticksPerMinute
	float pvTPM;			// AKA current rotation in ticks per minute
	float newAccel;
	long newLimit;
	long baseTacho;
	float baseTPM;
	float spAccel;
	float currPower;
	long accTacho;

	float power;			// result of last PID
	float previousError;
	float previousIntegral;
	float previousDerivative;
	bool pending;

	uint32_t timeoutAt;

	PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd);
	void Reset();
	uint32_t GetTacho() { return reversed ? -tacho[interruptIndex] : tacho[interruptIndex]; }
	void SetSpeed(int speed, int accelration, long limit);
	void Stop(bool sudden);
	float Pid1(float setPoint, float presentValue, float elasped);

	void Tick();
	void PinPower(int power);
	void NewMove();
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

PilotMotor::PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd)
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
	// +++ wait for stop before resetting tachos
	SetSpeed(0, 1000, NOLIMIT);
	previousError = previousIntegral = 0.0;
	tacho[interruptIndex] = lastTacho = 0L;
}

void PilotMotor::PinPower(int power)
{
	uint8_t newDir = LOW;
	int16_t realPower = 0;
	
	Serial.print("//SetPwr p("); Serial.print(power); 
	Serial.print(") currentP("); Serial.print(currPower);
	Serial.print(")\n");

	if (currPower != power)
	{
		newDir = (power >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
		realPower = map(abs(power), 0, 100, 0, 255 * MotorMax / 100);
		digitalWrite(dirPin, newDir);
		analogWrite(pwmPin, realPower);
#if 1
		Serial.print("// setPwr");
		Serial.print(" pwi("); Serial.print(power); Serial.print(")");
		Serial.print(" sp("); Serial.print(spTPM); Serial.print(")");
		Serial.print(" pv("); Serial.print(spTPM); Serial.print(")");
		Serial.print(" pe("); Serial.print(previousError); Serial.print("),");
		Serial.print(" dir("); Serial.print(newDir); Serial.print(")");
		Serial.print(" pwo("); Serial.print(realPower); Serial.print(")\n");
#endif
		//currPower = power;
	}
}

// +++as a percent of max speed
void PilotMotor::SetSpeed(int speed, int accelration, long limit)
{
	// +++spSpeed = calculate
	spTPM = 5000;	
	NewMove();
}

void PilotMotor::Stop(bool sudden)
{
	// +++ use ramping?
}

#if 0
float Pid1(float setPoint, float presentValue, float dt)
{
	float p = 0;
	if (dt > 0)
	{
		float error = setPoint - presentValue;
		previousIntegral = previousIntegral + error * dt;
		previousDerivative = (error - previousDerivative) / dt;
		p = Kp1 * error + Ki1 * previousIntegral + Kd1 * previousDerivative;
		p = constrain(p, -20, 20);	// +++???
		previousError = error;
	}
	return p;
}
#endif

float CalcPower(float error, float Kp, float Ki, float Kd)
{
	return 0;
}

void PilotMotor::NewMove()
{
}

void PilotMotor::SubMove(float speed, float accel, int limit)
{

}

void PilotMotor::EndMove(bool stalled)
{
	moving = pending;
	//++ stalled
	if (pending)
	{
		pending = false;
		SubMove(newSpeed, newAccel, newLimit);
	}
}

void PilotMotor::Tick()
{
	uint32_t now = millis();
	uint32_t moveElapsed = now - baseTime;
	uint32_t tickElapsed = now - lastTickTime;
	float error;
	long pvTacho;

	if (moving)
	{
		if (moveElapsed < accelTime)
		{
			pvTPM = baseTPM + spAccel * moveElapsed / 1000;
			pvTacho = (baseTacho + pvTPM) * moveElapsed / (2 * 1000);
			error = pvTacho - GetTacho();
		}
		else
		{
			pvTPM = spTPM;
			pvTacho = baseTacho + accTacho + pvTPM * (moveElapsed - accelTime) / 1000;
			error = pvTacho - GetTacho();
			// is move complete?
			if (spTPM == 0 && (pending ||
				(abs(error) < 12 && moveElapsed > accelTime + 100) ||
				moveElapsed > accelTime + 500))
				EndMove(false);
		}
		// +++ stalled?
		// PID
		power = CalcPower(error, Kp1, Ki1, Kd1);
		PinPower(power);	// ?? done by overall regulator
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



