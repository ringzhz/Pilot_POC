//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright (c) 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

uint8_t MotorMax = 100;
float Kp1, Ki1, Kd1;		// per motor regulator
float Kp2, Ki2, Kd2;		// synchronizing regulator

volatile uint32_t tacho[2];		// interrupt 0 & 1 tachometers

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

	uint32_t lastUpdateTime;
	uint32_t lastTacho;
	
	// speed is in ticks per second
	// +++ we will need to know max, to use mmax to determine max speed
	// +++ function calls are expressed as pct of max speed
	float targetSpeed;	

	float spSpeed;		// calculated with ramping
	float pvSpeed;		// aka current velocity

	float lastPower;
	float previousError;
	float previousIntegral;

	uint32_t timeoutAt;

	PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd);
	void Reset();
	uint32_t GetTacho();
	void SetSpeed(int spd);
	void SetPower(int power);
	void Move(float speed, uint32_t timeout);
	void Stop(bool sudden);
	void Tick();
};

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
	tacho[interruptIndex] = lastTacho = 0L;
	lastPower = targetSpeed = pvSpeed = 0.0;
	previousError = previousIntegral = 0.0;
	//lastUpdateTime = ? ? ? ;
}

uint32_t PilotMotor::GetTacho()
{
	return reversed ? -tacho[interruptIndex] : tacho[interruptIndex];
}

void PilotMotor::SetPower(int power)
{
	// +++ just for testing
	char t[64];
	// speed is a +/- percent of max	
	uint8_t newDir = (power >= 0) ? (reversed ? 1 : 0) : (reversed ? 0 : 1);
	int16_t newSpeed = map(abs(power), 0, 100, 0, 255 * MotorMax / 100);
	digitalWrite(dirPin, newDir);
	analogWrite(pwmPin, newSpeed);
	targetSpeed = newSpeed;
	sprintf(t, "// %d,%d >> %s\r\n", newDir, newSpeed, motorName); Serial.print(t);
}

void PilotMotor::SetSpeed(int power)
{
	SetPower(power);	// +++ for now
	//char t[64];
	//// speed is a +/- percent of max	
	//uint8_t newDir = (power >= 0) ? (reversed ? 1 : 0) : (reversed ? 0 : 1);
	//int16_t newSpeed = map(abs(power), 0, 100, 0, 255 * MotorMax / 100);
	//digitalWrite(dirPin, newDir);
	//analogWrite(pwmPin, newSpeed);
	//targetSpeed = newSpeed;
	//sprintf(t, "// %d,%d >> %s\r\n", newDir, newSpeed, motorName); Serial.print(t);
}

// as a percent of max speed
void PilotMotor::Move(float speed, uint32_t timeout)
{

}

void PilotMotor::Stop(bool sudden)
{

}

void PilotMotor::Tick()
{
	// if (time > timeoutAt)
	//	setPower(0);
}

extern PilotMotor M1, M2;

void MotorInit()
{
	Serial.print("// MotorInit ...\r\n");

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
}

void PilotRegulatorTick()
{
	// +++ regulation needed now! but waiting on hardware :|
	/*
	Ziegler�Nichols method
		Control Type	K_p		K_i				K_d
		P			0.50{K_u}	--
		PI			0.45{K_u}	1.2{K_p} / P_u	-
		PID			0.60{K_u}	2{K_p} / P_u	{ K_p }{P_u} / 8
	*/
}
