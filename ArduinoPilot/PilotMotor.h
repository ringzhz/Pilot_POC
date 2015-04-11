//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

class PilotMotor
{
public:
	char motorName[3];
	char safetyEOS = '\0';
	uint8_t pwmPin;
	uint8_t dirPin;
	uint8_t feedBackPin;
	uint8_t interruptIndex;
	bool reversed;

	uint32_t lastUpdateTime;
	uint32_t lastTacho;
	float desiredSpeed;
	float actualSpeed;

	float lastPower;
	float previousError;
	float previousIntegral;

	PilotMotor(const char *name, uint8_t pwm, uint8_t dir, uint8_t fb, uint8_t idx, bool revrsd);
	void Reset();
	uint32_t GetTacho();
	void SetSpeed(int spd);

	void Tick();
};

extern uint8_t MotorMax;
extern float Kp1, Ki1, Kd1;
extern float Kp2, Ki2, Kd2;

void MotorInit();
void PilotRegulatorTick();