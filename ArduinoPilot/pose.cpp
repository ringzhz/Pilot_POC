//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#include "pose.h"
#include "PilotMotor.h"

double X = 0.0;		// internally mm, broadcast in meters
double Y = 0.0;
double H = 0.0;		// internally using radians, broadcasts in deggrees

Geometry Geom;

float GyroOffset = -79, varianceTacho = 0.9, varianceGyro = 0.119,
	varianceFilterPredicted, varianceFilterUpdated,
	kalmanGain,
	tachoMeasured, tachoPredicted, tachoUpdated,
	gyroMeasured;

extern PilotMotor M1, M2;

unsigned long LastPoseTime = 0L;


bool CalcPoseWithGyro()
{
	char t[64];

	// milli-degree/second is what the gyro uses
	// time1 uses milliseconds
	unsigned long nowTime = millis();

	// milli-degree/second is what the gyro reports
	float dt = (nowTime - LastPoseTime) / 1000.0;

	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScaler / 2.0;

	double tachoMeasured = (delta2 - delta1) * 2.0 / Geom.wheelBase;

	float m = MeanGyroValue();

	sprintf(t, "// CalcPoseWithGyro MeanGyroValue(%d) GyroOffset(%d)\n", m * 100, GyroOffset * 100); Serial.write(t);
	gyroMeasured = m + GyroOffset;

	// predict
	tachoPredicted = tachoUpdated + dt * gyroMeasured;
	varianceFilterPredicted = varianceFilterUpdated + varianceGyro;

	// heading must be between 0 and 2*PI
	tachoPredicted = fmod(tachoPredicted, TWO_PI);

	// Kalman gain
	kalmanGain = varianceFilterPredicted / (varianceFilterPredicted + varianceTacho);

	// update
	tachoUpdated = tachoPredicted + kalmanGain * (tachoMeasured - tachoPredicted);
	varianceFilterUpdated = varianceFilterPredicted +
		kalmanGain * (varianceTacho - varianceFilterPredicted);

	H = fmod(tachoUpdated, TWO_PI);

	X += delta * sin(H);
	Y += delta * -cos(H);

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;
	LastPoseTime = nowTime;

	return true;
}

bool CalcPose()
{
	// +++ add gyro integration/kalman

	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScaler / 2.0;
	
	// +++ * 2.0 added but unexplained

	double headingDelta = (delta2 - delta1) * 2.0 / Geom.wheelBase;

	X += delta * sin(H + headingDelta / 2.0);
	Y += delta * cos(H + headingDelta / 2.0);
	H += headingDelta;
	H = fmod(H, TWO_PI);

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return true;
}