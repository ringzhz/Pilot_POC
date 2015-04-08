//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <digitalWriteFast.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"

#include "PilotMotor.h"
#include "Commands.h"
#include "broadcasts.h"
#include "ArduinoPilot.h"
#include "pose.h"

double X = 0.0;		// internally mm, broadcast in meters
double Y = 0.0;
double H = 0.0;		// internally using radians, broadcasts in deggrees

unsigned long LastPoseTime = 0L;
double previousHeading = 0.0;

bool CalcPose()
{
	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	if (abs(delta1) + abs(delta2) < 1)
		return false;	// no significant movement

	double delta = (delta2 + delta1) * Geom.EncoderScaler / 2.0;	
	
	// +++ use DMP for heading
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	Serial.print("ypr\t");
	Serial.print(ypr[0] * 180 / M_PI);
	Serial.print("\t");
	Serial.print(ypr[1] * 180 / M_PI);
	Serial.print("\t");
	Serial.println(ypr[2] * 180 / M_PI);

	// +++ handle wrapping!!
	double headingDelta = (ypr[0] * 180 / M_PI + previousHeading) * 2.0;

	X += delta * sin(H + headingDelta / 2.0);
	Y += delta * cos(H + headingDelta / 2.0);

	H += headingDelta;
	H = fmod(H, TWO_PI);

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;
	previousHeading = ypr[0] * 180 / M_PI;

	return true;
}