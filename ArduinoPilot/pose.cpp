//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

#include <digitalWriteFast.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "helper_3dmath.h"

#include "PilotMotor.h"
#include "Commands.h"
#include "broadcasts.h"
#include "ArduinoPilot.h"
#include "pose.h"

float X = 0.0;		// internally mm, broadcast in meters
float Y = 0.0;
float H = 0.0;		// internally using radians, broadcasts in degrees

uint64_t LastPoseTime = 0L;
float previousHeading = 0.0;

bool CalcPose()
{
	bool poseChanged = false;

	uint32_t tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	int32_t delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	// uses DMP for heading
	float headingDelta = (ypr[0] - previousHeading);

	if (abs(RAD_TO_DEG * headingDelta) > .1F)
		poseChanged = true;
	
	float delta = (delta1 + delta2) * Geom.EncoderScaler / 2.0F;

	if (abs(delta) > .1F)
		poseChanged = true;

	X += delta * sin(H + headingDelta / 2.0F);
	Y += delta * cos(H + headingDelta / 2.0F);

	H += headingDelta;
	if (H < 0)
		H += TWO_PI;
	if (H >= TWO_PI)
		H -= TWO_PI;

	previousHeading = ypr[0];

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return poseChanged;
}