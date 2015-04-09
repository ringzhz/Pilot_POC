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


double X = 0.0;		// internally mm, broadcast in meters
double Y = 0.0;
double H = 0.0;		// internally using radians, broadcasts in degrees

unsigned long LastPoseTime = 0L;
double previousHeading = 0.0;

bool CalcPose()
{
	bool poseChanged = false;

	long tachoNow1 = M1.GetTacho(),
		tachoNow2 = M2.GetTacho();

	long delta1 = tachoNow1 - M1.lastTacho,
		delta2 = tachoNow2 - M2.lastTacho;

	// use DMP for heading
	double headingDelta = (ypr[0] - previousHeading);
	if (abs(headingDelta) > .001)
		poseChanged = true;
	
	H += headingDelta;
	if (H < 0)
		H += TWO_PI;
	if (H > TWO_PI)
		H -= TWO_PI;

	previousHeading = ypr[0];

	if (abs(delta1) + abs(delta2) < 1)
		return poseChanged;	// no significant linear movement

	double delta = (delta2 + delta1) * Geom.EncoderScaler / 2.0;		

	X += delta * sin(H + headingDelta / 2.0);
	Y += delta * cos(H + headingDelta / 2.0);

	M1.lastTacho = tachoNow1;
	M2.lastTacho = tachoNow2;

	return true;
}