//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#ifndef __Pose__H
#define __Pose__H

#include <Arduino.h>

#include "ArduinoPilot.h"

extern double X;		// internally mm, broadcast in meters
extern double Y;
extern double H;		// internally using radians, broadcasts in deggrees

extern Geometry Geom;

extern float GyroOffset, varianceTacho, varianceGyro,
		varianceFilterPredicted, varianceFilterUpdated,
		kalmanGain,
		tachoMeasured, tachoPredicted, tachoUpdated,
		gyroMeasured;

bool CalcPoseWithGyro();
bool CalcPose();

#endif
