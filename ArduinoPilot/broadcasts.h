//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

#pragma once

extern PilotMotor M1, M2;
extern double X, Y, H;

void PublishGps();
void PublishPose();
void PublishHeartbeat();
