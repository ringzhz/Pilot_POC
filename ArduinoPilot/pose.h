//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

extern float X;		// internally mm, broadcast in meters
extern float Y;
extern float H;		// internally using radians, broadcasts in deggrees
extern float previousHeading;

extern Geometry Geom;
bool CalcPose();
