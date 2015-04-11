//* S3 Pilot Proof of Concept, Arduino UNO shield
//* Copyright © 2015 Mike Partain, MWPRobotics dba Spiked3.com, all rights reserved

extern double X;		// internally mm, broadcast in meters
extern double Y;
extern double H;		// internally using radians, broadcasts in deggrees
extern double previousHeading;

extern Geometry Geom;
bool CalcPose();
