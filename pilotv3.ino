//* S3 Pilot, V3 Arduino UNO shield prototype
//* Copyright (c) 2015 Mike Partain, Spiked3.com, all rights reserved

/*

  so after many tries to split this into logical files, I gave up trying to make arduino code
  act like code. It is now one big plate of spahgetti, as intended by Arduino designers. 
  Often I have to compile twice, as the first time causes a Java error. But it has always done 
  fine the second time (such a reliable consistent code generator that it is).

  todo:
  there is some inconsistencies depending on who wrote a piece of code (David v Mike). Nothing that 
  would break anything, just loop counters/debounce is done one way in some places and a different
  way in others.

  accelration/deceleration.

  rotateTo absolute encoder count
  use velocity * ET  +/- Accel = absoluteEncoderPosition as PID goals v velocity
  (this makes 'go straight', actually do that)

  implement turnTo heading.

*/  

#include <ArduinoJson.h>

#include "digitalWriteFast.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define CRLINE "S3 Pilot V3.7.6 (c) 2015 mike partain/spiked3.com"

// pins are defines to allow feastRead/Writes
// interrupt/phase pins are hardcoded in motor.cpp
#define BUMPER  4
#define LED   13
#define ESC_ENA 12
#define MPU_INT 7

// motor pins
#define M1_PWM  5
#define M2_PWM  6
#define M1_DIR  11
#define M2_DIR  17  // A3
#define M1_FB 16  // A2
#define M2_FB 15  // A1

#define NOLIMIT 0x7fffffff

#define KICK_POWER 60
#define VELOCITY_SAMPLE_RATE 10
#define AHRS_SETTLE_TIME 30 // seconds

typedef struct {
  int ticksPerMeter;
  int mMax;
} Geometry;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
} pidData;

typedef struct 
{
  const char *cmd;
  void (*f)(JsonObject&  j);
} CmdFunction;

volatile long rawTacho[2];    // interrupt 0 & 1 tachometers

class PilotMotor
{
public:
  friend void PilotRegulatorTick();
  friend bool CalcPose();
  friend void PublishHeartbeat();
  
  // pin level
  byte pwmPin;
  byte dirPin;
  byte feedBackPin;
  byte interruptIndex;
  bool reversed;
  float lastPinPower;

  // used by pose
  long lastPoseTacho;

  // used by motor regulator (tick)
  long tickTacho, lastTickTacho;

  // regulator variables
  // speed/velocity are ticks per second
  float power;
  float previousError, integral, derivative;
  float timeAccumulator; // used to calculate velocity at @ VELOCITY_SAMPLE_RATE
  int velocitySampleCtr;
  float tgtVelocity;    // is what we asked for
  float velocity;     // is what we have 

public:
  PilotMotor(unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd);
  void Reset();
  void SetSpeed(float speed, int acceleration);
  void Tick(unsigned int elapsed);

protected:
  long GetRawTacho() { return reversed ? -rawTacho[interruptIndex] : rawTacho[interruptIndex]; }
  void PinPower(int power);
};

// A0 was reserved for the buttons on prototype board, but is no longer used - so it is available

char *value = "Value";
char *intvl = "Int";

float X = 0.0;		// internally mm, published in meters
float Y = 0.0;
float H = 0.0;		// internally using radians, published in degrees

float previousYaw = 0.0;

bool AhrsEnabled = true;
// escEnabled serves 2 purposes. if it is false at startup, the pins are not initialized
// after startup it is used to actually enable/disable the speed controlers
bool escEnabled = true;
bool heartbeatEventEnabled = false;
bool BumperEventEnabled = true;
bool DestinationEventEnabled = true;
// PoseEventEnabled serves 2 purposes. it is false at startup until AHRS warmup elapses, then
// the pose is reset, an move completeevent is published and PoseEventEnabled is set to true
bool PoseEventEnabled = false;

// counter based (ie every X loops)
// +++ at the current frequency we overrun the MPU too often
// +++ that in itself is not a problem if we handled it instead of locking up
unsigned int CalcPoseFrequency = 50;
unsigned int pilotRegulatorFrequency = 50;
unsigned int heartbeatEventFrequency = 2000;
unsigned int checkBumperFrequency = 300;
unsigned int mpuSettledCheckFrequency = 10000;
unsigned long cntr = 0L;		// warning, will wrap after 70 years or so, do not use for extraplanetary exploration!

// MPU control/status vars
byte mpuIntStatus;		// holds actual interrupt status byte from MPU
byte packetSize;		// expected DMP packet size (default is 42 bytes)
byte devStatus;			// return status after each device operation (0 = success, !0 = error)
unsigned int fifoCount;	// count of all bytes currently in FIFO
byte fifoBuffer[64];	// FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

MPU6050 mpu;

Geometry Geom;

PilotMotor	M1(M1_PWM, M1_DIR, M1_FB, 0, true), M2(M2_PWM, M2_DIR, M2_FB, 1, false);

int  mqIdx = 0;
char mqRecvBuf[128];

#define bumperDebounce 3
byte bumperDebounceCntr = 0;
unsigned int lastBumperRead = 0xffff;

long ahrsSettledTime;
bool ahrsSettled = false;

unsigned long lastTickTime;

pidData MotorPID {
  .3, .9, 0.0025  // seems pretty good on 06/19/2015
};

ISR(MotorISR1)
{
  int b = digitalReadFast(8);
  if (digitalReadFast(2))
    b ? rawTacho[0]-- : rawTacho[0]++;
  else
    b ? rawTacho[0]++ : rawTacho[0]--;
}

ISR(MotorISR2)
{
  int b = digitalReadFast(9);
  if (digitalReadFast(3))
    b ? rawTacho[1]-- : rawTacho[1]++;
  else
    b ? rawTacho[1]++ : rawTacho[1]--;
}

PilotMotor::PilotMotor(unsigned short pwm, unsigned short dir, unsigned short fb, unsigned short idx, bool revrsd)
{
  pwmPin = pwm;
  dirPin = dir;
  feedBackPin = fb;
  interruptIndex = idx;
  reversed = revrsd;

  if (escEnabled)
  {
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    Reset();
  }
}

void PilotMotor::Reset()
{
  SetSpeed(0, 0);
  rawTacho[interruptIndex] = lastTickTacho = lastPoseTacho = 0L;
}

void PilotMotor::PinPower(int p)
{
  unsigned short newDir = LOW;
  int realPower = 0;

  if (p != lastPinPower)
  {   
    newDir = (p >= 0) ? (reversed ? HIGH : LOW) : (reversed ? LOW : HIGH);
    realPower = map(abs(p), 0, 100, 0, 255);
    digitalWrite(dirPin, newDir);
    analogWrite(pwmPin, realPower);
    lastPinPower = p;
  }
}

void PilotMotor::SetSpeed(float setSpeed, int setAccel)
{
  tgtVelocity = setSpeed * Geom.mMax / 100; // speed as % times max ticks speed
  
  if (setSpeed == 0)
    previousError = derivative = integral = 0;
}

void PilotMotor::Tick(unsigned int eleapsedMs)
{
  timeAccumulator += eleapsedMs;
  if (velocitySampleCtr++ == VELOCITY_SAMPLE_RATE)  // provides an average over X samples to smooth digital aliasing
  {
    float newVelocity = (tickTacho - lastTickTacho) * 1000 / timeAccumulator; // TPS
    velocity = (.90 * newVelocity) + (.10 * velocity);
    timeAccumulator = 0;
    velocitySampleCtr = 0;
    lastTickTacho = tickTacho;
  }

  // a breaking PID would be better
  // as is, regular drive pid is too violent for breaking
  // (see LeJOS)
  if (tgtVelocity == 0)
    power = previousError = integral = derivative = 0;

  // !!! fix suggested by jesseg - use pid output as power, not an adjustment
  // it finally acts like expected, thanks jesseg !!!
  float pid = Pid(tgtVelocity, velocity, MotorPID.Kp, MotorPID.Ki, MotorPID.Kd,
    previousError, integral, derivative, (float) eleapsedMs / 1000);

  power = constrain(pid, -100, 100);
}

//----------------------------------------------------------------------------

// this PID is heavily smoothed at the moment. This probably should be parameters as it will be 
//  different depending on how the PID is being used (steering v motor)
float Pid(float setPoint, float presentValue, float Kp, float Ki, float Kd, float& previousError, float& integral, float& derivative, float dt)
{
  float error = setPoint - presentValue;
  error = .4 * error + .6 * previousError / 2;  // smoothing
  integral = integral + error * dt;
  float newDerivative = (error - previousError) / dt; 
  derivative = (.25 * newDerivative) + (.75 * derivative);    // more smoothing
  float output = Kp * error + Ki * integral + Kd * derivative;  
  previousError = error;
  return output;
}

void MotorInit()
{
  rawTacho[0] = rawTacho[1] = 0L;

  // !! hardcoded interrupt handlers !!
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  attachInterrupt(PCINT0, MotorISR1, CHANGE); // pin 2
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

  escEnabled = false;
}

// +++ will need these evantually
//float Distance(float x1, float y1, float x2, float y2)
//{
//  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
//}
//
//void MoveCompleteEvent(bool success);
//bool headingStop = false;
//bool moveStop = false;
//float headingGoal = 0;
//
//float AngleBetween(float angle1, float angle2)
//{
//  while (angle1 < 0) angle1 += TWO_PI;
//  while (angle2 < 0) angle2 += TWO_PI;
//  while (angle1 > TWO_PI) angle1 -= TWO_PI;
//  while (angle2 > TWO_PI) angle2 -= TWO_PI;
//
//  float diff = angle1 - angle2;
//  float absDiff = abs(diff);
//
//  if (absDiff <= PI)
//    return absDiff == PI ? PI : diff; // so -180 is 180 (in radians)
//
//  else if (angle1 > angle2)
//    return absDiff - TWO_PI;
//
//  return TWO_PI - absDiff;
//}

void PilotRegulatorTick()
{
  // +++ if we just accessed the MPU yaw instead of depending on CalcPose
  //  we might not have to run CalcPose at the same speed
  // +++ this applies to when we try the turn to stuff

  unsigned long now = millis();
  unsigned int tickElapsedTime = now - lastTickTime;

  M1.tickTacho = M1.GetRawTacho();
  M2.tickTacho = M2.GetRawTacho();

  M1.Tick(tickElapsedTime);
  M2.Tick(tickElapsedTime);

  M1.PinPower(M1.power);
  M2.PinPower(M2.power);

  lastTickTime = now;
}

// ---------------------- commands

void cmdBump(JsonObject&  j)
{
  BumperEventEnabled = j[value] == 1;
}

void cmdDest(JsonObject&  j)
{
  DestinationEventEnabled = j[value] == 1;
}

void cmdHeartbeat(JsonObject&  j)
{
  heartbeatEventEnabled = j[value] == 1;
  if (j.containsKey(intvl))
    heartbeatEventFrequency = j[intvl];
}

void cmdReset(JsonObject&  j)
{
  // by including specific variables, you can set pose to a particular value
  char * xKey = "X";
  char * yKey = "Y";
  char * hKey = "H";
  
  M1.Reset();
  M2.Reset();
  
  X = Y = H = 0.0;
  if (j.containsKey(xKey))
    X = j[xKey];
  if (j.containsKey(yKey))
    Y = j[yKey];
  if (j.containsKey(hKey))
    H = DEG_TO_RAD * (float)j[hKey];
  
  NormalizeHeading(H);
  previousYaw = H + ypr[0]; // base value
  //Traveling = Rotating = false;
  PublishPose();
}

void cmdEsc(JsonObject&  j)
{
  escEnabled = j[value] == 1;
  digitalWriteFast(ESC_ENA, escEnabled);
}

void cmdPose(JsonObject&  j)
{
  PoseEventEnabled = j[value] == 1;
}

void cmdConfig(JsonObject&  j)
{
  // combines old PID, geom, and calibrate, adds some new stuff
  char *geomKey = "Geom";
  char *pidKey = "PID";
  char *mpuKey = "MPU";

  if (j.containsKey(geomKey))
  {
    Geom.ticksPerMeter = j[geomKey][0].as<float>();
    Geom.mMax = j[geomKey][1].as<float>();
  }

  if (j.containsKey(pidKey))
  {
    MotorPID.Kp = j[pidKey][0].as<float>();
    MotorPID.Ki = j[pidKey][1].as<float>();
    MotorPID.Kd = j[pidKey][2].as<float>();
    M1.previousError = M2.previousError = M1.integral = M2.integral = M1.derivative = M2.derivative = 0;
  }

  // MPU/DMP calibration values
  if (j.containsKey(mpuKey))
  {
    mpu.setXAccelOffset(j[mpuKey][0]);
    mpu.setYAccelOffset(j[mpuKey][1]);
    mpu.setZAccelOffset(j[mpuKey][2]);
    mpu.setXGyroOffset(j[mpuKey][3]);
    mpu.setYGyroOffset(j[mpuKey][4]);
    mpu.setZGyroOffset(j[mpuKey][5]);
  }
}

void cmdPower(JsonObject&  j)
{
  // +++acceleration not implemented - but planned
  char *m1Key = "M1";
  char *m2Key = "M2";

  if (j.containsKey(m1Key))
  {
    float s = j[m1Key];
    M1.SetSpeed(s, 0);
  }
  if (j.containsKey(m2Key))
  {
    float s = j[m2Key];
    M2.SetSpeed(s, 0);
  } 
}

////////////////////////////////////////////////////

CmdFunction cmdTable[] {
  { "Esc", cmdEsc },
  { "Pwr", cmdPower, },
  { "Reset", cmdReset },
  { "Bumper", cmdBump, },
  { "Heartbeat", cmdHeartbeat, },
  { "Pose", cmdPose, },
  { "Config", cmdConfig, },
};

void ProcessCommand(JsonObject& j)
{
  for (int i = 0; i < sizeof(cmdTable) / sizeof(CmdFunction); i++)
    if (strcmp(cmdTable[i].cmd, j["Cmd"]) == 0)
    {
      (*cmdTable[i].f)(j);
      break;
    }
}

////////////////////////////////////////////////////
// ---------------------- publish

void PublishPose()
{
  StaticJsonBuffer<64> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["T"] = "Pose";
  root["X"].set(X / 1000, 4);   // mm to meter
  root["Y"].set(Y / 1000, 4);
  root["H"].set(RAD_TO_DEG * H, 1); // in degrees
  root.printTo(Serial); Serial.println();
}

void PublishHeartbeat()
{
  // !!! seems like about 3-4 floats is all the arduino (serial) can handle
  // often carries some debugging payload with it
  StaticJsonBuffer<128> publishBuffer;
  JsonObject& root = publishBuffer.createObject();
  root["T"] = "Heartbeat"; 
  root["T1"].set(M1.tgtVelocity, 2);  // number of decimals to print
  root["V1"].set(M1.velocity, 2);
  root["I1"].set(M1.integral, 2);
  root["D1"].set(M1.derivative, 2);
  root["PW1"].set(M1.lastPinPower, 2);

  root["T2"].set(M2.tgtVelocity, 2);  // number of decimals to print
  root["V2"].set(M2.velocity, 2);
  root["I2"].set(M2.integral, 2);
  root["D2"].set(M2.derivative, 2);
  root["PW2"].set(M2.lastPinPower, 2);

  // used for ticks per meter calibration
  //  root["TA1"] = M1.GetRawTacho();

  root.printTo(Serial); Serial.println();
}

void BumperEvent(bool bumperPressed)
{
  StaticJsonBuffer<64> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["T"] = "Bumper";
  root["Value"] = bumperPressed ? 1 : 0;
  root.printTo(Serial); Serial.println();
}

// +++ log really needs to help with things like this;
// //DBGP("PinPower");  DBGV("Pin", pwmPin);  DBGV("", p); DBGE();
// once MPU code is separated out, maybe we will have enough room to include sprintf

void Log(char const *t)
{
  StaticJsonBuffer<128> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["T"] = "Log";
  root["Msg"] = t;
  root.printTo(Serial); Serial.println();
}

void MoveCompleteEvent(bool success)
{
  extern bool headingStop;
  extern bool moveStop;

  StaticJsonBuffer<64> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["T"] = "Moved";
  root["Value"] = success ? 1 : 0;
  root.printTo(Serial); Serial.println();
}


////////////////////////////////////////////////////////////

void BlinkOfDeath(int code)
{
  while (1)
  {
    for (int i = 0; i < code; i++)
    {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
    delay(600);
  }
}

////////////////////////////////////////////////////

void CheckMq()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')		// end of line, process
    {
      mqRecvBuf[mqIdx++] = '\0';
      StaticJsonBuffer<128> jsonBuffer;
      JsonObject& j = jsonBuffer.parseObject(mqRecvBuf);
      if (j.containsKey("Cmd"))
        ProcessCommand(j);
      else
        Log("ENoCmd");
      mqIdx = 0;
      return;
    }
    else
      mqRecvBuf[mqIdx++] = c;

    if (mqIdx >= sizeof(mqRecvBuf))
      BlinkOfDeath(4);
  }
}

void NormalizeHeading(float& h)
{
  while (h > PI || h < -PI)
    h += (h > PI) ? -TWO_PI : (h < -TWO_PI) ? TWO_PI : 0;
}

bool CalcPose()
{
  bool poseChanged = false;

  long tachoNow1 = M1.GetRawTacho(),
       tachoNow2 = M2.GetRawTacho();

  float delta1 = (tachoNow1 - M1.lastPoseTacho) * 1000 / Geom.ticksPerMeter,
        delta2 = (tachoNow2 - M2.lastPoseTacho) * 1000 / Geom.ticksPerMeter;

  // uses DMP for heading
  float headingDelta = (ypr[0] - previousYaw);

  if (abs(RAD_TO_DEG * headingDelta) > .1F)
    poseChanged = true;

  float delta = (delta1 + delta2) / 2;

  if (abs(delta) > 1)
    poseChanged = true;

  X += delta * sin(H + headingDelta / 2);
  Y += delta * cos(H + headingDelta / 2);

  H += headingDelta;
  NormalizeHeading(H);

  previousYaw = ypr[0];

  M1.lastPoseTacho = tachoNow1;
  M2.lastPoseTacho = tachoNow2;

  return poseChanged;
}

////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  pinMode(ESC_ENA, OUTPUT);
  pinMode(BUMPER, INPUT_PULLUP);

  digitalWrite(LED, false);
  digitalWrite(ESC_ENA, false);

  MotorInit();  // interrupt handler(s), pinmode(s)

  if (AhrsEnabled)
  {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    pinMode(MPU_INT, INPUT_PULLUP);

    mpu.initialize();
    delay(10);

    mpu.testConnection();
    devStatus = mpu.dmpInitialize();
    delay(10);

    if (devStatus == 0)
    {
      mpu.setDMPEnabled(true);
      delay(10);
      mpuIntStatus = mpu.getIntStatus();
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      // ERROR! 1 = initial memory load failed 2 = DMP configuration updates failed
      Log("E6050 failed");
      BlinkOfDeath(1 + devStatus);
    }

    // defaults from example is good default starting point
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    ahrsSettledTime = millis() + (AHRS_SETTLE_TIME * 1000);
  }

  Log(CRLINE);
}

////////////////////////////////////////////////////

void loop()
{
  CheckMq();

  // +++ check status flag / amp draw from mc33926??
  // FYI analogRead(pin) * .9 is ma (I read that somewhere)

  if (!ahrsSettled && (cntr % mpuSettledCheckFrequency == 0))
    if (millis() > ahrsSettledTime)
    {
      ahrsSettled = true;
      //M1.Reset();
      //M2.Reset();
      //X = Y = H = 0.0;
      //previousYaw = ypr[0];	// base value for heading 0
      PoseEventEnabled = true;
      MoveCompleteEvent(true);
    }

  // +++ note - v2r1 schematic is wrong - jumper should be tied to ground not vcc
  //  so for now use outside bumper pin and grnd (available on outside reset jumper) for bumper @v2r1
  // +++ im not convinced it is wrong, the intent was to use normally closed and trigger on open, and it should be ok as is (need to flip sign is all)?
  if (BumperEventEnabled && (cntr % checkBumperFrequency == 0))
  {
    if (bumperDebounceCntr == 0)
    {
      int thisBumper = digitalReadFast(BUMPER);
      if (thisBumper != lastBumperRead)
      {
        // new (debounced) event
        lastBumperRead = thisBumper;
        bumperDebounceCntr = bumperDebounce;
        BumperEvent(thisBumper != 0);
      }
    }
    else if (--bumperDebounceCntr < 0)
      bumperDebounceCntr = 0;
  }

  if (escEnabled && (cntr % pilotRegulatorFrequency == 0))
    PilotRegulatorTick();

  if (cntr % CalcPoseFrequency == 0)
    if (CalcPose())
      if (PoseEventEnabled)
        PublishPose();

  if (cntr % heartbeatEventFrequency == 0)  // heart beat blinky
  {
    digitalWriteFast(LED, !digitalRead(LED));
    if (heartbeatEventEnabled && digitalReadFast(LED))
      PublishHeartbeat();
  }

  if (AhrsEnabled && digitalReadFast(MPU_INT))
  {
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for overflow, this happens on occasion
    if (mpuIntStatus & 0x10 || fifoCount == 1024)
      Log("EmpuOvf");

    else if (mpuIntStatus & 0x02)
    {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    mpu.resetFIFO();
  }
  cntr++;
}

