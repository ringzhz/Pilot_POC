// digital pins

#define LED 13

#define PWM1 5
#define PWM2 6

#define M1_A 2
#define M2_B 3
#define M1_B 8
#define M2_A 9
#define M1_DIR 11
#define M2_DIR 10

#define ESC_EN 12

// analog pins
#define M1_FB 0
#define M2_FB 1

//#define SDA 4
//#define SCL 5

char t[64];

// +++ should be time based, eg 1/20 times per second

int debounceFrequency = 2;
int checkMqFrequency = 1000;
int checkButtonFrequency = 100;
int CalcPoseFrequency = 1000;
long cntr = 0L;

bool esc_enabled = false;
bool motorOn = false;
bool motorCW = true;

bool useGyro = true;

int motorPower = 0;

void setup()
{
	Serial.begin(57600);
	pinMode(LED, OUTPUT);
	
	pinMode(ESC_EN, OUTPUT);

	pinMode(PWM1, OUTPUT);
	pinMode(M1_DIR, OUTPUT);

	// +++ calc cycles in a second, set frequencies
}

char read_buttons()
{
	int btn = analogRead(0);      // read the value from the sensor 
	return	btn > 1000 ? ' ' :
		btn < 50 ? 'E' :
		btn < 250 ? 'D' :
		btn < 450 ? 'C' :
		btn < 650 ? 'B' :
		btn < 850 ? 'A' :
		' ';  // when all others fail, return this...
}

int debounceCount = 0;
char lastHandledButton;

void CheckButtons()
{
	char btn;
	bool handled;

	btn = read_buttons();

	if (btn == lastHandledButton)
		return;

	if (--debounceCount > 0)
		return;

	handled = false;
	lastHandledButton = btn;

	switch (read_buttons())
	{
	case 'A':
		esc_enabled = !esc_enabled;
		handled = true;
		break;
	case 'B':
		motorCW = false;
		handled = true;
		break;
	case 'E':
		motorCW = true;
		handled = true;
		break;
	case 'D':
		motorPower += 10;
		handled = true;
		break;
	case 'C':
		motorPower -= 10;
		handled = true;
		break;
	}
	if (handled)
		debounceCount = debounceFrequency;
}

void CheckMq()
{

}

void CalcPose()
{
	if (useGyro)
	{
	}
}

void loop()
{	
	// cheapo scheduler

	// check bumper

	// check SF ??

	if (cntr % checkMqFrequency == 0)
		CheckMq();

	if (cntr % checkButtonFrequency == 0)
		CheckButtons();

	if (cntr % CalcPoseFrequency == 0)
		CalcPose();

	motorPower = motorPower > 100 ? 100 : motorPower < 0 ? 0 : motorPower;	// clip

	if (cntr % 2000 == 0)
	{
		sprintf(t, "pwr: %d\n", motorPower);
		Serial.write(t);
	}


	if (cntr % 5000 == 0)
	{
		digitalWrite(ESC_EN, esc_enabled);
		digitalWrite(M1_DIR, motorCW);
		analogWrite(PWM1, map(motorPower,0,100,0,255));
	}

	if (cntr % 5000 == 0)  // blinky
		digitalWrite(LED, !digitalRead(LED));	

	cntr++;

}
