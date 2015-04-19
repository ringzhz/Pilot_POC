#include <SoftwareSerial.h>

const int GpxTx = 6;
const int GpxRx = 5;

SoftwareSerial  Gps(GpxRx, GpxTx);

void setup()
{
	pinMode(13, OUTPUT);
	Gps.begin(4800);
	Serial.begin(9600);
}

int blinkCtr = 0;

char gpsBuf[256];
int gpsIdx = 0;

void loop()
{
	char t[64];
	if (++blinkCtr % 20000 == 0)
		digitalWrite(13, !digitalRead(13));

	if (Gps.available() > 0)
	{
		int c = Gps.read();
		gpsBuf[gpsIdx] = c;
		//sprintf(t, "%02x ", c);  Serial.print(t);
		if (c == 0x0d)
		{
			Serial.print(gpsBuf);
			gpsIdx = 0;
		}
		else if (++gpsIdx > sizeof(gpsBuf))
		{
			// overflow +++ flush until EOL
			gpsIdx = 0;
		}
	}
}
