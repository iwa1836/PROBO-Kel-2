#include <Arduino.h>

#define MUX_A0 26
#define MUX_A1 27
#define MUX_A2 14
#define LT_SENS_ADC 32

uint16_t LTSensor[8], LTThreshold = 2048;

void setup() {
	Serial.begin(115200);

	pinMode(MUX_A0, OUTPUT);
	pinMode(MUX_A1, OUTPUT);
	pinMode(MUX_A2, OUTPUT);
	pinMode(LT_SENS_ADC, INPUT);
}

void loop() {
	for(uint8_t i = 0; i < 8; i++) {
		digitalWrite(MUX_A0, i & (1 << 2));
		digitalWrite(MUX_A1, i & (1 << 1));
		digitalWrite(MUX_A2, i & 1);
		delay(1);
		if(analogRead(LT_SENS_ADC) < LTThreshold) {
			LTSensor[i] = 255;
		} else {
			LTSensor[i] = 0;
		}
	}
	Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", LTSensor[6], LTSensor[0], LTSensor[4], LTSensor[2], LTSensor[5], LTSensor[7], LTSensor[3], LTSensor[1]);
}