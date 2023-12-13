#include <Arduino.h>

void setup() {
	Serial.begin(115200);
	analogReadResolution(8);
	pinMode(5, OUTPUT);
	digitalWrite(5, LOW);
}

void loop() {
	Serial.println(analogRead(33));
}