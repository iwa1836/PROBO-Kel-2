#include <Arduino.h>

#define M1_IN1 19
#define M1_IN2 18
#define M2_IN1 5
#define M2_IN2 17
#define EN1A 39
#define EN1B 36
#define EN2A 34
#define EN2B 35

int En1Dat, En2Dat, lastEn1Dat, lastEn2Dat, mot1PPS, mot2PPS, mot1RPM, mot2RPM;
uint32_t En1Time, En2Time, lastEn1Time, lastEn2Time;
unsigned long currTime, prevTime;
const uint16_t interval = 1000;
const uint8_t PPR = 204;

void Encoder1() {
	lastEn1Time = En1Time;
	En1Time = micros();
	mot1PPS = 1000000 / (En1Time - lastEn1Time);
	if(digitalRead(EN1A)) {
		if(digitalRead(EN1B)) {
			lastEn1Dat = En1Dat;
			En1Dat--;
			mot1PPS *= -1;
		} else {
			lastEn1Dat = En1Dat;
			En1Dat++;
		}
	} else {
		if(digitalRead(EN1B)) {
			lastEn1Dat = En1Dat;
			En1Dat++;
		} else {
			lastEn1Dat = En1Dat;
			En1Dat--;
			mot2PPS *= -1;
		}
	}
}

void Encoder2() {
	lastEn2Time = En2Time;
	En2Time = micros();
	mot2PPS = 1000000 / (En2Time - lastEn2Time);
	if(digitalRead(EN2A)) {
		if(digitalRead(EN2B)) {
			lastEn2Dat = En2Dat;
			En2Dat--;
			mot2PPS *= -1;
		} else {
			lastEn2Dat = En2Dat;
			En2Dat++;
		}
	} else {
		if(digitalRead(EN2B)) {
			lastEn2Dat = En2Dat;
			En2Dat++;
		} else {
			lastEn2Dat = En2Dat;
			En2Dat--;
			mot2PPS *= -1;
		}
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(EN1A, INPUT_PULLUP);
	pinMode(EN1B, INPUT_PULLUP);
	pinMode(EN2A, INPUT_PULLUP);
	pinMode(EN2B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(EN1A), Encoder1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(EN2A), Encoder2, CHANGE);

	ledcSetup(0, 1000, 8);
	ledcSetup(1, 1000, 8);
	ledcSetup(2, 1000, 8);
	ledcSetup(3, 1000, 8);

	ledcAttachPin(M1_IN1, 0);
	ledcAttachPin(M1_IN2, 1);
	ledcAttachPin(M2_IN1, 2);
	ledcAttachPin(M2_IN2, 3);

	ledcWrite(0, 0);
	ledcWrite(1, 255);
	ledcWrite(2, 64);
	ledcWrite(3, 0);
}

void loop() {
	currTime = millis();
	if(currTime - prevTime >= interval) {
		prevTime = currTime;
		if(En1Dat == (lastEn1Dat + 1) || En1Dat == (lastEn1Dat - 1)) {
			mot1PPS = 0;
		}
		if(En2Dat == (lastEn2Dat + 1) || En2Dat == (lastEn2Dat - 1)) {
			mot2PPS = 0;
		}
	}
	Serial.printf("Motor R = %d (%d PPS)\tMotor L = %d (%d PPS)\n", En1Dat, mot1PPS, En2Dat, mot2PPS);
}