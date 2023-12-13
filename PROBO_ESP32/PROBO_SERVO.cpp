#include <Arduino.h>
#include <ESP32Servo.h>

Servo servo;

#define SERVO_PIN 15

void setup() {
	Serial.begin(115200);

	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo.setPeriodHertz(50);
	servo.attach(SERVO_PIN);
}

void loop() {
	servo.writeMicroseconds(1470);
	delay(2000);
	servo.writeMicroseconds(2345);
	delay(2000);
	servo.writeMicroseconds(1470);
	delay(2000);
	servo.writeMicroseconds(700);
	delay(2000);
}