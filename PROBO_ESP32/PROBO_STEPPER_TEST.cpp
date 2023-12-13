#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_1 16
#define SERVO_2 4
#define SERVO_5 15

#define STEPPER_STEP 2
#define STEPPER_DIR 0

int step;
bool dir = 1;

Servo servo1;
Servo servo2;
Servo servo5;

void setup() {
	Serial.begin(115200);

	pinMode(STEPPER_DIR, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);

	servo1.setPeriodHertz(50);
	servo2.setPeriodHertz(50);
	servo5.setPeriodHertz(50);

	servo1.attach(SERVO_1);
	servo2.attach(SERVO_2);
	servo5.attach(SERVO_5);

	servo1.write(90);
	servo2.write(90);

	digitalWrite(STEPPER_DIR, dir);
}

void loop() {
	digitalWrite(STEPPER_STEP, LOW);
	delay(1);
	digitalWrite(STEPPER_STEP, HIGH);
	if(dir) {
		Serial.println(step++);
	} else {
		Serial.println(step--);
	}
		delay(1);
	if((step % 1024 == 0) && (step != 0)) {
		dir = !dir;
		digitalWrite(STEPPER_DIR, dir);
		servo1.write(0);
		servo2.write(180);
	}
	if(step % 512 == 0) {
		delay(2000);
		servo1.write(90);
		servo2.write(90);
	}
}