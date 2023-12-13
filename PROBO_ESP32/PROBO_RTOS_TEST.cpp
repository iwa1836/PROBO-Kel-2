#pragma region Include Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QuickPID.h>
#pragma endregion

#pragma region Pin Definitions
#define ENCR_A 39
#define ENCR_B 36
#define ENCL_A 35
#define ENCL_B 34

#define LT_SENS_ADC 32
#define PHOTODIODE_ADC 33
#define BATT_ADC 25

#define MUX_A0 26
#define MUX_A1 27
#define MUX_A2 14

#define BUTTON_1 12
#define BUTTON_2 13

#define BUZZ_PIN 23

#define MOTOR_S_1 1
#define MOTOR_S_2 3
#define MOTOR_R_1 19
#define MOTOR_R_2 18
#define MOTOR_L_1 5
#define MOTOR_L_2 17

#define SERVO_1 16
#define SERVO_2 4
#define SERVO_5 15

#define STEPPER_STEP 2
#define STEPPER_DIR 0
#pragma endregion

#pragma region Variables
#define AP_SSID "PROBO KEL 2"
#define AP_PASS "123321abccba"

unsigned long currMill, prevMill;
const uint16_t sendingInterval = 50;
uint8_t leftAxisX, leftAxisY, rightAxisX, rightAxisY;
uint16_t buttons;
uint8_t telemetryData[19], configData[16];
uint32_t packetCount;
uint8_t BattADC, LMotor, RMotor, SMotor, LTSensor[8], LTSensor_HIGH[8], LTSensor_LOW[8], speed, ballColor, LTThreshold;
bool startPoint, stepDir;
float PIDSetpoint = 0, PIDInput, PIDOutput;

typedef enum {
	PROBO_CONFIG,
	PROBO_READY,
	PROBO_AUTO_SEARCH_BALL,
	PROBO_AUTO_PICK_BALL,
	PROBO_AUTO_DELIVER_BALL,
	PROBO_AUTO_DROP_BALL_1,
	PROBO_AUTO_DROP_BALL_2,
	PROBO_AUTO_DROP_BALL_3,
	PROBO_AUTO_DROP_BALL_4,
	PROBO_AUTO_SEARCH_MANUAL,
	PROBO_MANUAL_RUNNING,
	PROBO_MANUAL_PICK_BALL,
	PROBO_MANUAL_SHOOT_BALL
} State;

typedef union {
	uint8_t bytes[4];
	float floatVal;
} ByteToFloat;

State proboState;
ByteToFloat Kp, Ki, Kd;

int32_t EncR_Count, EncL_Count;
#pragma endregion

#pragma region Object
TaskHandle_t LTSensRead_Handle;
TaskHandle_t BallColorRead_Handle;
TaskHandle_t BallShooting_Handle;
TaskHandle_t BallPicker_Handle;
TaskHandle_t PID_Handle;
TaskHandle_t Motor_Handle;
TaskHandle_t OLED_Handle;
TaskHandle_t ProboState_Handle;
TaskHandle_t RevolverMove90_Handle;

WebSocketsServer wsServer(81);

Servo servo1;
Servo servo2;
Servo servo5;

ESP32PWM rightMotor1;
ESP32PWM rightMotor2;
ESP32PWM leftMotor1;
ESP32PWM leftMotor2;

Preferences preferences;

QuickPID pid(&PIDInput, &PIDOutput, &PIDSetpoint);
#pragma endregion

#pragma region Task Codes
void LTSensRead(void *pvParameters) {
	for(;;) {
		for(uint8_t i = 0; i < 8; i++) {
			digitalWrite(MUX_A0, i & (1 << 2));
			digitalWrite(MUX_A1, i & (1 << 1));
			digitalWrite(MUX_A2, i & 1);
			vTaskDelay(1 / portTICK_PERIOD_MS);
			if(proboState == PROBO_CONFIG) {
				LTSensor[i] = 255 - analogRead(LT_SENS_ADC);
			} else {
				int LTSensCal = map(255 - analogRead(LT_SENS_ADC), LTSensor_LOW[i], LTSensor_HIGH[i], 0, 255);
				if(LTSensCal < LTThreshold) {
					LTSensor[i] = 0;
				} else if(LTSensCal > LTThreshold) {
					LTSensor[i] = 255;
				}
			}
		}
	}
}

void BallColorRead(void *pvParameters) {
	vTaskDelete(NULL);
}

void BallShooting(void *pvParameters) {
	servo5.write(0);
	vTaskDelay(1500 / portTICK_PERIOD_MS);
	servo5.write(90);
	vTaskDelete(NULL);
}

void BallPicker(void *pvParameters) {
	servo1.write(0);
	servo2.write(180);
	vTaskDelay(1500 / portTICK_PERIOD_MS);
	servo1.write(90);
	servo2.write(90);
	vTaskDelete(NULL);
}

void PID(void *pvParameters) {
	pid.SetMode(QuickPID::Control::automatic);
	pid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
	pid.SetSampleTimeUs(50000);
	for(;;) {
		if(proboState == PROBO_CONFIG) {
			pid.SetOutputLimits(speed - 255, 255 - speed);
			pid.Reset();
			vTaskDelay(10000 / portTICK_PERIOD_MS);
		} else if(proboState >= PROBO_MANUAL_RUNNING) {
			pid.Reset();
			vTaskDelay(30000 / portTICK_PERIOD_MS);
		} else {
			pid.SetTunings(Kp.floatVal, Ki.floatVal, Kd.floatVal);
			PIDInput = (LTSensor[5] * 0.1) + (LTSensor[7] * 0.2) + (LTSensor[3] * 0.4) + (LTSensor[1] * 0.8) - (LTSensor[6] * 0.1) - (LTSensor[0] * 0.2) - (LTSensor[4] * 0.4) - (LTSensor[2] * 0.8);
			pid.Compute();
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
	}
}

void Motor(void *pvParameters) {
	rightMotor1.attachPin(MOTOR_R_1, 1000, 8);
	rightMotor2.attachPin(MOTOR_R_2, 1000, 8);
	leftMotor1.attachPin(MOTOR_L_1, 1000, 8);
	leftMotor2.attachPin(MOTOR_L_2, 1000, 8);
	for(;;) {
		if(proboState == PROBO_AUTO_SEARCH_BALL || proboState == PROBO_AUTO_DELIVER_BALL || proboState == PROBO_AUTO_SEARCH_MANUAL) {
			RMotor = speed - PIDOutput;
			LMotor = speed + PIDOutput;
			rightMotor1.write(0);
			rightMotor2.write(RMotor);
			leftMotor1.write(LMotor);
			leftMotor2.write(0);
		} else if(proboState == PROBO_MANUAL_RUNNING || proboState == PROBO_MANUAL_PICK_BALL || proboState == PROBO_MANUAL_SHOOT_BALL) {
			rightMotor1.write(-(leftAxisY - 127) + (rightAxisX - 127));
			rightMotor2.write((leftAxisY - 127) - (rightAxisX - 127));
			leftMotor1.write((leftAxisY - 127) + (rightAxisX - 127));
			leftMotor2.write(-(leftAxisY - 127) - (rightAxisX - 127));
			RMotor = (leftAxisY - 127) - (rightAxisX - 127);
			LMotor = (leftAxisY - 127) + (rightAxisX - 127);
		} else {
			rightMotor1.write(0);
			rightMotor2.write(0);
			leftMotor1.write(0);
			leftMotor2.write(0);
			RMotor = 0;
			LMotor = 0;
		}
	}
}

void OLED(void *pvParameters) {
	vTaskDelete(NULL);
}

void ProboState(void *pvParameters) {
	for(;;) {
		if(proboState == PROBO_MANUAL_PICK_BALL) {
			xTaskCreatePinnedToCore(BallPicker, "BallPicker", 1024, NULL, 1, &BallPicker_Handle, 1);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			proboState = PROBO_MANUAL_RUNNING;
		} else if(proboState == PROBO_MANUAL_SHOOT_BALL) {
			xTaskCreatePinnedToCore(BallShooting, "BallShooting", 1024, NULL, 1, &BallShooting_Handle, 1);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			proboState = PROBO_MANUAL_RUNNING;
		} else if(proboState == PROBO_AUTO_PICK_BALL) {
			xTaskCreatePinnedToCore(BallPicker, "BallPicker", 1024, NULL, 1, &BallPicker_Handle, 1);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			proboState = PROBO_AUTO_DELIVER_BALL;
		}
	}
}

void RevolverMove90(void *pvParameters) {
	for(;;) {
		uint16_t step;
		digitalWrite(STEPPER_STEP, HIGH);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		digitalWrite(STEPPER_STEP, LOW);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		step++;
		if(step == 512) {
			vTaskDelete(NULL);
		}
	}
}
#pragma endregion

#pragma region Function Codes
void sendConfig() {
	startPoint = preferences.getBool("StartPoint", 0);
	Kp.floatVal = preferences.getFloat("Kp", 0);
	Ki.floatVal = preferences.getFloat("Ki", 0);
	Kd.floatVal = preferences.getFloat("Kd", 0);
	speed = preferences.getUChar("Speed", 0);
	LTThreshold = preferences.getUChar("LTThreshold", 0);
	
	configData[0] = 0x5A;
	configData[1] = startPoint;
	configData[2] = Kp.bytes[0];
	configData[3] = Kp.bytes[1];
	configData[4] = Kp.bytes[2];
	configData[5] = Kp.bytes[3];
	configData[6] = Ki.bytes[0];
	configData[7] = Ki.bytes[1];
	configData[8] = Ki.bytes[2];
	configData[9] = Ki.bytes[3];
	configData[10] = Kd.bytes[0];
	configData[11] = Kd.bytes[1];
	configData[12] = Kd.bytes[2];
	configData[13] = Kd.bytes[3];
	configData[14] = speed;
	configData[15] = LTThreshold;
	wsServer.broadcastBIN(configData, sizeof(configData));
}

void clientConnected() {
	digitalWrite(BUZZ_PIN, HIGH);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, LOW);
}

void clientDisconnected() {
	digitalWrite(BUZZ_PIN, HIGH);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, LOW);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, HIGH);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, LOW);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, HIGH);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	digitalWrite(BUZZ_PIN, LOW);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
	switch(type) {
		case WStype_CONNECTED:
			clientConnected();
			sendConfig();
			break;

		case WStype_DISCONNECTED:
			clientDisconnected();
			break;

		case WStype_BIN:
			if((payload[0] == 0xAA) && (proboState == PROBO_CONFIG)) {
				proboState = PROBO_READY;
			}
			if((payload[0] == 0x55) && (proboState != PROBO_CONFIG)) {
				proboState = PROBO_CONFIG;
				wsServer.broadcastBIN(configData, sizeof(configData));
			}
			if((payload[0] == 0xA5) && (proboState == PROBO_CONFIG)) {
				startPoint = payload[1];
				Kp.bytes[0] = payload[2];
				Kp.bytes[1] = payload[3];
				Kp.bytes[2] = payload[4];
				Kp.bytes[3] = payload[5];
				Ki.bytes[0] = payload[6];
				Ki.bytes[1] = payload[7];
				Ki.bytes[2] = payload[8];
				Ki.bytes[3] = payload[9];
				Kd.bytes[0] = payload[10];
				Kd.bytes[1] = payload[11];
				Kd.bytes[2] = payload[12];
				Kd.bytes[3] = payload[13];
				speed = payload[14];
				LTThreshold = payload[15];

				preferences.putBool("StartPoint", startPoint);
				preferences.putFloat("Kp", Kp.floatVal);
				preferences.putFloat("Ki", Ki.floatVal);
				preferences.putFloat("Kd", Kd.floatVal);
				preferences.putUChar("Speed", speed);
				preferences.putUChar("LTThreshold", LTThreshold);
			}
			if((payload[0] == 0x5A) && (proboState == PROBO_CONFIG)) {
				sendConfig();
			}
			if((payload[0] == 0xFF) && (proboState == PROBO_CONFIG)) {
				for(uint8_t i = 0; i < 8; i++) {
					LTSensor_HIGH[i] = LTSensor[i];
				}
			}
			if((payload[0] == 0x00) && (proboState == PROBO_CONFIG)) {
				for(uint8_t i = 0; i < 8; i++) {
					LTSensor_LOW[i] = LTSensor[i];
				}
			}
			if((payload[0] == 0xFF) && (proboState >= PROBO_READY)) {
				leftAxisX = payload[1];
				leftAxisY = payload[2];
				rightAxisX = payload[3];
				rightAxisY = payload[4];
				buttons = (payload[5] << 8);
				buttons |= payload[6];

				if((buttons & 0x0010) && (proboState > PROBO_AUTO_SEARCH_MANUAL || proboState < PROBO_AUTO_SEARCH_BALL)) {
					proboState = PROBO_AUTO_SEARCH_BALL;
				} else if(buttons & 0x0020) {
					proboState = PROBO_MANUAL_RUNNING;
				} else if((buttons & 0x1000) && proboState == PROBO_MANUAL_RUNNING) {
					proboState = PROBO_MANUAL_PICK_BALL;
				} else if((buttons & 0x2000) && proboState == PROBO_MANUAL_RUNNING) {
					proboState = PROBO_MANUAL_SHOOT_BALL;
				}
			}
			break;
			
		default:
			break;
	}
}
#pragma endregion

#pragma region ISR
void IRAM_ATTR EncR_ISR() {
	if(digitalRead(ENCR_A)) {
		if(digitalRead(ENCR_B)) {
		EncR_Count--;
		} else {
		EncR_Count++;
		}
	} else {
		if(digitalRead(ENCR_B)) {
		EncR_Count++;
		} else {
		EncR_Count--;
		}
	}
}

void IRAM_ATTR EncL_ISR() {
	if(digitalRead(ENCL_A)) {
		if(digitalRead(ENCL_B)) {
		EncL_Count--;
		} else {
		EncL_Count++;
		}
	} else {
		if(digitalRead(ENCL_B)) {
		EncL_Count++;
		} else {
		EncL_Count--;
		}
	}
}
#pragma endregion

void setup() {
	analogReadResolution(8);

	#pragma region Preferences Initialization
	preferences.begin("Config");
	startPoint = preferences.getBool("StartPoint", 0);
	Kp.floatVal = preferences.getFloat("Kp", 0);
	Ki.floatVal = preferences.getFloat("Ki", 0);
	Kd.floatVal = preferences.getFloat("Kd", 0);
	speed = preferences.getUChar("Speed", 0);
	LTThreshold = preferences.getUChar("LTThreshold", 0);
	#pragma endregion

	#pragma region pinMode
	pinMode(ENCR_A, INPUT_PULLUP);
	pinMode(ENCR_B, INPUT_PULLUP);
	pinMode(ENCL_A, INPUT_PULLUP);
	pinMode(ENCL_B, INPUT_PULLUP);

	pinMode(LT_SENS_ADC, INPUT);
	pinMode(PHOTODIODE_ADC, INPUT);
	pinMode(BATT_ADC, INPUT);
	
	pinMode(MUX_A0, OUTPUT);
	pinMode(MUX_A1, OUTPUT);
	pinMode(MUX_A2, OUTPUT);
	
	pinMode(BUTTON_1, INPUT_PULLUP);
	pinMode(BUTTON_2, INPUT_PULLUP);
	
	pinMode(BUZZ_PIN, OUTPUT);
	
	pinMode(MOTOR_S_1, OUTPUT);
	pinMode(MOTOR_S_2, OUTPUT);
	pinMode(MOTOR_R_1, OUTPUT);
	pinMode(MOTOR_R_2, OUTPUT);
	pinMode(MOTOR_L_1, OUTPUT);
	pinMode(MOTOR_L_2, OUTPUT);

	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_DIR, OUTPUT);
	#pragma endregion

	#pragma region attachInterrupt
	attachInterrupt(digitalPinToInterrupt(ENCR_A), EncR_ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCL_A), EncL_ISR, CHANGE);
	#pragma endregion

	#pragma region Task Creations
	xTaskCreatePinnedToCore(LTSensRead, "LTSensRead", 1024, NULL, 1, &LTSensRead_Handle, 0);
	//xTaskCreatePinnedToCore(BallColorRead, "BallColorRead", 1024, NULL, 1, &BallColorRead_Handle, 1);
	xTaskCreatePinnedToCore(PID, "PID", 1024, NULL, 2, &PID_Handle, 1);
	xTaskCreatePinnedToCore(Motor, "Motor", 1024, NULL, 1, &Motor_Handle, 1);
	//xTaskCreatePinnedToCore(OLED, "OLED", 1024, NULL, 1, &OLED_Handle, 1);
	xTaskCreatePinnedToCore(ProboState, "ProboState", 1024, NULL, 1, &ProboState_Handle, 1);
	#pragma endregion

	#pragma region Wi-Fi Initialization
	WiFi.mode(WIFI_MODE_AP);
	WiFi.softAP(AP_SSID, AP_PASS);

	wsServer.begin();
	wsServer.onEvent(onWebSocketEvent);
	#pragma endregion

	#pragma region Servo Initialization
	servo1.setPeriodHertz(50);
	servo2.setPeriodHertz(50);
	servo5.setPeriodHertz(50);

	servo1.attach(SERVO_1);
	servo2.attach(SERVO_2);
	servo5.attach(SERVO_5);

	servo1.write(90);
	servo2.write(90);
	servo5.write(90);
	#pragma endregion

	proboState = PROBO_CONFIG;
}

void loop() {
	wsServer.loop();
	
	currMill = millis();
	if(currMill - prevMill >= sendingInterval) {
		prevMill = currMill;
		wsServer.broadcastBIN(telemetryData, sizeof(telemetryData));

		packetCount++;
		if(packetCount % 200 == 0) {
			xTaskCreatePinnedToCore(RevolverMove90, "RevolverMove90", 1024, NULL, 1, &RevolverMove90_Handle, 1);
		}

		if(proboState == PROBO_CONFIG) {
			telemetryData[0] = 0x55;
		} else {
			telemetryData[0] = 0xAA;
		}
		telemetryData[1] = packetCount >> 24;
		telemetryData[2] = packetCount >> 16;
		telemetryData[3] = packetCount >> 8;
		telemetryData[4] = packetCount;
		telemetryData[5] = proboState;
		telemetryData[6] = BattADC;
		telemetryData[7] = LMotor;
		telemetryData[8] = RMotor;
		telemetryData[9] = SMotor;
		telemetryData[10] = LTSensor[6];
		telemetryData[11] = LTSensor[0];
		telemetryData[12] = LTSensor[4];
		telemetryData[13] = LTSensor[2];
		telemetryData[14] = LTSensor[5];
		telemetryData[15] = LTSensor[7];
		telemetryData[16] = LTSensor[3];
		telemetryData[17] = LTSensor[1];
		telemetryData[18] = ballColor;
	}
}