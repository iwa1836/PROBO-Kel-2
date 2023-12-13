#pragma region Include Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
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
#define SERVO_3 0
#define SERVO_4 2
#define SERVO_5 15
#pragma endregion

#pragma region Variables
#define AP_SSID "PROBO KEL 2"
#define AP_PASS "123321abccba"

unsigned long currMill, prevMill;
const uint16_t sendingInterval = 50;
uint8_t leftAxisX, leftAxisY, rightAxisX, rightAxisY;
uint16_t buttons;
uint8_t telemetryData[19], configData[15];
uint32_t packetCount;
uint8_t BattADC, LMotor, RMotor, SMotor, LTSensor[8], speed, ballColor;
bool startPoint;

typedef enum {
	PROBO_CONFIG,
	PROBO_READY,
	PROBO_RUN_AUTO,
	PROBO_RUN_MANUAL
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

WebSocketsServer wsServer(81);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

Preferences preferences;
#pragma endregion

#pragma region Task Codes
void LTSensRead(void *pvParameters) {
	for(;;) {
		/*servo4.write(30);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		servo4.write(60);*/
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void BallColorRead(void *pvParameters) {
	for(;;) {
		/*servo2.write(30);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		servo2.write(60);*/
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

void BallShooting(void *pvParameters) {
	for(;;) {
		/*servo3.write(45);
		vTaskDelay(3000 / portTICK_PERIOD_MS);
		servo3.write(135);*/
		vTaskDelay(3000 / portTICK_PERIOD_MS);
	}
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
	for(;;) {
		/*servo5.write(45);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		servo5.write(135);*/
		vTaskDelay(1000 / portTICK_PERIOD_MS);
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

				preferences.putBool("StartPoint", startPoint);
				preferences.putFloat("Kp", Kp.floatVal);
				preferences.putFloat("Ki", Ki.floatVal);
				preferences.putFloat("Kd", Kd.floatVal);
				preferences.putUChar("Speed", speed);
			}
			if((payload[0] == 0x5A) && (proboState == PROBO_CONFIG)) {
				sendConfig();
			}
			if((payload[0] == 0xFF) && (proboState >= PROBO_READY)) {
				leftAxisX = payload[1];
				leftAxisY = payload[2];
				rightAxisX = payload[3];
				rightAxisY = payload[4];
				buttons = (payload[5] << 8);
				buttons |= payload[6];

				if(buttons & 0x0010) {
					proboState = PROBO_RUN_AUTO;
				} else if(buttons & 0x0020) {
					proboState = PROBO_RUN_MANUAL;
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
	#pragma region Preferences Initialization
	preferences.begin("Config");
	startPoint = preferences.getBool("StartPoint", 0);
	Kp.floatVal = preferences.getFloat("Kp", 0);
	Ki.floatVal = preferences.getFloat("Ki", 0);
	Kd.floatVal = preferences.getFloat("Kd", 0);
	speed = preferences.getUChar("Speed", 0);
	#pragma endregion

	#pragma region pinMode
	pinMode(ENCR_A, INPUT_PULLUP);
	pinMode(ENCR_B, INPUT_PULLUP);
	pinMode(ENCL_A, INPUT_PULLUP);
	pinMode(ENCL_B, INPUT_PULLUP);
	
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
	#pragma endregion

	#pragma region attachInterrupt
	attachInterrupt(digitalPinToInterrupt(ENCR_A), EncR_ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCL_A), EncL_ISR, CHANGE);
	#pragma endregion

	#pragma region Task Creations
	xTaskCreatePinnedToCore(LTSensRead, "LTSensRead", 1024, NULL, 2, &LTSensRead_Handle, 1);
	xTaskCreatePinnedToCore(BallColorRead, "BallColorRead", 1024, NULL, 2, &BallColorRead_Handle, 1);
	xTaskCreatePinnedToCore(BallShooting, "BallShooting", 1024, NULL, 2, &BallShooting_Handle, 1);
	//xTaskCreatePinnedToCore(BallPicker, "BallPicker", 1024, NULL, 2, &BallPicker_Handle, 1);
	xTaskCreatePinnedToCore(PID, "PID", 1024, NULL, 3, &PID_Handle, 1);
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
	servo3.setPeriodHertz(50);
	servo4.setPeriodHertz(50);
	servo5.setPeriodHertz(50);

	servo1.attach(SERVO_1);
	servo2.attach(SERVO_2);
	servo3.attach(SERVO_3);
	servo4.attach(SERVO_4);
	servo5.attach(SERVO_5);

	servo1.write(90);
	servo2.write(90);
	servo3.write(0);
	servo4.write(0);
	servo5.write(0);
	#pragma endregion

	proboState = PROBO_CONFIG;
}

void loop() {
	wsServer.loop();

	currMill = millis();
	if(currMill - prevMill >= sendingInterval) {
		prevMill = currMill;
		wsServer.broadcastBIN(telemetryData, sizeof(telemetryData));
		Serial.print(packetCount);
		Serial.print('\t');
		Serial.println(proboState);

		packetCount++;
		BattADC--;
		LMotor += 2;
		RMotor += 4;
		SMotor += 8;
		LTSensor[0]++;
		LTSensor[1] += 2;
		LTSensor[2] += 4;
		LTSensor[3] += 8;
		LTSensor[4] += 8;
		LTSensor[5] += 4;
		LTSensor[6] += 2;
		LTSensor[7]++;
		ballColor++;

		if(ballColor == 128) {
			xTaskCreatePinnedToCore(BallPicker, "BallPicker", 1024, NULL, 2, &BallPicker_Handle, 1);
			speed++;
			preferences.putUChar("Speed", speed);
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
		telemetryData[10] = LTSensor[0];
		telemetryData[11] = LTSensor[1];
		telemetryData[12] = LTSensor[2];
		telemetryData[13] = LTSensor[3];
		telemetryData[14] = LTSensor[4];
		telemetryData[15] = LTSensor[5];
		telemetryData[16] = LTSensor[6];
		telemetryData[17] = LTSensor[7];
		telemetryData[18] = ballColor;
	}
}