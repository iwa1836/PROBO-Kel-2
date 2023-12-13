#include <Arduino.h>
#include <WiFi.h>
//#include <esp_wifi.h>
#include <WebSocketsServer.h>

WebSocketsServer wsServer(81);

#define AP_SSID "PROBO KEL 2"
#define AP_PASS "123321abccba"

unsigned long currMill, prevMill;
const uint16_t sendingInterval = 50;
uint8_t leftAxisX, leftAxisY, rightAxisX, rightAxisY;
uint16_t buttons;
uint8_t telemetryData[18], configData[15];
uint32_t packetCount;
uint8_t BattADC, LMotor, RMotor, SMotor, LTSensor[8], speed;
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

void sendConfig() {
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

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
	switch(type) {
		case WStype_CONNECTED:
			Serial.println("Client connected");
			sendConfig();
			break;

		case WStype_DISCONNECTED:
			Serial.println("Client disconnected");
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
				Serial.printf("SP = %s\tKp = %f\tKi = %f\tKd = %f\tSpeed = %d\n", startPoint?"BLUE":"RED", Kp.floatVal, Ki.floatVal, Kd.floatVal, speed);
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
				Serial.printf("LX = %d\tLY = %d\tRX = %d\tRY = %d\tButtons = %d\r\n", leftAxisX, leftAxisY, rightAxisX, rightAxisY, buttons);

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

void setup() {
	Serial.begin(115200);

	WiFi.mode(WIFI_MODE_AP);
	WiFi.softAP(AP_SSID, AP_PASS);
	Serial.println("AP Started with this parameter:");
	Serial.printf("SSID: %s\n", AP_SSID);
	Serial.printf("Password: %s\n", AP_PASS);
	Serial.print("IP Address: ");
	Serial.println(WiFi.softAPIP());

	wsServer.begin();
	wsServer.onEvent(onWebSocketEvent);

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
	}
}