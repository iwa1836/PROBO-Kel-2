#pragma region Include Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <QuickPID.h>
#pragma endregion

#pragma region Pin Definitions
#define ENCR_A 39
#define ENCR_B 36
#define ENCL_A 35
#define ENCL_B 34

#define LT_SENS_ADC 32
#define PHOTODIODE_ADC 33

#define MUX_A0 26
#define MUX_A1 27
#define MUX_A2 14

#define BUTTON_1 12
#define BUTTON_2 13

#define BUZZ_PIN 23

#define MOTOR_S_REL 1
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

unsigned long currMill, prevMill, currTimeout, startTimeout;
const uint16_t sendingInterval = 50, dropBallTimeout = 8000;
uint8_t leftAxisX, leftAxisY, rightAxisX, rightAxisY;
uint16_t buttons;
uint8_t telemetryData[20], configData[18];
uint32_t packetCount, simStartPacketCount;
uint8_t LTSensor[8], LTSensor_HIGH[8], LTSensor_LOW[8], speed, ballColor, ballADC, LTThreshold, ballThreshold, autoIndex;
bool startPoint, stepState, simModeState;
uint16_t stepCount;
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

typedef enum {
	KIRI,
	KANAN
} StepDirection;

typedef enum {
	INVALID,
	ORANGE,
	WHITE
} BallColor;

typedef union {
	uint8_t bytes[4];
	float floatVal;
} ByteToFloat;

State proboState;
ByteToFloat Kp, Ki, Kd;

int32_t EncR_Count, EncL_Count, turnEncR_Count, turnEncL_Count;
int16_t motR_PPS, motL_PPS;
uint32_t EncR_Time, EncL_Time, lastEncR_Time, lastEncL_Time;

bool proboRightTurnForward, proboLeftTurnForward, proboRightTurnReverse, proboLeftTurnReverse, proboForward, proboReverse, proboStop;
#pragma endregion

#pragma region Objects
TaskHandle_t ProboState_Handle;
TaskHandle_t LTSensRead_Handle;
TaskHandle_t RevolverMove90_Handle;
TaskHandle_t Motor_Handle;
TaskHandle_t PID_Handle;
TaskHandle_t BallPicker_Handle;
TaskHandle_t BallShooting_Handle;
TaskHandle_t BallDropping_Handle;
TaskHandle_t ArmPushBall_Handle;
TaskHandle_t BallInserting_Handle;
TaskHandle_t SimMode_Handle;

WebSocketsServer wsServer(81);

Servo servo1;
Servo servo2;
Servo servo5;

ESP32PWM rightMotor1;
ESP32PWM rightMotor2;
ESP32PWM leftMotor1;
ESP32PWM leftMotor2;
ESP32PWM buzzer;

Preferences preferences;

QuickPID pid(&PIDInput, &PIDOutput, &PIDSetpoint);
#pragma endregion

#pragma region Task Codes
void ProboState(void *pvParameters) {
	for(;;) {
		if(startPoint) { // Start Point Blue
			switch(proboState) {
				case PROBO_CONFIG:
					break;

				case PROBO_READY:
					break;

				case PROBO_AUTO_SEARCH_BALL:
					switch(autoIndex) {
						// Belok kanan 90
						case 0:
							if(LTSensor[6] && LTSensor[0] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								speed += 32;
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 220))) {
								speed -= 32;
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Belok kiri 90
						case 2:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 3:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 235))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Percabangan T
						case 4:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								autoIndex = 0;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
								proboState = PROBO_AUTO_PICK_BALL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_PICK_BALL:
					switch(autoIndex) {
						// Belok kiri
						case 0:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 100)) && (EncL_Count >= (turnEncL_Count + 100))) {
								proboForward = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								speed += 32;
								autoIndex++;
								proboForward = false;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 235))) {
								autoIndex++;
								proboLeftTurnForward = false;
							}
							break;

						// Ambil bola kanan
						case 2:
							autoIndex++;
							proboStop = true;
							break;

						case 3:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							vTaskResume(BallPicker_Handle);
							break;

						// Ambil bola kiri
						case 4:
							if(eTaskGetState(BallPicker_Handle) == eSuspended) {
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(RevolverMove90_Handle);
								proboStop = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 5:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 175)) && (EncL_Count <= (turnEncL_Count - 175))) {
								proboReverse = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								autoIndex++;
								proboReverse = false;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 6:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 245))) {
								proboLeftTurnForward = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 7:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 260))) {
								autoIndex++;
								proboRightTurnForward = false;
								proboStop = true;
							}
							break;

						case 8:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							vTaskResume(BallPicker_Handle);
							break;

						// Putar balik
						case 9:
							if(eTaskGetState(BallPicker_Handle) == eSuspended) {
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(RevolverMove90_Handle);
								proboStop = false;
								proboRightTurnReverse = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 10:
							if(proboRightTurnReverse && (EncL_Count == (turnEncL_Count - 525))) {
								speed -= 32;
								autoIndex = 0;
								pid.Reset();
								proboRightTurnReverse = false;
								proboState = PROBO_AUTO_DELIVER_BALL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DELIVER_BALL:
					if(eTaskGetState(BallInserting_Handle) == eSuspended) {
						vTaskResume(BallInserting_Handle);
					}
					switch(autoIndex) {
						// Percabangan kanan
						case 0:
							autoIndex++;
							proboForward = true;
							break;

						// Percabangan T
						case 1:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								proboForward = false;
								proboStop = true;
								vTaskDelay(5000 / portTICK_PERIOD_MS);
								proboStop = false;
								speed += 32;
								autoIndex = 0;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
								proboState = PROBO_AUTO_DROP_BALL_1;
							}
							break;

						default:
							break;

					}
					break;

				case PROBO_AUTO_DROP_BALL_1:
					if(eTaskGetState(BallInserting_Handle) != eSuspended) {
						vTaskSuspend(BallInserting_Handle);
					}
					if(ballColor != ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Belok kanan
						case 0:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 235))) {
								speed -= 32;
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Percabangan kiri drop bola 1
						case 1:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 100)) && (EncL_Count >= (turnEncL_Count + 100))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kiri
						case 2:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboLeftTurnReverse = true;
							turnEncR_Count = EncR_Count;
							break;

						case 3:
							if(proboLeftTurnReverse && (EncR_Count == (turnEncR_Count - 245))) {
								autoIndex++;
								proboLeftTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 4:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 30)) && (EncL_Count <= (turnEncL_Count - 30))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 1
						case 5:
							if(ballColor == ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 2
						case 6:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_2;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_2:
					if(ballColor != WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 1
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						// Belok kiri arah drop bola 2
						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Percabangan kiri drop bola 2
						case 2:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kiri
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboLeftTurnReverse = true;
							turnEncR_Count = EncR_Count;
							break;

						case 5:
							if(proboLeftTurnReverse && (EncR_Count == (turnEncR_Count - 276))) {
								autoIndex++;
								proboLeftTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 2
						case 7:
							if(ballColor == WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 3
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_3;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_3:
					if(ballColor != WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 2
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						// Belok kiri arah drop bola 3
						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Percabangan kiri drop bola 3
						case 2:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kiri
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboLeftTurnReverse = true;
							turnEncR_Count = EncR_Count;
							break;

						case 5:
							if(proboLeftTurnReverse && (EncR_Count == (turnEncR_Count - 276))) {
								autoIndex++;
								proboLeftTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 3
						case 7:
							if(ballColor == WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 4
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_4;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_4:
					if(ballColor != ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 3
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						// Belok kiri arah drop bola 4
						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Percabangan kiri drop bola 4
						case 2:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kiri
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboLeftTurnReverse = true;
							turnEncR_Count = EncR_Count;
							break;

						case 5:
							if(proboLeftTurnReverse && (EncR_Count == (turnEncR_Count - 276))) {
								autoIndex++;
								proboLeftTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 4
						case 7:
							if(ballColor == ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke mode manual
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_SEARCH_MANUAL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_SEARCH_MANUAL:
					switch(autoIndex) {
						// Percabangan T drop bola 3
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Belok kanan arah manual
						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Percabangan kanan drop bola 3
						case 2:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kanan drop bola 2
						case 3:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kanan drop bola 1
						case 4:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kiri
						case 5:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Belok kiri 90
						case 6:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 7:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Masuk kotak manual
						case 8:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && LTSensor[2] && LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
							}
							break;

						case 9:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 200)) && (EncL_Count >= (turnEncL_Count + 200))) {
								autoIndex = 0;
								proboForward = false;
								proboStop = true;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_MANUAL_RUNNING:
					break;

				case PROBO_MANUAL_PICK_BALL:
					break;

				case PROBO_MANUAL_SHOOT_BALL:
					break;

				default:
					buzzer.writeNote(NOTE_C, 6);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					buzzer.write(0);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					break;
			}
		} else { // Start Point Red
			switch(proboState) {
				case PROBO_CONFIG:
					break;

				case PROBO_READY:
					break;

				case PROBO_AUTO_SEARCH_BALL:
					switch(autoIndex) {
						// Belok kiri 90
						case 0:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 220))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Belok kanan 90
						case 2:
							if(LTSensor[6] && LTSensor[0] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								speed += 32;
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 235))) {
								speed -= 32;
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Percabangan T
						case 4:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								autoIndex = 0;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
								proboState = PROBO_AUTO_PICK_BALL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_PICK_BALL:
					switch(autoIndex) {
						// Belok kanan
						case 0:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 100)) && (EncL_Count >= (turnEncL_Count + 100))) {
								proboForward = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								speed += 32;
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 235))) {
								speed -= 32;
								autoIndex++;
								proboRightTurnForward = false;
							}
							break;

						// Ambil bola kiri
						case 2:
							autoIndex++;
							proboStop = true;
							break;

						case 3:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							vTaskResume(BallPicker_Handle);
							break;

						// Ambil bola kanan
						case 4:
							if(eTaskGetState(BallPicker_Handle) == eSuspended) {
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(RevolverMove90_Handle);
								proboStop = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 5:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 175)) && (EncL_Count <= (turnEncL_Count - 175))) {
								proboReverse = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								speed += 32;
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 6:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 245))) {
								speed -= 32;
								proboRightTurnForward = false;
								proboStop = true;
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								proboStop = false;
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 7:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 260))) {
								autoIndex++;
								proboLeftTurnForward = false;
								proboStop = true;
							}
							break;

						case 8:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							vTaskResume(BallPicker_Handle);
							break;

						// Putar balik
						case 9:
							if(eTaskGetState(BallPicker_Handle) == eSuspended) {
								vTaskDelay(1000 / portTICK_PERIOD_MS);
								speed += 32;
								autoIndex++;
								vTaskResume(RevolverMove90_Handle);
								proboStop = false;
								proboLeftTurnReverse = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						case 10:
							if(proboLeftTurnReverse && (EncR_Count == (turnEncR_Count - 525))) {
								speed -= 32;
								autoIndex = 0;
								pid.Reset();
								proboLeftTurnReverse = false;
								proboState = PROBO_AUTO_DELIVER_BALL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DELIVER_BALL:
					if(eTaskGetState(BallInserting_Handle) == eSuspended) {
						vTaskResume(BallInserting_Handle);
					}
					switch(autoIndex) {
						case 0:
							autoIndex++;
							proboForward = true;
							break;

						// Percabangan T
						case 1:
							if(!LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3]) {
								proboForward = false;
								proboStop = true;
								vTaskDelay(5000 / portTICK_PERIOD_MS);
								proboStop = false;
								speed += 32;
								autoIndex = 0;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
								proboState = PROBO_AUTO_DROP_BALL_1;
							}
							break;

						default:
							break;

					}
					break;

				case PROBO_AUTO_DROP_BALL_1:
					if(eTaskGetState(BallInserting_Handle) != eSuspended) {
						vTaskSuspend(BallInserting_Handle);
					}
					if(ballColor != ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Belok kiri
						case 0:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 235))) {
								speed -= 32;
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Percabangan kanan drop bola 1
						case 1:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 100)) && (EncL_Count >= (turnEncL_Count + 100))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kanan
						case 2:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							speed += 32;
							autoIndex++;
							proboStop = false;
							proboRightTurnReverse = true;
							turnEncL_Count = EncL_Count;
							break;

						case 3:
							if(proboRightTurnReverse && (EncL_Count == (turnEncL_Count - 245))) {
								speed -= 32;
								autoIndex++;
								proboRightTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 4:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 30)) && (EncL_Count <= (turnEncL_Count - 30))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 1
						case 5:
							if(ballColor == ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 2
						case 6:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_2;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_2:
					if(ballColor != WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 1
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Belok kanan arah drop bola 2
						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Percabangan kanan drop bola 2
						case 2:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kanan
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboRightTurnReverse = true;
							turnEncL_Count = EncL_Count;
							break;

						case 5:
							if(proboRightTurnReverse && (EncL_Count == (turnEncL_Count - 276))) {
								autoIndex++;
								proboRightTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 2
						case 7:
							if(ballColor == WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 3
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_3;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_3:
					if(ballColor != WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 2
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Belok kanan arah drop bola 3
						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Percabangan kanan drop bola 3
						case 2:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kanan
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboRightTurnReverse = true;
							turnEncL_Count = EncL_Count;
							break;

						case 5:
							if(proboRightTurnReverse && (EncL_Count == (turnEncL_Count - 276))) {
								autoIndex++;
								proboRightTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 3
						case 7:
							if(ballColor == WHITE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke drop bola 4
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_DROP_BALL_4;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_DROP_BALL_4:
					if(ballColor != ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended && eTaskGetState(BallDropping_Handle) == eSuspended) {
						vTaskResume(RevolverMove90_Handle);
					}
					switch(autoIndex) {
						// Percabangan T drop bola 3
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Belok kanan arah drop bola 4
						case 1:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Percabangan kanan drop bola 4
						case 2:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 3:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 218)) && (EncL_Count >= (turnEncL_Count + 218))) {
								autoIndex++;
								proboForward = false;
								proboStop = true;
							}
							break;

						// Mundur kanan
						case 4:
							vTaskDelay(500 / portTICK_PERIOD_MS);
							autoIndex++;
							proboStop = false;
							proboRightTurnReverse = true;
							turnEncL_Count = EncL_Count;
							break;

						case 5:
							if(proboRightTurnReverse && (EncL_Count == (turnEncL_Count - 276))) {
								autoIndex++;
								proboRightTurnReverse = false;
								proboReverse = true;
								turnEncR_Count = EncR_Count;
								turnEncL_Count = EncL_Count;
							}
							break;

						// Mundur lurus
						case 6:
							if(proboReverse && (EncR_Count <= (turnEncR_Count - 100)) && (EncL_Count <= (turnEncL_Count - 100))) {
								autoIndex++;
								proboReverse = false;
								proboStop = true;
								startTimeout = millis();
							}
							break;

						// Drop bola 4
						case 7:
							if(ballColor == ORANGE && eTaskGetState(RevolverMove90_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex++;
								vTaskResume(BallDropping_Handle);
							}
							currTimeout = millis();
							if((currTimeout - startTimeout) >= dropBallTimeout) {
								autoIndex++;
							}
							break;

						// Maju ke mode manual
						case 8:
							if(eTaskGetState(BallDropping_Handle) == eSuspended) {
								vTaskDelay(500 / portTICK_PERIOD_MS);
								autoIndex = 0;
								pid.Reset();
								proboStop = false;
								proboState = PROBO_AUTO_SEARCH_MANUAL;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_AUTO_SEARCH_MANUAL:
					switch(autoIndex) {
						// Percabangan T drop bola 3
						case 0:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboLeftTurnForward = true;
								turnEncR_Count = EncR_Count;
							}
							break;

						// Belok kiri arah manual
						case 1:
							if(proboLeftTurnForward && (EncR_Count == (turnEncR_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboLeftTurnForward = false;
							}
							break;

						// Percabangan kiri drop bola 3
						case 2:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kiri drop bola 2
						case 3:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kiri drop bola 1
						case 4:
							if(LTSensor[6] && !LTSensor[0] && !LTSensor[4] && !LTSensor[2] && !LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Percabangan kanan
						case 5:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								pid.Reset();
							}
							break;

						// Belok kanan 90
						case 6:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && !LTSensor[2] && !LTSensor[5] && !LTSensor[7] && !LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboRightTurnForward = true;
								turnEncL_Count = EncL_Count;
							}
							break;

						case 7:
							if(proboRightTurnForward && (EncL_Count == (turnEncL_Count + 276))) {
								autoIndex++;
								pid.Reset();
								proboRightTurnForward = false;
							}
							break;

						// Masuk kotak manual
						case 8:
							if(LTSensor[6] && LTSensor[0] && LTSensor[4] && LTSensor[2] && LTSensor[5] && LTSensor[7] && LTSensor[3] && LTSensor[1]) {
								autoIndex++;
								proboForward = true;
							}
							break;

						case 9:
							if(proboForward && (EncR_Count >= (turnEncR_Count + 200)) && (EncL_Count >= (turnEncL_Count + 200))) {
								autoIndex = 0;
								proboForward = false;
								proboStop = true;
							}
							break;

						default:
							break;
					}
					break;

				case PROBO_MANUAL_RUNNING:
					break;

				case PROBO_MANUAL_PICK_BALL:
					break;

				case PROBO_MANUAL_SHOOT_BALL:
					break;

				default:
					buzzer.writeNote(NOTE_C, 6);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					buzzer.write(0);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					break;
			}
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

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

void RevolverMove90(void *pvParameters) {
	for(;;) {
		digitalWrite(STEPPER_STEP, HIGH);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		digitalWrite(STEPPER_STEP, LOW);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		stepCount++;
		if(stepCount == 512) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			stepCount = 0;
			vTaskSuspend(RevolverMove90_Handle);
		}
	}
}

void Motor(void *pvParameters) {
	for(;;) {
		if(proboState >= PROBO_AUTO_SEARCH_BALL && proboState <= PROBO_AUTO_SEARCH_MANUAL) {
			if(proboRightTurnForward) {
				rightMotor1.write(0);
				rightMotor2.write(0);
				leftMotor1.write(speed * 1.25);
				leftMotor2.write(0);
			} else if(proboLeftTurnForward) {
				rightMotor1.write(0);
				rightMotor2.write(speed * 1.25);
				leftMotor1.write(0);
				leftMotor2.write(0);
			} else if(proboRightTurnReverse) {
				rightMotor1.write(0);
				rightMotor2.write(0);
				leftMotor1.write(0);
				leftMotor2.write(speed * 1.25);
			} else if(proboLeftTurnReverse) {
				rightMotor1.write(speed * 1.25);
				rightMotor2.write(0);
				leftMotor1.write(0);
				leftMotor2.write(0);
			} else if(proboForward) {
				rightMotor1.write(0);
				rightMotor2.write(speed * 1.2);
				leftMotor1.write(speed * 1.1);
				leftMotor2.write(0);
			} else if(proboReverse) {
				rightMotor1.write(speed * 1.2);
				rightMotor2.write(0);
				leftMotor1.write(0);
				leftMotor2.write(speed * 1.1);
			} else if(proboStop) {
				rightMotor1.write(0);
				rightMotor2.write(0);
				leftMotor1.write(0);
				leftMotor2.write(0);
			} else {
				rightMotor1.write(0);
				rightMotor2.write(speed - (PIDOutput * 0.1));
				leftMotor1.write(speed + (PIDOutput * 0.1));
				leftMotor2.write(0);
			}
		} else if(proboState >= PROBO_MANUAL_RUNNING) {
			rightMotor1.write(-(leftAxisY - 127) + (rightAxisX - 127));
			rightMotor2.write((leftAxisY - 127) - (rightAxisX - 127));
			leftMotor1.write((leftAxisY - 127) + (rightAxisX - 127));
			leftMotor2.write(-(leftAxisY - 127) - (rightAxisX - 127));
		} else {
			rightMotor1.write(0);
			rightMotor2.write(0);
			leftMotor1.write(0);
			leftMotor2.write(0);
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void PID(void *pvParameters) {
	for(;;) {
		PIDInput = (LTSensor[5] * 0.1) + (LTSensor[7] * 0.4) + (LTSensor[3] * 1.6) + (LTSensor[1] * 6.4) - (LTSensor[6] * 6.4) - (LTSensor[0] * 1.6) - (LTSensor[4] * 0.4) - (LTSensor[2] * 0.1);
		pid.Compute();
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void BallPicker(void *pvParameters) {
	for(;;) {
		if(proboState == PROBO_MANUAL_RUNNING) {
			proboState = PROBO_MANUAL_PICK_BALL;
		} else {
			proboState = PROBO_AUTO_PICK_BALL;
		}
		servo1.write(0);
		servo2.write(180);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		servo1.write(90);
		servo2.write(90);
		if(proboState == PROBO_MANUAL_PICK_BALL) {
			proboState = PROBO_MANUAL_RUNNING;
		}
		vTaskSuspend(BallPicker_Handle);
	}
}

void BallShooting(void *pvParameters) {
	for(;;) {
		if(proboState == PROBO_MANUAL_RUNNING) {
			proboState = PROBO_MANUAL_SHOOT_BALL;
		}
		digitalWrite(MOTOR_S_REL, HIGH);
		vTaskDelay(800 / portTICK_PERIOD_MS);
		servo5.write(0);
		vTaskDelay(700 / portTICK_PERIOD_MS);
		servo5.write(2);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		servo5.write(90);
		digitalWrite(MOTOR_S_REL, LOW);
		vTaskDelay(1500 / portTICK_PERIOD_MS);
		if(proboState == PROBO_MANUAL_SHOOT_BALL) {
			proboState = PROBO_MANUAL_RUNNING;
		}
		vTaskSuspend(BallShooting_Handle);
	}
}

void BallDropping(void *pvParameters) {
	for(;;) {
		if(proboState == PROBO_MANUAL_RUNNING) {
			proboState = PROBO_MANUAL_SHOOT_BALL;
		}
		servo5.write(0);
		vTaskDelay(700 / portTICK_PERIOD_MS);
		servo5.write(2);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		servo5.write(90);
		vTaskDelay(1500 / portTICK_PERIOD_MS);
		if(proboState == PROBO_MANUAL_SHOOT_BALL) {
			proboState = PROBO_MANUAL_RUNNING;
		}
		vTaskSuspend(BallDropping_Handle);
	}
}

void ArmPushBall(void *pvParameters) {
	for(;;) {
		servo1.write(123);
		servo2.write(57);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		servo1.write(90);
		servo2.write(90);
		vTaskSuspend(ArmPushBall_Handle);
	}
}

void BallInserting(void *pvParameters) {
	for(;;) {
		if(eTaskGetState(RevolverMove90_Handle) == eSuspended) {
			vTaskResume(RevolverMove90_Handle);
			vTaskDelay(1500 / portTICK_PERIOD_MS);
			vTaskResume(ArmPushBall_Handle);
		}
	}
}

void SimMode(void *pvParameters) {
	for(;;) {
		if(startPoint) { // Start Point Blue
			switch(simStartPacketCount) {
				#pragma region Sim LT 90 Degrees
				case 0: // Starting
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;

				case 20: // Kotak start
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;

				case 40: // Garis awal
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;

				case 100: // Belok kotak 90 kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;

				case 120: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 200: // Belok kotak 90 kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 220: // Garis lurus ketiga
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 300: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Pick Ball
				case 500: // Garis lurus dari bola
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 600: // Percabangan kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 610: // Garis lurus
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 700: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 720: // Garis lurus arah drop bola
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 760: // Percabangan drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 1
				case 770: // Garis lurus dari drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1100: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1120: // Garis lurus arah drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1200: // Percabangan drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 2
				case 1210: // Garis lurus dari drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1500: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1520: // Garis lurus arah drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1600: // Percabangan drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 3
				case 1610: // Garis lurus dari drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1900: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1920: // Garis lurus arah drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2000: // Percabangan drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 4
				case 2010: // Garis lurus dari drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2300: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2320: // Garis lurus arah drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2400: // Percabangan drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2410: // Garis lurus arah drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2500: // Percabangan drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2510: // Garis lurus arah drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2600: // Percabangan drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2610: // Garis lurus arah manual
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2700: // Percabangan kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2710: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2800: // Belok kotak 90 kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT Curve
				case 2820: // Garis lurus pertama
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3000: // Curve kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 0;
					LTSensor[1] = 0;
					break;
					
				case 3100: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3300: // Curve kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 0;
					LTSensor[0] = 0;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3400: // Garis lurus ketiga
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3600: // Curve kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 0;
					LTSensor[1] = 0;
					break;
					
				case 3700: // Garis lurus terakhir
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3740: // No line
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3800: // Kotak finish
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
				#pragma endregion
					
				default:
					buzzer.write(0);
					break;
			}
		} else { // Start Point Red
			switch(simStartPacketCount) {
				#pragma region Sim LT 90 Degrees
				case 0: // Starting
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;

				case 20: // Kotak start
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;

				case 40: // Garis awal
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;

				case 100: // Belok kotak 90 kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;

				case 120: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 200: // Belok kotak 90 kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 220: // Garis lurus ketiga
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 300: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Pick Ball
				case 500: // Garis lurus dari bola
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 600: // Percabangan kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 610: // Garis lurus
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 700: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 720: // Garis lurus arah drop bola
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 760: // Percabangan drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 1
				case 770: // Garis lurus dari drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1100: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1120: // Garis lurus arah drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1200: // Percabangan drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 2
				case 1210: // Garis lurus dari drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1500: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1520: // Garis lurus arah drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1600: // Percabangan drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 3
				case 1610: // Garis lurus dari drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 1900: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 1920: // Garis lurus arah drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2000: // Percabangan drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT After Drop Ball 4
				case 2010: // Garis lurus dari drop bola 4
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2300: // Percabangan T
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2320: // Garis lurus arah drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2400: // Percabangan drop bola 3
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2410: // Garis lurus arah drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2500: // Percabangan drop bola 2
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2510: // Garis lurus arah drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2600: // Percabangan drop bola 1
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2610: // Garis lurus arah manual
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2700: // Percabangan kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				case 2710: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 2800: // Belok kotak 90 kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
					
				#pragma endregion

				#pragma region Sim LT Curve
				case 2820: // Garis lurus pertama
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3000: // Curve kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 0;
					LTSensor[0] = 0;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3100: // Garis lurus kedua
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3300: // Curve kanan
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 0;
					LTSensor[1] = 0;
					break;
					
				case 3400: // Garis lurus ketiga
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3600: // Curve kiri
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 0;
					LTSensor[0] = 0;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3700: // Garis lurus terakhir
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3740: // No line
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 255;
					LTSensor[4] = 255;
					LTSensor[2] = 255;
					LTSensor[5] = 255;
					LTSensor[7] = 255;
					LTSensor[3] = 255;
					LTSensor[1] = 255;
					break;
					
				case 3800: // Kotak finish
					buzzer.writeNote(NOTE_E, 5);
					LTSensor[6] = 255;
					LTSensor[0] = 0;
					LTSensor[4] = 0;
					LTSensor[2] = 0;
					LTSensor[5] = 0;
					LTSensor[7] = 0;
					LTSensor[3] = 0;
					LTSensor[1] = 255;
					break;
				#pragma endregion
					
				default:
					buzzer.write(0);
					break;
			}
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
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
	ballThreshold = preferences.getUChar("BallThreshold", 0);
	
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
	configData[16] = ballThreshold;
	configData[17] = simModeState;
	wsServer.broadcastBIN(configData, sizeof(configData));
}

void clientConnected() {
	buzzer.writeNote(NOTE_C, 5);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	buzzer.writeNote(NOTE_G, 5);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	buzzer.write(0);
}

void clientDisconnected() {
	buzzer.writeNote(NOTE_G, 5);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	buzzer.writeNote(NOTE_C, 5);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	buzzer.write(0);
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
			// CONFIG to READY
			if((payload[0] == 0xAA) && (proboState == PROBO_CONFIG)) {
				proboState = PROBO_READY;
				if(eTaskGetState(PID_Handle) == eSuspended) {
					pid.SetOutputLimits(speed - 255, 255 - speed);
					pid.SetTunings(Kp.floatVal, Ki.floatVal, Kd.floatVal);
					pid.Reset();
					vTaskResume(PID_Handle);
				}
				if(eTaskGetState(Motor_Handle) == eSuspended) {
					vTaskResume(Motor_Handle);
				}
				servo1.write(123);
				servo2.write(57);
			}

			// Go to CONFIG
			if((payload[0] == 0x55) && (proboState != PROBO_CONFIG)) {
				proboState = PROBO_CONFIG;
				vTaskDelay(10 / portTICK_PERIOD_MS);
				vTaskSuspend(PID_Handle);
				vTaskSuspend(Motor_Handle);
				if(eTaskGetState(BallInserting_Handle) != eSuspended) {
					vTaskSuspend(BallInserting_Handle);
				}
				if(simModeState) {
					buzzer.write(0);
					vTaskSuspend(SimMode_Handle);
				}
			}

			// Write CONFIG
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
				ballThreshold = payload[16];
				simModeState = payload[17];

				if(simModeState) {
					vTaskSuspend(LTSensRead_Handle);
				} else {
					vTaskResume(LTSensRead_Handle);
				}

				preferences.putBool("StartPoint", startPoint);
				preferences.putFloat("Kp", Kp.floatVal);
				preferences.putFloat("Ki", Ki.floatVal);
				preferences.putFloat("Kd", Kd.floatVal);
				preferences.putUChar("Speed", speed);
				preferences.putUChar("LTThreshold", LTThreshold);
				preferences.putUChar("BallThreshold", ballThreshold);

				sendConfig();
			}

			// Calibrate LT Sensor HIGH
			if((payload[0] == 0xFF) && (proboState == PROBO_CONFIG)) {
				for(uint8_t i = 0; i < 8; i++) {
					LTSensor_HIGH[i] = LTSensor[i];
				}
			}

			// Calibrate LT Sensor LOW
			if((payload[0] == 0x00) && (proboState == PROBO_CONFIG)) {
				for(uint8_t i = 0; i < 8; i++) {
					LTSensor_LOW[i] = LTSensor[i];
				}
			}

			// Receive Joystick
			if((payload[0] == 0xFF) && (proboState >= PROBO_READY)) {
				leftAxisX = payload[1];
				leftAxisY = payload[2];
				rightAxisX = payload[3];
				rightAxisY = payload[4];
				buttons = (payload[5] << 8);
				buttons |= payload[6];

				if((buttons & 0x0010) && (proboState > PROBO_AUTO_SEARCH_MANUAL || proboState < PROBO_AUTO_SEARCH_BALL)) {
					proboState = PROBO_AUTO_SEARCH_BALL;
					pid.Reset();
					autoIndex = 0;
					proboRightTurnForward = false;
					proboLeftTurnForward = false;
					proboRightTurnReverse = false;
					proboLeftTurnReverse = false;
					proboForward = false;
					proboReverse = false;
					proboStop = false;
					if(simModeState && (eTaskGetState(SimMode_Handle) == eSuspended)) {
						simStartPacketCount = 0;
						vTaskResume(SimMode_Handle);
					}
				} else if(buttons & 0x0020) {
					proboState = PROBO_MANUAL_RUNNING;
					vTaskSuspend(PID_Handle);
					if(simModeState) {
						buzzer.write(0);
						vTaskSuspend(SimMode_Handle);
					}
				} else if((buttons & 0x1000) && (eTaskGetState(BallPicker_Handle) == eSuspended) && (eTaskGetState(ArmPushBall_Handle) == eSuspended) && (proboState == PROBO_MANUAL_RUNNING)) {
					vTaskResume(BallPicker_Handle);
				} else if((buttons & 0x8000) && (eTaskGetState(ArmPushBall_Handle) == eSuspended) && (eTaskGetState(BallPicker_Handle) == eSuspended) && (proboState == PROBO_MANUAL_RUNNING)) {
					vTaskResume(ArmPushBall_Handle);
				} else if((buttons & 0x2000) && (eTaskGetState(BallShooting_Handle) == eSuspended) && (eTaskGetState(BallDropping_Handle) == eSuspended) && (eTaskGetState(RevolverMove90_Handle) == eSuspended) && (proboState == PROBO_MANUAL_RUNNING)) {
					vTaskResume(BallShooting_Handle);
				} else if((buttons & 0x4000) && (eTaskGetState(BallDropping_Handle) == eSuspended) && (eTaskGetState(BallShooting_Handle) == eSuspended) && (eTaskGetState(RevolverMove90_Handle) == eSuspended) && (proboState == PROBO_MANUAL_RUNNING)) {
					vTaskResume(BallDropping_Handle);
				} else if((buttons & 0x0004) && (eTaskGetState(RevolverMove90_Handle) == eSuspended) && (eTaskGetState(BallShooting_Handle) == eSuspended) && (proboState >= PROBO_MANUAL_RUNNING)) {
					digitalWrite(STEPPER_DIR, KIRI);
					vTaskResume(RevolverMove90_Handle);
				} else if((buttons & 0x0008) && (eTaskGetState(RevolverMove90_Handle) == eSuspended) && (eTaskGetState(BallShooting_Handle) == eSuspended) && (proboState >= PROBO_MANUAL_RUNNING)) {
					digitalWrite(STEPPER_DIR, KANAN);
					vTaskResume(RevolverMove90_Handle);
				}
			}

			// Revolver Adjustment
			if((payload[0] == 0x5A) && (proboState == PROBO_CONFIG)) {
				buttons = (payload[1] << 8);
				buttons |= payload[2];

				if(buttons & 0x0004) {
					stepState = !stepState;
					digitalWrite(STEPPER_DIR, KIRI);
					digitalWrite(STEPPER_STEP, stepState);

				} else if(buttons & 0x0008) {
					stepState = !stepState;
					digitalWrite(STEPPER_DIR, KANAN);
					digitalWrite(STEPPER_STEP, stepState);
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
	lastEncR_Time = EncR_Time;
	EncR_Time = micros();
	motR_PPS = 1000000 / (EncR_Time - lastEncR_Time);
	if(digitalRead(ENCR_A)) {
		if(digitalRead(ENCR_B)) {
			EncR_Count--;
			motR_PPS = -motR_PPS;
		} else {
			EncR_Count++;
		}
	} else {
		if(digitalRead(ENCR_B)) {
			EncR_Count++;
		} else {
			EncR_Count--;
			motR_PPS = -motR_PPS;
		}
	}
}

void IRAM_ATTR EncL_ISR() {
	lastEncL_Time = EncL_Time;
	EncL_Time = micros();
	motL_PPS = 1000000 / (EncL_Time - lastEncL_Time);
	if(digitalRead(ENCL_A)) {
		if(digitalRead(ENCL_B)) {
			EncL_Count++;
		} else {
			EncL_Count--;
			motL_PPS = -motL_PPS;
		}
	} else {
		if(digitalRead(ENCL_B)) {
			EncL_Count--;
			motL_PPS = -motL_PPS;
		} else {
			EncL_Count++;
		}
	}
}
#pragma endregion

#pragma region Main Program
void setup() {
	simModeState = false;

	analogReadResolution(8);

	for(uint8_t i = 0; i < 8; i++) {
		LTSensor_LOW[i] = 0;
		LTSensor_HIGH[i] = 255;
	}

	#pragma region Preferences Initialization
	preferences.begin("Config");
	startPoint = preferences.getBool("StartPoint", 0);
	Kp.floatVal = preferences.getFloat("Kp", 0);
	Ki.floatVal = preferences.getFloat("Ki", 0);
	Kd.floatVal = preferences.getFloat("Kd", 0);
	speed = preferences.getUChar("Speed", 0);
	LTThreshold = preferences.getUChar("LTThreshold", 0);
	ballThreshold = preferences.getUChar("BallThreshold", 0);
	#pragma endregion

	#pragma region pinMode
	pinMode(ENCR_A, INPUT_PULLUP);
	pinMode(ENCR_B, INPUT_PULLUP);
	pinMode(ENCL_A, INPUT_PULLUP);
	pinMode(ENCL_B, INPUT_PULLUP);

	pinMode(LT_SENS_ADC, INPUT);
	pinMode(PHOTODIODE_ADC, INPUT);
	
	pinMode(MUX_A0, OUTPUT);
	pinMode(MUX_A1, OUTPUT);
	pinMode(MUX_A2, OUTPUT);
	
	pinMode(BUZZ_PIN, OUTPUT);
	
	pinMode(MOTOR_S_REL, OUTPUT);
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

	servo1.write(123);
	servo2.write(57);
	servo5.write(90);
	#pragma endregion

	#pragma region Buzzer Initialization
	buzzer.attachPin(BUZZ_PIN, 440, 8);
	buzzer.write(0);
	#pragma endregion

	#pragma region Motor Initialization
	rightMotor1.attachPin(MOTOR_R_1, 1000, 8);
	rightMotor2.attachPin(MOTOR_R_2, 1000, 8);
	leftMotor1.attachPin(MOTOR_L_1, 1000, 8);
	leftMotor2.attachPin(MOTOR_L_2, 1000, 8);
	#pragma endregion

	#pragma region PID Initialization
	pid.SetMode(QuickPID::Control::automatic);
	pid.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
	pid.SetSampleTimeUs(50000);
	#pragma endregion

	proboState = PROBO_CONFIG;

	#pragma region Task Creations
	xTaskCreatePinnedToCore(ProboState, "ProboState", 4096, NULL, 1, &ProboState_Handle, 1);
	xTaskCreatePinnedToCore(LTSensRead, "LTSensRead", 2048, NULL, 1, &LTSensRead_Handle, 0);
	xTaskCreatePinnedToCore(Motor, "Motor", 2048, NULL, 1, &Motor_Handle, 1);
	vTaskSuspend(Motor_Handle);
	xTaskCreatePinnedToCore(PID, "PID", 2048, NULL, 2, &PID_Handle, 1);
	vTaskSuspend(PID_Handle);
	xTaskCreatePinnedToCore(RevolverMove90, "RevolverMove90", 2048, NULL, 1, &RevolverMove90_Handle, 1);
	vTaskSuspend(RevolverMove90_Handle);
	xTaskCreatePinnedToCore(BallPicker, "BallPicker", 2048, NULL, 1, &BallPicker_Handle, 1);
	vTaskSuspend(BallPicker_Handle);
	xTaskCreatePinnedToCore(BallShooting, "BallShooting", 2048, NULL, 1, &BallShooting_Handle, 1);
	vTaskSuspend(BallShooting_Handle);
	xTaskCreatePinnedToCore(BallDropping, "BallDropping", 2048, NULL, 1, &BallDropping_Handle, 1);
	vTaskSuspend(BallShooting_Handle);
	xTaskCreatePinnedToCore(ArmPushBall, "ArmPushBall", 2048, NULL, 1, &ArmPushBall_Handle, 1);
	vTaskSuspend(ArmPushBall_Handle);
	xTaskCreatePinnedToCore(BallInserting, "BallInserting", 2048, NULL, 1, &BallInserting_Handle, 1);
	vTaskSuspend(BallInserting_Handle);
	xTaskCreatePinnedToCore(SimMode, "SimMode", 2048, NULL, 1, &SimMode_Handle, 1);
	vTaskSuspend(SimMode_Handle);
	#pragma endregion
}

void loop() {
	wsServer.loop();
	
	ballADC = analogRead(PHOTODIODE_ADC);
	if(ballADC <= ballThreshold) {
		ballColor = WHITE;
	} else if(ballADC > ballThreshold && ballADC < 224) {
		ballColor = ORANGE;
	} else {
		ballColor = INVALID;
	}
	
	currMill = millis();
	if(currMill - prevMill >= sendingInterval) {
		prevMill = currMill;
		wsServer.broadcastBIN(telemetryData, sizeof(telemetryData));

		if(motR_PPS < 100 && motR_PPS > -100) {
			motR_PPS = 0;
		}
		if(motL_PPS < 100 && motL_PPS > -100) {
			motL_PPS = 0;
		}

		packetCount++;
		if(simModeState) {
			simStartPacketCount++;
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
		telemetryData[6] = motR_PPS >> 8;
		telemetryData[7] = motR_PPS;
		telemetryData[8] = motL_PPS >> 8;
		telemetryData[9] = motL_PPS;
		telemetryData[10] = LTSensor[6];
		telemetryData[11] = LTSensor[0];
		telemetryData[12] = LTSensor[4];
		telemetryData[13] = LTSensor[2];
		telemetryData[14] = LTSensor[5];
		telemetryData[15] = LTSensor[7];
		telemetryData[16] = LTSensor[3];
		telemetryData[17] = LTSensor[1];
		telemetryData[18] = ballColor;
		telemetryData[19] = ballADC;
	}
}
#pragma endregion