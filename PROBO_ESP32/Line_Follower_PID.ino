#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QTRSensors.h>
#include <EEPROM.h>

#define OK      32
#define BACK    33
#define RIGHT   30
#define LEFT    17
#define UP      11
#define DOWN    21
#define LED     22
#define Buzzer  28
#define Volt    A0

#define motorBPin1  5
#define motorBPin2  6
#define motorAPin1  9
#define motorAPin2  10

const uint8_t SensorCount = 8; 
int addKp=10,addKi=12,addKd=14;
float nilaiP=0.0,nilaiI=0.0,nilaiD=0.0;

int SpeedA,SpeedB;
int L=0,R=0,ok=0,back=0,up=0,down=0,status_home=0,P,I,D,lastError=0,pilih=0,adcvolt;
uint16_t sensorValues[SensorCount], barValues[SensorCount];
boolean cal,go=false,pid=false,bt=false;
float voltage;
char rx;

const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = 80;
const uint8_t basespeedb = 80;

unsigned long previousMillis=0;
const unsigned long samplingTime = 10;
double elapsedTime=0,sumOut=0;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

Adafruit_SSD1306 lcd(128, 64, &Wire, -1);
QTRSensors qtr;

const unsigned char myBitmap [] PROGMEM = {
  // 'Logo_PENS_putih, 105x44px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x81, 0xf0, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf7, 0xfe, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe3, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0f, 0x3c, 0x3e, 0x1e, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x1f, 0x1c, 0xff, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x08, 
  0xff, 0xe7, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x18, 0xff, 0xff, 
  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x1c, 0xff, 0x80, 0xff, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x3e, 0x7e, 0x38, 0x7f, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, 0x7e, 0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xf9, 0xff, 0x80, 0xff, 0x8f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xf1, 0xff, 0xe3, 0xff, 0xc7, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x03, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 
  0xc1, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x7f, 
  0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x3c, 0x3f, 0xe0, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0, 0xff, 0x0f, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 
  0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0xc3, 0xc3, 
  0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0xdf, 0xc3, 0xfe, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0xc0, 0xc3, 0x06, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x7e, 0xc3, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char wifi [] PROGMEM = {
  //wifi
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x18, 0x06, 0x00, 0x20, 
  0x01, 0x00, 0x43, 0xf0, 0x80, 0x0e, 0x1c, 0x00, 0x18, 0x06, 0x00, 0x00, 0xc0, 0x00, 0x03, 0x30, 
  0x00, 0x04, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char signal_bar [] PROGMEM = {
  //Signal Bar 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x88, 0x00, 0x88, 0x00, 0x88, 
  0x04, 0x88, 0x04, 0x88, 0x04, 0x88, 0x44, 0x88, 0x44, 0x88, 0x44, 0x88
};

const unsigned char bluetooth [] PROGMEM = {
  // Bluetooth 13x16px
  0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x02, 0x40, 0x02, 0x10, 0x42, 0x10, 0x12, 0x40, 0x07, 0x00, 
  0x07, 0x00, 0x12, 0x40, 0x42, 0x10, 0x02, 0x10, 0x02, 0x40, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char battery [] PROGMEM = {
  // Battery Bar 18x18px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x60, 
  0x13, 0x40, 0x60, 0x3b, 0xc0, 0xe0, 0x3b, 0xc0, 0xe0, 0x3b, 0xc0, 0xe0, 0x3b, 0xc0, 0xe0, 0x3b, 
  0xc0, 0x60, 0x3b, 0xc0, 0x60, 0x13, 0x40, 0x7f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char robot [] PROGMEM = {
  // Robot 80x44px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 
  0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xc0, 0x07, 0xe0, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xfe, 0x7f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x3f, 0x80, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x7c, 0x3e, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x3e, 0x7c, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0x1f, 0xf8, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3f, 
  0xff, 0xff, 0xfc, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3f, 0x0f, 0xf0, 0xfc, 0x80, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0x3c, 0x03, 0xc0, 0x3c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x38, 0x00, 0x00, 
  0x1c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x38, 0x00, 0x00, 0x1c, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x38, 0x10, 0x08, 0x1c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x30, 0x30, 0x0c, 0x0c, 0x80, 
  0x00, 0x00, 0x00, 0x00, 0x01, 0x30, 0x30, 0x0c, 0x0c, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 
  0x30, 0x0c, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x38, 0x02, 0x40, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x07, 0xe0, 
  0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x03, 0xc0, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0c, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x70, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x80, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x03, 0x80, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Calibration [] PROGMEM = {
  //Calibation 35x35px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 
  0x00, 0xe0, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0x7e, 
  0xef, 0xc0, 0x00, 0x00, 0xf0, 0xe1, 0xe0, 0x00, 0x01, 0xe0, 0xe0, 0xf0, 0x00, 0x03, 0xc3, 0xf8, 
  0x78, 0x00, 0x03, 0x8f, 0xfe, 0x38, 0x00, 0x07, 0x1f, 0xff, 0x1c, 0x00, 0x07, 0x3c, 0xe7, 0x9c, 
  0x00, 0x0e, 0x38, 0x03, 0x8e, 0x00, 0x0e, 0x70, 0x01, 0xce, 0x00, 0x0e, 0x70, 0x01, 0xce, 0x00, 
  0x3f, 0xfc, 0x07, 0xff, 0x80, 0x3f, 0xfc, 0x07, 0xff, 0x80, 0x0e, 0x70, 0x01, 0xce, 0x00, 0x0e, 
  0x70, 0x01, 0xce, 0x00, 0x0e, 0x38, 0x03, 0x8e, 0x00, 0x07, 0x3c, 0xe7, 0x8c, 0x00, 0x07, 0x1e, 
  0xef, 0x1c, 0x00, 0x03, 0x8f, 0xfe, 0x3c, 0x00, 0x03, 0x87, 0xfc, 0x38, 0x00, 0x01, 0xc0, 0xe0, 
  0x70, 0x00, 0x00, 0xf0, 0xe1, 0xe0, 0x00, 0x00, 0x7c, 0xe7, 0xc0, 0x00, 0x00, 0x3f, 0xff, 0x80, 
  0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 
  0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Hardware [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x21, 0xf0, 
  0x00, 0x00, 0x00, 0x73, 0xf9, 0xc0, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x7f, 0xff, 0xc0, 
  0x00, 0x00, 0x7f, 0xff, 0xc0, 0x00, 0x00, 0x3f, 0xff, 0xc0, 0x00, 0x00, 0x7f, 0x1f, 0xc0, 0x00, 
  0x00, 0x7c, 0x07, 0xc0, 0x00, 0x03, 0xfc, 0x07, 0xfc, 0x00, 0x03, 0xfc, 0x07, 0xfc, 0x00, 0x03, 
  0xfc, 0x07, 0xfc, 0x00, 0x01, 0xfc, 0x07, 0xf0, 0x00, 0x00, 0x7e, 0x0f, 0xc0, 0x00, 0x00, 0x7f, 
  0x1f, 0xc0, 0x00, 0x00, 0x7f, 0x0f, 0xc0, 0x00, 0x00, 0x7e, 0x0f, 0xc0, 0x00, 0x00, 0x7c, 0x07, 
  0xc0, 0x00, 0x00, 0xfc, 0x07, 0xe0, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PID_tune [] PROGMEM = {
  0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 
  0x03, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x0f, 
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 
  0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x00, 
  0x00, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x01, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 
  0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x07, 0x87, 0x80, 0x00, 
  0x3c, 0x0f, 0xcf, 0x80, 0x00, 0x3c, 0x19, 0xdc, 0x00, 0x00, 0x3c, 0x01, 0xd8, 0x00, 0x00, 0x3c, 
  0x01, 0xf0, 0x00, 0x00, 0x3c, 0x00, 0xe0, 0x00, 0x00, 0x3c, 0x00, 0xe0, 0x00, 0x00, 0x3c, 0x00, 
  0xe0, 0x00, 0x00, 0x78, 0x01, 0xe0, 0x00, 0x00, 0x78, 0x03, 0xe0, 0x00, 0x00, 0x78, 0x07, 0x70, 
  0x00, 0x00, 0x78, 0x26, 0x73, 0x00, 0x00, 0xf0, 0x3c, 0x7e, 0x00, 0x01, 0xf0, 0x38, 0x3c, 0x00, 
  0x3f, 0xe0, 0x00, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00
};
const unsigned char track [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x0f, 0xf7, 0x80, 
  0x00, 0x00, 0x0f, 0xf7, 0x80, 0x00, 0x00, 0x0f, 0xf3, 0x80, 0x00, 0x00, 0x0f, 0xfb, 0xc0, 0x00, 
  0x00, 0x07, 0xfb, 0xe0, 0x00, 0x00, 0x0f, 0xf3, 0xf0, 0x00, 0x00, 0x1f, 0xe3, 0xfc, 0x00, 0x00, 
  0x3f, 0xe7, 0xfe, 0x00, 0x04, 0xff, 0xc7, 0xff, 0x00, 0x03, 0xff, 0x8f, 0xff, 0x80, 0x0f, 0xff, 
  0x0f, 0xff, 0xc0, 0x3f, 0xfe, 0x1f, 0xff, 0xc0, 0x7f, 0xfe, 0x1f, 0xff, 0xc0, 0x7f, 0xfc, 0x10, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Bluetooth_1 [] PROGMEM = {
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 
  0x7f, 0xff, 0xc0, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x01, 0xff, 
  0x3f, 0xf0, 0x00, 0x01, 0xfe, 0x1f, 0xf0, 0x00, 0x03, 0xfe, 0x07, 0xf8, 0x00, 0x03, 0xfe, 0x03, 
  0xf8, 0x00, 0x03, 0xfe, 0x31, 0xf8, 0x00, 0x03, 0xfe, 0x38, 0xf8, 0x00, 0x03, 0xc6, 0x38, 0x78, 
  0x00, 0x03, 0xc2, 0x30, 0xf8, 0x00, 0x03, 0xf0, 0x21, 0xf8, 0x00, 0x03, 0xf8, 0x07, 0xf8, 0x00, 
  0x03, 0xfc, 0x0f, 0xf8, 0x00, 0x03, 0xfe, 0x1f, 0xf8, 0x00, 0x03, 0xfc, 0x0f, 0xf8, 0x00, 0x03, 
  0xf8, 0x07, 0xf8, 0x00, 0x03, 0xf0, 0x21, 0xf8, 0x00, 0x03, 0xe2, 0x30, 0xf8, 0x00, 0x03, 0xc6, 
  0x38, 0x78, 0x00, 0x03, 0xfe, 0x38, 0xf8, 0x00, 0x03, 0xfe, 0x31, 0xf8, 0x00, 0x03, 0xfe, 0x03, 
  0xf8, 0x00, 0x03, 0xfe, 0x07, 0xf8, 0x00, 0x01, 0xfe, 0x1f, 0xf0, 0x00, 0x01, 0xff, 0x3f, 0xf0, 
  0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x7f, 0xff, 0xc0, 0x00, 
  0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00
};

const unsigned char zebracross [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xce, 0xee, 0x70, 0x00, 0x01, 0xdc, 
  0xe7, 0x70, 0x00, 0x03, 0x9c, 0xe7, 0x38, 0x00, 0x07, 0xbd, 0xf7, 0xbc, 0x00, 0x07, 0x3d, 0xf3, 
  0x9c, 0x00, 0x0f, 0x39, 0xf3, 0x9e, 0x00, 0x1e, 0x79, 0xf3, 0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// '1701019', 40x40px
const unsigned char ARM [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0xf8, 0x3f, 
  0xc0, 0x00, 0x03, 0xfc, 0x71, 0xff, 0xff, 0xff, 0x8e, 0x60, 0xff, 0xff, 0xff, 0x06, 0xe0, 0x60, 
  0x00, 0x06, 0x07, 0xc6, 0x70, 0x00, 0x06, 0x63, 0xc6, 0x78, 0x00, 0x06, 0x63, 0xe0, 0x7c, 0x00, 
  0x0e, 0x07, 0x70, 0xef, 0xff, 0xff, 0x0e, 0x7f, 0xc7, 0xff, 0xff, 0xfe, 0x1f, 0x83, 0x80, 0x01, 
  0xfc, 0x07, 0x81, 0xc0, 0x01, 0x9c, 0x01, 0xc0, 0xe0, 0x01, 0x9c, 0x00, 0xe0, 0x7c, 0x01, 0x9c, 
  0x00, 0x70, 0xff, 0x01, 0xfc, 0x00, 0x39, 0xe7, 0x83, 0xfc, 0x00, 0x1f, 0x81, 0x87, 0x0e, 0x00, 
  0x0f, 0x81, 0xc7, 0x07, 0x00, 0x07, 0x99, 0xc6, 0x03, 0x00, 0x03, 0x99, 0xc6, 0x03, 0x00, 0x03, 
  0x81, 0xc6, 0x03, 0x00, 0x01, 0xc3, 0x86, 0x03, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x01, 0xff, 
  0x80, 0x00, 0x00, 0x01, 0x81, 0x80, 0x00, 0x00, 0x01, 0x81, 0x80, 0x00, 0x00, 0x01, 0x81, 0xc0, 
  0x00, 0x00, 0x03, 0x81, 0xc0, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00, 
  0x00, 0x0e, 0x00, 0x70, 0x00, 0x00, 0x1c, 0x00, 0x38, 0x00, 0x00, 0x1c, 0x00, 0x38, 0x00, 0x00, 
  0x1c, 0x00, 0x38, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char Fan [] PROGMEM = {
  0x00, 0x00, 0x1c, 0x00, 0x00, 0x03, 0xe0, 0x03, 0xe7, 0xc0, 0x0f, 0xf8, 0x00, 0xff, 0xf0, 0x1f, 
  0xfc, 0x00, 0x3f, 0xf8, 0x3f, 0xfe, 0x00, 0x7f, 0xfc, 0x3f, 0xfe, 0x1f, 0xff, 0xfc, 0x7f, 0xff, 
  0x01, 0xff, 0xfe, 0x7f, 0xff, 0x80, 0xff, 0xfe, 0x7f, 0xff, 0x80, 0xff, 0xfe, 0x7f, 0xff, 0xc1, 
  0xff, 0xfe, 0x7f, 0xff, 0xc1, 0xff, 0xfe, 0x3f, 0xff, 0xc1, 0xff, 0xfc, 0x3f, 0xff, 0x81, 0xff, 
  0xfc, 0x7f, 0xfe, 0x3c, 0x7f, 0xf8, 0x6f, 0xfc, 0xff, 0x3f, 0xf0, 0x67, 0xf9, 0xff, 0x9f, 0xc0, 
  0x46, 0x7b, 0xff, 0xdf, 0x80, 0x44, 0x03, 0xff, 0xce, 0x00, 0x84, 0x07, 0xe3, 0xe8, 0x00, 0x84, 
  0x07, 0xc3, 0xe0, 0x00, 0x04, 0x07, 0xc3, 0xe0, 0x21, 0x00, 0x07, 0xc3, 0xe0, 0x21, 0x00, 0x73, 
  0xff, 0xc0, 0x23, 0x00, 0xfb, 0xff, 0xde, 0x62, 0x03, 0xf9, 0xff, 0x9f, 0xe2, 0x0f, 0xfc, 0xff, 
  0x3f, 0xf6, 0x1f, 0xfe, 0x3c, 0x7f, 0xfe, 0x3f, 0xff, 0x81, 0xff, 0xfc, 0x3f, 0xff, 0x87, 0xff, 
  0xfc, 0x7f, 0xff, 0x83, 0xff, 0xfe, 0x7f, 0xff, 0x83, 0xff, 0xfe, 0x7f, 0xff, 0x01, 0xff, 0xfe, 
  0x7f, 0xff, 0x00, 0xff, 0xfe, 0x7f, 0xff, 0x80, 0xff, 0xfe, 0x3f, 0xff, 0xf0, 0x7f, 0xfc, 0x3f, 
  0xfe, 0x00, 0x7f, 0xfc, 0x1f, 0xfc, 0x00, 0x3f, 0xf8, 0x0f, 0xff, 0x00, 0x1f, 0xf0, 0x03, 0xe7, 
  0xc0, 0x07, 0xc0, 0x00, 0x00, 0x78, 0x00, 0x00
};



void setup(){
  Serial1.begin(9600);
  Serial.begin(9600);   //Serial Monitor
  pinMode(UP,INPUT_PULLUP);
  pinMode(DOWN,INPUT_PULLUP);
  pinMode(LEFT,INPUT_PULLUP);
  pinMode(RIGHT,INPUT_PULLUP);
  pinMode(OK,INPUT_PULLUP);
  pinMode(BACK,INPUT_PULLUP);
  pinMode(Buzzer,OUTPUT);
  pinMode(LED,OUTPUT);
  pinMode(Volt,INPUT);
  
  pinMode(motorAPin1,OUTPUT);
  pinMode(motorAPin2,OUTPUT);
  pinMode(motorBPin1,OUTPUT);
  pinMode(motorBPin2,OUTPUT);

        nilaiP = EEPROM.read(addKp);
        nilaiI = EEPROM.read(addKi);
        nilaiD = EEPROM.read(addKd);
        
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  lcd.clearDisplay();
  lcd.drawBitmap(11.5,15,myBitmap,105,44,WHITE);
  lcd.display();

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5, A6, A7, A8}, SensorCount);

  for(int i=16;i<=20;i++){
  tone(Buzzer, i*1000, 200);
  delay(200);
  }
  noTone(Buzzer);

    for (int i = 14; i <= 100; i++)
    {
     if(i%5==0) digitalWrite(Buzzer,1),digitalWrite(LED,1);
     else digitalWrite(Buzzer,0),digitalWrite(LED,0);
     if(i==90) digitalWrite(Buzzer,1),delay(500);
     else if(i>100) digitalWrite(Buzzer,0);
     lcd.clearDisplay();
     lcd.drawRect(12, 25, 104, 10, WHITE);
     lcd.fillRect(14, 28, i, 4, WHITE);
     lcd.setTextSize(1.3);
     lcd.setTextColor(WHITE);
     lcd.setCursor(30, 45);
     lcd.print("WAITING ");
     lcd.print(i);
     lcd.print("%");
     lcd.display();
    }
}


void loop (){
while (go == true) //robot jalan maju dimulai
  {
            uint16_t position = qtr.readLineBlack(sensorValues);
            PID_Control();
            if(!digitalRead(BACK)) go=false,follow(0,0);
  }

while (pid == true) //geser menu
  {  
    if(!digitalRead(UP) | !digitalRead(DOWN) | !digitalRead(LEFT) | !digitalRead(RIGHT) ) digitalWrite(Buzzer,1);
    else digitalWrite(Buzzer,0);
    
    if(!digitalRead(DOWN)){
      delay(100);
      pilih++;
      if(pilih > 3) pilih=3;
    }
    if(!digitalRead(UP)){
      delay(100);
      pilih--;
      if(pilih < 1) pilih=1;
    }

    if(pilih==1 && !digitalRead(RIGHT)) Kp++;
    else if(pilih==1 && !digitalRead(LEFT)) Kp--;
    if(pilih==2 && !digitalRead(RIGHT)) Ki++;
    else if(pilih==2 && !digitalRead(LEFT)) Ki--;
    if(pilih==3 && !digitalRead(RIGHT)) Kd++;
    else if(pilih==3 && !digitalRead(LEFT)) Kd--;
    
        delay(100);
        
        if(!digitalRead(OK)){   //tulis nilai pada EEPROM
            EEPROM.update(addKp,abs(nilaiP));
            EEPROM.update(addKi,abs(nilaiI));
            EEPROM.update(addKd,abs(nilaiD)); 
        }
        
            lcd.clearDisplay();
            lcd.setTextSize(1.2);
            lcd.setTextColor(WHITE);
            lcd.setCursor(30,20);
            lcd.print("P :"); 
            lcd.setCursor(55,20);
            lcd.print(Kp); 
            lcd.setCursor(30,30);
            lcd.print("I :");
            lcd.setCursor(55,30);
            lcd.print(Ki);  
            lcd.setCursor(30,40);
            lcd.print("D :");
            lcd.setCursor(55,40);
            lcd.print(Kd);  
            lcd.display(); 
  
            if(!digitalRead(BACK)) pid=false;
  }

  while (bt == true)
  {
            lcd.clearDisplay();
            lcd.setTextSize(2);
            lcd.setTextColor(WHITE);
            lcd.setCursor(25,30);
            lcd.print("MODE BT"); 
            lcd.display();
//            HC05();      
            if(!digitalRead(BACK)) bt=false;
  }


 if(!digitalRead(UP) || !digitalRead(DOWN) ||!digitalRead(LEFT) ||!digitalRead(RIGHT) ||!digitalRead(OK) ||!digitalRead(BACK)) digitalWrite(Buzzer,1);
 else digitalWrite(Buzzer,0);
            
 if(status_home==0){

     Home();
     if(!digitalRead(LEFT)) L=1,R=0;
     if(!digitalRead(RIGHT))R=1,L=0;
 
     if(L==1){
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE);
            lcd.setCursor(18,55);
            lcd.print("<-");  
            lcd.display(); 
            if(!digitalRead(OK))  ok=1,back=0,go=true;
            if(ok==1)             status_home=2;
          }
  
     if(R==1){
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE);
            lcd.setCursor(90,55);
            lcd.print("->");  
            lcd.display(); 
            if(!digitalRead(OK))  ok=1,back=0;
            if(ok==1)             status_home=1;
          }
  }
  
      if(!digitalRead(BACK))       status_home=0,ok=0,back=1,up=0,pilih=0;
      
      else if(status_home==1){  
            if(!digitalRead(UP)){
              up++;
              delay(150);
              if(up>=7) up=7;
            }
            else if(!digitalRead(DOWN)){
              up--;
              delay(150);
              if(up<=0) up=0;
            }
            switch(up){
              case 0:
                lcd.clearDisplay();
                lcd.drawBitmap(0,16.5,Calibration,35,35,WHITE);
                lcd.setCursor(45,30);
                lcd.setTextSize(1.8);
                lcd.setTextColor(WHITE);
                lcd.print("CALIBRATE");
                lcd.display();
                delay(300);
                if(!digitalRead(OK)){
                  lcd.clearDisplay();
                  lcd.setCursor(45,30);
                  lcd.setTextSize(1.8);
                  lcd.setTextColor(WHITE);
                  lcd.print("LOADING");
                  lcd.display();
                  for (uint16_t i = 0; i < 200; i++)
                      {
                        qtr.calibrate();
                        if(i%5==0) digitalWrite(LED,1);
                        else digitalWrite(LED,0);
                      }
                        
                        up=8;
                  }   
                  
              break;

//              case 1:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,Hardware,35,35,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("HARDWARE TEST");
//                lcd.display();
//              break;

//              case 2:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,PID_tune,35,35,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("PID TUNE");
//                lcd.display();
//                if(!digitalRead(OK)) pid=true;
//                
//              break;

//              case 3:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,track,35,35,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("PATH PLANING");
//                lcd.display();
//              break;

//              case 4:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,10,zebracross,35,35,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("LINE COLOR");
//                lcd.display();
//              break;

//              case 5:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,Bluetooth_1,35,35,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("BLUETOOTH");
//                lcd.display();
//                if(!digitalRead(OK)) bt=true;
//              break;

//              case 6:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,ARM,40,40,WHITE);
//                lcd.setCursor(48,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("GRIPPER");
//                lcd.display();
//              break;

//              case 7:
//                lcd.clearDisplay();
//                lcd.drawBitmap(0,16.5,Fan,40,40,WHITE);
//                lcd.setCursor(45,30);
//                lcd.setTextSize(1.8);
//                lcd.setTextColor(WHITE);
//                lcd.print("FIRE FIGHTING");
//                lcd.display();
//              break;

              case 8:
                 uint16_t position = qtr.readLineBlack(sensorValues);
                 for(uint8_t i = 0; i < SensorCount; i++)
                      {
                        lcd.clearDisplay();
                        lcd.setCursor(20,15);
                        lcd.print(sensorValues[7]);
                        lcd.print(' ');
                        lcd.print(sensorValues[6]);
                        lcd.print(' ');
                        lcd.print(sensorValues[5]);
                        lcd.print(' ');
                        lcd.print(sensorValues[4]);
                        lcd.setCursor(20,35);
                        lcd.print(sensorValues[3]);
                        lcd.print(' ');
                        lcd.print(sensorValues[2]);
                        lcd.print(' ');
                        lcd.print(sensorValues[1]);
                        lcd.print(' ');
                        lcd.print(sensorValues[0]);
                        lcd.setCursor(55,55);
                        lcd.print(position);
                        lcd.display();
                      }
              if(!digitalRead(BACK)) up=0;
              break; 

            }
    }

}

void Voltage(){
  adcvolt = analogRead(Volt);
  voltage = (adcvolt * 8.2)/756.0;
  //Serial.println(voltage); 
}

void Home(){
  lcd.clearDisplay();
  lcd.drawBitmap(18,0,wifi,20,20,WHITE);
  lcd.drawBitmap(0,0,signal_bar,14,14,WHITE);
  lcd.drawBitmap(37,0,bluetooth,13,16,WHITE);
  lcd.drawBitmap(110,0,battery,18,18,WHITE);
//  lcd.drawBitmap(24,25,robot,80,44,WHITE);

  qtr.readLineBlack(sensorValues);
  for(uint8_t i = 0; i < SensorCount; i++){
      if(sensorValues[i] > 310) barValues[i]=1;
      if(sensorValues[i] < 250)barValues[i]=0;
      lcd.setCursor(20,35);
      lcd.print(barValues[7]);
      lcd.print(' ');
      lcd.print(barValues[6]);
      lcd.print(' ');
      lcd.print(barValues[5]);
      lcd.print(' ');
      lcd.print(barValues[4]);
      lcd.print(' ');
      lcd.print(barValues[3]); 
      lcd.print(' ');
      lcd.print(barValues[2]);
      lcd.print(' ');
      lcd.print(barValues[1]);
      lcd.print(' ');
      lcd.print(barValues[0]);
      lcd.print(' ');
  }
  Voltage();  
  lcd.setTextSize(1);
  lcd.setTextColor(WHITE);
  lcd.setCursor(0,55);
  lcd.print("RUNNING!");
  lcd.setCursor(107,55);
  lcd.print("SET");
  lcd.setCursor(90,5);
  lcd.print(voltage,1);
//  lcd.setCursor(25,55);
//  lcd.print(Kp);
//  lcd.setCursor(50,55);
//  lcd.print(Ki);
//  lcd.setCursor(65,55);
//  lcd.print(Kd);
  lcd.display();

}

void PID_Control(){
  Ki=Ki/10;
  Kd=Kd/10;
  long currentMillis = millis();
  elapsedTime=(currentMillis - previousMillis)/1000;
  
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = 3500 - position;
    
      P  = error * Kp;
      D  = ((error - lastError)*Kd) / elapsedTime;
      if (position & 3500){
        sumOut = 0;
        I=0;
      }else {
        sumOut +=error*elapsedTime;
        I=sumOut * Ki;
      }
      double rateError = error - lastError;
      lastError = error;
      
      int motorspeed = P+I+D;
      int motorspeeda = basespeeda + motorspeed;
      int motorspeedb = basespeedb - motorspeed;
    
        if (motorspeeda > 255) motorspeeda = 255;
        if (motorspeedb > 255) motorspeedb = 255;
        if (motorspeeda < -255) motorspeeda = -255;
        if (motorspeedb < -255) motorspeedb = -255;
    
     follow(motorspeeda, motorspeedb);
}

void follow(int posa, int posb) {
      if (posa > 0)
      {
        analogWrite(motorAPin2, 0);
        analogWrite(motorAPin1, posa);
      }
      else
      {
        analogWrite(motorAPin1, 0);
        analogWrite(motorAPin2, posa);
        posa = 100 + posa;
      }

      if (posb > 0)
      {
        analogWrite(motorBPin2, 0);
        analogWrite(motorBPin1, posb);
      }
      else
      {
        analogWrite(motorBPin1, 0);
        analogWrite(motorBPin2, posb);
        posb = 100 + posb;
      }        
}
//void HC05(){
//  noTone(Buzzer);
//  if(Serial1.available()) rx=Serial1.read();
//              if(rx == 'F') {
//                  analogWrite(motorAPin1, SpeedA);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, SpeedB);
//                  analogWrite(motorBPin2, 0);       //maju
//              }
//              else if(rx == 'B'){
//                  analogWrite(motorAPin1, 0);
//                  analogWrite(motorAPin2, SpeedA);
//                  analogWrite(motorBPin1, 0);
//                  analogWrite(motorBPin2, SpeedB);  //mundur
//              }
//              else if(rx == 'L'){
//                  analogWrite(motorAPin1, 0);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, SpeedB/2);
//                  analogWrite(motorBPin2, 0);       //kiri
//              }
//              else if(rx == 'R'){
//                  analogWrite(motorAPin1, SpeedA/2);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, 0);
//                  analogWrite(motorBPin2, 0);       //kanan
//              }
//              else if(rx == 'G'){
//                  analogWrite(motorAPin1, SpeedA/2);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, SpeedB);
//                  analogWrite(motorBPin2, 0);       //Majukiri
//              }
//              else if(rx == 'I'){
//                  analogWrite(motorAPin1, SpeedA);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, SpeedB/2);
//                  analogWrite(motorBPin2, 0);       //Majukanan
//              }
//              else if(rx == 'H'){
//                  analogWrite(motorAPin1, 0);
//                  analogWrite(motorAPin2, SpeedA/2);
//                  analogWrite(motorBPin1, 0);
//                  analogWrite(motorBPin2, SpeedB);  //Mundurkiri
//              }
//              else if(rx == 'J'){
//                  analogWrite(motorAPin1, 0);
//                  analogWrite(motorAPin2, SpeedA);
//                  analogWrite(motorBPin1, 0);
//                  analogWrite(motorBPin2, SpeedB/2);    //Mundurkanan
//              }
//              else if(rx == 'S'){
//                  analogWrite(motorAPin1, 0);
//                  analogWrite(motorAPin2, 0);
//                  analogWrite(motorBPin1, 0);
//                  analogWrite(motorBPin2, 0);         //Stop
//              }
//              else if(rx == 'W') digitalWrite(LED,1);       //LampuON
//              else if(rx == 'w') digitalWrite(LED,0);       //LampuOFF
//              else if(rx == 'V') digitalWrite(Buzzer,1);    //BuzON
//              else if(rx == 'v') digitalWrite(Buzzer,0);    //BuzOFF
              
//              else if(rx == 'X') //HazardON
//              else if(rx == 'x') //HazardOFF

              
//              else if(rx == '1') SpeedA=20,SpeedB=20;     //Speed 1
//              else if(rx == '2') SpeedA=30,SpeedB=30;     //Speed 2
//              else if(rx == '3') SpeedA=50,SpeedB=50;     //Speed 3
//              else if(rx == '4') SpeedA=70,SpeedB=70;     //Speed 4
//              else if(rx == '5') SpeedA=100,SpeedB=100;   //Speed 5
//              else if(rx == '6') SpeedA=120,SpeedB=120;   //Speed 6
//              else if(rx == '7') SpeedA=170,SpeedB=170;   //Speed 7
//              else if(rx == '8') SpeedA=190,SpeedB=190;   //Speed 8
//              else if(rx == '9') SpeedA=230,SpeedB=230;   //Speed 9
//              else if(rx == 'q') SpeedA=255,SpeedB=255;   //Speed 10
//}
