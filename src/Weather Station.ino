#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP085_U.h>
#include <RtcDS3231.h>
#include <SD.h>
#include <RF24.h>
#include <LiquidCrystal.h>

#define BAUD 9600
#define DHT_PIN
#define DHTTYPE DHT22
#define BMP_PIN
#define RTC_PIN
#define SD_CS
#define SD_SCK
#define SD_MOSI
#define SD_MISO
#define RF_CE
#define RF_CSN
#define RF_PIPE_1
#define RF_PIPE_2


void setup() {
  Serial.begin(BAUD);
}

void loop() {
  Serial.print("Hello World");
}
