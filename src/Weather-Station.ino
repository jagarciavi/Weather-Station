#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <SFE_BMP180.h>
#include <RtcDS3231.h>
#include <SD.h>
#include <RF24.h>
#include <LiquidCrystal.h>

#define CONNECTION_TYPE 0 // 0 is USB + SD, 1 is NRF24L01 + SD
#define BAUD 9600
#define DHT_ENABLE true
#define DHTPIN 2
#define DHTTYPE DHT22
#define SD_CS 4
#define SD_SCK
#define SD_MOSI
#define SD_MISO
#define RF_CE 7
#define RF_CSN 8
#define RF_PIPE_1
#define RF_PIPE_2
#define LCD_RS 8
#define LCD_ENABLE 9
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7

#define count(x) sizeof(x)/sizeof(unsigned long)
#define countof(a) (sizeof(a) / sizeof(a[0]))

#define BMP180T 0   // Temperature
#define BMP180P 1   // Pressure
#define DHT22T  2   // Temperature
#define DHT22H  3   // Humidity
#define DHT22HI 4   // Heat Index
#define LDR     5   // Light sensor (analogical)
#define DS3231  6   // Real time clock

#define NOTUSED -1  // For BMP180/DHT22/BH1750 sensor.
#define groundPin A0
#define ALTITUDE 826

// TODO: Create if with #define client o server
DHT dht(DHTPIN, DHTTYPE);
SFE_BMP180 pressure;
RtcDS3231 tc;
Sd2Card card;
SdVolume volume;
SdFile root;
RF24 radio(RF_CE, RF_CSN);
RtcDS3231 Rtc;
LiquidCrystal lcd (LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

int types[] = {BMP180T,BMP180P,DHT22T,DHT22H,DHT22HI,LDR,DS3231};
unsigned long lastUpdate[] = {0,0,0,0,0,0,0};
unsigned long updateTimes[] = {30000,30000,30000,30000,30000,30000,30000};
int pins[] = {NOTUSED, NOTUSED, 2, 2, NOTUSED, 15, NOTUSED};

int secureAnalogRead(int pin) {
  delay(0.01);
  analogRead(groundPin);
  delay(0.01);
  return analogRead(pin);
}

boolean bmp180Temperature(double &T) {
  char status;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if(status != 0) return true;
  }
  return false;
}

boolean bmp180Pressure(double &P) {
  double T, absP;
  char status;
  if(bmp180Temperature(T)) {
    status = pressure.startPressure(3);
    if (status != 0) {
        delay(status);
        status = pressure.getPressure(absP,T);
        if (status != 0) {
          P = pressure.sealevel(absP,ALTITUDE);
          return true;
        }
      }
  }
  return false;
}

int ldr(int pin) {
  return map(secureAnalogRead(pin), 0, 1023, 0, 100);
}

#if CONNECTION_TYPE == 0

void sendCommand(String c, String v) {
  RtcDateTime now = Rtc.GetDateTime();
  Serial.println(printDateTime(now) + "_" + c + "_" + v);
}

#elif CONNECTION_TYPE == 1

void sendCommand(String c, String v) {
  String data = c + "_" + v;
  const char* datachar = data.c_str();

  radio.powerUp();
  radio.write(datachar, strlen(datachar));
  radio.powerDown();
}
#endif

void update(int type, int pin) {
  String c, v;
  switch (type) {
    case BMP180T:
      {
        double t;
        c = "BMP180T";
        if (bmp180Temperature(t)) v = String(t);
        else v = c + " Error";
        break;
      }

    case BMP180P:
      {
        double p;
        c = "BMP180P";
        if (bmp180Pressure(p)) v = String(p);
        else v = c + " Error";
        break;
      }

    case DHT22T:
      {
        float t = dht.readTemperature();    // Default temperature unit is ºC.
        c = "DHT22T";
        v = String(t);
        break;
      }

    case DHT22H:
      {
        float h = dht.readHumidity();
        c = "DHT22H";
        v = String(h);
        break;
      }
    case DHT22HI:
      {
        float h, t, hi;
        h = dht.readHumidity();
        t = dht.readTemperature(true);    // We need value in farenheit.
        hi = dht.computeHeatIndex(t,h);
        c = "DHT22HI";
        v = String(dht.convertFtoC(hi));
        break;
      }
    case LDR:
      {
        c = "LDR";
        //v = secureAnalogRead(pin);
        v = ldr(pin);
        break;
      }
    case DS3231:
      {
        c = "DS3231";
        v = String(Rtc.GetTemperature().AsFloat());
      }
  }
  sendCommand(c, v);
}

String printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u_%02u:%02u:%02u "),
            dt.Day(),
            dt.Month(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );

    return String(datestring);
}

void updateAll() {
  for (int i = 0; i < count(updateTimes); i++) {
    unsigned long currentTime = millis();
    if (currentTime < lastUpdate[i]) {
      lastUpdate[i] = 0;
    }
    else if (currentTime - lastUpdate[i]  > updateTimes[i]) {
      update(types[i], pins[i]);
      lastUpdate[i] += updateTimes[i];
    }
  }
}

// TODO: Add SD write
void setup() {
  #if CONNECTION_TYPE == 0
    Serial.begin(BAUD,SERIAL_8E1);
  #elif CONNECTION_TYPE == 1
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS) ;
    radio.enableDynamicPayloads();
    radio.setRetries(5,15);
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.stopListening();
  #endif

  pressure.begin();
  dht.begin();
  Rtc.Begin();

  for (int i = 0; i < count(updateTimes); i++) {
    update(types[i],pins[i]);
  }
}

void loop() {
  updateAll();
}
