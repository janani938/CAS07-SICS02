#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SD.h>
#include <SPI.h>
#include <RTClib.h>

#define CROP_TEMP_PIN A0
#define AIR_TEMP_PIN 2
#define RAIN_SENSOR_PIN 3
#define TRIG_PIN 4
#define ECHO_PIN 5
#define TDS_SENSOR_PIN A1
#define PH_SENSOR_PIN A2
#define TURBIDITY_SENSOR_PIN A3
#define DHT_PIN 6
#define SD_CS_PIN 10
#define ERROR_LED_PIN 8

#define DHT_TYPE DHT22
#define RAIN_FACTOR 0.2794
#define CROP_HEIGHT_BASELINE 100

DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(AIR_TEMP_PIN);
DallasTemperature airTempSensor(&oneWire);
RTC_DS3231 rtc;
File dataFile;

volatile int rainTipCount = 0;
unsigned long lastRainTip = 0;
unsigned long lastReadingTime = 0;
unsigned long lastLogTime = 0;
bool sdCardAvailable = false;
float cropHeight = 0;

float cropTemp = 0;
float airTemp = 0;
float cropStress = 0;
float rainfall = 0;
float tdsValue = 0;
float pHValue = 0;
float turbidityValue = 0;
float humidity = 0;
float temperature = 0;

const float CROP_STRESS_THRESHOLD = 5.0;
const float RAINFALL_ALERT = 10.0;
const float TDS_MIN = 300.0;
const float TDS_MAX = 1500.0;
const float PH_MIN = 5.5;
const float PH_MAX = 7.5;
const float HUMIDITY_MIN = 40.0;
const float HUMIDITY_MAX = 80.0;

void readCropStressSensor();
void readRainfallSensor();
void readCropGrowthSensor();
void readWaterQualitySensors();
void readHumiditySensor();
void logData();
void handleErrors(const char* errorSource);
void displayReadings();
float calculateTdsValue(int rawReading);
float calculatePHValue(int rawReading);
float calculateTurbidityValue(int rawReading);


void rainTipInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastRainTip > 100) {
    rainTipCount++;
    lastRainTip = currentTime;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Agricultural Sensor System Initializing...");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT_PULLUP);
  
  dht.begin();
  airTempSensor.begin();
  
  attachInterrupt(digitalPinToInterrupt(RAIN_SENSOR_PIN), rainTipInterrupt, FALLING);
  
  if (!rtc.begin()) {
    handleErrors("RTC initialization failed");
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed!");
    handleErrors("SD card initialization failed");
  } else {
    Serial.println("Done!");
    sdCardAvailable = true;
    
    dataFile = SD.open("AGDATA.CSV", FILE_WRITE);
    if (dataFile) {
      if (dataFile.size() == 0) {
        dataFile.println("Date,Time,CropTemp,AirTemp,CropStress,Rainfall,CropHeight,TDS,pH,Turbidity,Humidity,Temperature");
      }
      dataFile.close();
    } else {
      handleErrors("Could not create data file");
    }
  }
  
  readCropGrowthSensor();
  cropHeight = 0;
  
  Serial.println("System initialization complete!");
  digitalWrite(ERROR_LED_PIN, LOW);
}

void loop() {
  if (millis() - lastReadingTime >= 15000) {
    readCropStressSensor();
    readRainfallSensor();
    readCropGrowthSensor();
    readWaterQualitySensors();
    readHumiditySensor();
    
    displayReadings();
    lastReadingTime = millis();
  }
  
  if (millis() - lastLogTime >= 600000) {
    logData();
    lastLogTime = millis();
  }
  
  checkAlerts();
  
  delay(100);
}

