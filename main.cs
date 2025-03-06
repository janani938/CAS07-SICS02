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

void readCropStressSensor() {
  for (int i = 0; i < 3; i++) {
    int cropTempRaw = analogRead(CROP_TEMP_PIN);
    cropTemp = map(cropTempRaw, 0, 1023, 0, 80) / 2.0;
    
    airTempSensor.requestTemperatures();
    airTemp = airTempSensor.getTempCByIndex(0);
    
    cropStress = cropTemp - airTemp;
    
    if (cropTemp > -20 && cropTemp < 60 && airTemp > -20 && airTemp < 60) {
      return;
    }
    
    delay(500);
  }
  
  handleErrors("Crop stress sensor reading failed");
}

void readRainfallSensor() {
  rainfall = rainTipCount * RAIN_FACTOR;
}

void readCropGrowthSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  float distance = duration * 0.034 / 2;
  
  if (distance > 0 && distance < 400) {
    float newHeight = CROP_HEIGHT_BASELINE - distance;
    
    if (abs(newHeight - cropHeight) < 10 || cropHeight == 0) {
      cropHeight = newHeight;
    } else {
      cropHeight = cropHeight * 0.9 + newHeight * 0.1;
    }
  } else {
    handleErrors("Invalid ultrasonic sensor reading");
  }
}

void readWaterQualitySensors() {
  int tdsRaw = analogRead(TDS_SENSOR_PIN);
  tdsValue = calculateTdsValue(tdsRaw);
  
  int pHRaw = analogRead(PH_SENSOR_PIN);
  pHValue = calculatePHValue(pHRaw);
  
  int turbidityRaw = analogRead(TURBIDITY_SENSOR_PIN);
  turbidityValue = calculateTurbidityValue(turbidityRaw);
  
  if (tdsValue < 0 || tdsValue > 5000 || 
      pHValue < 0 || pHValue > 14) {
    handleErrors("Water quality sensor readings out of range");
  }
}

float calculateTdsValue(int rawReading) {
  float voltage = rawReading * 5.0 / 1024.0;
  
  float temperature = 25.0;
  if (airTemp > -20 && airTemp < 60) {
    temperature = airTemp;
  }
  
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  
  float compensatedVoltage = voltage / compensationCoefficient;
  
  return (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage - 
          255.86 * compensatedVoltage * compensatedVoltage + 
          857.39 * compensatedVoltage) * 0.5;
}

float calculatePHValue(int rawReading) {
  float voltage = rawReading * 5.0 / 1024.0;
  
  return 7.0 + ((2.5 - voltage) / 0.18);
}

float calculateTurbidityValue(int rawReading) {
  float voltage = rawReading * 5.0 / 1024.0;
  
  return (voltage - 2.5) * (-1120.4) + 3000;
}

void readHumiditySensor() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  
  if (isnan(humidity) || isnan(temperature)) {
    handleErrors("DHT sensor reading failed");
    return;
  }
  
  if (humidity < 0 || humidity > 100 || 
      temperature < -40 || temperature > 80) {
    handleErrors("Humidity sensor readings out of range");
  }
}

void logData() {
  if (!sdCardAvailable) return;
  
  DateTime now = rtc.now();
  
  dataFile = SD.open("AGDATA.CSV", FILE_WRITE);
  if (dataFile) {
    dataFile.print(now.year());
    dataFile.print('/');
    dataFile.print(now.month());
    dataFile.print('/');
    dataFile.print(now.day());
    dataFile.print(',');
    dataFile.print(now.hour());
    dataFile.print(':');
    dataFile.print(now.minute());
    dataFile.print(':');
    dataFile.print(now.second());
    dataFile.print(',');
    dataFile.print(cropTemp);
    dataFile.print(',');
    dataFile.print(airTemp);
    dataFile.print(',');
    dataFile.print(cropStress);
    dataFile.print(',');
    dataFile.print(rainfall);
    dataFile.print(',');
    dataFile.print(cropHeight);
    dataFile.print(',');
    dataFile.print(tdsValue);
    dataFile.print(',');
    dataFile.print(pHValue);
    dataFile.print(',');
    dataFile.print(turbidityValue);
    dataFile.print(',');
    dataFile.print(humidity);
    dataFile.print(',');
    dataFile.println(temperature);
    
    dataFile.close();
    Serial.println("Data logged successfully");
  } else {
    handleErrors("Failed to open data file for logging");
  }
}

void checkAlerts() {
  bool alertTriggered = false;
  String alertMessage = "ALERT: ";
  
  if (cropStress > CROP_STRESS_THRESHOLD) {
    alertMessage += "High crop stress detected! ";
    alertTriggered = true;
  }
  
  if (rainfall > RAINFALL_ALERT) {
    alertMessage += "Heavy rainfall detected! ";
    alertTriggered = true;
  }
  
  if (tdsValue < TDS_MIN || tdsValue > TDS_MAX) {
    alertMessage += "TDS levels outside optimal range! ";
    alertTriggered = true;
  }
  
  if (pHValue < PH_MIN || pHValue > PH_MAX) {
    alertMessage += "pH levels outside optimal range! ";
    alertTriggered = true;
  }
  
  if (humidity < HUMIDITY_MIN || humidity > HUMIDITY_MAX) {
    alertMessage += "Humidity outside optimal range! ";
    alertTriggered = true;
  }
  
  if (alertTriggered) {
    Serial.println(alertMessage);
    
    if (sdCardAvailable) {
      File alertFile = SD.open("ALERTS.TXT", FILE_WRITE);
      if (alertFile) {
        DateTime now = rtc.now();
        alertFile.print(now.year());
        alertFile.print('/');
        alertFile.print(now.month());
        alertFile.print('/');
        alertFile.print(now.day());
        alertFile.print(' ');
        alertFile.print(now.hour());
        alertFile.print(':');
        alertFile.print(now.minute());
        alertFile.print(" - ");
        alertFile.println(alertMessage);
        alertFile.close();
      }
    }
    
    for (int i = 0; i < 5; i++) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(100);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(100);
    }
  }
}

void handleErrors(const char* errorSource) {
  Serial.print("ERROR: ");
  Serial.println(errorSource);
  
  if (sdCardAvailable) {
    File errorFile = SD.open("ERRORS.TXT", FILE_WRITE);
    if (errorFile) {
      DateTime now = rtc.now();
      errorFile.print(now.year());
      errorFile.print('/');
      errorFile.print(now.month());
      errorFile.print('/');
      errorFile.print(now.day());
      errorFile.print(' ');
      errorFile.print(now.hour());
      errorFile.print(':');
      errorFile.print(now.minute());
      errorFile.print(" - ");
      errorFile.println(errorSource);
      errorFile.close();
    }
  }
  
  digitalWrite(ERROR_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(ERROR_LED_PIN, LOW);
}

void displayReadings() {
  Serial.println("====== SENSOR READINGS ======");
  Serial.print("Time: ");
  DateTime now = rtc.now();
  Serial.print(now.hour());
  Serial.print(':');
  Serial.print(now.minute());
  Serial.print(':');
  Serial.println(now.second());
  
  Serial.print("Crop Temperature: ");
  Serial.print(cropTemp);
  Serial.println(" °C");
  
  Serial.print("Air Temperature: ");
  Serial.print(airTemp);
  Serial.println(" °C");
  
  Serial.print("Crop Stress Index: ");
  Serial.println(cropStress);
  
  Serial.print("Rainfall: ");
  Serial.print(rainfall);
  Serial.println(" mm");
  
  Serial.print("Crop Height: ");
  Serial.print(cropHeight);
  Serial.println(" cm");
  
  Serial.print("Water TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
  
  Serial.print("Water pH: ");
  Serial.println(pHValue);
  
  Serial.print("Water Turbidity: ");
  Serial.print(turbidityValue);
  Serial.println(" NTU");
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.println("=============================");
}