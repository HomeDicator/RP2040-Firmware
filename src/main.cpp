#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define SCD4X_SENSOR_READ_INTERVAL 5000  // 5 sec
#define DEBUG 1

SensirionI2CScd4x scd4x;

JsonDocument jsonData;

unsigned long SCD4XLastSensorReadTime = 0;

void send_json_data() {
  String output;
  serializeJson(jsonData, output); // serialize Json
  Serial1.println(output);  // Send to ESP32
  #if DEBUG
    Serial.println("Sent: " + output);  // Debug
  #endif
}

void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("SCD4x: Serial number: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

// PIN19 is power switch for i2c
void sensor_power_on(void) {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

float temperature = 0.0;
float humidity = 0.0;

uint16_t defaultCompenstaionRh = 0x8000;
uint16_t defaultCompenstaionT = 0x6666;

uint16_t compensationRh = defaultCompenstaionRh;
uint16_t compensationT = defaultCompenstaionT;

// ===============================
// Initialize sensor SCD4x
// ===============================
void sensor_scd4x_init(void) {
  delay(1000);
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
      Serial.print("Error trying to execute getSerialNumber(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
}

// ===============================
// Get SCD4x data to json
// ===============================
void sensor_scd4x_get(void) {
  uint16_t error;
  char errorMessage[256];
  // Read Measurement
  uint16_t co2;
  float temperature;
  float humidity;
  bool isDataReady = false;

  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    errorToString(error, errorMessage, 256);
    #if DEBUG
      Serial.print("SCD4x: Error trying to execute getDataReadyFlag(): ");
      Serial.println(errorMessage);
    #endif
    jsonData["scd4x"]["status"] = "fail";
    jsonData["scd4x"]["error_msg"] = String("Error trying to execute readMeasurement(): ") + errorMessage;
    return;
  }
  if (!isDataReady) {
    #if DEBUG
      Serial.print("SCD4x: data not ready");
    #endif
    jsonData["scd4x"]["status"] = "data not ready";
    return;
  }

  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    errorToString(error, errorMessage, 256);
    #if DEBUG
      Serial.print("SCD4x: ");
      Serial.println(errorMessage);
    #endif
    jsonData["scd4x"]["status"] = "fail";
    jsonData["scd4x"]["error_msg"] = String("Error trying to execute readMeasurement(): ") + errorMessage;
  } else if (co2 == 0) {
    jsonData["scd4x"]["status"] = "nan";
    jsonData["scd4x"]["error_msg"] = "Invalid sample detected, skipping.";
  } else {
    jsonData["scd4x"]["status"] = "ok";
    jsonData["scd4x"]["temp"] = temperature;
    jsonData["scd4x"]["humidity"] = humidity;
    jsonData["scd4x"]["co2"] = co2;
  }
}

// ===============================
// Buzzer
// ===============================
#define Buzzer 19  //Buzzer GPIO

void beep_init(void) {
  pinMode(Buzzer, OUTPUT);
}
void beep_off(void) {
  digitalWrite(19, LOW);
}
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}

/************************ recv cmd from esp32  ****************************/

static bool shutdown_flag = false;

void onPacketReceived(const uint8_t *buffer, size_t size) {

#if DEBUG
  Serial.printf("<--- recv len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", buffer[i]);
  }
  Serial.println("");
#endif
}

/************************ setuo & loop ****************************/

int cnt = 0;
int i = 0;

void setup() {
  // USB monitor serial
  Serial.begin(115200);

  // Setup Serial to ESP32-S3
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);

  sensor_power_on();

  // Init i2c bus
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  sensor_scd4x_init();

  beep_init();
  delay(500);
  beep_on();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - SCD4XLastSensorReadTime >= SCD4X_SENSOR_READ_INTERVAL) {
    SCD4XLastSensorReadTime = currentMillis;  // Zeitstempel aktualisieren
    sensor_scd4x_get();
  }

  send_json_data();
  // remove if we have other stuff to run
  delay(1000);
}
