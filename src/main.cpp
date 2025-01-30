#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SensirionI2CSgp40.h>
#include <VOCGasIndexAlgorithm.h>

#define SCD4X_SENSOR_READ_INTERVAL 5000  // 5 sec
#define SGP40_SENSOR_READ_INTERVAL 5000  // 5 sec
#define UART_SEND_INTERVAL 4000  // 4 sec
#define DEBUG 1

SensirionI2CScd4x scd4x;
SensirionI2CSgp40 sgp40;
VOCGasIndexAlgorithm voc_algorithm;

JsonDocument jsonData;

unsigned long lastSendTime = 0;
unsigned long SCD4XLastSensorReadTime = 0;
unsigned long SGP40LastSensorReadTime = 0;

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

void printSCD4xSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("SCD4x S/N: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

void printSGP40SerialNumber(uint16_t* serialNumber, size_t size) {
  Serial.print("SGP40 S/N: ");
  Serial.print("0x");
  for (size_t i = 0; i < size; i++) {
    uint16_t value = serialNumber[i];
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
  }
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
// SCD4x
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
      printSCD4xSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
      Serial.print("Error trying to execute startPeriodicMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
}

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
// SGP40 tvoc
// ===============================
void sensor_sgp40_init(void) {
  uint16_t error;
  char errorMessage[256];

  sgp40.begin(Wire);

  uint16_t serialNumber[3];
  uint8_t serialNumberSize = 3;

  error = sgp40.getSerialNumber(serialNumber, serialNumberSize);

  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    printSGP40SerialNumber(serialNumber, serialNumberSize);
  }

  uint16_t testResult;
  error = sgp40.executeSelfTest(testResult);
  if (error) {
    Serial.print("Error trying to execute executeSelfTest(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (testResult != 0xD400) {
    Serial.print("executeSelfTest failed with error: ");
    Serial.println(testResult);
  }
}

void sensor_sgp40_get(void) {
  uint16_t error;
  char errorMessage[256];
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t srawVoc = 0;

  error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
  if (error) {
    errorToString(error, errorMessage, 256);
    #if DEBUG
      Serial.print("SGP40: Error trying to execute measureRawSignal(): ");
      Serial.println(errorMessage);
    #endif
    jsonData["sgp40"]["status"] = "fail";
    jsonData["sgp40"]["error_msg"] = String("Error trying to execute measureRawSignal(): ") + errorMessage;
  } else {
    jsonData["sgp40"]["status"] = "ok";
    jsonData["sgp40"]["sraw_voc"] = srawVoc;

    int32_t voc_index = voc_algorithm.process(srawVoc);
    jsonData["sgp40"]["voc_index"] = voc_index;
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

  // ESPhome side
  // uart:
  //   id: uart_rp2040
  //   tx_pin: GPIO19  # ESP32 TX<>RP2040 RX (GPIO17)
  //   rx_pin: GPIO20  # ESP32 RX<>RP2040 TX (GPIO16)
  //   baud_rate: 115200  # Same as ESP32

  sensor_power_on();

  // Init i2c bus
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  sensor_scd4x_init();
  sensor_sgp40_init();

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

  if (currentMillis - SGP40LastSensorReadTime >= SCD4X_SENSOR_READ_INTERVAL) {
    SGP40LastSensorReadTime = currentMillis;  // Zeitstempel aktualisieren
    sensor_sgp40_get();
  }

  if (currentMillis - lastSendTime >= UART_SEND_INTERVAL) {
    lastSendTime = currentMillis;
    send_json_data();
  }
}
