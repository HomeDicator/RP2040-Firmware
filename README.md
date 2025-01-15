# HomeDicator - RP2040 Firmware

This is an early work-in-progress version for an RP2040 firmware for the SenseCAP Indicator and HomeDicator.
The goal is to have bi-directional communicaton over UART to get the sensor data in ESPHome and to do things like sensor calibration on the SCD4x.

While inspired by https://github.com/Seeed-Solution/SenseCAP_Indicator_RP2040, which use PacketSerial, I started from scratch, trying to use Json over UART to the ESP32, hoping to make reading on the ESPHome side easier.

## Working so far
* Read SCD4x co2, temprature and humidty and send over serial.
* Status and error information over serial.

## Planned
* SGP40 reading and send over serial

## Not Planned
* External Sensors like AHT20 - but will be easy to implement that and any other sensor connected to I2C or analog.
