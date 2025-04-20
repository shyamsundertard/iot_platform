# Digital IoT Platform for ESP32-based Devices

- Website - https://iot-frontend-tau.vercel.app/dashboard
- Backend - https://github.com/shyamsundertard/iot-backend
- Frontend - https://github.com/shyamsundertard/iot-frontend

### Login credentials
- username - tardshyamsunder
- password - iotpassword

A comprehensive IoT platform that connects ESP32 microcontrollers with a modern web dashboard for real-time monitoring and control of various sensors and actuators.

## Features

### Hardware Features (ESP32)
- **Environmental Monitoring**:
  - DHT11 sensor for temperature and humidity readings
- **Motion Detection**:
  - PIR sensor for motion detection
- **Distance Measurement**:
  - HC-SR04 ultrasonic sensor for object distance measurement
- **Motion Tracking**:
  - MPU6050 accelerometer/gyroscope for motion tracking
- **Device Control**:
  - 4-channel relay module for controlling connected devices
- **Connectivity**:
  - WiFi connectivity for cloud communication
  - Automatic reconnection logic

### Dashboard Features
- **Real-time Monitoring**:
  - Temperature and humidity visualization
  - Motion detection status
  - Distance measurement display
  - Relay status indicators
- **Data Visualization**:
  - Historical charts for temperature and humidity
- **Control Interface**:
  - Manual/Automatic mode switching
  - Direct relay control in manual mode
  - Automatic control logic based on sensor readings
- **Alert System**:
  - Visual alerts for important events
- **Responsive Design**:
  - Works on desktop and mobile devices
  - Modern UI with animated elements

## Hardware Requirements

- ESP32 development board
- DHT11 temperature/humidity sensor
- PIR motion sensor
- HC-SR04 ultrasonic distance sensor
- MPU6050 accelerometer/gyroscope
- 4-channel relay module
- Jumper wires and breadboard (for prototyping)
- 5V power supply

## Software Requirements

### For ESP32
- Arduino IDE or PlatformIO
- Required libraries:
  - WiFi.h
  - HTTPClient.h
  - DHT.h (Adafruit DHT library)
  - ArduinoJson.h
  - Wire.h
  - MPU6050.h

## Installation

### ESP32 Setup

1. Clone this repository or download the ESP32 code
2. Open the code in Arduino IDE or PlatformIO
3. Install the required libraries
4. Update the following constants in the code:
   - `ssid`: Your WiFi network name
   - `password`: Your WiFi password
   - `serverEndpoint`: Your backend server URL
5. Upload the code to your ESP32
