#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>

// DHT sensor GPIO pin
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Relay GPIO pins
#define RELAY_1 26
#define RELAY_2 2
#define RELAY_3 27
#define RELAY_4 23

// PIR sensor pin
#define PIR_PIN 4

// HC-SR04 Pins
#define TRIG_PIN 33
#define ECHO_PIN 35

// // MPU-9250 configuration
#define MPU_ADDR 0x68
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Timing variables
unsigned long lastSensorUpdate = 0;
unsigned long lastPIRUpdate = 0;
unsigned long lastMPUCheck = 0;
unsigned long lastUltrasonicUpdate = 0;
const unsigned long DHT_INTERVAL = 30000;
const unsigned long PIR_INTERVAL = 100;
const unsigned long MPU_INTERVAL = 20000;
const unsigned long ULTRASONIC_INTERVAL = 100;

// Wifi credentials
const char *ssid = "wifi_ssid";
const char *password = "wifi_password";

// Server base endpoint
const char *serverEndpoint = "server_url";

void sendToServer(float temperature, float humidity);
void sendPIRStatus(bool motionDetected);
void fetchRelayStates();
void sendUltrasonicData(float distance);
float readDHTTemperature();
float readDHTHumidity();
float readDistance();
float readDistance();


void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  // Initialize the DHT sensor
  dht.begin();

  // Initialize MPU sensor
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Connect to WiFi (STA mode)
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  // if WiFi not connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // When connects to WiFi
  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);
  digitalWrite(RELAY_4, LOW);

  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  fetchRelayStates();

  float temperature = readDHTTemperature();
  float humidity = readDHTHumidity();

  // Distance from UAV sensor
  float distance = readDistance();

  // Check if the readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Print the temperature and humidity readings to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Distance: "); 
  Serial.print(distance); 
  Serial.println(" cm");

  Serial.print("accelX: "); 
  Serial.print(accelX); 
  Serial.println(" m/s^2");

  Serial.print("accelY: "); 
  Serial.print(accelY); 
  Serial.println(" m/s^2");

  Serial.print("accelZ: "); 
  Serial.print(accelZ); 
  Serial.println(" m/s^2");

  
  // Sending data (of multiple sensors/devices connected to ESP32) to server
  
  // Send DHT sensor data every 10 seconds
  if (currentMillis - lastSensorUpdate >= DHT_INTERVAL) {
    sendDHTData(temperature, humidity);
    lastSensorUpdate = currentMillis;
  }

  // Send PIR sensor data every 1 second
  if (currentMillis - lastPIRUpdate >= PIR_INTERVAL) {
    bool motionDetected = digitalRead(PIR_PIN) == HIGH;
    sendPIRStatus(motionDetected);
    lastPIRUpdate = currentMillis;
  }

  // Check for relay updates every 1 second
  // if (currentMillis - lastRelayCheck >= RELAY_INTERVAL) {
  //   fetchRelayStates();
  //   lastRelayCheck = currentMillis;
  // }

  if (currentMillis - lastUltrasonicUpdate >= ULTRASONIC_INTERVAL) {
    sendUltrasonicData(distance);
    lastUltrasonicUpdate = currentMillis;
  }

  // Send MPU sensor data every 5 seconds
  if (currentMillis - lastMPUCheck >= MPU_INTERVAL) {
    sendMPUData();
    lastMPUCheck = currentMillis;
  }

  // delay(2000);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034 / 2);
  
  // If the reading exceeds a valid range (for example, above 50 cm), consider it invalid.
  if(distance > 50) {
    return -1;
  }
  return distance;
}

// Function to read temperature from DHT sensor
float readDHTTemperature() {
  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  return t;
}
// Function to read humidity from DHT sensor
float readDHTHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return -1;
  }
  return h;
}


// Function to send Ultrasonic sensor data to the server
void sendUltrasonicData(float distance) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    StaticJsonDocument<100> doc;
    doc["distance"] = distance;
    String jsonData;
    serializeJson(doc, jsonData);

    http.begin(String(serverEndpoint) + "/ultrasonic");
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.POST(jsonData);
    if (httpResponseCode > 0) {
        Serial.println("Ultrasonic sensor distance sent successfully");
    } else {
        Serial.println("failed to send Ultrasonic sensor distance. Retrying...");
        delay(1000);
        httpResponseCode = http.POST(jsonData);
    }

    http.end();
  }
}


// Function to send DHT sensor data to the server
void sendDHTData(float temperature, float humidity) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        StaticJsonDocument<100> doc;
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;
        String jsonData;
        serializeJson(doc, jsonData);

        String url = String(serverEndpoint) + "/dht";
        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        int httpResponseCode = http.POST(jsonData);
        if (httpResponseCode > 0) {
            Serial.println("DHT data sent successfully");
        } else {
            Serial.println("DHT data send failed. Retrying...");
            delay(1000);
            httpResponseCode = http.POST(jsonData);
        }

        http.end();
    } else {
    Serial.println("WiFi connection lost...");
    // Trying to reconnect to WiFi
    WiFi.begin(ssid, password);
    }
}


// Function to send PIR sensor data to the server
void sendPIRStatus(bool motionDetected) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Create a JSON string with the data
    String jsonData = "{";
    jsonData += "\"motionDetected\": " + String(motionDetected ? "true" : "false");
    jsonData += "}";

    // Construct the full URL for the PIR endpoint
    String url = String(serverEndpoint) + "/pir";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Server Response:");
      Serial.println(response);
    } else {
      Serial.print("Error sending PIR data: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi connection lost...");
    // Trying to reconnect to WiFi
    WiFi.begin(ssid, password);
  }
}

// Function to fetch relay states from the server
void fetchRelayStates() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Construct the full URL for the relays endpoint
    http.begin(String(serverEndpoint) + "/relays");
    http.addHeader("Connection", "keep-alive");

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
      String response = http.getString();
      // Serial.println("Server Response:");
      // Serial.println(response);

      // Parse JSON response
      StaticJsonDocument<64> jsonDoc;
      DeserializationError error = deserializeJson(jsonDoc, response);

      // if (!error) {
        digitalWrite(RELAY_1, jsonDoc["relay1"] ? HIGH : LOW);
        digitalWrite(RELAY_2, jsonDoc["relay2"] ? HIGH : LOW);
        digitalWrite(RELAY_3, jsonDoc["relay3"] ? HIGH : LOW);
        digitalWrite(RELAY_4, jsonDoc["relay4"] ? HIGH : LOW);

        // Serial.println("Relay states updated successfully");
      // } else {
      //   Serial.println("JSON parsing failed");
      // }
    } else {
      Serial.println("Error fetching relay states");
      Serial.println("Error code: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("WiFi connection lost...");
    // Trying to reconnect to WiFi
    WiFi.begin(ssid, password);
  }
}

// Function to read MPU sensor data
void readMPUData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read();
  Wire.read();
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
  gyroX = rawGyroX / 131.0;
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;
}

// // Function to send MPU sensor data to server
void sendMPUData() {
  if (WiFi.status() == WL_CONNECTED) {
    readMPUData();
    HTTPClient http;

    // Construct full url of MPU sensor endpoint
    String url = String(serverEndpoint) + "/mpu";
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    String payLoad = "{\"accelX\": " + String(accelX) +
                     ", \"accelY\": " + String(accelY) + 
                     ", \"accelZ\": " + String(accelZ) + 
                     ", \"gyroX\": " + String(gyroX) + 
                     ", \"gyroY\": " + String(gyroY) + 
                     ", \"gyroZ\": " + String(gyroZ) + "}";
    
    int httpResponseCode = http.POST(payLoad);

    if (httpResponseCode > 0) {
      Serial.println("MPU sensor data sent to server");
    } else {
      Serial.println("Error sending data to MPU sensor");
    }

    http.end();

  } else {
    Serial.println("WiFi connection lost...");
    // Trying to reconnect to WiFi
    WiFi.begin(ssid, password);
  }
}