#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "RoboCore_Vespa.h"

// Motor pin definitions
#define MOTOR_A1 13 // Motor A (forward)
#define MOTOR_A2 14 // Motor A (backward)
#define MOTOR_B1 27 // Motor B (forward)
#define MOTOR_B2 4  // Motor B (backward)

// PWM configuration
#define PWM_FREQ 5000       // PWM frequency
#define PWM_RESOLUTION 8    // PWM resolution
#define PWM_CHANNEL_A 0     // PWM channel for motor A
#define PWM_CHANNEL_B 1     // PWM channel for motor B
#define MAX_PWM 255         // Maximum PWM value
#define MIN_PWM 150         // Minimum PWM value to overcome motor inertia

uint16_t max_duty_cycle = (uint16_t)(pow(2, PWM_RESOLUTION) - 1); // Max duty cycle

// Batery voltage
VespaBattery vbat;

// Wi-Fi credentials
const char* wifi_ssid = "";
const char* wifi_password = "";

WiFiServer server(8008);

void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void startServer(int maxRetries = 5, int retryDelayMs = 2000) {
  int attempts = 0;
  
  while (attempts < maxRetries) {
    server.begin();
    
    if (server) {
      Serial.println("Server started.");
      return;
    }
    
    delay(retryDelayMs);
    Serial.println("Retrying server startup...");
    attempts++;
  }
  
  Serial.println("Failed to start server after retries.");
}

void processJsonCommand(String jsonCommand) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonCommand);

  if (error) {
    Serial.print("Error parsing JSON: ");
    Serial.println(error.c_str());
    return;
  }

  const char* command = doc["command"];
  int speed = doc["speed"];
  int leftSpeed = doc["leftSpeed"];
  int rightSpeed = doc["rightSpeed"];

  if (strcmp(command, "forward") == 0) {
    forward(speed);
  } else if (strcmp(command, "backward") == 0) {
    backward(speed);
  } else if (strcmp(command, "turn") == 0) {
    turn(leftSpeed, rightSpeed);
  } else if (strcmp(command, "stop") == 0) {
    stop();
  } else if (strcmp(command, "test") == 0) {
    Serial.println("Start test");
    forward(speed);
    delay(2000);
    stop();
    delay(2000);
    backward(speed);
    delay(2000);
    stop();
    delay(2000);
    turn(-speed, speed);
    delay(2000);
    stop();
    delay(2000);
    turn(speed, -speed);
    delay(2000);
    stop();
    Serial.println("End test");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

void setupPWM() {
  // Configure motor pins as output
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  // Setup PWM channels for the motors
  ledcAttachChannel(MOTOR_A1, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_A);
  ledcAttachChannel(MOTOR_B1, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_B);
}

// Function to map percentage (0-100%) to duty cycle (0-MAX_PWM)
uint16_t mapSpeedToPWM(uint8_t speedPercent) {
  uint16_t pwmValue = map(speedPercent, 0, 100, 0, max_duty_cycle);

  if (pwmValue > MAX_PWM) {
    return MAX_PWM;
  }

  if (pwmValue != 0 && pwmValue < MIN_PWM) {
    return MIN_PWM;
  }

  return pwmValue;
}

void forward(uint8_t speedPercent) {
  uint16_t pwmValue = mapSpeedToPWM(speedPercent);

  // Turn off the backward pin
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, LOW);

  // Apply PWM
  ledcWriteChannel(PWM_CHANNEL_A, pwmValue);
  ledcWriteChannel(PWM_CHANNEL_B, pwmValue);

  // Activate the forward pin
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
}

void backward(uint8_t speedPercent) {
  uint16_t pwmValue = mapSpeedToPWM(speedPercent);

  // Turn off the forward pin
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_B1, LOW);

  // Apply PWM
  ledcWriteChannel(PWM_CHANNEL_A, pwmValue);
  ledcWriteChannel(PWM_CHANNEL_B, pwmValue);
  
  // Activate the backward pin
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B2, HIGH);
}

void stop() {
  // Set PWM to 0 for both motors
  ledcWriteChannel(PWM_CHANNEL_A, 0);
  ledcWriteChannel(PWM_CHANNEL_B, 0);

  // Turn off all control pins
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void turn(int8_t leftSpeedPercent, int8_t rightSpeedPercent) {
  uint8_t pwmLeft = mapSpeedToPWM(abs(leftSpeedPercent));
  uint8_t pwmRight = mapSpeedToPWM(abs(rightSpeedPercent));

  if (leftSpeedPercent >= 0) { // Forward
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
  } else { // Backward
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
  }

  if (rightSpeedPercent >= 0) { // Forward
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  } else { // Backward
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  }

  ledcWriteChannel(PWM_CHANNEL_A, pwmLeft);
  ledcWriteChannel(PWM_CHANNEL_B, pwmRight);
}

void setup() {
  Serial.begin(115200);

  setupPWM();
  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    startServer(5, 2000);
  }
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    String jsonCommand = "";
    
    while (client.connected()) {
      if (client.available()) {
        jsonCommand = client.readStringUntil('\n');

        if (jsonCommand.length() > 0) {
          Serial.print("Received JSON: ");
          Serial.println(jsonCommand);
          processJsonCommand(jsonCommand);
          jsonCommand = "";

          Serial.print("Battery voltage: ");
          Serial.print(vbat.readVoltage());
          Serial.println(" mV");
        }
      }
    }

    client.stop();
    Serial.println("Client Disconnected.");
  }
}
