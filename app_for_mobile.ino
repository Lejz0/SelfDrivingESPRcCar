#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>

// Motor A (First L298N)
int motor1Pin1 = 21;
int motor1Pin2 = 22;
int enable1Pin = 5;

// Motor B (First L298N)
int motor2Pin1 = 19;
int motor2Pin2 = 18;
int enable2Pin = 23;

// Motor C (Second L298N)
int motor3Pin1 = 25;
int motor3Pin2 = 26;
int enable3Pin = 16;

// Motor D (Second L298N)
int motor4Pin1 = 14;
int motor4Pin2 = 12;
int enable4Pin = 4;

const int buzPin = 13;
const int ledPin = 15;
const int wifiLedPin = 2;

int SPEED_HIGH = 80; // Initial high speed
int SPEED_LOW = 50;  // Reduced speed after initial movement

WebServer server(80); // HTTP server on port 80

String sta_ssid = "iPhone13mini";
String sta_password = "11112222";

void setup() {
    Serial.begin(115200);
    Serial.println("*WiFi Robot Remote Control Mode - L298N 2A*");
    Serial.println("------------------------------------------------");

    pinMode(buzPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(wifiLedPin, OUTPUT);
    digitalWrite(buzPin, LOW);
    digitalWrite(ledPin, LOW);
    digitalWrite(wifiLedPin, HIGH);

    pinMode(enable1Pin, OUTPUT);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);

    pinMode(enable2Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    pinMode(enable3Pin, OUTPUT);
    pinMode(motor3Pin1, OUTPUT);
    pinMode(motor3Pin2, OUTPUT);

    pinMode(enable4Pin, OUTPUT);
    pinMode(motor4Pin1, OUTPUT);
    pinMode(motor4Pin2, OUTPUT);

    analogWrite(enable1Pin, SPEED_HIGH);
    analogWrite(enable2Pin, SPEED_HIGH);
    analogWrite(enable3Pin, SPEED_HIGH);
    analogWrite(enable4Pin, SPEED_HIGH);

    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);

    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
    unsigned long startMillis = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMillis < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n*WiFi-STA-Mode*");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        digitalWrite(wifiLedPin, LOW);
    } else {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP8266_Robot");
        Serial.println("\n*WiFi-AP-Mode*");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        digitalWrite(wifiLedPin, HIGH);
    }

    server.on("/control", handleControl);
    server.onNotFound(handleNotFound);
    server.begin();

    ArduinoOTA.begin();

    // 15-second delay before starting the movement
    delay(15000);

    // Start moving forward with higher speed
    Forward(SPEED_HIGH);

    // After moving for 2 seconds, reduce the speed
    delay(200);
    Forward(SPEED_LOW);
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
}

void handleControl() {
    if (server.hasArg("cmd")) {
        String cmd = server.arg("cmd");
        if (cmd == "STRAIGHT") {
            Forward(SPEED_LOW);  // After the initial burst, the default speed is low
        } else if (cmd == "RIGHT" && server.hasArg("force")) {
            int force = server.arg("force").toInt();
            TurnRight(force);
        } else if (cmd == "LEFT" && server.hasArg("force")) {
            int force = server.arg("force").toInt();
            TurnLeft(force);
        }
        server.send(200, "text/plain", "Command executed");
    } else {
        server.send(400, "text/plain", "Invalid command");
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not found");
}

// Forward function with variable speed
void Forward(int speed) {
    analogWrite(enable1Pin, speed);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);

    analogWrite(enable2Pin, speed);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);

    analogWrite(enable3Pin, speed);
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);

    analogWrite(enable4Pin, speed);
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
}

void TurnRight(int force) {
    int turnSpeed = SPEED_LOW - force;
    analogWrite(enable1Pin, turnSpeed);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);

    analogWrite(enable2Pin, SPEED_LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);

    analogWrite(enable3Pin, turnSpeed);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);

    analogWrite(enable4Pin, SPEED_LOW);
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);

    delay(200); // Run motors for 1 second

    // Stop motors
    StopMotors();
}

void TurnLeft(int force) {
    int turnSpeed = SPEED_LOW - force;
    analogWrite(enable1Pin, SPEED_LOW);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);

    analogWrite(enable2Pin, turnSpeed);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);

    analogWrite(enable3Pin, SPEED_LOW);
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);

    analogWrite(enable4Pin, turnSpeed);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);

    delay(200); // Run motors for 1 second

    // Stop motors
    StopMotors();
}

void StopMotors() {
    analogWrite(enable1Pin, 0);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);

    analogWrite(enable2Pin, 0);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);

    analogWrite(enable3Pin, 0);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);

    analogWrite(enable4Pin, 0);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
}
