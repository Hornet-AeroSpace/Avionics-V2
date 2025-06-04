#include <MAVLink.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <SPI.h> 
#include <SD.h>
#include <ESP32Servo.h>

int num = -5;
float startingPressure;
float prevTime = 0, prevAlt = 0.0, deltAlt = 0.0;
float altitude;
float holdAlt = 0.0;
float groundspeed = 40; // meters per second updates km/h on HUD for MAVLink
unsigned long currentTime = 0; 
const int inputPin = 13;
const int inputPin2 = 12;
const int inputPin3 = 14;
const int Pyro = 4;
const int Screw = 26;
const int chipSelect = 5;
const int servoClose = 50;
const int servoOpen = 8;
int x = 0;
Servo myservo;
bool closed = false;

#define BMP390_I2C_ADDRESS 0x77 // Default I2C address for BMP390
Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(9600);
    myservo.attach(2);
    delay(1000);
    if (!bmp.begin_I2C(BMP390_I2C_ADDRESS)) {
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        while (1);
    }
    pinMode(inputPin, INPUT);
    pinMode(inputPin2, INPUT);
    pinMode(inputPin3, INPUT);
    pinMode(Pyro, OUTPUT);
    digitalWrite(Pyro, LOW);
    pinMode(Screw, INPUT);
    

    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized successfully!");
}

void logData(const String &data) {
    File file = SD.open("/datalog.txt", FILE_APPEND);
    if (file) {
        file.println(data);
        file.close();
    } else {
        Serial.println("Failed to open file for writing");
    }
}

void loop() {
    bool screwState = digitalRead(Screw);
    if (num <= 10) {
        startingPressure = bmp.pressure / 100.0;
        Serial.print("Starting Pressure: ");
        Serial.println(startingPressure);
        num++;
        delay(500);
    }
    if (screwState == HIGH && !closed) {
    myservo.write(servoClose); // Close
    closed = true;
    Serial.println("closed");
    }

    if (screwState == LOW && closed) {
    myservo.write(servoOpen); // Open
    closed = false;
    Serial.println("open");
    }

    if (apogeeReached(altitude)) {
        logData("Apogee reached at " + String(holdAlt));
    }

    altitude = bmp.readAltitude(startingPressure);
    Serial.println(altitude);

    if (digitalRead(inputPin) == LOW) {
        logData("Pin 1 Disconnected at " + String(altitude));
    }
    if (digitalRead(inputPin2) == LOW) {
        logData("Pin 2 Disconnected at " + String(altitude));
    }
    if (digitalRead(inputPin3) == LOW) {
        logData("Pin 3 Disconnected at "+ String(altitude));
    }

    if ((digitalRead(inputPin) == LOW && digitalRead(inputPin2) == LOW) ||
        (digitalRead(inputPin) == LOW && digitalRead(inputPin3) == LOW) ||
        (digitalRead(inputPin2) == LOW && digitalRead(inputPin3) == LOW)) {

        Serial.print("Altitude: ");
        Serial.println(altitude);
        Serial.print("Starting Pressure: ");
        Serial.println(startingPressure);

        if (altitude <= 1020 && altitude >= 970) {
            digitalWrite(Pyro, HIGH);
            logData("Pyro set off at " + String(altitude));
            delay(1500);
            digitalWrite(Pyro, LOW);
        }
        if (altitude >= 370&& altitude <= 410) {
            myservo.write(servoOpen);
            logData("Servo opened at " + String(altitude));
        }
    }
  }

bool apogeeReached(int altitude) {
    float deltaT = currentTime - prevTime;
    float currentAlt = altitude;
    bool potentialApogee = false;
    const unsigned long interval = 1000;

    deltAlt = currentAlt - prevAlt;
    prevTime = currentTime;

    if (deltAlt < 0 && !potentialApogee) {
        potentialApogee = true;
        holdAlt = currentAlt;
    }

    if (potentialApogee && (holdAlt - currentAlt >= 6.096)) {
        return true;
    }

    if (potentialApogee && deltAlt > 0) {
        potentialApogee = false;
    }

    prevAlt = currentAlt;
    return false;
}
