#include <Servo.h>
#include <Wire.h>               // Include Wire library for I2C communication
#include <Adafruit_Sensor.h>     // Include Adafruit Sensor library
#include <Adafruit_BME280.h>     // Include Adafruit BME280 sensor library

Servo myservo;  // create servo object to control a servo
int closeServo = 90; // Adjust the fine movement
int openServo = 270; // Adjust the fine movement

const int inputPin = 13; // Renamed variable to avoid conflict
const int inputPin2 = 12;
const int inputPin3 = 11;
const int Pyro = 10;

#define BME280_I2C_ADDRESS  0x76  // Set the I2C address of your BME280 sensor

Adafruit_BME280 bme280;           // Create an instance of the BME280 sensor

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  pinMode(inputPin, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
  pinMode(Pyro, OUTPUT);
  if (!bme280.begin(BME280_I2C_ADDRESS)) {  // Initialize the sensor
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);  // Infinite loop if sensor is not found
  }
  Serial.println("Looking for signal");
}

void loop() {
  /*
  if(digitalRead(inputPin) == LOW){
    // When the signal is lost, print the message
    Serial.println("Signal Lost Pin 1");
  }
  if(digitalRead(inputPin2) == LOW){
    // When the signal is lost, print the message
    Serial.println("Signal Lost Pin 2");
  }
  if(digitalRead(inputPin3) == LOW){
    // When the signal is lost, print the message
    Serial.println("Signal Lost Pin 3");
  }*/
  if(digitalRead(inputPin) == LOW & digitalRead(inputPin2) == LOW || digitalRead(inputPin) == LOW & digitalRead(inputPin3) == LOW || digitalRead(inputPin2) == LOW & digitalRead(inputPin3) == LOW)
  {
    float temperature = bme280.readTemperature();          // Read temperature
    float pressure = bme280.readPressure();                // Read pressure
    float altitude = bme280.readAltitude(1013.25);

    if(altitude == 1000){
      digitalWrite(Pyro, HIGH);
      delay(2000);
      digitalWrite(Pyro, LOW);
    }
    if(altitude == 400){
      myservo.write(closeServo);
    }
  }
  
}
