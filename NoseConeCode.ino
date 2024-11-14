#include <ESP32Servo.h>          // Include the ESP32Servo library for controlling the servo
#include <Wire.h>                // Include Wire library for I2C communication
#include <Adafruit_Sensor.h>     // Include Adafruit Sensor library
#include <Adafruit_BME280.h>     // Include Adafruit BME280 sensor library
#include <BluetoothSerial.h>     // Include BluetoothSerial library for ESP32

Servo myservo;  // create servo object to control a servo
int lock = 90; // Adjust the fine movement
int unlock = -90; // Adjust the fine movement

const int inputPin = 13; // Pin for input signal 1
const int inputPin2 = 12; // Pin for input signal 2
const int inputPin3 = 14; // Pin for input signal 3 (Changed to a different pin for ESP32)
const int Pyro = 15; // Pin for Pyro (Changed to a different pin for ESP32)

#define BME280_I2C_ADDRESS  0x76  // Set the I2C address of your BME280 sensor

Adafruit_BME280 bme280;           // Create an instance of the BME280 sensor

BluetoothSerial ESP_BT;  // Create BluetoothSerial object

void setup() {
  Serial.begin(9600); // ESP32 typically uses a higher baud rate
  ESP_BT.begin("ESP32_Servo_Control"); // Name of the Bluetooth device
  Serial.println("Bluetooth device is ready to pair");
  
  myservo.attach(5);    // Adjust the pin for the servo (Pin 5 is commonly PWM compatible on ESP32)
  
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
  // Bluetooth data reception
  if (ESP_BT.available()) {
    char receivedChar = ESP_BT.read();  // Read the received character

    if (receivedChar == '1') {
      myservo.write(unlock); // Close the servo when '1' is received
      Serial.println("Servo closed");
    } else if (receivedChar == '2') {
      myservo.write(lock);  // Open the servo when '2' is received
      Serial.println("Servo opened");
    }
  }

  if (digitalRead(inputPin) == LOW && digitalRead(inputPin2) == LOW || 
      digitalRead(inputPin) == LOW && digitalRead(inputPin3) == LOW || 
      digitalRead(inputPin2) == LOW && digitalRead(inputPin3) == LOW) {

    float temperature = bme280.readTemperature();          // Read temperature
    float pressure = bme280.readPressure();                // Read pressure
    float altitude = bme280.readAltitude(1013.25);         // Read altitude with standard pressure

    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Pressure: "); Serial.println(pressure);
    Serial.print("Altitude: "); Serial.println(altitude);

    if (altitude == 1000) {
      digitalWrite(Pyro, HIGH); // Trigger pyro when altitude matches
    }
    if (altitude == 400) {
      myservo.write(unlock); // Move servo to close position when altitude matches
    }
  }
}
