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
int pos = 0;
int x = 0;
Servo myservo;

#define BMP390_I2C_ADDRESS 0x77 // Default I2C address for BMP390
Adafruit_BMP3XX bmp;

void setup() {
    Serial.begin(115200);
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
    pinMode(Screw, INPUT);
    digitalWrite(Pyro, LOW);
    myservo.detach();

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
    if (num <= 10) {
        startingPressure = bmp.pressure / 100.0;
        Serial.print("Starting Pressure: ");
        Serial.println(startingPressure);
        num++;
        delay(500);
        logData("hi");
    }

    if (apogeeReached(altitude)) {
        logData("Apogee reached at " + String(holdAlt));
    }

    altitude = bmp.readAltitude(startingPressure);
    Serial.println(altitude);

    if (digitalRead(inputPin) == LOW) {
        logData("Pin 1 Disconnected");
    }
    if (digitalRead(inputPin2) == LOW) {
        logData("Pin 2 Disconnected");
    }
    if (digitalRead(inputPin3) == LOW) {
        logData("Pin 3 Disconnected");
    }

    if ((digitalRead(inputPin) == LOW && digitalRead(inputPin2) == LOW) ||
        (digitalRead(inputPin) == LOW && digitalRead(inputPin3) == LOW) ||
        (digitalRead(inputPin2) == LOW && digitalRead(inputPin3) == LOW)) {

        if (!bmp.performReading()) {
            Serial.println("Failed to perform reading from BMP390 sensor!");
            return;
        }

        Serial.print("Altitude: ");
        Serial.println(altitude);
        Serial.print("Starting Pressure: ");
        Serial.println(startingPressure);

        if (altitude >= 1000 && altitude <= 995) {
            digitalWrite(Pyro, HIGH);
            logData("Pyro set off at " + String(altitude));
            delay(1500);
            digitalWrite(Pyro, LOW);
        }
        if (altitude >= 389 && altitude <= 399) {
            for (pos = 180; pos >= 0; pos -= 1) {
                myservo.write(pos);
                delay(15);
            }
            logData("Servo opened at " + String(altitude));
        }
    }
    // Create a MAVLink message container
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  float heading = (200 * 100);      // Heading in degrees (0-360)
  float throttle = 30;    // Throttle percentage (0-100)
  float airspeed = 20;
  float Alt = altitude*1000;
  float climb_rate = 20;
  
  mavlink_msg_vfr_hud_pack(
    0,                           
    0,                         
    &msg,
    0,                    
    groundspeed,                 // [m/s] Groundspeed updtaes hud updates hyd
    heading,                     // [degrees] Heading (yaw angle) // updates hud
    throttle,                   
    0,             
    climb_rate                        // [m/s] Climb rate updates hud
  );
 uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  //Serial.write(buf, len);


  float relative_altitude = altitude*1000;



mavlink_msg_global_position_int_pack(
    0, //sys id not requied 
    0, //componet id not required
    &msg,
    0,
    0,
    0,
    0,
    relative_altitude,
    0,
    0,
    0,
    heading
);
len = mavlink_msg_to_send_buffer(buf, &msg);

Serial.write(buf, len);
uint16_t voltage_battery = 8000;  // Example: 11V (in millivolts) //updates voltage 
int16_t current_battery = 200;     // Example: 1A (in centiamperes) //updates current 


mavlink_msg_sys_status_pack(
    0,
    0,
    &msg,
    0,  
    0,  
    0,  
    0,
    voltage_battery, // updates hud
    current_battery, // updates hud
    0,  
    0, 
    0, 
    0,
    0, 
    0, 
    0,
    0,
    0,
    0
); 
  

len = mavlink_msg_to_send_buffer(buf, &msg);
Serial.write(buf, len);
    delay(1000);
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
