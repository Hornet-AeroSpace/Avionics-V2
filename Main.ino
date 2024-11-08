
/* 

External Libraries: https://github.com/olliw42/fastmavlink
Index:
1. " $$ " means please check back on this
2. method details in README.txt
---------------------------------------------

 Name:           Perserverance Avionics c++ file
 Summary:        Code is written to run on a Teensy 4.1 to preform the functions of an avionics subsytem 
		 


*/ 


// Import necessary libraries 

#include <SPI.h>
#include <LoRa.h>
#include <vector>
#include "Arduino.h"
#include <Wire.h>
#include <string>

// **** Necessary dependencies to download
#include "Adafruit_ICM20948.h"
#include "Adafruit_ICM20X.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_ADXL375.h"
#include <Wire.h>

//******


using namespace std;

// *****! Define constants (Set up 2 more i^2C busses and one SPI) 

/*
#define SCK    system clock 
#define MISO   Master in slave out 
#define MOSI   Master out slave in
       #define SS     clock #10
       #define RST    reset pin #32 
       #define DIO    interrupt request #26



*/

#define ICM_SDA 17
#define ICM_SCL 16

#define BMP_SDA 18
#define BMP_SCL 19

#define ACC_SDA 25
#define ACC_SCL 24


#define SS 10
#define RST 32
#define DIO 26

// Funciton prototypes 




// Global variables 


Adafruit_BMP3XX bmp;   // barometric pressure sensor
Adafruit_ICM20948 icm; // Inertial measurment unit sensor 
Adafruit_ADXL375 acc   // Accelerometer

wire.begin();


void setup() {


	Serial.begin(5000000); 


  Serial.println("ICM20948 Test");
  if (!icm.begin_I2C(0x69,&Wire1)) {

    Serial.println("Failed to find ICM20948 chip");
    delay(10);
  }else{
    Serial.println("ICM20948 Found!");
  }


 Serial.println("ADXL375 Accelerometer Test"); 
  /* Initialize the sensor */
  if(!accel375.begin(0x53))
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
  }else{
    Serial.println("ADXL375 Found!");
  }


  Serial.println("Adafruit BMP388 / BMP390 test");
  //Test to see if communicating properly
  if (!bmp.begin_I2C(0x77,&Wire)) { 
  	Serial.println("Could not find a valid BMP3 sensor, check wiring!");
	  } else{ 
    Serial.println("BMP Found !");
	  }



}



void loop() {




}




//----------- Function Definitons ----------------------------

bool turnOn() {




}

bool apogeeReached() {




}



void admin() {



}

void deployCharges() {


}


//---------  MAINTANCE METHODS  --------------------

/*
* Methods to add: 
* transmit method
* promiscuous mode method
* 
* 
* 
* Notes: 
* 1. dont forget to write an index at the top i.g. Lines 23 find() is defined, etc..
* 2. adjust coding rate when RSSI drops
* 
* 2. low battery, signal strength indicator, 
*/