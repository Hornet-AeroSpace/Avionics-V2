
/* 

External Libraries: https://github.com/olliw42/fastmavlink
Index:
1. " $$ " means please check back on this. INSTRUCTION: hit cntrl + f and enter "$$"
2. method details in README.txt
---------------------------------------------

 Name:           Perserverance Avionics c++ file
 Summary:        Code is written to run on an ESP 32 based microcontroller to preform the functions of a traditional flight computer for a level 3 rocket



*/ 



// $$ Identify & remove any unused librares. 

#include "Arduino.h"
#include <string>
#include <MAVLink.h>
#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_ICM20X.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_ADXL375.h"
#include "Adafruit_INA260.h"


//  MANUALLY VERIFY OR SET VALUE(S) BEFORE RUNNING 

int STAGENUMBER = 0; 

const int MAIN_DEPLOY_ALT = 1400; 
const float BMP_INIT_READ = 1013.25;

const int DROUGEPIN = 12; //  Pin to set off charges, confirm this is correct pin
const int MAINPIN = 11;   // 


unsigned long startTime = 0; // Initalized at Stage 1


float altitude, a_magnitude, velocity = 0;  // Acceleration magnitude and velocity      
float prevTime = 0, prevAlt = 0.0, deltAlt = 0.0, power;


Adafruit_BMP3XX bmp;                    // barometric pressure sensor
Adafruit_ADXL375 accel(0x53, &Wire);    // Accelerometer

/*
$$ uncomment & remove once the hex address for the INA is found 
Adafruit_INA260 ina260 = Adafruit_INA260(IICaddr, &Wire); // voltage & current sensor 
*/ 

void setup() {

Wire.begin();

	Serial.begin(9600);


  Serial.println("ICM20948 Test");
  if (!icm.begin_I2C(0x69,&Wire1)) {

    Serial.println("Failed to find ICM20948 chip");
    delay(10);
  }else{
    Serial.println("ICM20948 Found!");
  }


 Serial.println("ADXL375 Accelerometer Test"); 
  /* Initialize the sensor */
  accel = Adafruit_ADXL375(0x53,&Wire);
  if(!accel.begin(0x53))
  {
    /* There was a problem detecting the ADXL375 ... check your connections */
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
  }else{
    Serial.println("ADXL375 Found!");
  }


  Serial.println("Adafruit BMP390 test");
  //Test to see if communicating properly
  if (!bmp.begin_I2C(0x77,&Wire)) { 
  	Serial.println("Could not find a valid BMP3 sensor, check wiring!");
	  } else{ 
    Serial.println("BMP Found !");
	  }

 //  ina260.begin(); 
}



void loop() {
// Add a StringBuilder { Time, altitude, acc(x), acc(y), acc(z), Stage#, Battery Life }

  unsigned long elapsedTime = millis();

  if(elapsedTime> 250){ 
// Values are initalized in setup, and updated here 
// power = ina260.readPower(); 
// Volt = readBusVoltage(); 
  }

  sensors_event_t event;
  accel.getEvent(&event);  // return acceleration magnitude (in m/s²)

  float ax = event.acceleration.x;  
  float ay = event.acceleration.y;   
  float az = event.acceleration.z;   

  //Calculating altitude using Pressure and Temperature
 altitude = bmp.readAltitude(BMP_INIT_READ);
  


  switch (STAGENUMBER) {
    case 1:
    stageOne(az,ay,ax);   // Checking For accelration, ( needs 3 components bc the sensor can have multiple orientations)
      break;
    case 2:
    stageTwo();           // Wating 5.5s
      break;
    case 3:
    stageThree();        // Checking for Apogee 
      break;
    case 4: 
    stageFour();         // Checking for 1400ft
    default:
      break;
  }







}
// intentionally left out an else block. the following functions are supposed to occur regardless of current state.

  // 1. Create a MAVLink message container
  





void trasmitMavlink(){  //$$ function definition incomplete

/*

*/

}


//----------- Function Definitons ----------------------------


bool apogeeReached(float altitude) {
    const unsigned long interval = 500; // 1/2 second
    unsigned long prevTime = 0;
    float prevAlt = 0;
    float deltAlt = 0;

    unsigned long currentTime = millis();
    float deltaT = currentTime - prevTime;

    // Accumulate delta altitude
    deltAlt += altitude - prevAlt;
    prevAlt = altitude;

    if (deltaT >= interval) {
      prevTime = currentTime;

      if (deltAlt < 0) {
        // Altitude decreased over 1/2 second -> apogee passed
        deltAlt = 0;
        return true;
      }

      // Reset delta for next interval
      deltAlt = 0;
    }

  return false;
}


void stageOne(float az, float ay, float ax){ 
if (az > 1 || ay >1 || ax>1){ 
  startTime = millis();
  STAGENUMBER++;  
} else
{}
  return; 
}



void stageTwo(){ 
if ((millis()-startTime) > 5500){ 
  STAGENUMBER++;  
} 

}

void stageThree(){

  // STAGE 3 LOOPING CONDITION 
if (apogeeReached()){ 
  deployCharges(DROUGEPIN); 
  STAGENUMBER++;  
} 


}

void stageFour(){ 

  if(altitude<MAIN_DEPLOY_ALT){ 
  deployCharges(MAINPIN);
  STAGENUMBER++;  
  }

}





void deployCharges(int pin) {

  /* 
  Set the predefined pin to HIGH
  */ 
 digitalWrite(pin, HIGH); 

}



/*
* Methods to add: 
* transmit method
* promiscuous mode method
* 
* 
* 
* Notes: 
* 1. dont forget to write an index at the top i.g. Lines 23 find() is defined, etc..
* 
* 
* 2. low battery, signal strength indicator, 
*/
