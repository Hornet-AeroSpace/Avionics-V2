
/* 

External Libraries: https://github.com/olliw42/fastmavlink
Index:
1. " $$ " means please check back on this. INSTRUCTION: hit cntrl + f and enter "$$"
2. method details in README.txt
---------------------------------------------

 Name:           Perserverance Avionics c++ file
 Summary:        Code is written to run on an ESP 32 based microcontroller to preform the functions of a traditional flight computer for a level 3 rocket



*/ 



// ** indentify unused libraries & remove em.

#include "Arduino.h"
#include <Wire.h>
#include <string>
#include <Adafruit_ADXL345_U.h>
#include <MAVLink.h>
#include <Wire.h>
#include "Sensors/Adafruit_ICM20948.h"
#include "Sensors/Adafruit_ICM20X.h"
#include <Sensors/Adafruit_Sensor.h>
#include "Sensors/Adafruit_BMP3XX.h"
#include "Sensors/Adafruit_ADXL375.h"

// *****! Define constants (Set up 2 more i^2C busses and one SPI) 

/*
#define SCK    system clock 
#define MISO   Master in slave out 
#define MOSI   Master out slave in
       #define SS     clock #10
       #define RST    reset pin #32 
       #define DIO    interrupt request #26

*/


// Global Variables 

float a_magnitude, velocity = 0;  // Acceleration magnitude and velocity
unsigned long prevTime = 0;       // For time tracking
float prevAlt = 0.0;
float deltAlt = 0.0;

const int LOCKOUTVELOCITY = 257;  // Units  = m/s "$$" derived by (.75 * 375 m/s) 375 m/s is the speed of sound in m/s


Adafruit_BMP3XX bmp;   // barometric pressure sensor
Adafruit_ICM20948 icm; // Inertial measurment unit sensor 
Adafruit_ADXL375 acc;   // Accelerometer

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


  sensors_event_t event;
  accel.getEvent(&event);  // Get acceleration data
  // Calculate acceleration magnitude (in m/s²)

  // Grab acceleration values & calculate speed via kinematics in returnVelocity(). Velocity is calculated as a scalar within the function

  float ax = event.acceleration.x;  // X acceleration
  float ay = event.acceleration.y;  // Y acceleration
  float az = event.acceleration.z;  // Z acceleration

  //Calculating altitude using Pressure and Temperature
  float altitude = bmp.readAltitude(1013.25);
  
if(!(returnVelocity(ax,ay,az) > LOCKOUTVELOCITY)){ 
//$$  preform functions if you are under .75 * (speed of sound)

if (apogeeReached()){ 

  deployCharges(); 
}

}
// intentionally left out an else block. the following functions are supposed to occur regardless of current state.

  // 1. Create a MAVLink message container
  



}

void trasmitMavlink(){  //$$ function definition incomplete

/*
  mavlink_msg_vfr_hud_pack(
    0,                           
    0,                         
    &msg,
    0,                    
    speed,                 // [m/s] Groundspeed updtaes hud updates hyd
    pitch,                     // [degrees] Heading (yaw angle) // updates hud
    throttle,                   
    0,             
    climb_rate                        // [m/s] Climb rate updates hud
  );
 uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 // Serial.write(buf, len);



int32_t relative_altitude = static_cast<int32_t>(2000 * 1000); // Relative altitude in millimeters //this updates height on hud


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

//Serial.write(buf, len);
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
//Serial.write(buf, len);

*/ 

}

float returnVelocity(float ax, float ay, float az ){ 

  // 1. initalize gravity values for 3 axis

  float gx = 0, gy = 0, gz = 9.8; 
  rotationMatrix(gx,gy,gz); //2. transform values to adjust the components of g

   ax -= gx; 
   ay -= gy; 
   az -= gz; 
   

  a_magnitude = sqrt(ax * ax + ay * ay + az * az);  // Total magnitude
  

  // Time calculation (in seconds)
  unsigned long currentTime = millis();
  float deltaT = (currentTime - prevTime) / 1000.0;  // Convert ms to seconds
  prevTime = currentTime;

  // Integrate to calculate velocity
  velocity += a_magnitude * deltaT;

  return velocity;
  

}

//----------- Function Definitons ----------------------------


bool apogeeReached() {
// logic: if altitude greater than current, with a margin that is GREATER than a defined constant;  return TRUE. 

/*$$  Needed variable: `const int altPast1;` deltAltPast1 will be the delta of the altitude over a single second. 

another variable will be needed to track delay. the delay(n); function cannot be used at all. 


if (deltAltPast1 < 0){ 
  return true; 
}
*/  
  prevAlt = altitude;
  if(altitude - prevAlt < 0){
  	return true;
  }
	
  const unsigned long interval = 1000;  
  unsigned long currentTime = millis();
  float deltaT = (currentTime - prevTime);  
 
if(deltT>= interval){
  prevTime = currentTime;
  deltAlt = 0; 

  }else{ 
  deltAlt += altitude - previousAlt; 
}


// track whatever components you want. 
float rotationMatrix(float &x, float &y, float &z){ 


  sensors_event_t gyro;
  IMU.getEvent(&gyro);  



  // return gyro values
  float roll = gyro.gyro.z;  
  float pitch = gyro.gyro.z;  
  float yaw = gyro.gyro.z; 
  

  //$$ verify if gryo values from the sensor tracks  values in one of two ways 1. incremental rotation or 2. total rotation. 

  // if 1( leave everything alone). if 2(in loop, sum the total changes of the gyro values)

  // Initalize gyro values from the IMU here. 

  float prevX = x, prevY = y, prevZ = z; 



  float theta = toRadians(pitch); // Pitch
  float phi = toRadians(roll);    // Roll
  float psi = toRadians(yaw);     // Yaw


  
  // Rotation matrices      
  float R_roll[3][3] = {
    {1, 0, 0},
    {0, cos(phi), -sin(phi)},
    {0, sin(phi), cos(phi)}
  };

  float R_pitch[3][3] = {
    {cos(theta), 0, sin(theta)},
    {0, 1, 0},
    {-sin(theta), 0, cos(theta)}
  };

  float R_yaw[3][3] = {
    {cos(psi), -sin(psi), 0},
    {sin(psi), cos(psi), 0},
    {0, 0, 1}
  };



// computing R total,  A*B*C    <  

  float R[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        R[i][j] += R_yaw[i][k] * R_pitch[k][j]; // =  A*B
      }
    }
  }


  // pt2  

  float R_combined[3][3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R_combined[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        R_combined[i][j] += R[i][k] * R_roll[k][j]; //  = (A*B)* C
      }
    }
  }


  x = R_combined[0][0] * prevX + R_combined[0][1] * prevY + R_combined[0][2] * prevZ;
  y = R_combined[1][0] * prevX + R_combined[1][1] * prevY + R_combined[1][2] * prevZ;
  z = R_combined[2][0] * prevX + R_combined[2][1] * prevY + R_combined[2][2] * prevZ;

// >
// $$ Source: https://math.libretexts.org/Bookshelves/Applied_Mathematics/Mathematics_for_Game_Developers_(Burzynski)/04%3A_Matrices/4.06%3A_Rotation_Matrices_in_3-Dimensions

}

void admin() {

// will be a function that return batter life or any other needed data via bluetooth. Not a priority atm. 


}

void deployCharges(int pin) {

  /* 
  Set the predefined pin to HIGH

  */ 
 SetPin(pin, HIGH); 

}

// HELPER FUNCTIONS 

// Function to convert degrees to radians
float toRadians(float degrees) {
  return degrees * M_PI / 180.0;
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
* 
* 
* 2. low battery, signal strength indicator, 
*/
