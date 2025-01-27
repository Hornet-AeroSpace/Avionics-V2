#include <MAVLink.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SPI.h> 


Servo myservo;
int lock = 90;    // Lock position
int unlock = 0;   // Unlock position

const int inputPin = 13;
const int inputPin2 = 12;
const int inputPin3 = 14;
const int Pyro = 15;
const int Screw = 26;
int num = -5;
float prevTime = 0, prevAlt = 0.0, deltAlt = 0.0;
float holdAlt = 0.0;
float altitude = 0.0;
float  groundspeed = 0; //meters per second  updates km/h on hud for MAVlink
unsigned long currentTime = 0; 

#define BMP390_I2C_ADDRESS 0x77 // Default I2C address for BMP390
Adafruit_BMP3XX bmp;
void setup() {
  Serial.begin(115200);

  myservo.attach(5);
  
  pinMode(inputPin, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
  pinMode(Pyro, OUTPUT);
  pinMode(Screw, INPUT);
  digitalWrite(Pyro, LOW);  

  // Initialize BMP390 sensor
  if (!bmp.begin_I2C(BMP390_I2C_ADDRESS)) {
    Serial.println("Could not find a valid BMP390 sensor, check wiring!");
    while (1);
  }

  // Set sensor parameters
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

}

void loop() {
  currentTime = millis();
  float startingPressure;
  if (num<= 5){
    startingPressure = bmp.pressure / 100.0;
    Serial.print("Starting Pressure" + num);Serial.println(startingPressure);
    num = num + 1;
  }
  if(apogeeReached){
      groundspeed = holdAlt;
  }
  float temperature = bmp.temperature;
    float pressure = bmp.pressure / 100.0; // Convert pressure from Pa to hPa
  altitude = bmp.readAltitude(startingPressure);
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

float heading = (200 * 100);      // Heading in degrees (0-360) this updates heading 
uint16_t throttle = 0;    // Throttle percentage (0-100) on hud
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

  delay(1000);  // Send once per second
  
  if (digitalRead(Screw) == LOW) {
    myservo.write(lock);
  }
     
    // Perform sensor reading
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading from BMP390 sensor!");
      return;
    }

    

    // Calculate altitude manually using barometric formula
    

    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Pressure: "); Serial.println(pressure);
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Starting Pressure: "); Serial.println(startingPressure);

    // Altitude-based actions
    if (altitude <= 1000 && altitude >= 995) { // Adjust range as needed for small altitude variations
      digitalWrite(Pyro, HIGH);
      delay(2000);
      digitalWrite(Pyro, LOW);
      
    }
    if (altitude >= 389 && altitude <= 399) {
      myservo.write(unlock);
      
    }

  }
bool apogeeReached() {
// logic: if altitude greater than current, with a margin that is GREATER than a defined constant;  return TRUE. 

/*$$  Needed variable: `const int altPast1;` deltAltPast1 will be the delta of the altitude over a single second. 

another variable will be needed to track delay. the delay(n); function cannot be used at all. 


if (deltAltPast1 < 0){ 
  return true; 
}
*/  
  float deltaT = currentTime - prevTime;
  float currentAlt = altitude;
  bool potentialApogee = false;
  const unsigned long interval = 1000;
  // Only execute logic if the interval has passed
  if (deltaT >= interval) {
    // Calculate the change in altitude
    deltAlt = currentAlt - prevAlt;

    // Update prevTime for the next interval
    prevTime = currentTime;

    // Case 1: Altitude is decreasing (potential apogee detected)
    if (deltAlt < 0 && !potentialApogee) {
      potentialApogee = true;  // Mark as potential apogee
      holdAlt = currentAlt;   // Save the altitude for comparison
    }

    // Case 2: Confirm apogee (altitude drops by 20 feet from holdAlt)
    if (potentialApogee && (holdAlt - currentAlt >= 6.096)) {
      return true;  // Confirmed apogee
    }

    // Case 3: Reset potential apogee if altitude starts increasing again
    if (potentialApogee && deltAlt > 0) {
      potentialApogee = false;  // Reset the flag
    }

    // Update prevAlt for the next interval
    prevAlt = currentAlt;
  }

  return false;  // Apogee not reached yet
}
