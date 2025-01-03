#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SPI.h> 
#include <SD.h> 

#define CS_PIN 5 

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


  // initalize SDCard Writer 

  if(!SD.begin(CS_PIN)){ 
    Serial.println(" Card writer failed, or not present"); 
  }else{ 
    unsigned long startTime = millis(); 
    Serial.println(" Card writer initalized "); 
  }
// end 

// Open File for writing

  File dataFile = SD.open( "data.csv", FILE_WRITE); 

  if(dataFile){ 
    dataFile.println("Time, Air Pressure, Derived Altitude,  Temprature "); 
  }
  // end 

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
  float startingPressure;
  if (num<= 5){
    startingPressure = bmp.pressure / 100.0;
    Serial.print("Starting Pressure" + num);Serial.println(startingPressure);
    num = num + 1;
  }
  if (digitalRead(Screw) == LOW) {
    myservo.write(lock);
  }

  if (apogeeReached()) {
      
    // Perform sensor reading
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading from BMP390 sensor!");
      return;
    }

    float temperature = bmp.temperature;
    float pressure = bmp.pressure / 100.0; // Convert pressure from Pa to hPa

    // Calculate altitude manually using barometric formula
    float altitude = bmp.readAltitude(startingPressure);

    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Pressure: "); Serial.println(pressure);
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Starting Pressure: "); Serial.println(startingPressure);

    // Altitude-based actions
    if (altitude >= 1000 && altitude <= 995) { // Adjust range as needed for small altitude variations
      digitalWrite(Pyro, HIGH);
      delay(2000);
      digitalWrite(Pyro, LOW);
      dataFile.print("Nose-Cone Parachute Deploy at "); dataFile.println(altitude);
    }
    if (altitude >= 389 && altitude <= 399) {
      myservo.write(unlock);
      dataFile.print("Drone Launch at "); dataFile.write(altitude);
    }

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
  prevAlt = altitude;
  if(altitude - prevAlt < 0){
	dataFile.print("Apogee Reached at "); dataFile.write(altitude);
  	return true;
  }
	
  const unsigned long interval = 1000;  
  unsigned long currentTime = millis();
  float deltaT = (currentTime - prevTime);  
 
if(deltaT>= interval){
  prevTime = currentTime;
  deltAlt = 0; 

  }else{ 
  deltAlt += altitude - prevAlt; 
}

}

