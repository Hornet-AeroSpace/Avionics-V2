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

  if ((digitalRead(inputPin) == LOW && digitalRead(inputPin2) == LOW) || 
      (digitalRead(inputPin) == LOW && digitalRead(inputPin3) == LOW) || 
      (digitalRead(inputPin2) == LOW && digitalRead(inputPin3) == LOW)) {
      
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
    }
    if (altitude >= 389 && altitude <= 399) {
      myservo.write(unlock);
    }

   // recording data 
     dataFile.print((millis()-startTime));  dataFile.print( ",");
     dataFile.print(pressure);  dataFile.print( ",");
     dataFile.print(altitude);  dataFile.print( ",");
    dataFile.println(startingPressure); 


  }
}
