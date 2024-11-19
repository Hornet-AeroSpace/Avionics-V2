#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <ICM_20948.h>

// Define BMP390 sensor object
Adafruit_BMP3XX bmp;

// Define ICM-20948 sensor object
ICM_20948_I2C icm;

// Define ADXL375 I2C address
#define ADXL375_ADDRESS 0x53

// ADXL375 Registers
#define ADXL375_REG_DATA_FORMAT 0x31
#define ADXL375_REG_DATAX0 0x32
#define ADXL375_REG_PWR_CTL 0x2D

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to initialize
  Wire.begin();

  // Initialize BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find BMP390!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  // Initialize ICM-20948
  if (icm.begin(Wire, 0x69) != ICM_20948_Stat_Ok) { // Default I2C address is 0x69
    Serial.println("Could not find ICM-20948!");
    while (1);
  }

  // Configure ICM-20948
  if (icm.initializeDMP() != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 initialization failed!");
    while (1);
  }

  Serial.println("ICM-20948 Initialized!");

  // Initialize ADXL375
  Wire.beginTransmission(ADXL375_ADDRESS);
  Wire.write(ADXL375_REG_PWR_CTL);
  Wire.write(0x08); // Set measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(ADXL375_ADDRESS);
  Wire.write(ADXL375_REG_DATA_FORMAT);
  Wire.write(0x0B); // Full resolution, +/-200g range
  Wire.endTransmission();

  Serial.println("All sensors initialized!");
}

void loop() {
  // Read BMP390 data
  if (bmp.performReading()) {
    Serial.print("BMP390 - Temperature: ");
    Serial.print(bmp.temperature);
    Serial.print(" °C, Pressure: ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");
  } else {
    Serial.println("Failed to read BMP390");
  }

  // Read ICM-20948 data
  if (icm.dataReady()) {
    icm.getAGMT(); // Updates internal AGMT structure

    Serial.print("ICM20948 - Accel (g): X=");
    Serial.print(icm.accX());
    Serial.print(", Y=");
    Serial.print(icm.accY());
    Serial.print(", Z=");
    Serial.println(icm.accZ());

    Serial.print("ICM20948 - Gyro (dps): X=");
    Serial.print(icm.gyrX());
    Serial.print(", Y=");
    Serial.print(icm.gyrY());
    Serial.print(", Z=");
    Serial.println(icm.gyrZ());

    Serial.print("ICM20948 - Magnetometer (uT): X=");
    Serial.print(icm.magX());
    Serial.print(", Y=");
    Serial.print(icm.magY());
    Serial.print(", Z=");
    Serial.println(icm.magZ());
  } else {
    Serial.println("ICM20948 - No new data");
  }

  // Read ADXL375 data
  int16_t x, y, z;
  x = readADXL375Axis(ADXL375_REG_DATAX0);
  y = readADXL375Axis(ADXL375_REG_DATAX0 + 2);
  z = readADXL375Axis(ADXL375_REG_DATAX0 + 4);

  Serial.print("ADXL375 - Accel (g): X=");
  Serial.print(x * 0.049); // Convert to g assuming 12-bit resolution
  Serial.print(", Y=");
  Serial.print(y * 0.049);
  Serial.print(", Z=");
  Serial.println(z * 0.049);

  delay(500); // Delay between readings
}

int16_t readADXL375Axis(uint8_t reg) {
  Wire.beginTransmission(ADXL375_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ADXL375_ADDRESS, 2);

  int16_t value = Wire.read();
  value |= (Wire.read() << 8);
  return value;
}
