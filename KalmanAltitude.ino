#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>

Adafruit_BMP3XX bmp;
Adafruit_ADXL375 adxl(-1, &Wire);  // Initialize the ADXL375 with I2C

// Kalman Filter Class
class KalmanFilter {
public:
    KalmanFilter(float R = 1.0, float Q = 1.0, float A = 1.0, float B = 0.0, float C = 1.0)
        : R(R), Q(Q), A(A), B(B), C(C), cov(NAN), x(NAN) {}

    float filter(float z, float u = 0.0) {
        if (isnan(x)) {
            x = (1.0 / C) * z;
            cov = (1.0 / C) * Q * (1.0 / C);
        } else {
            float predX = predict(u);
            float predCov = uncertainty();
            float K = predCov * C * (1.0 / (C * predCov * C + Q));
            x = predX + K * (z - C * predX);
            cov = predCov - K * C * predCov;
        }
        return x;
    }

    float predict(float u = 0.0) const {
        return A * x + B * u;
    }

    float uncertainty() const {
        return A * cov * A + R;
    }

    float lastMeasurement() const {
        return x;
    }

    void setMeasurementNoise(float noise) {
        Q = noise;
    }

    void setProcessNoise(float noise) {
        R = noise;
    }

private:
    float R;
    float Q;
    float A;
    float B;
    float C;
    float cov;
    float x;
};

KalmanFilter kf(0.1, 0.5);

void setup() {
    Serial.begin(9600);

    if (!bmp.begin_I2C()) {
        Serial.println("Could not find BMP390 sensor!");
        while (1);
    }

    if (!adxl.begin()) {
        Serial.println("Could not find ADXL375 sensor!");
        while (1);
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    kf.setMeasurementNoise(0.5);
    kf.setProcessNoise(0.1);
}

void loop() {
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
    float altitude = bmp.readAltitude(1013.25);

    sensors_event_t accel;
    adxl.getEvent(&accel);
    float velocity = accel.acceleration.z;

    float filteredAltitude = kf.filter(altitude, velocity);
    Serial.print("Velocity: ");
    Serialprintln(velocity);

    Serial.print("Altitude: ");
    Serial.println(altitude);

    Serial.print("Filtered Altitude: ");
    Serial.println(filteredAltitude);

    delay(1000);
}
