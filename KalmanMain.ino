class KalmanFilter {
public:
    KalmanFilter(float R = 1.0, float Q = 1.0, float A = 1.0, float B = 0.0, float C = 1.0)
        : R(R), Q(Q), A(A), B(B), C(C), cov(NAN), x(NAN) {}

    // Filter a new value
    float filter(float z, float u = 0.0) {
        if (isnan(x)) {
            x = (1.0 / C) * z;
            cov = (1.0 / C) * Q * (1.0 / C);
        } else {
            // Compute prediction
            float predX = predict(u);
            float predCov = uncertainty();

            // Kalman gain
            float K = predCov * C * (1.0 / (C * predCov * C + Q));

            // Correction
            x A= predX + K * (z - C * predX);
            cov = predCov - K * C * predCov;
        }
        return x;
    }

    // Predict next value
    float predict(float u = 0.0) const {
        return A * x + B * u;
    }

    // Return uncertainty of filter
    float uncertainty() const {
        return A * cov * A + R;
    }

    // Return the last filtered measurement
    float lastMeasurement() const {
        return x;
    }

    // Set measurement noise Q
    void setMeasurementNoise(float noise) {
        Q = noise;
    }

    // Set the process noise R
    void setProcessNoise(float noise) {
        R = noise;
    }

private:
    float R; // Process noise
    float Q; // Measurement noise
    float A; // State vector
    float B; // Control vector
    float C; // Measurement vector
    float cov; // Covariance
    float x;   // Estimated signal
};

// Example usage
KalmanFilter kf;

void setup() {
    Serial.begin(9600);

    // Example: Set specific noise values if needed
    kf.setMeasurementNoise(0.5);
    kf.setProcessNoise(0.1);
}

void loop() {
    // Example measurement and control values
    float measurement = 5.0;
    float control = 1.0;

    // Filter the measurement
    float filteredValue = kf.filter(measurement, control);

    // Print the filtered value
    Serial.print("Filtered Value: ");
    Serial.println(filteredValue);

    delay(1000);  // Delay for readability
}
