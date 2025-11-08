#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Arduino.h>


class KalmanFilter {
public:
    KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialValue = 0.0f);

    // Update the filter with a new measurement
    float update(float measurement);

    // Setters
    void setProcessNoise(float q);
    void setMeasurementNoise(float r);
    void setEstimatedError(float p);
    void setGains(float q, float r, float p);

    // Getters
    float getProcessNoise() const;
    float getMeasurementNoise() const;
    float getEstimatedError() const;
    float getValue() const;

    // Reset the filter
    void reset(float initialValue = 0.0f);

private:
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float p; // Estimation error covariance
    float k; // Kalman gain
    float x; // Value (state estimate)
};

#endif
