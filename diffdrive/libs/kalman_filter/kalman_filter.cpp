#include "kalman_filter.hpp"


KalmanFilter::KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialValue)
    : q(processNoise), r(measurementNoise), p(estimatedError), x(initialValue), k(0.0f) {}

float KalmanFilter::update(float measurement) {

    if (measurement != measurement || measurement > 1e6f || measurement < -1e6f) {  // NaN is the only value where (x != x) is true
        return x;  // Return current estimate without updating
    }

    // Prediction update
    p += q;

    // Measurement update
    k = p / (p + r);
    x += k * (measurement - x);

    // p *= (1.0f - k);
    p = (1.0f - k) * p * (1.0f - k) + k * r * k;  // Joseph form

    return x;
}

void KalmanFilter::setProcessNoise(float qVal) {
    q = qVal;
}

void KalmanFilter::setMeasurementNoise(float rVal) {
    r = rVal;
}

void KalmanFilter::setEstimatedError(float pVal) {
    p = pVal;
}

void KalmanFilter::setGains(float qVal, float rVal, float pVal) {
    q = qVal;
    r = rVal;
    p = pVal;
}

float KalmanFilter::getProcessNoise() const {
    return q;
}

float KalmanFilter::getMeasurementNoise() const {
    return r;
}

float KalmanFilter::getEstimatedError() const {
    return p;
}

float KalmanFilter::getValue() const {
    return x;
}

void KalmanFilter::reset(float initialValue) {
    x = initialValue;
    p = 1.0f;
}
