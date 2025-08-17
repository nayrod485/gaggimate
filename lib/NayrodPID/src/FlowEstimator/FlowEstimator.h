#ifndef FLOW_ESTIMATOR_H
#define FLOW_ESTIMATOR_H

#include <cmath>
#include <algorithm>

class FlowEstimator {
public:
    FlowEstimator(float dt_, float CmlPerBar_);
    void reset();

    // Set process & measurement noise variances
    void setNoise(float qP, float qQ, float rMeas);

    // Update one step with pump flow (mL/s) and measured pressure (bar)
    void update(float Qin_ml_s, float P_meas_bar);

    // Getters
    float getPressureBar() const { return x[0]; }
    float getFlowMlPerSec() const { return x[1]; }
    float getFlowVariance() const { return P[1][1]; }

private:
    float dt;            // sampling time [s]
    float CmlPerBar;     // compliance [mL/bar]
    float x[2];          // state [Pressure (bar), Qout (mL/s)]
    float P[2][2];       // covariance
    float Q[2][2];       // process noise covariance
    float R;             // measurement noise variance (bar^2)
};

#endif // FLOW_ESTIMATOR_H
