#ifndef HYDRAULICPARAMETERESTIMATOR_H
#define HYDRAULICPARAMETERESTIMATOR_H

#include <deque>
#include <math.h>

class HydraulicParameterEstimator {
  public:
    HydraulicParameterEstimator(float dt_ = 0.03f);

    bool update(float Q_in, float P_raw);
    void reset();
    bool hasConverged();
    void setPhysicalNoises(float sigmaQin, float kDrift, float qOutDrift, float pressureNoise);
<<<<<<< HEAD
=======
    float getEffectiveCompliance(float Vin);
>>>>>>> 0ef912e1aa9d4336fec9477805dd266698c240ac
    float getResistance() { return X_state[1]; };   // k
    float getQout() { return X_state[2]; };        // Qout
    float getPressure() { return X_state[0]; };    // P

    float getCovarianceK() { return P_cov[1][1]; };
    float getCovarianceQout() { return P_cov[2][2]; };
<<<<<<< HEAD
    float getCeff() {return C_eff;};
    float setMinPressureStart(float press){minPressureStart = press;};
    
  private:
    float getEffectiveCompliance(float Vin);
=======
    float getCeff(){return C_eff;};
>>>>>>> 0ef912e1aa9d4336fec9477805dd266698c240ac
    float C_fixed;
    float K_est_init;
    float K_est;
    float C_eff;
<<<<<<< HEAD
  // === State & parameter for EKF ===
=======

    float P_filtered;
    float dPdt_filtered;

    // === Etat & hyperparamètres EKF ===
>>>>>>> 0ef912e1aa9d4336fec9477805dd266698c240ac
    float X_state[3] = {0.0f, 0.0f, 0.0f}; // [P, k, Qout]
    float P_cov[3][3] = {0};
    float Qk[3][3] = {0};   // process noise
    float meas_noise_var;   // noise on P
<<<<<<< HEAD
    float dt;
    float epsilon = 1e-6f;
    float Vin_cum = 0.0f; 
    float minPressureStart = 1.0f; // Minimal pressure to start estimator (ie: boiler filled, pressure start to build up)
=======
    float lambda;           // forget factor

  private:
    float dt;
    float epsilon = 1e-6f;
    int counter;
    bool isValid(float x) { return fabs(x) < 1e30f; }
    float Vin_cum = 0.0f; 
>>>>>>> 0ef912e1aa9d4336fec9477805dd266698c240ac
};

#endif
