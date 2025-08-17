#include "FlowEstimator.h"

FlowEstimator::FlowEstimator(float dt_, float CmlPerBar_) : dt(dt_), CmlPerBar(CmlPerBar_) {
    reset();
    float sigmaQin = 0.1f; //mL/s Uncertainty of the pump flow model 
    float qOutDrift  = 1.5f;// mL/s^2 Expected max variation of output flow  
    float pressureNoise = 0.01; // bar Pressure sensor noise.  
    setNoise(powf(dt_/CmlPerBar*sigmaQin,2.0f), pow(qOutDrift*dt_,2), powf(pressureNoise, 2.0f)); // default tuning
}

void FlowEstimator::reset() {
    x[0] = 0.0f; // pressure [bar]
    x[1] = 0.0f; // flow [mL/s]

    P[0][0] = 1.0f; 
    P[0][1] = 0.0f;
    P[1][0] = 0.0f; 
    P[1][1] = 1.0f;
}

void FlowEstimator::setNoise(float qP, float qQ, float rMeas) {
    Q[0][0] = qP; 
    Q[0][1] = 0.0f;
    Q[1][0] = 0.0f; 
    Q[1][1] = qQ;
    R = rMeas;
}

void FlowEstimator::update(float Qin_ml_s, float P_meas_bar) {
    // State transition F and input B
    float F[2][2] = {
        {1.0f, -dt / CmlPerBar},
        {0.0f, 1.0f}
    };
    float B[2] = {dt / CmlPerBar, 0.0f};

    // --- Prediction ---
    float x_pred[2];
    x_pred[0] = F[0][0]*x[0] + F[0][1]*x[1] + B[0]*Qin_ml_s;
    x_pred[1] = F[1][0]*x[0] + F[1][1]*x[1] + B[1]*Qin_ml_s;

    // P_pred = F P F^T + Q
    float P_pred[2][2] = {{0,0},{0,0}};
    for (int i=0; i<2; i++)
        for (int j=0; j<2; j++)
            for (int k=0; k<2; k++)
                for (int l=0; l<2; l++)
                    P_pred[i][j] += F[i][k] * P[k][l] * F[j][l];
    P_pred[0][0]+=Q[0][0]; P_pred[1][1]+=Q[1][1];

    // --- Update ---
    float H[2] = {1.0f, 0.0f}; // we measure only P
    float S = P_pred[0][0] + R;
    if (S < 1e-12f) S = 1e-12f;

    float K[2] = {P_pred[0][0]/S, P_pred[1][0]/S};

    float y_pred = x_pred[0];
    float residual = P_meas_bar - y_pred;

    x[0] = x_pred[0] + K[0]*residual;
    x[1] = x_pred[1] + K[1]*residual;

    float I_KH[2][2] = {
        {1-K[0]*H[0], -K[0]*H[1]},
        {-K[1]*H[0], 1-K[1]*H[1]}
    };

    float T[2][2] = {{0,0},{0,0}};
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
            for(int k=0;k<2;k++)
                T[i][j] += I_KH[i][k]*P_pred[k][j];

    float P_new[2][2] = {{0,0},{0,0}};
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++) {
            for(int k=0;k<2;k++)
                P_new[i][j] += T[i][k]*I_KH[j][k];
            P_new[i][j] += K[i]*R*K[j];
        }

    // Copy back
    P[0][0] = std::max(P_new[0][0], 1e-12f);
    P[1][1] = std::max(P_new[1][1], 1e-12f);
    float s01 = 0.5f*(P_new[0][1]+P_new[1][0]);
    P[0][1] = P[1][0] = s01;
}
