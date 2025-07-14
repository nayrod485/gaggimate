// PressureController.h
#ifndef PRESSURE_CONTROLLER_H
#define PRESSURE_CONTROLLER_H
#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif
#include "RLS_R_estimator.h"
#include "SimpleKalmanFilter.h"
class PressureController {
  public:
    enum class ControlMode { POWER, PRESSURE, FLOW };
    PressureController(float dt, float *rawSetpoint, float *sensorOutput, float *controllerOutput, int *valveStatus);
    void filterSetpoint(float rawSetpoint);
    void initSetpointFilter();
    void setupSetpointFilter(float freq, float damping);
    
    void setFlowLimit(float lim){_flowLimit = lim;};
    void setPressureLimit(float lim){_pressureLimit = lim;};

    float getFilteredSetpoint() const { return _r; };
    float getFilteredSetpointDeriv() const { return _dr; };

    void update(ControlMode Mode);
    void filterSensor();
    void tare();

    void computePumpDutyCycle();
    void virtualScale();
    void reset();

    float getFlowPerSecond() { return flowPerSecond; };
    float getcoffeeOutputEstimate() { return coffeeOutput; };
    float getFilteredPressure() { return _filteredPressureSensor; };


  private:
    float _dt = 1; // Controler frequency sampling

    float *_rawSetpoint = nullptr; // pointer to the controller pressure setpoint
    float *_rawPressure = nullptr; // pointer to the pressure measurement ,raw output from sensor
    float *_ctrlOutput = nullptr;  // pointer to controller output value of power ratio 0-100%
    int *valveStatus = nullptr;     // pointer to OPV status regarding group head canal open/closed

    float _filteredPressureSensor = 0.0f;
    float _filtfreqHz = 1.0f; // Setpoint filter cuttoff frequency
    float _filtxi = 1.2f;     // Setpoint filter damping ratio
    float _r = 0.0f;          // r[n]     : filtered setpoint
    float _dr = 0.0f;         // dr[n]     : derivative of filtered setpoint
    bool _filterInitialised = false;
    float _flowLimit = 0.0f;
    float _pressureLimit = 0.0f;

    // === Paramètres système ===
    const float _Co = 1e-8f; // Compliance (m^3/bar)
    float _R = 5e6f;
    const float _Q0 = 14e-6f;  // Débit max à P = 0 (m^3/s)
    const float _Pmax = 15.0f; // Pression max (bar)

    // === Paramètres Controller ===
    float _K = 0.3; // Commutation gain
    float _lambda = 3 / 1;
    ;                       // Convergence gain
    float _epsilon = 1.5f;  // Limite band
    float _rho = _K * 0.0f; // Uncertainty
    float _Ki = 0.4f;
    float _integLimit = 1000.0f;

    float _P_previous = 0.0f;
    float _errorInteg = 0.0f;

    float alpha = 0.0f;

    float flowPerSecond = 0.0f;
    float coffeeOutput = 0.0f;
    float retroCoffeeOutputPressureHistory = 0.0f;

    SimpleKalmanFilter *pressureKF;
    RLSFilter *R_estimator;

    float computeEstimatedFlowRate(float p, float r);

};

#endif // PRESSURE_CONTROLLER_H
