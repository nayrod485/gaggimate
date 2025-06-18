#include "PressureController.h"
#include "RLS_R_estimator.h"
#include "SimpleKalmanFilter.h"
#include <math.h>
// Helper function to return the sign of a float
inline float sign(float x) { return (x > 0.0f) - (x < 0.0f); }

PressureController::PressureController(float dt, float *rawSetpoint, float *sensorOutput, float *controllerOutput,int *valveStatus) {
    this->_rawSetpoint = rawSetpoint;
    this->_rawPressure = sensorOutput;
    this->_ctrlOutput = controllerOutput;
    this->valveStatus = valveStatus;
    this->_dt = dt;

    this->pressureKF = new SimpleKalmanFilter(0.1f, 10.0f, powf(3 * _dt, 2));
    this->_P_previous = *sensorOutput;
    this->R_estimator = new RLSFilter();
}

void PressureController::filterSetpoint(float rawSetpoint) {
    if (!_filterInitialised)
        initSetpointFilter();
    float _wn = 2 * M_PI * _filtfreqHz;
    float d2r = (_wn * _wn) * (rawSetpoint - _r) - 2.0f * _filtxi * _wn * _dr;
    _dr += d2r * _dt;
    _r += _dr * _dt;
}

void PressureController::initSetpointFilter() {
    _r = *_rawSetpoint;
    _dr = 0.0f;
    _filterInitialised = true;
}

void PressureController::setupSetpointFilter(float freq, float damping) {
    // Reset the filter if values have changed
    if (_filtxi != damping || _filtfreqHz != freq)
        initSetpointFilter();
    _filtfreqHz = freq;
    _filtxi = damping;
}

void PressureController::filterSensor() { _filteredPressureSensor = this->pressureKF->updateEstimate(*_rawPressure); }

void PressureController::tare() { coffeeOutput = 0.0; }

void PressureController::update(ControlMode mode ) {
    
    bool isRconverged = R_estimator->getConvergenceScore() > 0.9f;
    float pressureSetpoint = *_rawSetpoint;
    switch(mode){
        case ControlMode::PRESSURE: {
            if(isRconverged){// With R estimated we can gestimate the appropriate pressure setpoint to not go above flow rate limite
                if(flowPerSecond > _flowLimit){
                    *_rawSetpoint = _flowLimit * this->R_estimator->getEstimate();
                }
            }else{
                if(fabs(_r-_filteredPressureSensor) <0.2 ){ // We consider the pressure to be established so pump flow = coffee flow
                    if( flowPerSecond > _flowLimit ){
                        *_rawSetpoint *= _flowLimit / flowPerSecond;
                    }
                }
            }
            filterSensor();
            filterSetpoint(*_rawSetpoint);
            computePumpDutyCycle();
            break;
        }
        case ControlMode::FLOW: {
            // Coffee flow  = Pressure / R, with the estimated R we can find the appropiate pressure.
            // Without R we can only set the pump to the desired flow rate (which does not say anything about coffee flow rate)
            float pressureSetpointForFlow = *_rawSetpoint * this->R_estimator->getEstimate();
            pressureSetpointForFlow = std::clamp(pressureSetpointForFlow, 0.0f, _pressureLimit);
            filterSetpoint(pressureSetpointForFlow);
            if(isRconverged){ 
                computePumpDutyCycle();
            }else{
                *_ctrlOutput = 100.0f  * _r /( _Q0 * (1 - _filteredPressureSensor / _Pmax));
            }
            break;
        }

        default:
            break;
    }
    virtualScale();
}

void PressureController::virtualScale() {
    float Qi_estim = *_ctrlOutput / 100.0f * _Q0 * (1 - _filteredPressureSensor / _Pmax);
    bool isPpressurized = this->R_estimator->update(Qi_estim, _filteredPressureSensor);
    bool isRconverged = R_estimator->getConvergenceScore() > 0.9f;
    bool isPrefReached = fabsf( (_r - _filteredPressureSensor)/_r) < 0.2;
    if (isPpressurized && *valveStatus == 1 && isRconverged) {
        flowPerSecond = computeEstimatedFlowRate(_filteredPressureSensor + retroCoffeeOutputPressureHistory,this->R_estimator->getEstimate()  );
        coffeeOutput += flowPerSecond * _dt;
        retroCoffeeOutputPressureHistory = 0.0f;
    } else {
        flowPerSecond = Qi_estim;
        if (isPrefReached)
            retroCoffeeOutputPressureHistory += _filteredPressureSensor;
    }
    // ESP_LOGI("","%1.2e",this->R_estimator->getEstimate());
}

void PressureController::computePumpDutyCycle() {
    float P = _filteredPressureSensor;
    float P_ref = _r;
    float dP_ref = _dr;

    float error = P - P_ref;
    float dP_actual = (P - _P_previous) / _dt;
    float error_dot = dP_actual - dP_ref;

    _P_previous = P;

    float s = _lambda * error + 0.1 * error_dot;
    float sat_s = tanhf(s / _epsilon);

    _errorInteg += error * _dt;

    float iterm = 0.0f;
    if(P>1.0f){ // Start integrator only if empty group head volume is filled and pressure starts to build up
        iterm = _Ki * _errorInteg;
        if ((sign(error) == sign(_errorInteg)) && (fabs(iterm) > _integLimit)) {
            _errorInteg -= error * _dt;
            iterm = _Ki * _errorInteg;
        }
    }else
    _K = _K * (1.0f - 0.5 * P_ref / _Pmax);
    alpha = -(_K + 0.1 * fabsf(s)) * sat_s + _rho * sign(s) - _Ki * iterm;
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    *_ctrlOutput = alpha * 100.0f;
}

void PressureController::reset() {
    this->R_estimator->reset();
    initSetpointFilter();
    _errorInteg = 0.0f;
    retroCoffeeOutputPressureHistory = 0;
}

float PressureController::computeEstimatedFlowRate(float pressure, float resistance)
{// Formula in a separeted function for future improvement on flow calculation
    return powf(pressure / resistance * 1e6f, 1 / 1.2);
}