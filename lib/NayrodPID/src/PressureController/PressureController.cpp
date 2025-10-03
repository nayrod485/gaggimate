#include "PressureController.h"
#include "SimpleKalmanFilter/SimpleKalmanFilter.h"
#include <algorithm>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// Helper function to return the sign of a float
inline float sign(float x) { return (x > 0.0f) - (x < 0.0f); }

// Static utility function for first-order low-pass filtering
void PressureController::applyLowPassFilter(float *filteredValue, float rawValue, float cutoffFreq, float dt) {
    if (filteredValue == nullptr)
        return;

    float alpha = dt / (1.0f / (2.0f * M_PI * cutoffFreq) + dt);
    *filteredValue = alpha * rawValue + (1.0f - alpha) * (*filteredValue);
}

PressureController::PressureController(float dt, float* rawPressureSetpoint, float* rawFlowSetpoint, 
                                      float* sensorOutput, float* controllerOutput, int* valveStatus)
    : _dt(dt), _filterConfig(), _rawPressureSetpoint(rawPressureSetpoint), _rawFlowSetpoint(rawFlowSetpoint),
      _rawPressure(sensorOutput), _ctrlOutput(controllerOutput), _valveStatus(valveStatus) {
    
    // Input validation
    if (!rawPressureSetpoint || !rawFlowSetpoint || !sensorOutput || !controllerOutput || !valveStatus) {
        // Handle error - could throw exception or set error flag
        return;
    }
    
    // Initialize Kalman filter with proper memory management
    _pressureKalmanFilter = std::make_unique<SimpleKalmanFilter>(0.1f, 10.0f, powf(4 * _dt, 2));
    
    // Initialize derivative filters with internal configuration
    _pressureDerivativeFilter = FilterFactory::createFilter(_filterConfig.pressureDerivativeFilter);
    
    // Initialize subsystems
    _pumpModel = std::make_unique<PumpModel>(_dt);
    
    // Configure CoffeeEstimator filters
    CoffeeEstimator::CoffeeFilterConfig coffeeConfig;
    // CoffeeEstimator::CoffeeFilterConfig coffeeConfig(FilterConfig::SAVITZKY_GOLAY);
    
    _coffeeEstimator = std::make_unique<CoffeeEstimator>(_dt, coffeeConfig);
    
    _previousPressure = *sensorOutput;
    _sampleCounter = 0;
}

// Destructor for proper cleanup
PressureController::~PressureController() {
    // unique_ptr will automatically clean up the Kalman filter
}

void PressureController::filterSetpoint(float rawSetpoint) {
    if (!_setpointFilterInitialized) {
        initSetpointFilter();
    }
    
    float omega = 2.0 * M_PI * _setpointFilterFreq;
    float d2r = (omega * omega) * (rawSetpoint - _filteredSetpoint) - 
                2.0f * _setpointFilterDamping * omega * _filteredSetpointDerivative;
    
    // Apply rate limiting to prevent sudden changes
    float derivativeChange = std::clamp(d2r * _dt, -_maxPressureRate, _maxPressureRate);
    _filteredSetpointDerivative += derivativeChange;
    _filteredSetpoint += _filteredSetpointDerivative * _dt;
}

void PressureController::initSetpointFilter(float val) {
    _filteredSetpoint = *_rawPressureSetpoint;
    if (val != 0.0f)
        _filteredSetpoint = val;
    _filteredSetpointDerivative = 0.0f;
    _setpointFilterInitialized = true;
}

void PressureController::filterSensor() {
    if (!_pressureKalmanFilter || !_pressureDerivativeFilter) {
        return; // Safety check
    }
    
    // Use Kalman filter for pressure sensor noise reduction
    float newFiltered = _pressureKalmanFilter->updateEstimate(*_rawPressure);
    
    // Add sample to derivative filter
    _pressureDerivativeFilter->addSample(newFiltered, _sampleCounter * _dt);
    
    // Get derivative using the new filter
    if (_pressureDerivativeFilter->isReady()) {
        _filteredPressureDerivative = _pressureDerivativeFilter->getDerivative(_dt);
    } else {
        // Fallback to simple difference for initial samples
        _filteredPressureDerivative = (newFiltered - _lastFilteredPressure) / _dt;
    }
    
    _lastFilteredPressure = newFiltered;
    _filteredPressureSensor = newFiltered;
    _sampleCounter++;
}


void PressureController::update(ControlMode mode) {
    // Input validation
    if (!_rawPressureSetpoint || !_rawFlowSetpoint || !_rawPressure || !_ctrlOutput || !_valveStatus) {
        return; // Skip update if pointers are invalid
    }
    
    filterSetpoint(*_rawPressureSetpoint);
    filterSensor();

    // Handle different control modes
    if ((mode == ControlMode::FLOW || mode == ControlMode::PRESSURE) && 
        *_rawPressureSetpoint > 0.0f && *_rawFlowSetpoint > 0.0f) {
        // Combined flow and pressure control - use the more restrictive output
        float flowOutput = getPumpDutyCycleForFlowRate();
        float pressureOutput = getPumpDutyCycleForPressure();
        *_ctrlOutput = std::min(flowOutput, pressureOutput);
        
        // Reset integral term when flow is the limiting factor
        if (flowOutput < pressureOutput) {
            _errorIntegral = 0.0f;
        }
    } else if (mode == ControlMode::FLOW) {
        *_ctrlOutput = getPumpDutyCycleForFlowRate();
    } else if (mode == ControlMode::PRESSURE) {
        *_ctrlOutput = getPumpDutyCycleForPressure();
    } else {
        // POWER mode or invalid mode - use manual control
        *_ctrlOutput = 0.0f;
    }
    
    // Update pump model (calculates and filters flow rate internally)
    _pumpModel->update(_filteredPressureSensor, *_ctrlOutput);
    
    // Update coffee estimator with the filtered flow rate
    _coffeeEstimator->update(
        _pumpModel->getPumpFlowRate(),
        _filteredPressureSensor,
        _filteredPressureDerivative,
        *_valveStatus
    );
}

float PressureController::pumpFlowModel(float alpha) const {
    const float availableFlow = getAvailableFlow();
    return availableFlow * alpha / 100.0f;
}

float PressureController::getAvailableFlow() const {
    const float P = std::fmax(0.0f, _filteredPressureSensor); // Ensure non-negative pressure
    const float P2 = P * P;
    const float P3 = P2 * P;
    const float Q = _pumpFlowCoefficients[0] * P3 + _pumpFlowCoefficients[1] * P2 + 
                    _pumpFlowCoefficients[2] * P + _pumpFlowCoefficients[3];

    return std::fmax(0.0f, Q); // Ensure non-negative flow
}

float PressureController::getPumpDutyCycleForFlowRate() const {
    const float availableFlow = getAvailableFlow();
    if (availableFlow <= 0.0f || *_rawFlowSetpoint <= 0.0f) {
        return 0.0f;
    }
    
    float duty = (*_rawFlowSetpoint / availableFlow) * 100.0f;
    return std::clamp(duty, 0.0f, 100.0f);
}

void PressureController::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    // Update controller coefficients
    _pumpFlowCoefficients[0] = 0.0f;
    _pumpFlowCoefficients[1] = 0.0f;
    _pumpFlowCoefficients[2] = (nineBarFlow - oneBarFlow) / 8;
    _pumpFlowCoefficients[3] = oneBarFlow - _pumpFlowCoefficients[2] * 1.0f;
    
    // Update pump model
    if (_pumpModel) {
        _pumpModel->setFlowCoeff(oneBarFlow, nineBarFlow);
    }
}

void PressureController::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    // Update controller coefficients
    _pumpFlowCoefficients[0] = a;
    _pumpFlowCoefficients[1] = b;
    _pumpFlowCoefficients[2] = c;
    _pumpFlowCoefficients[3] = d;
    
    // Update pump model
    if (_pumpModel) {
        _pumpModel->setFlowCoefficients(a, b, c, d);
    }
}

void PressureController::tare() { 
    // Reset subsystems
    if (_pumpModel) _pumpModel->reset();
    if (_coffeeEstimator) _coffeeEstimator->tare();
    
    // Reset derivative filters
    if (_pressureDerivativeFilter) _pressureDerivativeFilter->reset();
    _sampleCounter = 0;
}


float PressureController::getPumpDutyCycleForPressure() {
    // Handle low pressure setpoint (e.g., blooming phase)
    if (*_rawPressureSetpoint < 0.2f) {
        initSetpointFilter();
        _errorIntegral = 0.0f;
        *_ctrlOutput = 0.0f;
        _previousPressure = 0.0f;
        return 0.0f;
    }

    // Get current system state
    float P = _filteredPressureSensor;
    float P_ref = _filteredSetpoint;
    float error = P - P_ref;
    _previousPressure = P;

    // Calculate switching surface parameters
    float epsilon = _epsilonCoefficient * _filteredSetpoint;
    float deadband = _deadbandCoefficient * _filteredSetpoint;
    float s = _convergenceGain * error;
    
    // Calculate saturation function
    float sat_s = 0.0f;
    if (error > 0) {
        float tan = std::tanh(s / epsilon - deadband * _convergenceGain / epsilon);
        sat_s = std::max(0.0f, tan);
    } else if (error < 0) {
        float tan = std::tanh(s / epsilon + deadband * _convergenceGain / epsilon);
        sat_s = std::min(0.0f, tan);
    }

    // Calculate integral term with pressure-dependent gain
    float Ki = _integralGain / (1 - P / _maxPressure);
    _errorIntegral += error * _dt;
    float iterm = Ki * _errorIntegral;

    // Get available flow and calculate control gains
    float Qa = std::fmax(getAvailableFlow(), 1e-3f);
    float Ceq = _systemCompliance;
    float pressureRatio = 1.0f - P / _maxPressure;
    float K = _commutationGain / pressureRatio * Qa / Ceq;
    
    // Calculate control output
    _pumpDutyCycle = Ceq / Qa * (-_convergenceGain * error - K * sat_s) - iterm;

    // Anti-windup protection
    if ((sign(error) == -sign(_pumpDutyCycle)) && (std::fabs(_pumpDutyCycle) > 1.0f)) {
        _errorIntegral -= error * _dt;
        iterm = Ki * _errorIntegral;
        _pumpDutyCycle = Ceq / Qa * (-_convergenceGain * error - K * sat_s) - iterm;
    }

    return std::clamp(_pumpDutyCycle * 100.0f, 0.0f, 100.0f);
}

void PressureController::reset() {
    initSetpointFilter(_filteredPressureSensor);
    _errorIntegral = 0.0f;
    
    // Reset subsystems
    if (_pumpModel) _pumpModel->reset();
    if (_coffeeEstimator) _coffeeEstimator->reset();
    
    // Reset derivative filters
    if (_pressureDerivativeFilter) _pressureDerivativeFilter->reset();
    _sampleCounter = 0;
}
