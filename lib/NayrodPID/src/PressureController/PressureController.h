// PressureController.h
#ifndef PRESSURE_CONTROLLER_H
#define PRESSURE_CONTROLLER_H

#ifndef M_PI
static constexpr float M_PI = 3.14159265358979323846f;
#endif

#include "SimpleKalmanFilter/SimpleKalmanFilter.h"
#include "ImprovedFiltering.h"
#include "PumpModel.h"
#include "CoffeeEstimator.h"
#include <algorithm>
#include <memory>

/**
 * PressureController - Advanced pressure and flow control system for espresso machines
 * 
 * This controller manages pressure and flow control using a combination of:
 * - Kalman filtering for sensor noise reduction
 * - Advanced control algorithms for pressure regulation
 * - Flow estimation and puck resistance calculation
 * - Virtual scale functionality for coffee output measurement
 */
class PressureController {
public:
    enum class ControlMode { POWER, PRESSURE, FLOW };
    
    // Filter configuration structure
    struct FilterConfig {
        ::FilterConfig pressureDerivativeFilter;
        
        // Default constructor with sensible defaults (uses Savitzky-Golay for pressure derivative)
        FilterConfig() : 
            pressureDerivativeFilter(::FilterConfig::SAVITZKY_GOLAY, 15, 2) {}
        
        // Constructor to specify filter type and parameters
        FilterConfig(::FilterConfig::FilterType filterType, int windowSize, int polynomialOrder = 2) :
            pressureDerivativeFilter(filterType, windowSize, polynomialOrder) {}
    };
    
    // Constructor with input validation
    PressureController(float dt, float* rawPressureSetpoint, float* rawFlowSetpoint, 
                      float* sensorOutput, float* controllerOutput, int* valveStatus);
    
    // Destructor for proper cleanup
    ~PressureController();
    
    // Control interface
    void update(ControlMode mode);
    void tare();
    void reset();
    void initSetpointFilter(float val = 0.0f);
    
    // Configuration methods
    void setPumpFlowCoeff(float oneBarFlow, float nineBarFlow);
    void setPumpFlowPolyCoeffs(float a, float b, float c, float d);
    void setDeadVolume(float deadVol) { 
        if (_coffeeEstimator) _coffeeEstimator->setDeadVolume(deadVol); 
    }
    
    // Filter configuration tuning (if needed)
    void setPressureFilterWindow(int windowSize) { const_cast<::FilterConfig&>(_filterConfig.pressureDerivativeFilter).windowSize = windowSize; }
    void setPressureFilterOrder(int polynomialOrder) { const_cast<::FilterConfig&>(_filterConfig.pressureDerivativeFilter).polynomialOrder = polynomialOrder; }
    
    // Getters for system state
    float getCoffeeOutputEstimate() const { return _coffeeEstimator ? _coffeeEstimator->getCoffeeOutput() : 0.0f; }
    float getPumpFlowRate() const { return _pumpModel ? _pumpModel->getExportFlowRate() : 0.0f; }
    float getCoffeeFlowRate() const { return *_valveStatus == 1 ? (_coffeeEstimator ? _coffeeEstimator->getCoffeeFlowRate() : 0.0f) : 0.0f; }
    float getPuckResistance() const { return _coffeeEstimator ? _coffeeEstimator->getPuckResistance() : 1e7f; }
    
    // Legacy methods (currently not implemented)
    void setFlowLimit(float lim) { /* Flow limit not currently implemented */ }
    void setPressureLimit(float lim) { /* Pressure limit not currently implemented */ }

private:
    // Utility functions
    static void applyLowPassFilter(float* filteredValue, float rawValue, float cutoffFreq, float dt);
    
    // Control algorithms
    float getPumpDutyCycleForPressure();
    float getPumpDutyCycleForFlowRate() const;
    float getAvailableFlow() const;
    
    // Signal processing
    void filterSensor();
    void filterSetpoint(float rawSetpoint);
    
    // Flow modeling
    float pumpFlowModel(float alpha = 100.0f) const;

private:
    // === Configuration Parameters ===
    const float _dt;
    const FilterConfig _filterConfig;                    // Controller sampling period (seconds)
    const float _systemCompliance = 1.4f;  // System compliance (ml/bar)
    const float _maxPressure = 15.0f;       // Maximum pressure (bar)
    const float _maxPressureRate = 9.0f;    // Maximum pressure rate (bar/s)
    
    // === Input/Output Pointers ===
    float* _rawPressureSetpoint;        // Pressure profile current setpoint/limit (bar)
    float* _rawFlowSetpoint;            // Flow profile current setpoint/limit (ml/s)
    float* _rawPressure;                // Raw pressure measurement from sensor (bar)
    float* _ctrlOutput;                 // Controller output power ratio (0-100%)
    int* _valveStatus;                  // 3-way valve status (group head open/closed)
    
    // === Filtering and Signal Processing ===
    std::unique_ptr<SimpleKalmanFilter> _pressureKalmanFilter;
    std::unique_ptr<DerivativeFilter> _pressureDerivativeFilter;
    float _filteredPressureSensor = 0.0f;      // Filtered pressure sensor reading (bar)
    float _filteredSetpoint = 0.0f;            // Filtered pressure setpoint (bar)
    float _filteredSetpointDerivative = 0.0f; // Derivative of filtered setpoint (bar/s)
    float _filteredPressureDerivative = 0.0f;   // Derivative of filtered pressure (bar/s)
    float _lastFilteredPressure = 0.0f;        // Previous filtered pressure for derivative calculation
    int _sampleCounter = 0;                    // Sample counter for timestamping
    
    // Setpoint filter parameters
    float _setpointFilterFreq = 1.0f;         // Setpoint filter cutoff frequency (Hz)
    float _setpointFilterDamping = 1.2f;       // Setpoint filter damping ratio
    bool _setpointFilterInitialized = false;
    float _filterEstimatorFrequency = 1.0f;    // Filter frequency for estimator
    
    // === Controller Parameters ===
    float _commutationGain = 0.7f;              // Commutation gain
    float _convergenceGain = 1.0f;              // Convergence gain
    float _epsilonCoefficient = 0.3f;           // Limit band coefficient
    float _deadbandCoefficient = 0.1f;           // Dead band coefficient
    float _integralGain = 0.25f;                // Integral gain (dt/tau)
    
    // === Controller State ===
    float _previousPressure = 0.0f;             // Previous pressure reading (bar)
    float _errorIntegral = 0.0f;                // Integral of pressure error
    float _pumpDutyCycle = 0.0f;                // Calculated pump duty cycle (0-100%)
    float _pumpFlowCoefficients[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Pump flow model coefficients
    
    // === Subsystems ===
    std::unique_ptr<PumpModel> _pumpModel;
    std::unique_ptr<CoffeeEstimator> _coffeeEstimator;
};

#endif // PRESSURE_CONTROLLER_H
