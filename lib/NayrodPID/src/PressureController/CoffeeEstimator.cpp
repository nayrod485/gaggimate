// CoffeeEstimator.cpp - Coffee output and puck behavior estimation implementation
#include "CoffeeEstimator.h"
#include "ImprovedFiltering.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <esp_log.h>

CoffeeEstimator::CoffeeEstimator(float dt, const CoffeeFilterConfig& config) 
    : _dt(dt), _filterConfig(config), _coffeeOutput(0.0f), _coffeeFlowRate(0.0f), _waterThroughPuckFlowRate(0.0f),
      _tempFlow(0.0f), _filteredPressureSensor(0.0f), _puckSaturationVolume(0.0f), 
      _puckSaturatedVolume(35.0f), _puckConductance(0.0f), _puckConductanceDerivative(0.0f), 
      _puckResistance(1e7f), _puckCounter(0), _inPuckVol(0.0f), _settledInitialized(false) {
    
    // Initialize filters with config-specified types
    _conductanceFilter = FilterFactory::createFilter(_filterConfig.conductanceFilter);
    _puckInputFlowFilter = FilterFactory::createFilter(_filterConfig.puckInputFlowFilter);
    
    // Initialize puck state
    setPuckSoaking(false);
    setPuckSettled(false);
    setFirstDropDetected(false);
    
    // Initialize conductance values to prevent startup spikes
    _puckConductance = 0.0f;
    _puckConductanceDerivative = 0.0f;
}

void CoffeeEstimator::update(float pumpFlowRate, float pressure, float pressureDerivative, int valveStatus) {
    // Store current pressure for calculations
    _filteredPressureSensor = pressure;
    
    // Estimate puck input flow rate
    estimatePuckInputFlow(pumpFlowRate, pressure, pressureDerivative);
    
    // Early exit if conditions are not met for puck analysis
    if (_waterThroughPuckFlowRate <= 0.0f || valveStatus != 1 || pressure <= 0.8f) {
        return;
    }

    // Increment puck counter
    _puckCounter++;

    
    // Compute conductance and derivative
    if (!computeConductance(pressure)) {
        return; // Wait for the filter to be ready
    }
    _puckResistance = _puckConductance;
    // Detect puck soaking and settling
    detectPuckSoaking();
    detectPuckSettling();
    
    // Update coffee flow rate and output only when puck is settled
    if (isPuckSettled()) {
        updateCoffeeFlow();
    } else {
        // Reset coffee flow during soaking phase
        _coffeeFlowRate = _waterThroughPuckFlowRate;
    }
    
}

void CoffeeEstimator::estimatePuckInputFlow(float pumpFlowRate, float pressure, float pressureDerivative) {
    // Calculate raw water flow through the puck
    float effectiveCompliance = 4.0f / std::fmax(0.2f, pressure); // ml*s/bar
    float flowRaw = pumpFlowRate - effectiveCompliance * pressureDerivative;
    
    // Add sample to Savitzky-Golay filter
    _puckInputFlowFilter->addSample(flowRaw, _puckCounter * _dt);
    
    // Use filtered value when ready, otherwise use raw value
    if (_puckInputFlowFilter && _puckInputFlowFilter->isReady()) {
        _waterThroughPuckFlowRate = _puckInputFlowFilter->getFilteredValue();
    } 
    
    _puckSaturationVolume += std::fmax(0.0f, _waterThroughPuckFlowRate * _dt);
}

bool CoffeeEstimator::computeConductance(float pressure) {
    float currentConductance = _waterThroughPuckFlowRate / std::sqrt(pressure);
    
    _conductanceFilter->addSample(currentConductance, _puckCounter * _dt);
    // Use filtered conductance from Savitzky-Golay filter
    if (_conductanceFilter && _conductanceFilter->isReady()) {
        _puckConductance = _conductanceFilter->getFilteredValue();
        ESP_LOGD("CoffeeEstimator", "Conductance: raw=%.4f, filtered=%.4f", currentConductance, _puckConductance);
    } else {
        ESP_LOGI("CoffeeEstimator", "conductance filter not ready");
        return false;
    }
    
    // Calculate conductance derivative using the filter
    if (_conductanceFilter && _conductanceFilter->isReady()) {
        _puckConductanceDerivative = _conductanceFilter->getDerivative(_dt);
        ESP_LOGD("CoffeeEstimator", "Conductance derivative: %.4f (dt=%.4f)", _puckConductanceDerivative, _dt);
    }else {
        ESP_LOGI("CoffeeEstimator", "Derivative conductance filter not ready");
        return false;
    }
    
    return true; // Always succeed, use raw values if needed
}


void CoffeeEstimator::detectPuckSoaking() {
    float timeElapsed = static_cast<float>(_puckCounter) * _dt;
    // Only set to true if not already true (prevents toggling)
    if (!isPuckSoaking() && _puckConductanceDerivative < -0.5f && timeElapsed > 1.0f) {
        setPuckSoaking(true); // Puck is soaking up water (conductance decreasing)
    }
}

void CoffeeEstimator::detectPuckSettling() {
    // Puck is settled when:
    // 1. Puck was soaking AND conductance derivative is stabilizing (less negative)
    // 2. OR saturation volume has been reached (35ml default)
    // 3. AND puck hasn't settled yet
    bool condition1 = isPuckSoaking() && _puckConductanceDerivative > -0.2f;
    bool condition2 = _puckSaturationVolume > _puckSaturatedVolume;
    
    if ((condition1 || condition2) && !isPuckSettled()) {
        setPuckSettled(true); // Puck is now settled and ready for extraction
    }
}

void CoffeeEstimator::updateCoffeeFlow() {
    // One-time initialization when puck first settles - irreversible state
    if (isPuckSettled() && !_settledInitialized) {
        // Reset estimation from zero 
        _puckConductance = 0.0f;
        _coffeeFlowRate = 0.0f; // Initialize immediately to current flow
        _tempFlow = 0.0f; // Initialize temp flow
        _settledInitialized = true; // Irreversible flag - prevents re-initialization
        ESP_LOGI("CoffeeEstimator", "Coffee flow initialized: %.2f ml/s, Conductance: %.4f", _coffeeFlowRate, _puckConductance);
    }
    
    // Update coffee flow rate continuously when puck is settled
    if (isPuckSettled()) {
        applyLowPassFilter(&_coffeeFlowRate, _waterThroughPuckFlowRate, 0.3f, _dt);
        float deltaOutput = _coffeeFlowRate * _dt;
        _coffeeOutput += deltaOutput;
        
        ESP_LOGD("CoffeeEstimator", "Coffee: input=%.3f, filtered=%.3f, delta=%.4f, total=%.2f", 
                 _waterThroughPuckFlowRate, _coffeeFlowRate, deltaOutput, _coffeeOutput);
    }
}

void CoffeeEstimator::applyLowPassFilter(float* filteredValue, float rawValue, float cutoffFreq, float dt) {
    if (filteredValue == nullptr) return;
    
    float alpha = dt / (1.0f / (2.0f * 3.14159f * cutoffFreq) + dt);
    *filteredValue = alpha * rawValue + (1.0f - alpha) * (*filteredValue);
}

void CoffeeEstimator::setDeadVolume(float deadVol) {
    _puckSaturatedVolume = deadVol;
}

void CoffeeEstimator::reset() {
    _coffeeOutput = 0.0f;
    _coffeeFlowRate = 0.0f;
    _waterThroughPuckFlowRate = 0.0f;
    _tempFlow = 0.0f;
    _filteredPressureSensor = 0.0f;
    _puckSaturationVolume = 0.0f;
    _puckConductance = 0.0f;
    _puckConductanceDerivative = 0.0f;
    _puckResistance = 1e7f;
    setPuckSoaking(false);
    setPuckSettled(false);
    setFirstDropDetected(false);
    _puckCounter = 0;
    _inPuckVol = 0.0f;
    _settledInitialized = false;
    
    if (_conductanceFilter) {
        _conductanceFilter->reset();
    }
    if (_puckInputFlowFilter) {
        _puckInputFlowFilter->reset();
    }
}

void CoffeeEstimator::tare() {
    reset();
}
