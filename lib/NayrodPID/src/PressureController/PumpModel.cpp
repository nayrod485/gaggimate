// PumpModel.cpp - Pump flow modeling implementation
#include "PumpModel.h"
#include <algorithm>
#include <cmath>

PumpModel::PumpModel(float dt) : _dt(dt) {}

void PumpModel::setFlowCoefficients(float a, float b, float c, float d) {
    _coefficients[0] = a;
    _coefficients[1] = b;
    _coefficients[2] = c;
    _coefficients[3] = d;
}

void PumpModel::setFlowCoeff(float oneBarFlow, float nineBarFlow) {
    // Set the affine pump flow model coefficients based on flow measurement at 1 bar and 9 bar
    _coefficients[0] = 0.0f;
    _coefficients[1] = 0.0f;
    _coefficients[2] = (nineBarFlow - oneBarFlow) / 8;
    _coefficients[3] = oneBarFlow - _coefficients[2] * 1.0f;
}

void PumpModel::update(float pressure, float dutyCycle) {
    // Calculate raw flow rate
    const float availableFlow = getAvailableFlow(pressure);
    const float rawFlowRate = availableFlow * dutyCycle / 100.0f;
    
    // Apply low-pass filter to smooth the flow rate
    float alpha = _dt / (1.0f / (2.0f * 3.14159f * _filterFrequency) + _dt);
    _pumpFlowRate = alpha * rawFlowRate + (1.0f - alpha) * _pumpFlowRate;
    _pumpVolume += _pumpFlowRate * _dt;
    _exportFlowRate = _pumpFlowRate;
}

float PumpModel::getAvailableFlow(float pressure) const {
    const float P = std::fmax(0.0f, pressure); // Ensure non-negative pressure
    const float P2 = P * P;
    const float P3 = P2 * P;
    const float Q = _coefficients[0] * P3 + _coefficients[1] * P2 + 
                    _coefficients[2] * P + _coefficients[3];
    return std::fmax(0.0f, Q); // Ensure non-negative flow
}

void PumpModel::reset() {
    _pumpFlowRate = 0.0f;
    _exportFlowRate = 0.0f;
    _pumpVolume = 0.0f;
}
