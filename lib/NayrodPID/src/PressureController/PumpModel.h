// PumpModel.h - Dedicated pump flow modeling
#ifndef PUMP_MODEL_H
#define PUMP_MODEL_H

#include <array>

/**
 * PumpModel - Models pump flow characteristics
 * Handles the "crappy but functional" pump model for flow estimation
 */
class PumpModel {
public:
    PumpModel(float dt);
    
    // Configuration
    void setFlowCoefficients(float a, float b, float c, float d);
    void setFlowCoeff(float oneBarFlow, float nineBarFlow);
    
    // Flow calculation and state update
    void update(float pressure, float dutyCycle);
    float getAvailableFlow(float pressure) const;
    void reset();
    
    // Getters
    float getPumpVolume() const { return _pumpVolume; }
    float getPumpFlowRate() const { return _pumpFlowRate; }
    float getExportFlowRate() const { return _exportFlowRate; }

private:
    const float _dt;
    std::array<float, 4> _coefficients = {0.0f, 0.0f, -0.5854f, 10.79f};
    float _pumpFlowRate = 0.0f;
    float _exportFlowRate = 0.0f;
    float _pumpVolume = 0.0f;
    float _filterFrequency = 1.0f;
};

#endif // PUMP_MODEL_H
