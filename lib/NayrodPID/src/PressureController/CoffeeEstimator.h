// CoffeeEstimator.h - Coffee output and puck behavior estimation
#ifndef COFFEE_ESTIMATOR_H
#define COFFEE_ESTIMATOR_H

#include "ImprovedFiltering.h"
#include <memory>

// Puck state enumeration for better readability
enum class PuckState : int {
    SOAKING = 0,    // Water is soaking into the puck
    SETTLED = 1,    // Puck is settled and ready for extraction
    FIRST_DROP = 2  // First drop detected
};

/**
 * CoffeeEstimator - Estimates coffee output and analyzes puck behavior
 * Handles the virtual scale functionality and puck analysis
 */
class CoffeeEstimator {
public:
    // CoffeeEstimator-specific filter configuration
    struct CoffeeFilterConfig {
        ::FilterConfig conductanceFilter;  // For conductance smoothing and derivatives
        ::FilterConfig puckInputFlowFilter; // For puck input flow smoothing
        
        // Default constructor - uses Linear Regression (recommended)
        CoffeeFilterConfig() :
            conductanceFilter(::FilterConfig::LINEAR_REGRESSION, 30, 2),
            puckInputFlowFilter(::FilterConfig::LINEAR_REGRESSION, 50, 2) {}
        
        // Constructor to use same filter type for both
        CoffeeFilterConfig(::FilterConfig::FilterType filterType) :
            conductanceFilter(filterType),
            puckInputFlowFilter(filterType) {}
        
        // Full constructor for custom configuration
        CoffeeFilterConfig(const ::FilterConfig& condConfig, const ::FilterConfig& flowConfig) :
            conductanceFilter(condConfig),
            puckInputFlowFilter(flowConfig) {}
    };
    
    CoffeeEstimator(float dt, const CoffeeFilterConfig& config);
    
    // Main estimation update
    void update(float pumpFlowRate, float pressure, float pressureDerivative, int valveStatus);
    
    // Configuration
    void setDeadVolume(float deadVol);
    void reset();
    void tare();
    
    // Filter configuration tuning (if needed)
    void setConductanceFilterWindow(int windowSize) { const_cast<::FilterConfig&>(_filterConfig.conductanceFilter).windowSize = windowSize; }
    
    // Getters
    float getCoffeeOutput() const { return _coffeeOutput; }
    float getCoffeeFlowRate() const { return _coffeeFlowRate; }
    float getPuckResistance() const { return _puckConductance; }
    float getWaterThroughPuckFlowRate() const { return _waterThroughPuckFlowRate; }
    
    // Puck analysis state
    int getPuckCounter() const { return _puckCounter; }
    float getPuckConductanceDerivative() const { return _puckConductanceDerivative; }

private:
    // Flow estimation
    void estimatePuckInputFlow(float pumpFlowRate, float pressure, float pressureDerivative);
    std::unique_ptr<DerivativeFilter> _puckInputFlowFilter;
    
    // Puck analysis
    bool computeConductance(float pressure);
    void detectPuckSoaking();
    void detectPuckSettling();
    void updateCoffeeFlow();
    
    // Puck state helpers
    bool isPuckSoaking() const { return _puckState[static_cast<int>(PuckState::SOAKING)]; }
    bool isPuckSettled() const { return _puckState[static_cast<int>(PuckState::SETTLED)]; }
    bool isFirstDropDetected() const { return _puckState[static_cast<int>(PuckState::FIRST_DROP)]; }
    void setPuckSoaking(bool state) { _puckState[static_cast<int>(PuckState::SOAKING)] = state; }
    void setPuckSettled(bool state) { _puckState[static_cast<int>(PuckState::SETTLED)] = state; }
    void setFirstDropDetected(bool state) { _puckState[static_cast<int>(PuckState::FIRST_DROP)] = state; }
    
    // Helper functions
    void applyLowPassFilter(float* filteredValue, float rawValue, float cutoffFreq, float dt);
    
    const float _dt;
    const CoffeeFilterConfig _filterConfig;
    
    // Flow estimation
    float _waterThroughPuckFlowRate = 0.0f;
    float _coffeeOutput = 0.0f;
    float _coffeeFlowRate = 0.0f;
    float _tempFlow = 0.0f;  // Intermediate filtering variable
    float _filteredPressureSensor = 0.0f;  // Current pressure for calculations
    float _puckSaturationVolume = 0.0f;
    float _puckSaturatedVolume = 35.0f;
    
    // Puck analysis
    std::unique_ptr<DerivativeFilter> _conductanceFilter;
    float _puckConductance = 0.0f;
    float _puckConductanceDerivative = 0.0f;
    float _puckResistance = 1e7f;
    bool _puckState[3] = {}; // TODO: Replace with enum-based approach
    int _puckCounter = 0;
    float _inPuckVol = 0.0f;
    bool _settledInitialized = false;  // Irreversible flag - once true, never false until reset
};

#endif // COFFEE_ESTIMATOR_H
