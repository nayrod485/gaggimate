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
    // Filter configuration structure
    struct FilterConfig {
        FilterFactory::FilterType conductanceFilterType;
        FilterFactory::FilterType puckInputFlowFilterType;
        int conductanceWindowSize;
        int conductancePolynomialOrder;
        int puckInputFlowWindowSize;
        int puckInputFlowPolynomialOrder;
        
        // Default constructor with sensible defaults
        FilterConfig() : 
            conductanceFilterType(FilterFactory::LINEAR_REGRESSION),
            conductanceWindowSize(30),
            conductancePolynomialOrder(2),

            puckInputFlowFilterType(FilterFactory::LINEAR_REGRESSION),
            puckInputFlowWindowSize(50),
            puckInputFlowPolynomialOrder(2) {}
        
    };
    
    CoffeeEstimator(float dt, const FilterConfig& config);
    
    // Main estimation update
    void update(float pumpFlowRate, float pressure, float pressureDerivative, int valveStatus);
    
    // Configuration
    void setDeadVolume(float deadVol);
    void reset();
    void tare();
    
    // Filter configuration tuning (if needed)
    void setConductanceFilterWindow(int windowSize) { const_cast<FilterConfig&>(_filterConfig).conductanceWindowSize = windowSize; }
    
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
    const FilterConfig _filterConfig;
    
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
