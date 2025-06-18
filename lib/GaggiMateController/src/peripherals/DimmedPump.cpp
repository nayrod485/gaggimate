#include "DimmedPump.h"

#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), _psm(_sense_pin, _ssr_pin, 100, FALLING, 1, 4), _pressureSensor(pressure_sensor),
      _pressureController(0.03f, &_ctrlTarget, &_currentPressure, &_controllerPower, &_valveStatus) {
    _psm.set(0);
}

void DimmedPump::setup() {
    _cps = _psm.cps();
    if (_cps > 70) {
        _cps = _cps / 2;
    }
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void DimmedPump::loop() {
    _currentPressure = _pressureSensor->getRawPressure();
    updatePower();
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _mode = ControlMode::POWER;
    _targetPressure = setpoint == 100 ? 20.0f : 0.0f;
    _flowLimit = 0.0f;
    _power = std::clamp(setpoint, 0.0f, 100.0f);
    _psm.set(static_cast<int>(_power));
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getcoffeeOutputEstimate(); }

float DimmedPump::getFlow() { return _pressureController.getFlowPerSecond(); }

void DimmedPump::tare() {
    _pressureController.tare();
    _pressureController.reset();
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    while (true) {
        pump->loop();
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

void DimmedPump::updatePower() {
    _pressureController.update(static_cast<PressureController::ControlMode>(_mode));
    switch (_mode) {
    case ControlMode::PRESSURE:
        _power = calculatePowerForPressure(_targetPressure, _currentPressure, _flowLimit);
        ESP_LOGI("DimmedPump", "Calculating power for pressure: %.2f", _power);
        break;

    case ControlMode::FLOW:
        _power = calculatePowerForFlow(_targetFlow, _currentPressure, _pressureLimit);
        break;

    case ControlMode::POWER:
        break;
    }

    _psm.set(static_cast<int>(_power));
}

float DimmedPump::calculateFlowRate(float pressure) const {
    float flow = BASE_FLOW_RATE;
    float pressurePercentage = pressure / MAX_PRESSURE;
    return flow * pressurePercentage;
}

float DimmedPump::calculatePowerForPressure(float targetPressure, float currentPressure, float flowLimit) {
    return _controllerPower;
}

float DimmedPump::calculatePowerForFlow(float targetFlow, float currentPressure, float pressureLimit) const {
     return _controllerPower;
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlTarget = targetFlow;
    _pressureLimit = pressureLimit;
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlTarget = targetPressure;
    _flowLimit = flowLimit;
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }
