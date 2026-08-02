#include <algorithm>
#include <cmath>

#include "gen/pid.hpp"

namespace gen {

float AsymptoticGains::at(float setpoint) const {
    const float magnitude = std::pow(std::fabs(setpoint), power);
    const float scaleMagnitude = std::pow(std::fabs(scale), power);
    const float denominator = magnitude + scaleMagnitude;
    if (denominator == 0.0f) return initial;
    return (final - initial) * magnitude / denominator + initial;
}

PID::PID(PIDConfig config)
    : config(config), proportionalGain(config.proportional.at(0.0f)) {}

float PID::update(float nextError) {
    error = nextError;

    if (config.scheduleMode == GainScheduleMode::FilteredCurrentError) {
        const float alpha = std::clamp(config.scheduleAlpha, 0.0f, 1.0f);
        filteredScheduleMagnitude += alpha * (std::fabs(error) - filteredScheduleMagnitude);
        proportionalGain = config.proportional.at(filteredScheduleMagnitude);
    }

    if (std::fabs(error) > config.integralRange) {
        integral = 0.0f;
    } else if (config.integralSignReset && std::signbit(error) != std::signbit(previousError)) {
        integral = 0.0f;
    } else {
        integral += error * 0.01f;
    }

    const float rawDerivative = (error - previousError) * 100.0f;
    const float derivativeAlpha = std::clamp(config.derivativeAlpha, 0.0f, 1.0f);
    filteredDerivative += derivativeAlpha * (rawDerivative - filteredDerivative);
    previousError = error;
    return proportionalGain * error + config.kI * integral + config.kD * filteredDerivative;
}

void PID::reset(float initialError) {
    error = initialError;
    integral = 0.0f;
    previousError = initialError;
    filteredScheduleMagnitude = std::fabs(initialError);
    filteredDerivative = 0.0f;
    if (config.scheduleMode == GainScheduleMode::FilteredCurrentError) {
        proportionalGain = config.proportional.at(filteredScheduleMagnitude);
    }
}

void PID::setTarget(float setpoint) {
    if (config.scheduleMode == GainScheduleMode::MotionSetpoint) {
        proportionalGain = config.proportional.at(setpoint);
    }
}

float PID::getError() const { return error; }

float PID::getProportionalGain() const { return proportionalGain; }

} // namespace gen
