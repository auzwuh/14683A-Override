#pragma once

namespace arc {

struct AsymptoticGains {
    float initial = 0.0f;
    float final = 0.0f;
    float scale = 1.0f;
    float power = 1.0f;

    float at(float setpoint) const;
};

enum class GainScheduleMode {
    // Select Kp once from the motion's requested setpoint.
    MotionSetpoint,
    // Continuously select Kp from a low-pass-filtered current error magnitude.
    FilteredCurrentError
};

struct PIDConfig {
    AsymptoticGains proportional{};
    GainScheduleMode scheduleMode = GainScheduleMode::MotionSetpoint;
    // Exponential filter weights in [0, 1]. A value of 1 disables smoothing.
    float scheduleAlpha = 1.0f;
    float derivativeAlpha = 1.0f;
    float kI = 0.0f;
    float kD = 0.0f;
    float integralRange = 0.0f;
    bool integralSignReset = false;
};

class PID {
    public:
        explicit PID(PIDConfig config = {});

        float update(float error);
        void reset(float initialError = 0.0f);
        void setTarget(float setpoint);

        float getError() const;
        float getProportionalGain() const;

    private:
        PIDConfig config;
        float proportionalGain = 0.0f;
        float error = 0.0f;
        float integral = 0.0f;
        float previousError = 0.0f;
        float filteredScheduleMagnitude = 0.0f;
        float filteredDerivative = 0.0f;
};

} // namespace arc
