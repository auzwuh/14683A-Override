#include "gen/path_following/types.hpp"

#include <algorithm>
#include <limits>
#include <numbers>

namespace arc::path {

double wrapAngle(double angle) { return std::remainder(angle, 2.0 * std::numbers::pi); }

double sinc(double value) {
    if (std::abs(value) < 1e-4) {
        const double square = value * value;
        return 1.0 - square / 6.0 + square * square / 120.0;
    }
    return std::sin(value) / value;
}

Pose stepUnicycle(const Pose& pose, const Control& control, double dt) {
    // Exact zero-order-hold integration avoids Euler curvature bias.
    Pose next = pose;
    const double turn = control.angular * dt;
    if (std::abs(control.angular) < 1e-8) {
        next.x += control.linear * std::cos(pose.theta) * dt;
        next.y += control.linear * std::sin(pose.theta) * dt;
    } else {
        const double radius = control.linear / control.angular;
        next.x += radius * (std::sin(pose.theta + turn) - std::sin(pose.theta));
        next.y -= radius * (std::cos(pose.theta + turn) - std::cos(pose.theta));
    }
    next.theta = wrapAngle(pose.theta + turn);
    return next;
}

std::array<double, 3> poseError(const Pose& pose, const Pose& reference) {
    return {pose.x - reference.x, pose.y - reference.y, wrapAngle(pose.theta - reference.theta)};
}

Control clampControl(const Control& requested, const Control& previous, const Limits& limits, double dt) {
    Control result;
    result.linear = std::clamp(requested.linear, -limits.maxLinear, limits.maxLinear);
    result.angular = std::clamp(requested.angular, -limits.maxAngular, limits.maxAngular);
    if (dt > 0.0) {
        result.linear = std::clamp(result.linear,
                                   previous.linear - limits.maxLinearAcceleration * dt,
                                   previous.linear + limits.maxLinearAcceleration * dt);
        result.angular = std::clamp(result.angular,
                                    previous.angular - limits.maxAngularAcceleration * dt,
                                    previous.angular + limits.maxAngularAcceleration * dt);
    }
    return result;
}

double trajectoryCost(const Pose& initial, const std::vector<Control>& controls,
                      const Horizon& reference, const CostWeights& weights,
                      const Control& previous, double dt) {
    if (reference.empty()) return std::numeric_limits<double>::infinity();
    Pose pose = initial;
    Control prior = previous;
    double result = 0.0;
    const std::size_t count = std::min(controls.size(), reference.size() - 1);
    for (std::size_t index = 0; index < count; ++index) {
        const auto error = poseError(pose, reference[index].pose);
        const double dv = controls[index].linear - reference[index].velocity.linear;
        const double dw = controls[index].angular - reference[index].velocity.angular;
        const double drv = controls[index].linear - prior.linear;
        const double drw = controls[index].angular - prior.angular;
        for (std::size_t i = 0; i < 3; ++i) result += weights.state[i] * error[i] * error[i];
        result += weights.control[0] * dv * dv + weights.control[1] * dw * dw;
        result += weights.controlRate[0] * drv * drv + weights.controlRate[1] * drw * drw;
        pose = stepUnicycle(pose, controls[index], dt);
        prior = controls[index];
    }
    const auto terminalError = poseError(pose, reference[std::min(count, reference.size() - 1)].pose);
    for (std::size_t i = 0; i < 3; ++i) result += weights.terminal[i] * terminalError[i] * terminalError[i];
    return result;
}

} // namespace arc::path
