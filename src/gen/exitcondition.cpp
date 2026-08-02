#include <cmath>

#include "gen/exitcondition.hpp"
#include "gen/util.hpp"
#include "pros/rtos.hpp"

namespace gen {

bool ExitCondition::error(float currentError, float threshold) {
    return threshold >= 0.0f && std::fabs(currentError) > threshold;
}

bool ExitCondition::velocity(float currentVelocity, float threshold) {
    return threshold >= 0.0f && std::fabs(currentVelocity) > threshold;
}

bool ExitCondition::time(std::uint32_t startTime, int timeoutMs) {
    return timeoutMs < 0 || pros::millis() - startTime < static_cast<std::uint32_t>(timeoutMs);
}

bool ExitCondition::halfPlane(const Pose& pose, const Pose& target, float headingDeg, float tolerance) {
    const float heading = degToRad(headingDeg);
    return (pose.y - target.y) * -std::cos(heading) >=
           std::sin(heading) * (pose.x - target.x) + tolerance;
}

Settler::Settler(std::function<bool()> condition) : condition(std::move(condition)) {}

Settler Settler::operator&(const Settler& other) const {
    const Settler left = *this;
    const Settler right = other;
    return Settler([left, right] { return left.isSettled() && right.isSettled(); });
}

Settler Settler::operator|(const Settler& other) const {
    const Settler left = *this;
    const Settler right = other;
    return Settler([left, right] { return left.isSettled() || right.isSettled(); });
}

bool Settler::isSettled() const { return condition(); }

} // namespace gen
