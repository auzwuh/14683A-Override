#include "gen/damp/measurement.hpp"
#include <cmath>
namespace arc::damp::measurement {
Motion Tracker::update(Sample sample, double forwardOffset, double leftOffset) {
    if (!sample.valid || !std::isfinite(sample.forwardDistance) ||
        !std::isfinite(sample.leftDistance) || !std::isfinite(sample.clockwiseHeading) ||
        !std::isfinite(forwardOffset) || !std::isfinite(leftOffset)) {
        reset();
        return {};
    }
    const auto elapsed = sample.timestampMs - previous.timestampMs;
    if (!initialized || elapsed == 0 || elapsed > 0x7fffffffu) {
        previous = sample;
        initialized = true;
        return {};
    }
    Motion result;
    result.dtSeconds = elapsed * .001;
    const double turn = sample.clockwiseHeading - previous.clockwiseHeading;
    result.forwardVelocity = (sample.forwardDistance - previous.forwardDistance + forwardOffset * turn) /
                             result.dtSeconds;
    result.leftVelocity = (sample.leftDistance - previous.leftDistance + leftOffset * turn) /
                          result.dtSeconds;
    result.clockwiseAngularVelocity = turn / result.dtSeconds;
    result.valid = std::isfinite(result.forwardVelocity) && std::isfinite(result.leftVelocity) &&
                   std::isfinite(result.clockwiseAngularVelocity);
    previous = sample;
    if (!result.valid) { reset(); return {}; }
    return result;
}
void Tracker::reset() { initialized = false; }
Displacement fieldDisplacement(double forward, double left, double heading, double deltaHeading) {
    const double midpoint = heading + deltaHeading / 2;
    const double chord = std::abs(deltaHeading) < 1e-9 ? 1 : 2 * std::sin(deltaHeading / 2) / deltaHeading;
    return {chord * (forward * std::sin(midpoint) - left * std::cos(midpoint)),
            chord * (forward * std::cos(midpoint) + left * std::sin(midpoint))};
}
}
