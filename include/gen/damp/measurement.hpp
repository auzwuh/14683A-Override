#pragma once
#include <cstdint>

namespace arc::damp::measurement {
struct Sample {
    double forwardDistance, leftDistance, clockwiseHeading;
    std::uint32_t timestampMs;
    bool valid;
};
struct Motion {
    double forwardVelocity = 0, leftVelocity = 0, clockwiseAngularVelocity = 0;
    double dtSeconds = 0;
    bool valid = false;
};
struct Displacement { double x, y; };
// Integrates body arc distances using the midpoint heading and chord correction.
Displacement fieldDisplacement(double forward, double left, double heading, double deltaHeading);
class Tracker {
public:
    Motion update(Sample sample, double forwardOffset, double leftOffset);
    void reset();
private:
    Sample previous{};
    bool initialized = false;
};
}
