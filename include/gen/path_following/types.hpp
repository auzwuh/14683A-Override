#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace arc::path {

// All controller math uses the conventional unicycle frame: +x forward at
// theta=0, +y to the left, theta positive counter-clockwise, radians/seconds.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct State {
    Pose pose{};
    double linearVelocity = 0.0;
    double angularVelocity = 0.0;
};

struct Control {
    double linear = 0.0;
    double angular = 0.0;
};

struct Reference {
    Pose pose{};
    Control velocity{};
};

using Horizon = std::vector<Reference>;

struct Limits {
    double maxLinear = 60.0;
    double maxAngular = 8.0;
    double maxLinearAcceleration = 180.0;
    double maxAngularAcceleration = 30.0;
};

struct CostWeights {
    // Inverse-square tolerances: roughly 2.5 in, 5 in, 2 rad for pose and
    // 1 in/s, 2 rad/s for command deviations.
    std::array<double, 3> state{0.16, 0.04, 0.25};
    std::array<double, 2> control{1.0, 0.25};
    std::array<double, 2> controlRate{0.02, 0.02};
    std::array<double, 3> terminal{1.6, 0.4, 2.5};
};

double wrapAngle(double angle);
double sinc(double value);
Pose stepUnicycle(const Pose& pose, const Control& control, double dt);
std::array<double, 3> poseError(const Pose& pose, const Pose& reference);
Control clampControl(const Control& requested, const Control& previous, const Limits& limits, double dt);
double trajectoryCost(const Pose& initial, const std::vector<Control>& controls,
                      const Horizon& reference, const CostWeights& weights,
                      const Control& previous, double dt);

} // namespace arc::path
