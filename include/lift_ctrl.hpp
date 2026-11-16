// lift_ctrl.hpp - the parts of the DR4B lift controller that need no robot, so they
// can be checked on a PC (tests/lift_ctrl_check.cpp). Ideas taken from Bal207/Liftlib.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace lift_ctrl {

// Mean of the readings that are real numbers, NAN if none is. An unplugged V5 motor
// reports infinity, and one bad reading must not drag the lift angle with it.
inline double meanFinite(const std::vector<double>& v) {
    double sum = 0.0;
    int n = 0;
    for (double x : v)
        if (std::isfinite(x)) { sum += x; ++n; }
    return n ? sum / n : NAN;
}

// Integral term in volts. It only builds inside zoneDeg of the target and is capped at
// maxV, so it trims the last degree or two and can not wind up on a long move.
struct Integral {
    double sum = 0.0;   // deg seconds
    double step(double errDeg, double dt, double kI, double zoneDeg, double maxV) {
        if (!(kI > 0.0) || !(std::fabs(errDeg) < zoneDeg)) return sum = 0.0;
        sum = std::clamp(sum + errDeg * dt, -maxV / kI, maxV / kI);
        return kI * sum;
    }
};

// True once inTol has held for holdMs without a break. Touching the tolerance on the
// way past does not count as settled.
struct Dwell {
    int ms = 0;
    bool step(bool inTol, int dtMs, int holdMs) {
        ms = inTol ? ms + dtMs : 0;
        return ms >= holdMs;
    }
};

}  // namespace lift_ctrl
