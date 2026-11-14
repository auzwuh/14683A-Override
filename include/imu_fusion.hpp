#pragma once
// Heading fusion for two inertial sensors. No PROS includes so
// tests/imu_fusion_check.cpp can run it on a PC. The PROS side is include/dual_imu.hpp.
//
// How it works. Every update reads each sensor's raw rotation (deg, unwrapped) and
// takes the CHANGE since the last update, times that sensor's scale. The fused
// heading adds up the weighted average of those changes. Working in changes, not
// absolute readings, means a sensor can drop out mid-run without the heading
// jumping, because the other sensor just keeps adding its own changes.
//
// Rules
//  - Only sensors that finished calibrating are used. One that was missing or
//    failed at calibration stays out until the next calibration.
//  - A sensor that stops reporting mid-run (unplugged, static shock, it starts
//    recalibrating) is dropped for the rest of the run. When an IMU reconnects it
//    recalibrates while the robot moves, so its readings can no longer be trusted.
//  - Both sensors should agree within a small amount each update. If they do not,
//    one has glitched. Keep the change closest to the last fused change (the robot
//    cannot change turn rate that fast). A sensor that is the odd one out for
//    kOutlierDropTicks updates in a row is dropped.
//  - No sensor left means no heading. Odometry then sees an invalid reading, the
//    same as a single unplugged IMU.

#include <algorithm>
#include <cmath>
#include <limits>

namespace imu_fusion {

inline constexpr int kCount = 2;

struct Config {
    // Degrees reported per real degree turned, corrected. Measure each sensor on the
    // robot (see dual_imu.hpp). Negative if that sensor counts backwards.
    double scale[kCount] = {1.0, 1.0};
    // How much each sensor counts when both are working. Normalised, so {1, 1} is 50/50.
    double weight[kCount] = {1.0, 1.0};
    // Allowed disagreement in one update: this many degrees, or this share of the
    // change, whichever is bigger. Real sensors differ by about 0.01 deg per 10 ms
    // update, so these only catch glitches.
    double maxDisagreeDeg = 2.0;
    double maxDisagreeFrac = 0.05;
    int outlierDropTicks = 25;  // about 0.25 s of updates at 10 ms
};

struct Sample {
    bool valid = false;   // connected, calibrated, finite reading
    double rawDeg = 0.0;  // raw unwrapped rotation from the sensor, before scale
};

class Fuser {
  public:
    explicit Fuser(Config config = {}) : config_(config) {}

    // Call when calibration finishes, with a reading from each sensor. Sensors
    // valid now are used for this run.
    void restart(const Sample (&s)[kCount], double fusedDeg = 0.0) {
        fused_ = fusedDeg;
        lastDelta_ = 0.0;
        for (int i = 0; i < kCount; ++i) {
            inUse_[i] = s[i].valid && std::isfinite(s[i].rawDeg);
            last_[i] = s[i].rawDeg;
            outlierStreak_[i] = 0;
        }
    }

    // One update. Returns the fused rotation in degrees, or NaN once no sensor is left.
    double step(const Sample (&s)[kCount]) {
        double delta[kCount] = {0.0, 0.0};
        for (int i = 0; i < kCount; ++i) {
            if (!inUse_[i]) continue;
            if (!s[i].valid || !std::isfinite(s[i].rawDeg)) { inUse_[i] = false; continue; }
            delta[i] = config_.scale[i] * (s[i].rawDeg - last_[i]);
            last_[i] = s[i].rawDeg;
        }

        double d = 0.0;
        if (inUse_[0] && inUse_[1]) {
            const double gap = std::fabs(delta[0] - delta[1]);
            const double allowed = std::max(config_.maxDisagreeDeg,
                                             config_.maxDisagreeFrac * std::max(std::fabs(delta[0]), std::fabs(delta[1])));
            if (gap <= allowed) {
                const double w0 = std::max(0.0, config_.weight[0]), w1 = std::max(0.0, config_.weight[1]);
                d = w0 + w1 > 0.0 ? (w0 * delta[0] + w1 * delta[1]) / (w0 + w1) : 0.5 * (delta[0] + delta[1]);
                outlierStreak_[0] = outlierStreak_[1] = 0;
            } else {
                const int keep = std::fabs(delta[0] - lastDelta_) <= std::fabs(delta[1] - lastDelta_) ? 0 : 1;
                const int odd = 1 - keep;
                d = delta[keep];
                outlierStreak_[keep] = 0;
                if (++outlierStreak_[odd] >= config_.outlierDropTicks) inUse_[odd] = false;
            }
        } else if (inUse_[0]) {
            d = delta[0];
        } else if (inUse_[1]) {
            d = delta[1];
        } else {
            return std::numeric_limits<double>::quiet_NaN();
        }

        fused_ += d;
        lastDelta_ = d;
        return fused_;
    }

    void setRotation(double deg) { fused_ = deg; }
    double rotation() const { return activeCount() > 0 ? fused_ : std::numeric_limits<double>::quiet_NaN(); }
    bool inUse(int i) const { return i >= 0 && i < kCount && inUse_[i]; }
    int activeCount() const { return static_cast<int>(inUse_[0]) + static_cast<int>(inUse_[1]); }
    Config& config() { return config_; }

  private:
    Config config_;
    double fused_ = 0.0;
    double lastDelta_ = 0.0;
    double last_[kCount] = {0.0, 0.0};
    bool inUse_[kCount] = {false, false};
    int outlierStreak_[kCount] = {0, 0};
};

}  // namespace imu_fusion
