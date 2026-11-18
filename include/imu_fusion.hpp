#pragma once
// Heading fusion for two inertial sensors. No PROS includes so
// tests/imu_fusion_check.cpp can run it on a PC. The PROS side is include/dual_imu.hpp.
//
// How it works. Every update reads each sensor's raw rotation (deg, unwrapped), takes
// the CHANGE since the last update times that sensor's scale, and adds it to that
// sensor's own running total. The fused heading is the weighted average of the two
// totals. A sensor can drop out mid-run without the heading jumping, because the
// other one just keeps adding its own changes.
//
// Why totals and not the changes of one update. The two sensors sample at their own
// moments. In a fast turn one of them can show no change in one update and a double
// change in the next. Judging each update on its own ("which change looks right")
// switched between the sensors in the middle of that and counted the rotation twice
// or not at all, 2 to 4 deg per fast turn in the simulator. The average of the two
// totals cannot lose or double anything.
//
// Rules
//  - Only sensors that finished calibrating are used. One that was missing or
//    failed at calibration stays out until the next calibration.
//  - A sensor that stops reporting mid-run (unplugged, static shock, it starts
//    recalibrating) is dropped for the rest of the run. When an IMU reconnects it
//    recalibrates while the robot moves, so its readings can no longer be trusted.
//  - The two totals should stay close. A late sample pulls them apart by about one
//    update of turning, and that is allowed. If they are further apart than that, one
//    sensor has jumped. From then on the heading follows the sensor whose change
//    looked normal, until they agree again.
//  - If the jump was a one-off (both turn at the same rate again), the sensor that
//    jumped rejoins after forgiveTicks updates with the jump left out.
//  - If it keeps disagreeing for outlierDropTicks updates it is dropped.
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
    // How far apart the two totals may be: this many degrees, or this many updates of
    // turning at the current rate, whichever is bigger. One late sample is 1 update,
    // a lost one is 2.
    double maxDisagreeDeg = 2.0;
    double maxDisagreeFrac = 3.0;
    int forgiveTicks = 3;       // a one-off jump is left out after this many updates
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
        slowGap_ = agreedGap_ = lastTarget_ = normalDelta_ = 0.0;
        streak_ = follow_ = 0;
        for (int i = 0; i < kCount; ++i) {
            inUse_[i] = s[i].valid && std::isfinite(s[i].rawDeg);
            last_[i] = s[i].rawDeg;
            total_[i] = 0.0;
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
            const double w0 = std::max(0.0, config_.weight[0]), w1 = std::max(0.0, config_.weight[1]);
            const double share1 = w0 + w1 > 0.0 ? w1 / (w0 + w1) : 0.5;
            total_[0] += delta[0];
            total_[1] += delta[1];
            const double gap = total_[0] - total_[1];
            const bool sameRate = std::fabs(delta[0] - delta[1]) <= config_.maxDisagreeDeg;
            if (sameRate) normalDelta_ = delta[0] - share1 * (delta[0] - delta[1]);
            const double allowed = std::max(config_.maxDisagreeDeg,
                                             config_.maxDisagreeFrac * std::min(std::fabs(delta[0]), std::fabs(delta[1])));
            // slowGap_ follows the gap slowly, so drift and a small scale mismatch never count as a jump
            if (std::fabs(gap - slowGap_) <= allowed) {
                agreedGap_ = gap;
                slowGap_ += 0.02 * (gap - slowGap_);
                streak_ = 0;
            } else {
                // Pick who to follow once, when the disagreement starts, and stay with it.
                if (streak_ == 0)
                    follow_ = std::fabs(delta[0] - normalDelta_) <= std::fabs(delta[1] - normalDelta_) ? 0 : 1;
                const int other = 1 - follow_;
                ++streak_;
                if (sameRate && streak_ >= config_.forgiveTicks) {
                    total_[other] += (other == 1 ? 1.0 : -1.0) * (gap - agreedGap_);   // rejoin, jump left out
                    slowGap_ = agreedGap_;
                    streak_ = 0;
                } else if (streak_ >= config_.outlierDropTicks) {
                    inUse_[other] = false;
                    agreedGap_ = slowGap_;   // take back what it added just before it was caught
                }
            }
            // While they agree this is the weighted average of the two totals. While they do
            // not, it is that average as it last stood plus the followed sensor's turning since.
            const double target = streak_ > 0 && follow_ == 1 ? total_[1] + (1.0 - share1) * agreedGap_
                                                              : total_[0] - share1 * agreedGap_;
            d = target - lastTarget_;
            lastTarget_ = target;
        } else if (inUse_[0]) {
            d = delta[0];
        } else if (inUse_[1]) {
            d = delta[1];
        } else {
            return std::numeric_limits<double>::quiet_NaN();
        }

        fused_ += d;
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
    double last_[kCount] = {0.0, 0.0};
    double total_[kCount] = {0.0, 0.0};   // each sensor's own scaled rotation since restart
    double slowGap_ = 0.0;      // total_[0] - total_[1], followed slowly
    double agreedGap_ = 0.0;    // that gap at the last update the sensors agreed
    double lastTarget_ = 0.0;
    double normalDelta_ = 0.0;  // fused change at the last update both turned at the same rate
    bool inUse_[kCount] = {false, false};
    int streak_ = 0;            // updates in a row the totals have been too far apart
    int follow_ = 0;            // sensor followed during that streak
};

}  // namespace imu_fusion
