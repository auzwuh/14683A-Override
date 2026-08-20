#pragma once

#include "gen/path_following/ramsete.hpp"

namespace arc::path {

struct RamseteLQRConfig {
    double b = 0.00129032;
    double zeta = 0.7;
    double linearVelocityTolerance = 2.0;
    double angularVelocityTolerance = 0.5;
    double linearCommandTolerance = 12.0;
    double angularCommandTolerance = 3.0;
    double velocityTimeConstant = 0.10;
    Limits limits{};
};

// RAMSETE supplies global-pose-corrected chassis velocity references. A
// discrete LQR then closes the first-order drivetrain velocity loop.
class RamseteLQRController {
  public:
    explicit RamseteLQRController(RamseteLQRConfig config = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    static double scalarLQRGain(double a, double b, double q, double r);
    RamseteLQRConfig config_;
    RamseteController ramsete_;
    Control previous_{};
};

} // namespace arc::path
