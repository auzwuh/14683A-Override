#pragma once

#include "gen/path_following/types.hpp"

namespace arc::path {

class RamseteController {
  public:
    // b has units rad^2 / distance^2. 0.00129032 is the common b=2 SI
    // tuning converted for this repository's inch-based trajectories.
    RamseteController(double b = 0.00129032, double zeta = 0.7, Limits limits = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    double b_;
    double zeta_;
    Limits limits_;
    Control previous_{};
};

} // namespace arc::path
