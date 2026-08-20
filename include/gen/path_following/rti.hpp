#pragma once

#include "gen/path_following/types.hpp"

namespace arc::path {

struct RTIConfig {
    double regularization = 1e-4;
    CostWeights weights{};
    Limits limits{};
};

// One Gauss-Newton/SQP-style trajectory update per sampling instant, warm
// started from the shifted solution at the previous instant.
class RTIController {
  public:
    explicit RTIController(RTIConfig config = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    RTIConfig config_;
    std::vector<Control> warmStart_;
    Control previous_{};
};

} // namespace arc::path
