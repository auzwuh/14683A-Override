#pragma once

#include "gen/path_following/types.hpp"

namespace arc::path {

class LTVLQRController {
  public:
    LTVLQRController(CostWeights weights = {}, Limits limits = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    CostWeights weights_;
    Limits limits_;
    Control previous_{};
};

using TVLQRController = LTVLQRController;

} // namespace arc::path
