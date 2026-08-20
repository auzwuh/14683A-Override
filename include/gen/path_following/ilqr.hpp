#pragma once

#include "gen/path_following/types.hpp"

namespace arc::path {

struct ILQRConfig {
    std::size_t iterations = 6;
    double regularization = 1e-5;
    CostWeights weights{};
    Limits limits{};
};

class ILQRController {
  public:
    explicit ILQRController(ILQRConfig config = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    ILQRConfig config_;
    std::vector<Control> warmStart_;
    Control previous_{};
};

} // namespace arc::path
