#pragma once

#include <cstdint>

#include "gen/path_following/types.hpp"

namespace arc::path {

struct SamplingMPCConfig {
    std::size_t samples = 128;
    std::size_t refinementPasses = 3;
    double linearNoise = 18.0;
    double angularNoise = 2.5;
    std::uint32_t seed = 0x14683u;
    CostWeights weights{};
    Limits limits{};
};

class SamplingMPCController {
  public:
    explicit SamplingMPCController(SamplingMPCConfig config = {});
    Control calculate(const State& state, const Horizon& reference, double dt);
    void reset(Control previous = {});

  private:
    double uniformSigned();
    SamplingMPCConfig config_;
    std::vector<Control> warmStart_;
    Control previous_{};
    std::uint32_t rng_;
};

using SMPCController = SamplingMPCController;

} // namespace arc::path
