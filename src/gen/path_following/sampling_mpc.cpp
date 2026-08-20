#include "gen/path_following/sampling_mpc.hpp"

#include <algorithm>
#include <limits>

namespace arc::path {

SamplingMPCController::SamplingMPCController(SamplingMPCConfig config)
    : config_(config), rng_(config.seed ? config.seed : 1u) {}

double SamplingMPCController::uniformSigned() {
    // Deterministic xorshift32: repeatable benchmarks and no heavyweight RNG.
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return 2.0 * (static_cast<double>(rng_) / 4294967295.0) - 1.0;
}

Control SamplingMPCController::calculate(const State& state, const Horizon& reference, double dt) {
    if (reference.size() < 2 || !(dt > 0.0)) return {};
    const std::size_t count = reference.size() - 1;
    std::vector<Control> nominal(count);
    for (std::size_t i = 0; i < count; ++i)
        nominal[i] = i < warmStart_.size() ? warmStart_[i] : reference[i].velocity;

    Control prior = previous_;
    for (Control& input : nominal) {
        input = clampControl(input, prior, config_.limits, dt);
        prior = input;
    }
    double bestCost = trajectoryCost(state.pose, nominal, reference, config_.weights, previous_, dt);
    std::vector<Control> best = nominal;

    for (std::size_t pass = 0; pass < config_.refinementPasses; ++pass) {
        const double scale = 1.0 / static_cast<double>(1u << pass);
        for (std::size_t sample = 0; sample < config_.samples; ++sample) {
            std::vector<Control> candidate = best;
            Control candidatePrior = previous_;
            double filteredLinear = 0.0;
            double filteredAngular = 0.0;
            for (Control& input : candidate) {
                // Low-pass-correlated perturbations explore smooth command
                // sequences much more efficiently than independent white noise.
                filteredLinear = 0.65 * filteredLinear + 0.35 * uniformSigned();
                filteredAngular = 0.65 * filteredAngular + 0.35 * uniformSigned();
                input.linear += scale * config_.linearNoise * filteredLinear;
                input.angular += scale * config_.angularNoise * filteredAngular;
                input = clampControl(input, candidatePrior, config_.limits, dt);
                candidatePrior = input;
            }
            const double candidateCost = trajectoryCost(state.pose, candidate, reference,
                                                        config_.weights, previous_, dt);
            if (candidateCost < bestCost) {
                bestCost = candidateCost;
                best = std::move(candidate);
            }
        }
    }

    previous_ = best.front();
    warmStart_.resize(count);
    for (std::size_t i = 0; i + 1 < count; ++i) warmStart_[i] = best[i + 1];
    warmStart_.back() = reference.back().velocity;
    return previous_;
}

void SamplingMPCController::reset(Control previous) {
    previous_ = previous;
    warmStart_.clear();
    rng_ = config_.seed ? config_.seed : 1u;
}

} // namespace arc::path
