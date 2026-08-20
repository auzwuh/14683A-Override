#include "gen/path_following/ramsete_lqr.hpp"

#include <algorithm>
#include <cmath>

namespace arc::path {

RamseteLQRController::RamseteLQRController(RamseteLQRConfig config)
    : config_(config), ramsete_(config.b, config.zeta, config.limits) {}

double RamseteLQRController::scalarLQRGain(double a, double b, double q, double r) {
    // Stabilizing solution of the scalar discrete algebraic Riccati equation.
    double p = q;
    for (int i = 0; i < 100; ++i) {
        const double denominator = r + b * b * p;
        const double next = q + a * a * p - (a * b * p) * (a * b * p) / denominator;
        if (std::abs(next - p) < 1e-12) {
            p = next;
            break;
        }
        p = next;
    }
    return (b * p * a) / (r + b * b * p);
}

Control RamseteLQRController::calculate(const State& state, const Horizon& reference, double dt) {
    if (!(dt > 0.0)) return {};
    ramsete_.reset(previous_);
    const Control desired = ramsete_.calculate(state, reference, dt);
    const double decay = std::exp(-dt / std::max(config_.velocityTimeConstant, 1e-4));
    const double input = 1.0 - decay;
    const double kv = scalarLQRGain(decay, input,
                                    1.0 / (config_.linearVelocityTolerance * config_.linearVelocityTolerance),
                                    1.0 / (config_.linearCommandTolerance * config_.linearCommandTolerance));
    const double kw = scalarLQRGain(decay, input,
                                    1.0 / (config_.angularVelocityTolerance * config_.angularVelocityTolerance),
                                    1.0 / (config_.angularCommandTolerance * config_.angularCommandTolerance));
    const Control raw{desired.linear + kv * (desired.linear - state.linearVelocity),
                      desired.angular + kw * (desired.angular - state.angularVelocity)};
    previous_ = clampControl(raw, previous_, config_.limits, dt);
    return previous_;
}

void RamseteLQRController::reset(Control previous) {
    previous_ = previous;
    ramsete_.reset(previous);
}

} // namespace arc::path
