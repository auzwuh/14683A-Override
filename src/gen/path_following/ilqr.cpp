#include "gen/path_following/ilqr.hpp"

#include "optimizer.hpp"

namespace arc::path {

ILQRController::ILQRController(ILQRConfig config) : config_(config) {}

Control ILQRController::calculate(const State& state, const Horizon& reference, double dt) {
    const std::vector<Control> solution = detail::optimize(
        state.pose, previous_, reference, config_.weights, config_.limits, dt,
        config_.iterations, config_.regularization, warmStart_);
    if (solution.empty()) return {};
    previous_ = solution.front();
    warmStart_ = detail::shiftWarmStart(solution, reference);
    return previous_;
}

void ILQRController::reset(Control previous) {
    previous_ = previous;
    warmStart_.clear();
}

} // namespace arc::path
