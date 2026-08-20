#include "gen/path_following/rti.hpp"

#include "optimizer.hpp"

namespace arc::path {

RTIController::RTIController(RTIConfig config) : config_(config) {}

Control RTIController::calculate(const State& state, const Horizon& reference, double dt) {
    const std::vector<Control> solution = detail::optimize(
        state.pose, previous_, reference, config_.weights, config_.limits, dt,
        1, config_.regularization, warmStart_);
    if (solution.empty()) return {};
    previous_ = solution.front();
    warmStart_ = detail::shiftWarmStart(solution, reference);
    return previous_;
}

void RTIController::reset(Control previous) {
    previous_ = previous;
    warmStart_.clear();
}

} // namespace arc::path
