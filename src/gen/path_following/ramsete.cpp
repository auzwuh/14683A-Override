#include "gen/path_following/ramsete.hpp"

#include <cmath>
#include <stdexcept>

namespace arc::path {

RamseteController::RamseteController(double b, double zeta, Limits limits)
    : b_(b), zeta_(zeta), limits_(limits) {
    if (!(b > 0.0) || !(zeta > 0.0 && zeta < 1.0)) {
        throw std::invalid_argument("RAMSETE requires b > 0 and 0 < zeta < 1");
    }
}

Control RamseteController::calculate(const State& state, const Horizon& reference, double dt) {
    if (reference.empty()) return {};
    const Reference& desired = reference.front();
    const double dx = desired.pose.x - state.pose.x;
    const double dy = desired.pose.y - state.pose.y;
    const double c = std::cos(state.pose.theta);
    const double s = std::sin(state.pose.theta);
    const double ex = c * dx + s * dy;
    const double ey = -s * dx + c * dy;
    const double etheta = wrapAngle(desired.pose.theta - state.pose.theta);
    const double vr = desired.velocity.linear;
    const double wr = desired.velocity.angular;
    const double k = 2.0 * zeta_ * std::sqrt(wr * wr + b_ * vr * vr);
    const Control raw{vr * std::cos(etheta) + k * ex,
                      wr + k * etheta + b_ * vr * sinc(etheta) * ey};
    previous_ = clampControl(raw, previous_, limits_, dt);
    return previous_;
}

void RamseteController::reset(Control previous) { previous_ = previous; }

} // namespace arc::path
