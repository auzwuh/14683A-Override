#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>

#include "gen/path_following/all.hpp"

using namespace arc::path;

int main() {
    constexpr double pi = std::numbers::pi;
    assert(std::abs(wrapAngle(3.0 * pi) - pi) < 1e-12 ||
           std::abs(wrapAngle(3.0 * pi) + pi) < 1e-12);
    assert(std::abs(sinc(0.0) - 1.0) < 1e-12);

    const Pose quarter = stepUnicycle({}, {2.0, 1.0}, pi / 2.0);
    assert(std::abs(quarter.x - 2.0) < 1e-9);
    assert(std::abs(quarter.y - 2.0) < 1e-9);
    assert(std::abs(quarter.theta - pi / 2.0) < 1e-9);

    Limits loose;
    loose.maxLinear = 1000.0;
    loose.maxAngular = 1000.0;
    loose.maxLinearAcceleration = 1e9;
    loose.maxAngularAcceleration = 1e9;
    Horizon exact(2, {{3.0, -2.0, 0.4}, {17.0, -0.7}});
    State state{exact.front().pose, 17.0, -0.7};
    RamseteController ramsete(2.0, 0.7, loose);
    const Control output = ramsete.calculate(state, exact, 0.01);
    assert(std::abs(output.linear - 17.0) < 1e-9);
    assert(std::abs(output.angular + 0.7) < 1e-9);

    // Every implementation must return finite, bounded output on a wrapped
    // heading error near the -pi/pi discontinuity.
    Horizon wrapped(8);
    for (std::size_t i = 0; i < wrapped.size(); ++i)
        wrapped[i] = Reference{{static_cast<double>(i), 0.0, -pi + 0.01}, {10.0, 0.0}};
    State displaced{{0.0, 1.0, pi - 0.01}, 0.0, 0.0};
    SamplingMPCController smpc;
    ILQRController ilqr;
    RTIController rti;
    LTVLQRController ltv;
    RamseteLQRController hybrid;
    for (Control value : {smpc.calculate(displaced, wrapped, 0.05),
                          ilqr.calculate(displaced, wrapped, 0.05),
                          rti.calculate(displaced, wrapped, 0.05),
                          ltv.calculate(displaced, wrapped, 0.05),
                          hybrid.calculate(displaced, wrapped, 0.05)}) {
        assert(std::isfinite(value.linear));
        assert(std::isfinite(value.angular));
        assert(std::abs(value.linear) <= 60.0 + 1e-9);
        assert(std::abs(value.angular) <= 8.0 + 1e-9);
    }

    std::cout << "path_following_math_test: PASS\n";
}
