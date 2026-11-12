// PC check for include/drive_mix.hpp. Run:
//   g++ -std=c++17 -I include tests/drive_mix_check.cpp -o drive_mix_check && ./drive_mix_check
#include <cassert>
#include <cstdio>
#include <cstdlib>

#include "drive_mix.hpp"

using drive_mix::mix;

int main() {
    // Centre and deadband give nothing, just past the deadband is tiny (no step)
    assert(mix(0, 0) == std::make_pair(0, 0));
    assert(mix(5, 5) == std::make_pair(0, 0));
    assert(std::abs(mix(7, 0).first) <= 2);

    // Full sticks
    assert(mix(127, 0) == std::make_pair(127, 127));
    assert(mix(-127, 0) == std::make_pair(-127, -127));
    assert(mix(0, 127) == std::make_pair(127, -127));      // point turn when stopped
    {   // full speed full right turns harder than "inner wheel stops" but still rolls forward
        const auto [l, r] = mix(127, 127);
        assert(l == 127 && r < 0 && l + r > 0);
    }

    // Full forward, sweep right. Outer stays full, inner keeps slowing (tighter with
    // more stick), the robot never stops rolling forward, and no jumps
    int prevR = 127;
    for (int turn = 0; turn <= 127; ++turn) {
        const auto [l, r] = mix(127, turn);
        assert(l == 127);
        assert(r <= prevR && l + r > 0);
        assert(prevR - r <= 6);
        prevR = r;
    }

    // Any throttle, any turn: right input turns right, outputs move smoothly
    for (int thr = -127; thr <= 127; ++thr) {
        auto prev = mix(thr, 0);
        for (int turn = 1; turn <= 127; ++turn) {
            const auto cur = mix(thr, turn);
            assert(cur.first - cur.second >= prev.first - prev.second - 1);  // turn never weakens
            assert(std::abs(cur.first - prev.first) <= 6 && std::abs(cur.second - prev.second) <= 6);
            prev = cur;
        }
    }

    // Lift soft stops, top at 115 deg
    using drive_mix::liftPower;
    assert(liftPower(127, 50, 115) == 127);            // mid stroke, full power both ways
    assert(liftPower(-127, 50, 115) == -127);
    assert(liftPower(127, 115, 115) == 0);             // at the top, up is blocked
    assert(liftPower(-127, 115, 115) == -127);         // but down is full
    assert(liftPower(-127, 0, 115) == 0);              // at the bottom, down is blocked
    assert(liftPower(127, 0, 115) == 127);             // but up is full
    assert(liftPower(-127, -2, 115) == 0);             // past the bottom stays blocked
    assert(liftPower(127, 50, NAN) == 127);            // unknown top, no top limit
    assert(liftPower(127, INFINITY, 115) == 127);      // encoder unplugged, no limits
    {   // ramps down smoothly toward the top, ending near kLiftEdgeFrac
        int prev = 127;
        for (double a = 95; a < 115; a += 0.5) {
            const int p = liftPower(127, a, 115);
            assert(p <= prev && p >= 31 && prev - p <= 4);
            prev = p;
        }
    }

    std::puts("drive_mix ok");
    return 0;
}
