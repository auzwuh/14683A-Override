// PC check for include/lift_ctrl.hpp. Run:
//   g++ -std=c++17 -I include tests/lift_ctrl_check.cpp -o lift_ctrl_check && ./lift_ctrl_check
#include <cassert>
#include <cstdio>

#include "lift_ctrl.hpp"

using namespace lift_ctrl;

int main() {
    // Position: an unplugged motor (infinity) is left out, nothing left gives NAN
    assert(meanFinite({40.0, 44.0}) == 42.0);
    assert(meanFinite({40.0, INFINITY}) == 40.0);
    assert(meanFinite({NAN, 44.0}) == 44.0);
    assert(std::isnan(meanFinite({INFINITY, INFINITY})));
    assert(std::isnan(meanFinite({})));

    // Integral: off when kI is 0, off outside the zone, builds inside it, capped at maxV
    Integral i;
    assert(i.step(1.0, 0.01, 0.0, 3.0, 2.0) == 0.0);
    assert(i.step(10.0, 0.01, 4.0, 3.0, 2.0) == 0.0);              // 10 deg off, outside the 3 deg zone
    for (int k = 0; k < 10; ++k) i.step(1.0, 0.01, 4.0, 3.0, 2.0);  // 1 deg for 0.1 s
    assert(std::fabs(i.step(1.0, 0.01, 4.0, 3.0, 2.0) - 4.0 * 0.11) < 1e-9);
    for (int k = 0; k < 1000; ++k) i.step(2.0, 0.01, 4.0, 3.0, 2.0);
    assert(std::fabs(i.step(2.0, 0.01, 4.0, 3.0, 2.0) - 2.0) < 1e-9);   // capped at 2 V
    assert(i.step(5.0, 0.01, 4.0, 3.0, 2.0) == 0.0 && i.sum == 0.0);     // left the zone, cleared
    for (int k = 0; k < 1000; ++k) i.step(-2.0, 0.01, 4.0, 3.0, 2.0);
    assert(std::fabs(i.step(-2.0, 0.01, 4.0, 3.0, 2.0) + 2.0) < 1e-9);  // and the other way

    // Settle: passing through the tolerance does not count, staying in it for 60 ms does
    Dwell d;
    assert(!d.step(true, 10, 60));
    assert(!d.step(false, 10, 60));          // left again, the count restarts
    for (int k = 0; k < 5; ++k) assert(!d.step(true, 10, 60));
    assert(d.step(true, 10, 60));            // sixth loop in a row
    assert(Dwell{}.step(true, 10, 0));       // 0 ms hold = the old behaviour

    std::puts("lift_ctrl ok");
    return 0;
}
