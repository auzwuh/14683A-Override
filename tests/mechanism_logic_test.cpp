#include "macros.hpp"

#include <cassert>
#include <cstring>

int main() {
    assert(robot::rollerCommand(false, false) == 0);
    assert(robot::rollerCommand(true, false) == 127);
    assert(robot::rollerCommand(false, true) == -127);
    assert(robot::rollerCommand(true, true) == 127);

    const robot::MechanismDebugText text = robot::mechanismDebugText(2, 400, 387, 91);
    assert(std::strcmp(text.cascade, "Cascade: 387 deg") == 0);
    assert(std::strcmp(text.target, "Stage 2 target: 400") == 0);
    assert(std::strcmp(text.rotator, "Claw rot: 91 deg") == 0);
}
