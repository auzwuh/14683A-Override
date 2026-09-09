#include "gen/damp/measurement.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace arc::damp::measurement;
static void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-8); }
int main() {
    // Catches integrating cumulative startup positions and assuming a fixed 10 ms loop.
    Tracker tracker;
    assert(!tracker.update({100, -20, 2, 1000, true}, 3, -4).valid);
    auto measured = tracker.update({99, -18, 2, 1020, true}, 3, -4);
    assert(measured.valid);
    near(measured.forwardVelocity, -50);
    near(measured.leftVelocity, 100);
    near(measured.dtSeconds, .02);
    // A clockwise 0.1 rad turn moves the offset pods -0.3 and +0.4 inches.
    measured = tracker.update({98.7, -17.6, 2.1, 1070, true}, 3, -4);
    near(measured.forwardVelocity, 0);
    near(measured.leftVelocity, 0);
    near(measured.clockwiseAngularVelocity, 2);
    // Compass north maps forward to +Y and left to -X; east maps left to +Y.
    auto field = fieldDisplacement(2, 3, 0, 0);
    near(field.x, -3); near(field.y, 2);
    field = fieldDisplacement(2, 3, 1.5707963267948966, 0);
    near(field.x, 2); near(field.y, 3);
    // A quarter-circle of radius one ends one inch east and one inch north.
    field = fieldDisplacement(1.5707963267948966, 0, 0, 1.5707963267948966);
    near(field.x, 1); near(field.y, 1);
    // Fault recovery and explicit resets must rebaseline without synthetic motion.
    assert(!tracker.update({0, 0, 0, 1080, false}, 3, -4).valid);
    assert(!tracker.update({0, 0, 0, 1090, true}, 3, -4).valid);
    assert(tracker.update({0, 0, 0, 1100, true}, 3, -4).valid);
    tracker.reset();
    assert(!tracker.update({300, 400, 10, 1110, true}, 3, -4).valid);
    measured = tracker.update({300, 400, 10, 1120, true}, 3, -4);
    near(measured.forwardVelocity, 0); near(measured.leftVelocity, 0);
    assert(!tracker.update({300, 400, 10, 1120, true}, 3, -4).valid);
    assert(!tracker.update({NAN, 400, 10, 1130, true}, 3, -4).valid);
    assert(!tracker.update({300, 400, 10, 1140, true}, INFINITY, -4).valid);
    tracker.update({0, 0, 0, 2000, true}, 0, 0);
    assert(!tracker.update({1, 0, 0, 1990, true}, 0, 0).valid);
    tracker.reset();
    tracker.update({0, 0, 0, 0xfffffff5u, true}, 0, 0);
    measured = tracker.update({.02, 0, 0, 9, true}, 0, 0);
    assert(measured.valid); near(measured.forwardVelocity, 1);
    std::cout << "DAMP measurement tests passed\n";
}
