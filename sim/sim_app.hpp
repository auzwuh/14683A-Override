// sim_app.hpp - shared between sim_main.cpp and sim_selftest.cpp.
#pragma once
#include "world.hpp"

namespace simapp {
struct TraceRow { double t, x, y, thDeg, v, wDeg, liftDeg; };   // true state, in, deg, in/s, deg/s
extern std::vector<TraceRow> trace;   // filled every ms by the kernel hook while traceOn
extern bool traceOn;
extern int motionTimeouts;            // motions that ended because ExitCondition::time() expired
extern int motionsEnded;
[[noreturn]] void exitNow(int code);
void runSelftest();                   // body of the "selftest" task
}  // namespace simapp
