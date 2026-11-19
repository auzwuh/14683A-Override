// Replaces src/GenSelector/selector.cpp (needs LVGL). main.cpp only constructs the
// global autonSelector and never calls start(), so this is the whole surface.
#include "GenSelector/selector.hpp"
robot::AutonSelector::AutonSelector(SelectorConfig config, AutonRoutineList routines)
    : config_(std::move(config)), routines_(std::move(routines)) {}
