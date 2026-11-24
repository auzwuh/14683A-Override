// Replaces src/GenSelector/selector.cpp (needs LVGL). main.cpp only constructs the
// global autonSelector and never calls start(), so this is the whole surface.
#include "GenSelector/selector.hpp"
robot::AutonSelector::AutonSelector(SelectorConfig config, AutonRoutineList routines)
    : config_(std::move(config)), routines_(std::move(routines)) {}
// The sim has no screen: SIM_AUTON=<row> picks the routine (0 = Do Nothing, 1 = Skills, 2 to 5 = the match starts). Unset = Skills, so the skills tools keep working.
#include <cstdlib>
void robot::AutonSelector::start() {}
void robot::AutonSelector::runSelected(AutonFunc fallback) const {
    const char* e = std::getenv("SIM_AUTON");
    const std::size_t i = e ? std::strtoul(e, nullptr, 10) : 1;
    (i < routines_.size() && routines_[i].second ? routines_[i].second : fallback)();
}
std::size_t robot::AutonSelector::selectedIndex() const { const char* e = std::getenv("SIM_AUTON"); return e ? std::strtoul(e, nullptr, 10) : 1; }
