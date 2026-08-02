#pragma once

// Autonomous routines generated from ATTICUS_TERMINAL and adapted to the
// GenClient chassis API.  Register these in autonRoutines in main.cpp.

namespace Auton {

// V5RC Override Autonomous Win Point, red alliance.
// Starts with the rear of the robot flush against the LEFT field perimeter.
void overrideAwpRed();

// Same routine rotated 180 degrees onto the blue alliance.
// Starts flush against the RIGHT field perimeter.
void overrideAwpBlue();

}  // namespace Auton
