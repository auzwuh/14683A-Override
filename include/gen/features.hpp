#pragma once

// Per-robot feature toggles for this template. Edit these, not the
// implementation files, when reusing this project on a new robot build.
//
// ARC_RAMSETE_LQR_ENABLED: set to 0 to turn RAMSETE-LQR path following off
// everywhere in this codebase with one edit - e.g. this robot build has no
// odometry (or unreliable odometry) and RamseteLQR's pose-feedback tracking
// isn't safe to run blind. With this at 0:
//   - arc::followRamseteLQR / Chassis::followRamseteLQR compile down to a
//     no-op (see src/gen/chassis/motion.cpp) - they don't move the chassis
//     or touch odometry at all, regardless of who calls them.
//   - Auton::ramseteLqrTestRoutine and Auton::ramseteLqrTauTest (src/main.cpp)
//     compile out entirely, and their auton-selector entries disappear
//     instead of sitting there doing nothing.
// This does not delete or modify the RamseteLQR implementation itself - it
// is still here as a template. Flip back to 1 once this robot (or the next
// one built from this template) has tracking wheels/IMU wired up.
#define ARC_RAMSETE_LQR_ENABLED 0

// Enable after configuring and calibrating both unpowered pods and the IMU.
#ifndef ARC_DAMP_ENABLED
#define ARC_DAMP_ENABLED 0
#endif
