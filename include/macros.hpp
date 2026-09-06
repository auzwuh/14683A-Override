#pragma once

// Driver-control macros for the cascade lift, claw rotator and claw.
//
// Everything here is NON-BLOCKING. There is not a single pros::delay() in the
// implementation - it is ticked once per opcontrol loop.
//
// THIS REPLACES THE OLD WINCH-SPECIFIC STACK-LADDER MECHANISM (stall
// detection, per-level trims, Short/Tall Base selection, the multi-stage
// Snapping/Releasing/ToIntake state machine) with a much simpler, generic
// template:
//
//   - The cascade lift holds a small list of DISCRETE STAGES (raw motor
//     degrees off the tare position set in init()). There is no winch/spool
//     model and no auto-sequencing - just "go to stage N".
//   - The lift is driven by a plain proportional loop run every tick -
//     output = kP * (target_degrees - current_degrees), clamped to a motor
//     command. No PROS move_absolute(), no motor-firmware position PID: this
//     file owns the whole control loop. Being P-only is deliberate - far from
//     the target the (clamped) output saturates and the lift moves at full
//     command, and it eases off smoothly as the error shrinks near the
//     target. There is no I or D term, so expect some steady-state droop
//     under load (a real spring/gravity load will settle a little short of
//     target) - that is expected of a P-only loop, not a bug; add a small
//     feedforward or an I term later if that droop matters for this robot.
//   - Macros: L2 taps step the stage UP by one (tap it 3 times to reach
//     stage 3); R2 tapped steps the stage DOWN by one; R2 HELD for
//     R2_RESET_HOLD_MS resets straight to stage 0 instead of stepping down.
//
// The claw rotator now auto-preps for scoring instead of needing a manual
// jog every time: whenever a scoring stage is selected AND the lift has
// cleared LIFT_ROT_CLEAR, it drives itself to ROT_SCORE_DEG automatically.
// Stage 0 (stowed) - or the lift not yet clear - drives it back to
// ROT_STOW_DEG instead. Holding Right/Y still overrides this live, same as
// before, for whenever 90 degrees isn't exactly where you want it.
//
// TEMPORARY: Up/Down jog the lift freely (see LIFT_JOG_CMD) so LIFT_STAGE_DEG
// can be measured on the real robot - remove once that table is filled in.
//
// Usage in main.cpp:
//
//     #include "macros.hpp"
//     void initialize() { ...; robot::mech.init(); }
//     void opcontrol() {
//         while (true) {
//             ...drive...
//             robot::mech.driverTick();
//             pros::delay(10);
//         }
//     }

#include <cstdint>

namespace robot {

// ===========================================================================
// TUNING BLOCK - every number a mechanic needs is here and nowhere else.
// Every value below is a PLACEHOLDER, never measured against real hardware -
// same warning as src/autons.cpp. Measure and edit these on the real robot.
// ===========================================================================
namespace tune {

// --- cascade lift stages -----------------------------------------------------
// Index 0 is the stowed/bottom position. Resize this list to add or remove
// stages - nothing else needs to change. Raw motor degrees off the tare
// position set in Mechanism::init().
constexpr int LIFT_STAGE_DEG[] = {0, 200, 400, 600, 800};
constexpr int LIFT_STAGE_COUNT = sizeof(LIFT_STAGE_DEG) / sizeof(LIFT_STAGE_DEG[0]);

// Proportional gain: motor command (out of 127) per degree of error, i.e.
// output = LIFT_KP * (target - current). Too low and the lift is sluggish or
// never quite gets there under load; too high and it slams into each stage
// or oscillates. Tune this first, before anything else here.
constexpr double LIFT_KP = 1.2;

// Soft limits the lift is clamped to regardless of what a stage entry says.
// LIFT_MAX_DEG must be the real mechanical top of the cascade.
constexpr int LIFT_MIN_DEG = 0;
constexpr int LIFT_MAX_DEG = 900;

// How long R2 must be held before it resets to stage 0 instead of just
// stepping down one stage.
constexpr std::uint32_t R2_RESET_HOLD_MS = 1000;

// TEMPORARY - manual jog for measuring LIFT_STAGE_DEG on the real robot.
// Hold Up/Down to drive the lift freely (overrides the P-only loop while
// held) and read its degrees off the readout below at each position you
// want as a stage. Delete LIFT_JOG_CMD and the jog block in driverTick()
// once every stage has been measured - it has no place in a finished bind.
constexpr int LIFT_JOG_CMD = 80;  // raw motor command out of 127, both directions

// Controller-screen refresh for the tuning readout below. The controller
// link is slow and shared with rumble; writing faster than ~50ms starves it.
// 200ms is plenty to read numbers off while tuning LIFT_KP/LIFT_STAGE_DEG.
constexpr std::uint32_t READOUT_MS = 200;

// --- claw rotator / claw -----------------------------------------------------
constexpr int ROT_JOG_CMD = 90;      // raw motor command while manually jogging
constexpr int ROT_MIN = 0;
constexpr int ROT_MAX = 300;

// Auto-prep-to-score: whenever a scoring stage is selected (stage != 0) AND
// the lift has cleared LIFT_ROT_CLEAR, the rotator drives itself out to
// ROT_SCORE_DEG automatically - no manual jog needed to get ready to score.
// Stage 0 (stowed), or the lift not yet clear, drives it back to
// ROT_STOW_DEG instead. Holding Right/Y still overrides this while held.
constexpr int ROT_STOW_DEG = 0;
constexpr int ROT_SCORE_DEG = 90;
constexpr int ROT_VEL = 100;   // move_absolute velocity, motor RPM

constexpr int CLAW_GRIP = 0;
constexpr int CLAW_OPEN = -75;
constexpr int CLAW_VEL = 100;

// The rotator may not swing out until the lift is at least this high, so it
// cannot hit the frame - a real physical interlock, kept regardless of the
// control-scheme rewrite above.
constexpr int LIFT_ROT_CLEAR = 180;

static_assert(LIFT_STAGE_COUNT > 1, "need at least a stow stage and one more");
static_assert(LIFT_STAGE_DEG[0] == 0, "stage 0 is the stow/reset target for R2-hold");
static_assert(LIFT_MAX_DEG > LIFT_MIN_DEG, "LIFT_MAX_DEG must be above LIFT_MIN_DEG");
static_assert(LIFT_KP > 0.0, "LIFT_KP must be positive - a negative or zero gain never drives toward the target");

}  // namespace tune

// ===========================================================================

class Mechanism {
public:
    // Sets brake modes and zeroes the encoders. Call from initialize() with
    // the lift on its bottom hard stop, the rotator stowed and the claw
    // closed.
    void init();

    // Reads the controller and drives the lift/rotator/claw. Call every loop.
    void driverTick();

    // Which lift stage is currently targeted (0 = stowed).
    int stage() const { return stage_; }

private:
    void driveLift();     // P-only loop toward LIFT_STAGE_DEG[stage_]
    void driveRotClaw();  // rotator jog + interlock, claw toggle, intake

    // Bottom line of the controller screen: which stage you're on, its
    // tuned target, and where the lift actually is - the numbers you need to
    // tune LIFT_KP and LIFT_STAGE_DEG on the real robot. Throttled to
    // READOUT_MS and only writes when the text actually changes.
    void updateReadout();

    int stage_{0};
    int clawCmd_{tune::CLAW_GRIP};

    // R2 tap-vs-hold bookkeeping: timestamp R2 was first seen held this
    // press, and whether the hold threshold already fired a reset (so
    // releasing afterward doesn't ALSO step the stage down).
    std::uint32_t r2HeldSince_{0};
    bool r2ResetFired_{false};

    std::uint32_t lastReadoutAt_{0};
    char lastReadout_[20]{};
};

extern Mechanism mech;

}  // namespace robot
