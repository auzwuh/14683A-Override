#pragma once

// Driver-control macros for the cascade lift, claw rotator and claw.
//
// Everything here is NON-BLOCKING.  There is not a single pros::delay() in the
// implementation - it is a state machine that opcontrol ticks once per loop.
// A macro that blocks is a macro that freezes the drivetrain.
//
// The driver has direct jog control of the cascade and the rotator at all times
// (L2/R2 and Right/Y).  The macros sit on top of that: they snap the cascade to
// a computed stack level, and they run the score-and-reset cycle.
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

// Which Goal the stack ladder is anchored on.  Level 0 is an empty Goal; each
// level up is one more (Pin + Cup) already on the tower.
enum class Base { Short, Tall };

// ===========================================================================
// TUNING BLOCK - every number a mechanic needs is here and nowhere else.
//
// Lift, rotator and claw values are MOTOR DEGREES.  The starting values are the
// ATTICUS mech presets, which are placeholders that have never been measured
// against hardware (same warning as src/autons.cpp).
//
// ---------------------------------------------------------------------------
// THE CASCADE IS WINCH-DRIVEN.  Three consequences run through this whole file:
//
// 1. It can only PULL.  Descent is gravity paying line off the spool.  Command
//    the motor down harder than the carriage actually falls and the spool
//    outruns the load, the line goes slack and birds-nests.  So descent
//    velocities and the down-jog authority are deliberately much lower than the
//    up values - that asymmetry is not a typo.
//
// 2. Degrees per inch is NOT constant.  As line layers onto the spool the
//    effective radius grows, so the carriage travels further per motor degree
//    the higher it already is.  A single "degrees per level" constant is
//    therefore wrong by construction, which is why the stack ladder below is a
//    TABLE with a per-level trim rather than a multiplication.
//
// 3. Holding a height draws high current at zero velocity BY DESIGN - the motor
//    is carrying the load on the line the whole time.  Stall detection has to
//    ignore holding entirely or it fires every time the lift parks.
// ---------------------------------------------------------------------------
// ===========================================================================
namespace tune {

// --- lift anchors -----------------------------------------------------------
constexpr int LIFT_STOW   = 0;
constexpr int LIFT_SHORT  = 310;   // claw at an EMPTY Short Goal  = Short level 0
constexpr int LIFT_TALL   = 470;   // claw at an EMPTY Tall Goal   = Tall  level 0
constexpr int LIFT_TOGGLE = 700;   // flipper at Toggle height

// Soft limits.  LIFT_MAX_DEG must be the real mechanical top of the cascade -
// the level ladder is clamped to it, so an over-optimistic value is what lets a
// macro drive the carriage into its own hard stop.
constexpr int LIFT_MIN_DEG = 0;
constexpr int LIFT_MAX_DEG = 900;

// The lift must be at least this high before the rotator may leave stow, and the
// rotator must be back at stow before the lift drops below it.  This interlock
// applies to the driver's manual jog too, not just to the macros.
constexpr int LIFT_ROT_CLEAR = 180;

// --- the stack model --------------------------------------------------------
//
// A tower on a Goal alternates Pin, Cup, Pin, Cup...  (<SC2>: a Pin is Placed if
// it is nested with a Goal, or with a Cup that is nested to another Placed Pin.)
// One "level" is one Pin plus one Cup, so the claw has to rise by the same fixed
// pitch for every additional level.
//
// The physical rise per level is an ESTIMATE.  A Pin is 6.50 in long and
// consists of two halves; a Cup takes one half of a Pin, so the tower repeats
// every half Pin:  LEVEL_PITCH_IN = PIN_LEN_IN / 2 = 3.25 in.  The Pin length is
// solid (it is an Appendix A number, already in the ATTICUS field model); how
// deep a Cup seats over a Pin is not, and that is what could make it smaller.
// Check Appendix A sheets A5 (Pin) and A6 (Cup), or measure a real stack.
constexpr double LEVEL_PITCH_IN = 3.25;

// Degrees for the FIRST level step, off a nearly bare spool.  Seed only.
// Starting point: an 18T 6P sprocket pays 4.5 in of line per motor revolution
// (80 deg/in), and a 2-stage cascade doubles carriage travel, so ~33 deg/in
// near the bottom.  3.25 in x 33 = ~107.
constexpr int LEVEL_PITCH_DEG_FIRST = 107;

// Each level costs slightly FEWER degrees than the one below it, because the
// spool is fatter by then.  This is the winch correction: 0.96 means every step
// is 4% smaller than the last.  Estimate it as (bare spool radius) / (radius one
// layer up), or just tune the table by driving it - see LEVEL TRIM below.
constexpr double SPOOL_GROWTH = 0.96;

// Highest level the driver can select.  The ladder is clamped by LIFT_MAX_DEG
// anyway; this just stops the counter running away.
constexpr int MAX_LEVEL = 5;

// Degrees the claw must rise for each Pin ALREADY nested on a Goal, with no Cup
// between them.  Pins nest directly into each other - Figure SC2-1 calls them
// Placed "as they are all at least partially nested within each other" - so a
// Goal that already holds a Pin needs the claw one nest-depth higher, or the Pin
// being placed arrives at the height of the stack instead of above it and
// knocks it off.
//
// This is NOT the same as a stack level: a level is Pin + Cup, this is Pin into
// Pin, and Pins nest deeper into each other than a Cup sits over a Pin.  Seeded
// at half a Pin for want of a better number.
//
// MEASURE THIS EARLY: nest two Pins on a Goal, drive the claw to each in turn,
// read the difference off the controller.  It decides whether the autons stack
// or scatter.
constexpr int PIN_NEST_DEG = 107;

// Degrees for the step INTO `level` (level 1 is the first step above an empty
// Goal).  Shrinks geometrically as the spool fattens.
constexpr int levelStepDeg(int level) {
    double step = LEVEL_PITCH_DEG_FIRST;
    for (int i = 1; i < level; ++i) step *= SPOOL_GROWTH;
    return static_cast<int>(step);
}

// Seeded lift position for `level` above `anchor`.  This is the table the
// driver's trims edit - it is a starting guess, not a measurement.
constexpr int levelSeedDeg(int anchor, int level) {
    int deg = anchor;
    for (int i = 1; i <= level; ++i) deg += levelStepDeg(i);
    return deg;
}

// --- rotator ---------------------------------------------------------------
constexpr int ROT_STOW  = 0;
constexpr int ROT_SCORE = 160;
constexpr int ROT_FLIP  = 300;
constexpr int ROT_MIN   = 0;
constexpr int ROT_MAX   = 300;

// --- claw ------------------------------------------------------------------
constexpr int CLAW_GRIP = 0;
constexpr int CLAW_OPEN = -75;

// --- velocities, motor RPM (blue cartridge = 600 free) ----------------------
// Down is far slower than up ON PURPOSE.  Winding in is limited by torque;
// paying out is limited by how fast the carriage will actually fall, and
// exceeding that is what puts slack on the spool.
constexpr int LIFT_UP_VEL   = 200;
constexpr int LIFT_DOWN_VEL = 70;
constexpr int ROT_VEL       = 200;
constexpr int CLAW_VEL      = 100;

// --- jog authority, raw motor command [-127, 127] ---------------------------
constexpr int LIFT_JOG_UP_CMD   = 110;
constexpr int LIFT_JOG_DOWN_CMD = 35;   // just enough to pay out under load
constexpr int ROT_JOG_CMD       = 90;

// --- tolerances ------------------------------------------------------------
constexpr int LIFT_TOL = 15;
constexpr int ROT_TOL  = 12;

// --- timings, ms -----------------------------------------------------------
constexpr int RELEASE_DWELL_MS = 220;   // claw open -> Cup actually clear

// Controller-screen refresh.  The controller link is slow and shared with the
// rumble; writing to it faster than ~50 ms starves it.  200 ms is plenty for
// reading numbers off while tuning and leaves the link headroom.
constexpr int READOUT_MS = 200;

// --- LEVEL TRIM ------------------------------------------------------------
// A jog that ends while the lift is snapped to a level is remembered as a trim
// on THAT LEVEL of THAT Goal, not on the anchor.  On a winch the error is not a
// constant offset - it grows with spool radius - so a per-level table is the
// only correction that converges.  Drive one full stack, nudge each level with
// L2/R2, and the ladder is tuned for good.
constexpr int TRIM_LIMIT = 150;

// --- stall protection ------------------------------------------------------
// A winch jams for real: line hops the spool flange, a stage binds, the claw
// catches the frame.  Without this the motors sit at full current until they
// cook.
//
// The current threshold is high because a winch pulls hard by definition -
// hauling a loaded cascade up is a genuine multi-amp job, and only a jam
// produces that current with the spool not turning.  Holding a height is
// excluded outright in checkStall(), since on a winch that is high current at
// zero velocity all day long.
constexpr int STALL_MA      = 2500;
constexpr int STALL_VEL_RPM = 8;
constexpr int STALL_MS      = 350;

// Invariants the state machine relies on.  These fire at compile time so a
// mis-edit during tuning is a build error rather than a mechanism that silently
// never finishes a sequence.
static_assert(LIFT_STOW < LIFT_ROT_CLEAR,
              "rotator clearance must be above the bottom hard stop");
static_assert(LIFT_ROT_CLEAR < LIFT_SHORT,
              "LIFT_ROT_CLEAR must be below the lowest scoring anchor");
static_assert(LIFT_SHORT < LIFT_TALL, "Short Goal anchor must be below Tall Goal");
static_assert(LIFT_TALL < LIFT_MAX_DEG, "LIFT_MAX_DEG must be above every anchor");
static_assert(levelStepDeg(MAX_LEVEL) > LIFT_TOL,
              "even the smallest level step must be bigger than the position "
              "tolerance, or high levels are indistinguishable from each other");
static_assert(SPOOL_GROWTH > 0.5 && SPOOL_GROWTH <= 1.0,
              "SPOOL_GROWTH is a shrink factor per level; 1.0 means a constant "
              "pitch, i.e. no winch correction at all");

}  // namespace tune

// ===========================================================================

class Mechanism {
public:
    // Sets brake modes and zeroes the encoders.  Call from initialize() with the
    // lift on its bottom hard stop, the rotator stowed and the claw closed.
    void init();

    // Reads the controller and advances the state machine.  Call every loop.
    void driverTick();

    // Advances the state machine only, for custom bindings.
    void tick();

    // ---- intents, all non-blocking ----

    // Everything down and open, ready to take the next Cup.
    void intakePose();

    // Snap the cascade to a stack level on the current Base and present.
    void gotoLevel(int level);
    void nextLevel();
    void prevLevel();

    // Score whatever is in the claw onto the tower: release, let it settle,
    // return to the intake pose, and advance the level counter so the next
    // raise is already aimed one Cup higher.
    void scoreCup();

    // Lift to flipper height with the rotator stowed, for a Toggle ram.
    void flipToggle();

    void setBase(Base b);
    void abort();

    // ---- state ----
    int level() const { return level_; }
    Base base() const { return base_; }
    bool faulted() const { return stage_ == Stage::Faulted; }
    // Commanded lift position for a level, clamped to the soft limits.
    int levelDegrees(Base b, int level) const;

private:
    enum class Stage {
        Manual,     // holding, or being jogged by the driver
        ToIntake,
        Snapping,   // driving to snapTarget_
        Releasing,
        Faulted,
    };

    void enterStage(Stage s);
    void commandLift(int target, int velocity);
    void commandRot(int target);
    void commandClaw(int target);

    void jogLift(int dir);          // -1, 0, +1
    void jogRot(int dir);
    void captureLiftHold();
    void captureRotHold();

    bool liftAt(int target) const;
    bool rotAt(int target) const;
    bool rotStowed() const;
    int liftFloor() const;          // lowest the lift may go right now

    void checkStall();
    void fault();

    // Bottom line of the controller screen: which rung you are on, the tuned
    // position for it, and where the lift actually is.  This is how a trim gets
    // out of RAM and into the source - walk the stack, read each rung, paste.
    void updateReadout();

    Stage stage_{Stage::Manual};
    Base base_{Base::Short};
    int level_{0};

    int liftCmd_{tune::LIFT_STOW};
    int liftVel_{tune::LIFT_UP_VEL};
    int rotCmd_{tune::ROT_STOW};
    int clawCmd_{tune::CLAW_GRIP};

    int snapTarget_{tune::LIFT_STOW};
    bool snapRotStow_{false};       // Toggle ram: keep the rotator home
    bool snapValid_{false};         // a jog ending now should trim this anchor

    // [Base][level] - a winch's error grows with height, so each rung of the
    // ladder carries its own correction.
    int trims_[2][tune::MAX_LEVEL + 1]{};

    int liftJogDir_{0};
    int rotJogDir_{0};

    std::uint32_t stageAt_{0};
    std::uint32_t stallSince_{0};
    std::uint32_t lastReadoutAt_{0};
    char lastReadout_[20]{};
};

extern Mechanism mech;

}  // namespace robot
