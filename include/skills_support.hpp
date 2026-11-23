// skills_support.hpp - robot-side functions called by the ATTICUS skills routine
// (src/skills_dr4b.cpp). 14683A DR4B, V5RC Override, skills only.
//
// Status. Syntax-checked against the arc/GenClient headers in this repo, NOT run
// on a robot. Physical values (ports, mount positions, lift angles, bumper
// offsets) start as 0 / NAN so the build or the call refuses to run on a guess.
// Policy values (gates, ranges, speeds) have starting defaults marked (start),
// change them from the [reset] / [track] lines printed to the terminal.
//
// Frame (same as skills_dr4b.cpp) is the REAL 140.41 in field, inches, origin at
// field centre, +Y toward the red wall, heading 0 = facing the red wall, clockwise.
#pragma once
#include "main.h"
#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"   // arc::getSpeed for the stop reset stillness check
#include "gen/electronics.h"
#include "lift_ctrl.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

extern arc::Chassis chassis;
extern arc::MotorGroup leftDrive, rightDrive;  // declared in main.cpp

// ============================================================== match clock
namespace skills {
// Clock checks at the Tall Goal. If a leg cannot finish inside 60 s the routine
// stops there, already touching the Midfield. Legs measured in ATTICUS v5 (no
// settling or lift waits), so each limit keeps about 1.4 s of margin. Tune from runs.
// Route v21, simulated: A takes 4.8 to 5.4 s from the Tall Goal release to its own release, B 4.7 s.
inline uint32_t kSkipALegAfterMs = 54200;   // A, (96,96)
inline uint32_t kLastLegStartMs = 54800;    // B, (120,120)
// The two red Midfield Pins, 5 points each, checked at the back off point after the first Tall Goal Pin.
// S (72,48) costs 3.0 s (it ends where the route goes next). W (48,72) costs 4.6 s and runs first, so it
// may only start while S, A and B all still fit behind it. Past kMidSStartMs neither runs and the robot
// goes straight on to A. Raise them only after timing real runs.
inline uint32_t kMidWStartMs = 42300, kMidSStartMs = 47000;   // simulated: the check comes at 46.3 s on average, S runs in 3 of 4 runs
// Checked at the Alliance Goal before each late piece. Past the limit the robot parks
// in the Midfield instead (+8). Start values are 60 s minus the simulated time for the
// piece and the park after it (sim-tuned drive gains). Retime from the [t] lines.
inline uint32_t kD24StartMs = 44000;          // back square + D24 unit onto the Alliance Goal, 12.1 s, then the park
inline uint32_t kLoaderCycleStartMs = 49500;  // one more bottom Loader load onto the Alliance Goal, 6.0 s (7.5 worst), then the park, 3.9 s
inline uint32_t kTallPinStartMs = 52300;      // bottom Loader Pin onto the Tall Goal, ends in the Midfield
inline uint32_t& startMs() { static uint32_t s = 0; return s; }
inline void markStart() { startMs() = pros::millis(); }
inline uint32_t elapsedMs() { return pros::millis() - startMs(); }
}  // namespace skills

// ===================================================================== ports
// The port NUMBERS are in src/main.cpp, with the drive, IMU and tracking wheel ports
// (search for "namespace skills_ports"). Negative = reversed. Set them there.
namespace skills_ports {
extern const int kLiftLeft, kLiftRight;      // DR4B motors, left and right tower
extern const int kClawRollers;               // vertical roller claw motor
extern const int kLiftRotation;              // rotation sensor on the stage-1 bottom pivot shaft (only with kUseMotorEncoders off)
extern const int kClawSensor;                // distance sensor looking across the claw throat
extern const int kDistFront, kDistLeft, kDistRight;   // wall distance sensors
extern const int kToggleLeft, kToggleRight;  // 5.5 W Toggle spinner motors
}  // namespace skills_ports

// ====================================================================== DR4B
namespace dr4b {
// Lift angle (deg) = stage 1 rotation from the rest pose (lift down on its stop
// at power-on). Read from the lift motor encoders through the 12T to 48T crank
// (kUseMotorEncoders) or from a rotation sensor on a stage 1 pivot.
// NAN means not tuned, and dr4b::to() ignores NAN targets.
inline bool kUseMotorEncoders = true;
// 12T to 48T: 4 lift motor degrees per degree of CRANK angle. The lift angle follows the crank
// through the link, see crankTurnForLift() below.
inline double kMotorDegPerLiftDeg = 4.0;
// LIFT LIMITS, in lift MOTOR degrees, zeroed with the lift resting on its stop. The driver
// soft stops and every auton target stay inside them. With the 6.0 in link the crank runs out of
// travel at 394 motor deg (stage 1 65 deg above level), so the top limit is a little under that.
inline double kBottomMotorDeg = 0.0, kTopMotorDeg = 388.0;
// The 17 Nov height table (motor deg -> height: 0 3.2, 80 11.5, 160 20.2, 240 28.3, 320 34.9, 400 39.3,
// 412 39.7 in) is no longer used. It was for the lift before the hard stop and the link. Goal
// levels now come from the linkage and the link geometry below, or from a measured kLiftTable.
inline double kLoader = 0.0;      // pickup at the rest pose, object standing on the tiles (ASSUMED)
inline double kFloor = 0.0;       // same pickup pose for floor Cup+Pins (ASSUMED)
// The BACK OF THE LIFT decides these two (see rearIn() below). Folded, the rear end of the upper
// stage 1 bar is 10.27 in behind the robot origin, 1.72 in past the frame. It swings further back as
// the lift starts up (10.88 at 6 deg, 11.72 at 18, 12.0 at 33) and is over the wall top from 35 deg.
// So next to a wall the lift is either LOW or above 35 deg, never in between.
inline double kCarry = 4.0;       // STARTING VALUE. travel height with a held object, its bottom 2.2 in off the tiles. The lift's back
                                  // corners are then 11.7 in from the origin, and the Loader lane turns are 12.4 in from the wall
inline double kPreRaise = 4.0;    // STARTING VALUE. Was 18, a head start for tall stacks on the way out of the Loader. At 18 deg the
                                  // lift's back corners now reach 12.5 in and scrape the wall on the Loader lane turn. Keep it at kCarry
inline double kToggleUp = 38.4;   // STARTING VALUE (Pin bottom 15.7 in up, 1 in over the turning Toggle). claw and held Pin clear above the wall and Toggle while the paddle spins
                                  // (Toggle top 14.3 in up, 14.4 as it turns, paddle sweeps to 14.9 on the right)
// Goal placements used by the skills route, in route order. Filled by
// computeRouteTargets() from the stacking model below, do not type them in.
inline double kShort[6] = {NAN, NAN, NAN, NAN, NAN, NAN};     // Short Goal, placement 1..
inline double kAlliance[6] = {NAN, NAN, NAN, NAN, NAN, NAN};  // Alliance Goal, placement 1..
inline double kTall[5] = {NAN, NAN, NAN, NAN, NAN};                     // Tall Goal, placement 1..
inline double kAlliance2[2] = {NAN, NAN};                     // second red Alliance Goal (48,24), placement 1..
// Controller (volts per deg, volts per deg/s, gravity volts at the heaviest lift
// angle, see gravityShare)
// STARTING VALUES from the simulator (17 Nov), not tuned on the robot, for 200 rpm lift
// cartridges WITH rubber band assist. Useful ranges: kP 1.2 to 2.5 (above 3 it overshoots),
// kD 0.07 to 0.15, kG 2.5 to 4.1 (heavier claw or weaker bands = more).
// Set kG by hand first: with kP, kD and kI at 0, raise it until the lift holds a Cup+Pin at 37 deg.
// WITHOUT bands the two 200 rpm motors are at their limit: about 7.6 N m at the 48T against a
// 6.3 to 7.9 N m load at 37 deg, so the lift stalls part way up and a hold needs 5 to 6.4 V.
// Bands should carry a third to a half of the lift's weight (2 to 4 N m at the 48T axle).
inline double kP = 1.8, kD = 0.1, kG = 3.3;
inline double kTolDeg = 0.75;  // STARTING VALUE (3 motor deg). The lift counts as there inside this.
                               // Keep kTolDeg x 0.42 in per deg under kDropClearIn, or releases land low
// Integral, volts per deg second. 0 = off. Tune kP, kD and kG first, then add a little if the
// lift stops a degree or two off target. It only acts inside kIZoneDeg and is capped at kIMaxV.
inline double kI = 4.0, kIZoneDeg = 3.0, kIMaxV = 2.0;
// Most volts the controller may use to push the lift DOWN. The rubber bands leave the lift almost
// weightless, so it needs little, and a hard push goes straight into the link and the standoff
// the lift rests on (near the stop the crank and the link are almost in line, which multiplies it).
inline double kMaxDownV = 4.0;   // STARTING VALUE. Simulated: 4, 6 and 12 V give the same run time and score
inline int kSettleMs = 60;   // waitSettled wants the lift inside kTolDeg for this long, not just passing through

inline arc::MotorGroup& motors() {
    static arc::MotorGroup m({skills_ports::kLiftLeft, skills_ports::kLiftRight}, 200.0, 1.0);   // 200 rpm cartridges
    return m;
}

// ---- Link drive (Austin's numbers, 19 and 23 Nov) ----
// The 48T gear turns a crank arm. A RIGID LINK, 6.0 in hole to hole (13 holes), runs from the far
// hole of that arm to hole 27 on the tail of the lower stage 1 bar, in front of the bar's tower
// pivot. Pulling the tail down raises the lift, pushing it up lowers the lift. It was going to be
// a string (5.918 in): a string cannot push, and with 6 rubber bands carrying most of the lift's
// weight the simulated lift hung in the air on the way down.
// Build the link stiff: two flat bars with the crank arm and the tail between them, bolted
// together through spacers, or one piece of C-channel. One flat bar bows when it is pushed.
// The lift rests on a hard stop with stage 1 at kStage1FoldedDeg.
// So the lift angle is NOT motor degrees / 4. Right at the stop the crank turns 1.8 deg per lift
// degree, mid stroke about 0.9, and over the whole stroke 98.6 deg of crank for 98 deg of lift.
inline double kCrankFwdIn = 0.250, kCrankDownIn = 6.750;   // crank axle (48T centre) from the stage 1 tower pivot
inline double kCrankArmIn = 2.061;                         // 48T centre to the link hole
inline double kTieIn = 2.000;                              // bar's tower pivot to hole 27 (4 holes)
inline double kLinkIn = 6.000;                             // link, hole to hole. MEASURE the built one
inline double kStage1FoldedDeg = -33.0;                    // stage 1 above level, resting on the 4.35 in standoff on the crank axle
// Crank arm angle (deg from straight forward, + up) with stage 1 at aDeg above level.
inline double crankForStage(double aDeg) {
    const double a = aDeg * M_PI / 180.0;
    const double dx = kTieIn * std::cos(a) - kCrankFwdIn, dy = -kTieIn * std::sin(a) + kCrankDownIn;
    const double d = std::hypot(dx, dy);
    const double c = std::clamp((kCrankArmIn * kCrankArmIn + d * d - kLinkIn * kLinkIn) / (2.0 * kCrankArmIn * d), -1.0, 1.0);
    return (std::atan2(dy, dx) + std::acos(c)) * 180.0 / M_PI;
}
// Crank turn from the stop (deg) for a lift angle (stage 1 turn from the stop, deg), and back.
inline double crankTurnForLift(double liftDeg) {
    return crankForStage(kStage1FoldedDeg + liftDeg) - crankForStage(kStage1FoldedDeg);
}
inline double liftForCrankTurn(double crankDeg) {
    double lo = 0.0, hi = 115.0;   // crankTurnForLift rises all the way through this range
    if (crankDeg <= 0.0) return crankDeg / 1.8;   // below the stop only while taring, same slope as just above it
    for (int i = 0; i < 40; ++i) {
        const double m = 0.5 * (lo + hi);
        (crankTurnForLift(m) < crankDeg ? lo : hi) = m;
    }
    return 0.5 * (lo + hi);
}
inline pros::Rotation& sensor() { static pros::Rotation r(skills_ports::kLiftRotation); return r; }
// NAN when nothing can be read. An unplugged motor reports infinity and is left out, so the
// other motor's encoder still gives the angle (both are on one shaft).
inline double angle() {
    if (kUseMotorEncoders) return liftForCrankTurn(lift_ctrl::meanFinite(motors().get_position_all()) / kMotorDegPerLiftDeg);
    const std::int32_t c = sensor().get_position();
    return c == PROS_ERR ? NAN : c / 100.0;   // centidegrees -> deg
}
inline std::atomic<double>& target() { static std::atomic<double> t{NAN}; return t; }

// Call in initialize() with the lift resting down on its stop.
inline void zeroAtRest() {
    motors().set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    motors().tare_position_all();
    if (!kUseMotorEncoders) sensor().reset_position();
}

inline double gravityShare(double a);   // defined with the stacking model below

inline void start() {
    static bool started = false;
    if (started) return;
    started = true;
    pros::Task([] {
        double prev = angle(), lastT = NAN;
        lift_ctrl::Integral integral;
        bool lost = false;
        while (true) {
            const double a = angle(), t = target();
            if (!std::isnan(t) && !std::isnan(kP) && !std::isnan(kD) && !std::isnan(kG)) {
                if (!std::isfinite(a)) {
                    // no lift angle at all: stop driving. Chasing a target blind would run it into a stop at 12 V
                    if (!lost) std::printf("[dr4b] NO ANGLE, lift motors or sensor unplugged. Lift stopped\n");
                    lost = true;
                    integral.sum = 0.0;
                    motors().brake();
                } else {
                    lost = false;
                    if (t != lastT) integral.sum = 0.0;   // new target, start the trim again
                    lastT = t;
                    const double vel = std::isfinite(prev) ? (a - prev) / 0.010 : 0.0;
                    const double v = std::clamp(kP * (t - a) - kD * vel + integral.step(t - a, 0.010, kI, kIZoneDeg, kIMaxV) +
                                                    kG * gravityShare(a), -kMaxDownV, 12.0);
                    motors().move_voltage(static_cast<int>(v * 1000.0));
                }
            }
            prev = a;
            pros::delay(10);
        }
    }, "dr4b");
}
inline void to(double deg) {
    if (std::isnan(deg)) { std::printf("[dr4b] target not tuned, ignored\n"); return; }
    start();
    target() = std::clamp(deg, liftForCrankTurn(kBottomMotorDeg / kMotorDegPerLiftDeg), liftForCrankTurn(kTopMotorDeg / kMotorDegPerLiftDeg));   // never past the limits
}
inline bool waitSettled(int timeoutMs) {
    if (std::isnan(target()) || std::isnan(kTolDeg)) return false;
    const auto end = pros::millis() + timeoutMs;
    lift_ctrl::Dwell dwell;
    while (pros::millis() < end) {
        if (dwell.step(std::fabs(target() - angle()) < kTolDeg, 10, kSettleMs)) return true;
        pros::delay(10);
    }
    std::printf("[dr4b] NOT settled after %d ms, target %.1f deg, at %.1f\n", timeoutMs, target().load(), angle());
    return false;
}

// ------------------------------------------------------------ stacking model
// Every Goal level is computed, nothing per level is typed in by hand.
//  0. Lift geometry. With kLiftTable empty the rise comes from the dr4b.step
//     linkage below. Once built, a measured table (angle, Pin bottom height)
//     with 8 to 10 points replaces it.
//  1. kLiftTable. Height of the BOTTOM of a held bare Pin above the tiles (in,
//     tape) at several lift angles (deg) across the stroke, sorted by angle.
//  2. kSeatIn. Height of the bottom of one object sitting in the EMPTY Goal.
//  3. kPitchIn. How much each nested object raises a stack. Stand 1 object and
//     a 5 object stack (Pin, Cup, Pin, Cup, Pin) on the tiles, pitch = (top of
//     5 - top of 1) / 4. Object CAD gives 3.32, an older reading 3.55, so measure.
//  4. kUnitBelowPinIn. How much lower the bottom of a held Cup+Pin sits than the
//     bottom of a held bare Pin, claw at the same angle.
// The claw drives in level, so the held object must pass OVER whatever is already
// on the Goal and then drop to nest. Release height = top of the stack on the Goal
// (or the Goal base when empty) + claw underside + drop clearance.
enum Goal { kShortGoal = 0, kAllianceGoal = 1, kTallGoal = 2 };
enum Held { kPin = 0, kCupPin = 1 };
// {lift angle deg, rise in}, MEASURED on the robot. Empty = the linkage below is used.
// It is empty since 19 Nov: the 17 Nov height table was made for a lift that rested 51 deg
// below level and turned 1 to 1 with the crank. With the rest pose at -33 deg and the link
// that table no longer fits. Fill this with 8
// or so tape measurements once the lift is built: {lift angle from dr4b::angle(), rise of the
// held Pin's bottom above its pickup height}.
inline std::vector<std::pair<double, double>> kLiftTable = {};
// Side view. Directions in deg, 0 = forward, 90 = up. CHECK both rest directions with an angle
// gauge on the built robot, or fill kLiftTable instead.
inline double kL1In = 11.0, kL2In = 13.5;     // stage lengths, hole to hole (22 and 27 holes)
// 19 Nov, Austin's CAD: the lift rests with stage 2's lower bars on the 4.35 in standoff on the
// crank axle. Stage 1 is then 33 deg below level and stage 2 32.17 deg below level (the two are
// 0.83 deg apart, re-meshed by 4 teeth). Lower carriage pivot above the tiles, with a = stage 1
// above level: h = 17.395 + 11.0 sin(a) + 13.5 sin(a + 0.83). 4.22 in folded, 39.68 in at a = 65.
// riseIn() and clawBackIn() below are that same linkage.
inline double kStage1RestDir = -147.0;        // tower pivot to middle pivot, lift on its stop (= -180 - kStage1FoldedDeg, keep the two in step)
inline double kStage2RestDir = -32.17;        // middle pivot to front pivot, lift on its stop
// Goal and object numbers. Heights from Austin, 18 Nov: Short (neutral) Goals 146.5 mm = 5.77 in,
// Tall (centre) Goal 222.7 mm = 8.77 in. Alliance Goal 3.25 in from the Goal CAD and the field
// drawing. No Pins are in any Goal at the start.
inline double kGoalTopIn[3] = {5.77, 3.25, 8.77};   // top of the Goal base: Short, Alliance, Tall
inline double kSeatIn[3] = {2.605, 0.085, 5.605};   // object bottom when seated in an empty Goal (top - 3.165 nest)
inline double kPitchIn = 3.32;                     // 6.49 tall, 3.165 nest. Check with a tape
inline double kObjTallIn = 6.49;                   // height of one Pin or Cup standing alone
inline double kUnitBelowPinIn = 0.0;   // both picked standing on the tiles at the same pose
inline double kClawBelowObjIn = 0.25;  // lowest claw part below the held object's bottom (ASSUMED)
inline double kDropClearIn = 0.9;      // gap between the claw underside and the stack top / Goal base at release.
                                       // Must be more than the lift settle error (kTolDeg x about 0.45 in per deg)
inline double kTopDeg = liftForCrankTurn(kTopMotorDeg / kMotorDegPerLiftDeg);   // 96 deg of lift angle at 388 motor deg (the link's stroke ends at 394)

// Rise of the held object's bottom above its pickup height at lift angle th.
inline double riseIn(double th) {
    auto z = [](double t) {
        return kL1In * std::sin((kStage1RestDir - t) * M_PI / 180.0) +
               kL2In * std::sin((kStage2RestDir + t) * M_PI / 180.0);
    };
    return z(th) - z(0.0);
}
// How far the claw has moved BACK toward the robot at lift angle th. Both stages swing on
// arcs, so the higher the lift, the closer the robot has to stand to a Goal.
inline double clawBackIn(double th) {
    auto x = [](double t) {
        return kL1In * std::cos((kStage1RestDir - t) * M_PI / 180.0) +
               kL2In * std::cos((kStage2RestDir + t) * M_PI / 180.0);
    };
    return x(0.0) - x(th);
}
// Back of the lift (Austin's CAD, 19 Nov): how far behind the robot origin the rear end of the
// upper stage 1 bar is at lift angle th. Folded it is 10.27 in, 1.72 in past the frame (8.55), so
// the LIFT touches a wall before the frame does. It grows to 12.0 in as stage 1 comes level.
// Returns 0 once the stage 1 middle joint is above the wall top and the lift passes over the wall.
inline double kTowerAheadIn = 0.487;    // stage 1 tower pivots ahead of the robot origin
inline double kBarEndIn = 12.5;         // tower pivot to the rear end of the upper stage 1 bar (11.0 + 1.5 past the middle pivot)
inline double kBarHalfIn = 0.5;         // half the height of that bar
inline double kLowPivotUpIn = 11.875;   // lower stage 1 tower pivot above the tiles
inline double kWallTopIn = 11.58;       // top of the field wall (field CAD, CHECK on a real field)
inline double rearIn(double th) {
    const double a = (kStage1FoldedDeg + th) * M_PI / 180.0;
    if (kLowPivotUpIn + kL1In * std::sin(a) - kBarHalfIn >= kWallTopIn) return 0.0;
    return kBarEndIn * std::cos(a) + kBarHalfIn * std::fabs(std::sin(a)) - kTowerAheadIn;
}
// Gravity load on the lift motors at lift angle a, as a share of the heaviest
// angle. The load follows how far the claw rises per lift degree, which both
// stages set (stage 1 rests 51 deg below level, stage 2 25, and L2 > L1), so the
// peak is between where the two stages are level, not where stage 1 is level.
// Assumes the claw end carries most of the weight.
inline double gravityShare(double a) {
    // rise per degree of CRANK, which is what the motors feel through the link
    auto slope = [](double t) {
        return (riseIn(t + 0.5) - riseIn(t - 0.5)) / std::max(0.2, crankTurnForLift(t + 0.5) - crankTurnForLift(std::max(0.0, t - 0.5)));
    };
    static const double peak = [&] {
        double m = 0.0;
        for (double t = 0.0; t <= 140.0; t += 1.0) m = std::max(m, slope(t));
        return m;
    }();
    return slope(a) / peak;
}
// Angle that puts the bottom of a held Pin at heightIn. NAN if out of reach.
inline double angleForPinBottom(double heightIn) {
    if (std::isnan(heightIn)) return NAN;
    if (kLiftTable.size() < 2) {
        const double top = std::isnan(kTopDeg) ? 100.0 : kTopDeg;
        if (heightIn < 0.0 || heightIn > riseIn(top)) return NAN;
        double lo = 0.0, hi = top;
        for (int i = 0; i < 50; ++i) {
            const double m = 0.5 * (lo + hi);
            (riseIn(m) < heightIn ? lo : hi) = m;
        }
        return 0.5 * (lo + hi);
    }
    const auto& t = kLiftTable;
    for (size_t i = 1; i < t.size(); ++i) {
        const double h0 = t[i - 1].second, h1 = t[i].second;
        if ((heightIn - h0) * (heightIn - h1) <= 0.0 && h1 != h0)
            return t[i - 1].first + (heightIn - h0) / (h1 - h0) * (t[i].first - t[i - 1].first);
    }
    return NAN;
}
// Angle to release an object on a Goal that already holds `objectsOnGoal`
// objects (Pins and Cups both count, a Cup+Pin is 2). NAN if not calibrated or
// above kTopDeg.
inline double levelAngle(Goal g, int objectsOnGoal, Held held) {
    if (std::isnan(kSeatIn[g]) || std::isnan(kPitchIn)) return NAN;
    const double below = held == kCupPin ? kUnitBelowPinIn : 0.0;
    if (std::isnan(below)) return NAN;
    // What the claw has to clear on the way in: the top of the stack already on the
    // Goal, or the Goal base when it is empty. The object is released just above
    // that and drops the rest of the way (kObjTallIn - kPitchIn onto a stack).
    double top = objectsOnGoal > 0 ? kSeatIn[g] + (objectsOnGoal - 1) * kPitchIn + kObjTallIn
                                   : kSeatIn[g] - kClawBelowObjIn;
    if (objectsOnGoal == 0 && !std::isnan(kGoalTopIn[g])) top = std::max(top, kGoalTopIn[g]);
    const double h = top + kClawBelowObjIn + kDropClearIn + below;
    const double a = angleForPinBottom(h);
    if (!std::isnan(kTopDeg) && a > kTopDeg) return NAN;
    return a;
}
inline void toLevel(Goal g, int objectsOnGoal, Held held) { to(levelAngle(g, objectsOnGoal, held)); }

// Driver lift stages. Stage k (1, 2, ...) is the release height for the k-th object
// on a Short or Alliance Goal, so k - 1 objects are already on it. They come from the
// same stacking model as the skills route and stop where the lift runs out.
inline const std::vector<double>& driverStages() {
    static const std::vector<double> stages = [] {
        std::vector<double> v;
        for (int n = 0; n < 20; ++n) {
            const double a = levelAngle(kShortGoal, n, kCupPin);
            if (std::isnan(a)) break;
            v.push_back(a);
        }
        return v;
    }();
    return stages;
}

// Prints every level for every Goal (bare Pin and Cup+Pin) until the lift runs
// out, so the limit is whatever the lift can reach.
inline void printLevels() {
    const char* name[3] = {"Short", "Alliance", "Tall"};
    for (int g = 0; g < 3; ++g)
        for (int n = 0; n < 20; ++n) {
            const double ap = levelAngle(static_cast<Goal>(g), n, kPin);
            const double au = levelAngle(static_cast<Goal>(g), n, kCupPin);
            if (std::isnan(ap) && std::isnan(au)) {
                std::printf("[levels] %s max %d objects already on the Goal\n", name[g], n - 1);
                break;
            }
            std::printf("[levels] %s, %2d on Goal, Pin %6.1f deg, Cup+Pin %6.1f deg\n", name[g], n, ap, au);
        }
}

// The route's placements, as (objects already on that Goal, what is held).
// Loader loads and diagonal floor objects are Cup+Pin, the preload and the two
// last Loader objects are bare Pins.
inline void computeRouteTargets() {
    const std::pair<int, Held> sh[4] = {{0, kPin}, {1, kCupPin}, {3, kCupPin}, {5, kCupPin}};
    const std::pair<int, Held> al[5] = {{0, kPin}, {1, kCupPin}, {3, kCupPin}, {5, kCupPin}, {7, kCupPin}};
    const std::pair<int, Held> ta[5] = {{0, kPin}, {1, kCupPin}, {3, kCupPin}, {5, kCupPin}, {7, kCupPin}};
    for (int i = 0; i < 4; ++i) kShort[i] = levelAngle(kShortGoal, sh[i].first, sh[i].second);
    for (int i = 0; i < 5; ++i) kAlliance[i] = levelAngle(kAllianceGoal, al[i].first, al[i].second);
    for (int i = 0; i < 5; ++i) kTall[i] = levelAngle(kTallGoal, ta[i].first, ta[i].second);
    for (int i = 0; i < 2; ++i) kAlliance2[i] = levelAngle(kAllianceGoal, al[i].first, al[i].second);
    printLevels();
}
}  // namespace dr4b

// ============================================================ Toggle spinner
// Two 5.5 W motors on the 45 deg braces chain-drive a paddle at the top front of
// the robot (final CAD: 8T to 18T, paddle axis 13.0 in up, on the right; the left
// one is not assembled in the CAD). It works from the FRONT only. The long end of
// the paddle reaches the Toggle with the front rails 0.8 to 1.4 in from the wall,
// and inside 0.92 the short end hits it too. Closer than 0.78 the chain jams on
// the Toggle. Before a run park the paddle with its long end pointing BACK:
// flip() turns whole turns, so it ends where it started.
namespace toggle {
inline double kMotorTurnsPerPaddleTurn = 18.0 / 8.0;   // CHECK the real sprockets
inline double kFlipTurns = 2.0;   // paddle turns per skills visit (TUNE until the Toggle ends red; yellow to red is 2 steps)
inline int kRpm = 200;            // 5.5 W motor speed. Flip the port signs if it spins the wrong way
inline pros::MotorGroup& motors() {
    static pros::MotorGroup m({skills_ports::kToggleLeft, skills_ports::kToggleRight});
    static const bool held = (m.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD), true);  // a stopped paddle stays parked
    (void)held;
    return m;
}
// Driver. Spins while the button is held.
inline void spin(bool on) {
    if (on) motors().move(127);
    else motors().brake();
}
// Skills. Spins the paddle kFlipTurns turns and waits for it (at most twice the expected time).
inline void flip() {
    const double deg = kFlipTurns * kMotorTurnsPerPaddleTurn * 360.0;
    motors().set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    motors().tare_position_all();
    motors().move_relative(deg, kRpm);
    const auto end = pros::millis() + static_cast<uint32_t>(2000.0 * std::fabs(deg) / (kRpm * 6.0)) + 200;
    // Every motor that answers has to finish. The two are not linked, so the loaded one lags.
    auto done = [&] {
        for (double p : motors().get_position_all())
            if (std::isfinite(p) && std::fabs(p - deg) > 15.0) return false;
        return true;
    };
    while (pros::millis() < end && !done()) pros::delay(10);
    motors().brake();
    const auto pos = motors().get_position_all();
    std::printf("[t %5.2f] toggle spun L %.0f R %.0f of %.0f motor deg\n", skills::elapsedMs() / 1000.0,
                pos.size() > 0 ? pos[0] : NAN, pos.size() > 1 ? pos[1] : NAN, deg);
}
}  // namespace toggle

// =============================================================== roller claw
namespace rclaw {
inline double kSeesObjectMm = NAN;  // claw sensor reading with a Cup/Pin in the throat (MEASURE)
inline int kHoldMv = 0;             // small inward voltage that keeps the grip. 0 = motor HOLD brake (tune)
inline int kReleaseMs = 300;        // outtake time to drop the object (start value, tune). 0 would drop nothing

inline arc::MotorGroup& motor() {
    static arc::MotorGroup m({skills_ports::kClawRollers}, 200.0, 1.0);
    static const bool braked = (m.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD), true);  // a stopped roller keeps the grip
    (void)braked;
    return m;
}
inline pros::Distance& sensor() { static pros::Distance d(skills_ports::kClawSensor); return d; }
inline bool hasObject() {
    if (std::isnan(kSeesObjectMm)) return false;
    const int mm = sensor().get();
    return mm > 0 && mm < kSeesObjectMm;
}
inline void intake() { motor().move_voltage(12000); }
inline void hold() {
    if (kHoldMv == 0) motor().brake();
    else motor().move_voltage(kHoldMv);
}
inline void release() {
    std::printf("[t %5.2f] release\n", skills::elapsedMs() / 1000.0);
    motor().move_voltage(-12000);
    pros::Task([] { pros::delay(kReleaseMs); motor().move_voltage(0); });
}
// Returns as soon as the object is seen, waits up to timeoutMs if it is late.
inline bool confirmGrip(int timeoutMs) {
    const auto end = pros::millis() + timeoutMs;
    while (pros::millis() < end && !hasObject()) pros::delay(10);
    hold();
    const bool ok = hasObject();
    std::printf("[t %5.2f] grip %s\n", skills::elapsedMs() / 1000.0, ok ? "ok" : "NOT SEEN");
    return ok;
}
// Floor grabs. Started at the beginning of the slow creep toward a standing
// Cup+Pin. Cancels the drive motion the moment the Cup is in the throat, so the
// robot never keeps pushing a Cup it has already reached (that is what tips them).
inline void stopDriveOnObject(int timeoutMs) {
    pros::Task([timeoutMs] {
        pros::delay(30);  // let the motion start
        const auto end = pros::millis() + timeoutMs;
        while (pros::millis() < end && chassis.isInMotion()) {
            if (hasObject()) { chassis.cancelMotion(); std::printf("[claw] object seen, creep stopped\n"); return; }
            pros::delay(10);
        }
    }, "claw-stop");
}
}  // namespace rclaw

// ================================================================= field map
// Everything a low distance beam can hit before the wall. Structures plus every
// Cup that starts standing (Cups the route collects stay listed, which only
// makes the check stricter). ATTICUS tile coordinates scaled by 140.41/144.
namespace field {
constexpr double kScale = 140.41 / 144.0;
constexpr double kHalf = 140.41 / 2.0;
struct Obstacle { double x, y, r; };
constexpr Obstacle fromAtticus(double xf, double yf, double r) { return {(yf - 72.0) * kScale, (72.0 - xf) * kScale, r}; }
constexpr double kGoalR = 3.22, kLoaderR = 3.06, kCupR = 1.58;
constexpr Obstacle kObstacles[] = {
    fromAtticus(24, 96, kGoalR),
    fromAtticus(48, 120, kGoalR),
    fromAtticus(96, 24, kGoalR),
    fromAtticus(120, 48, kGoalR),
    fromAtticus(72, 72, kGoalR),
    fromAtticus(24, 48, kGoalR),
    fromAtticus(48, 24, kGoalR),
    fromAtticus(96, 120, kGoalR),
    fromAtticus(120, 96, kGoalR),
    fromAtticus(1.87, 11.7, kLoaderR),
    fromAtticus(1.87, 132.3, kLoaderR),
    fromAtticus(142.13, 11.7, kLoaderR),
    fromAtticus(142.13, 132.3, kLoaderR),
    fromAtticus(44.84, 1.58, kCupR),
    fromAtticus(48, 1.58, kCupR),
    fromAtticus(51.16, 1.58, kCupR),
    fromAtticus(92.84, 1.58, kCupR),
    fromAtticus(96, 1.58, kCupR),
    fromAtticus(99.16, 1.58, kCupR),
    fromAtticus(44.84, 142.42, kCupR),
    fromAtticus(48, 142.42, kCupR),
    fromAtticus(51.16, 142.42, kCupR),
    fromAtticus(92.84, 142.42, kCupR),
    fromAtticus(96, 142.42, kCupR),
    fromAtticus(99.16, 142.42, kCupR),
    fromAtticus(1.58, 44.84, kCupR),
    fromAtticus(1.58, 48, kCupR),
    fromAtticus(1.58, 51.16, kCupR),
    fromAtticus(1.58, 92.84, kCupR),
    fromAtticus(1.58, 96, kCupR),
    fromAtticus(1.58, 99.16, kCupR),
    fromAtticus(142.42, 44.84, kCupR),
    fromAtticus(142.42, 48, kCupR),
    fromAtticus(142.42, 51.16, kCupR),
    fromAtticus(142.42, 92.84, kCupR),
    fromAtticus(142.42, 96, kCupR),
    fromAtticus(142.42, 99.16, kCupR),
    fromAtticus(24, 120, kCupR),
    fromAtticus(48, 96, kCupR),
    fromAtticus(96, 48, kCupR),
    fromAtticus(120, 24, kCupR),
    fromAtticus(24, 24, kCupR),
    fromAtticus(48, 48, kCupR),
    fromAtticus(96, 96, kCupR),
    fromAtticus(120, 120, kCupR),
    fromAtticus(72, 96, kCupR),
    fromAtticus(96, 72, kCupR),
    fromAtticus(72, 48, kCupR),
    fromAtticus(48, 72, kCupR),
};
}  // namespace field

// ===================================================== distance localization
// Two ways the wall sensors correct odometry (vertical wheel + IMU, no
// horizontal wheel, so sideways slip is invisible to odometry).
//
// 1. Stop resets, localizer.applyImmediateCorrectionAuto(). ATTICUS emits this
//    at every Loader and lane-Goal stop. Samples all sensors together in the
//    background while the robot is parked, then writes the corrected axis.
// 2. Continuous tracking, localizer.startContinuous(). Every 25 ms, any sensor
//    that sees a wall square-on, in range, clear of the map, and is not moving
//    along its own beam nudges its axis toward the wall reading. On the x = 24
//    lanes and the Loader approaches this fixes the sideways drift a horizontal
//    wheel would have caught.
//
// A reading is used only if the beam is within the skew limit of a wall
// normal, the predicted beam clears every Goal, Loader and standing Cup (with a
// cone margin), the samples agree, and it matches odometry within the gate.
// Heading always comes from the IMU, or from squaring a bumper on a wall.
inline pros::Distance& distFront() { static pros::Distance d(skills_ports::kDistFront); return d; }
inline pros::Distance& distLeft() { static pros::Distance d(skills_ports::kDistLeft); return d; }
inline pros::Distance& distRight() { static pros::Distance d(skills_ports::kDistRight); return d; }

// ROBOT ORIGIN. Every robot-relative distance here and in gen_route.py is measured from
// the middle of the TRACTION wheel axle, not from the middle of the frame. The robot
// turns about its traction wheels, and odometry (one tracking wheel + IMU) follows
// exactly that point. Final CAD: it is 1.5 in ahead of the frame centre, so the front
// rail ends are 6.5 in ahead of it and the back of the frame 8.55 in behind it.
// The plate between the front rails has a V notch: a Goal base slides in until the Goal
// centre is 4.75 in from the origin. gen_route.py reads kL1In, kL2In, the rest directions
// and the stacking numbers from THIS file to give every stack level its own Goal stop,
// so regenerate the route after changing any of them.
struct DistanceMount {
    pros::Distance* dev;
    double rightIn, forwardIn;  // from the robot origin, the traction axle centre (MEASURE)
    double angleDeg;            // 0 = forward, 90 = right, -90 = left
    double biasIn;              // true distance minus reading (CALIBRATE)
};

class WallLocalizer {
  public:
    // stop resets (start values)
    double kGateIn = 3.0, kRecoveryGateIn = 8.0, kTrustRangeIn = 8.0;
    double kMaxRangeIn = 24.0, kMaxSkewDeg = 8.0, kMaxSpreadIn = 0.5;
    double kMoveTolIn = 0.3, kMoveTolDeg = 1.0;
    int kSamples = 4, kSamplePeriodMs = 30;
    // A move ends while the robot can still roll at up to the 6 in/s velocity exit.
    // Wait (at most kStillWaitMs) for it to be nearly stopped before sampling, so
    // the readings and the pose they are checked against describe the same spot.
    double kStillSpeedIn = 1.0, kStillTurnDeg = 5.0;
    int kStillWaitMs = 150;
    // continuous tracking (start values)
    double kContGateIn = 2.0, kContAlpha = 0.15, kContMaxStepIn = 0.15;
    double kContSkewDeg = 6.0, kContMaxBeamSpeed = 8.0, kContMaxTurnRate = 30.0;
    int kContPeriodMs = 25;
    // shared
    double kConeHalfDeg = 15.0;
    // Readings closer than this are not trusted (V5 sensor is least accurate right at
    // the face, same 2 in floor as ozzymcg's Advanced-Distance-Sensor-Reset-Command)
    double kMinRangeIn = 2.0;
    // wall squaring
    int kSquareMv = 3000;
    // Robot origin (traction axle) to the part that touches the wall (MEASURE). Final CAD: 6.50
    // to the front rail ends, 8.55 at the back of the FRAME (1x1 angles). The low lift sticks out
    // past that, see backBumperIn(). The front can NOT square at the Toggle, the spinner hits it first.
    double kFrontBumperIn = 6.50, kBackBumperIn = 8.55;   // CAD values, CHECK with a tape
    // What touches a wall behind the robot right now: the frame, or the back of the lift while it
    // is low (10.27 in folded, 10.88 at kCarry). NAN when kBackBumperIn or the lift angle is.
    double backBumperIn() const {
        const double lift = dr4b::rearIn(dr4b::angle());
        return std::isnan(lift) ? NAN : std::max(kBackBumperIn, lift);
    }
    // After the push the IMU heading must be this close to the wall heading, or the
    // robot hit the wall at an angle (or is lost) and nothing is snapped. The IMU is
    // the only judge, so a square robot whose IMU is itself further off than this is
    // skipped too. Watch for "[square] SKIPPED" and calibrate kImuScaleA/B first.
    double kSquareMaxSkewDeg = 10.0;
    // Still driving this fast at the end of the push = never reached the wall
    // (simulated: 9 to 11 in/s pushing on open tiles, under 5 against the wall).
    double kSquareMaxSpeedIn = 6.0;
    // Mounts (MEASURE rightIn / forwardIn from the robot origin, the traction axle centre, CALIBRATE
    // biasIn). A mount with any NAN is ignored, so nothing corrects until filled.
    DistanceMount mounts[3] = {
        // planned from the CAD (robot origin to sensor face), re-measure after mounting
        {&distFront(), 5.71, 6.25, 0.0, NAN},   // front, right outer rail front end, 4.5 in up
        // Sides, AS BUILT (Austin's drawing, 19 Nov): flat on the outer face of each tower, centred on
        // the tower fore and aft, window 6.6 in above the tiles, no spacer. The tower is 1.49 to 2.49 in
        // ahead of the frame centre (final CAD), so its middle is 0.49 in AHEAD of the traction axle.
        // The plan was 1.0 in behind the axle: from further forward than 0.1 the beam to the red wall
        // passes close to the wall stack Cups at the lane Goal stops and those fixes are rejected.
        // rightIn is the tower face (4.25) plus the sensor. calib::run() measures the offset along the
        // beam (biasIn), so only forwardIn has to be right here.
        {&distLeft(), -4.96, 0.49, -90.0, NAN},  // left tower outer face, 6.6 in up
        {&distRight(), 4.96, 0.49, 90.0, NAN},   // right tower outer face, 6.6 in up, chain 0.18 in outside it
    };
    bool verbose = true;

    // ---------------- stop reset
    void applyImmediateCorrectionAuto() {
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) return;
        pros::Task([this] { runStopReset(); busy_ = false; }, "wall-reset");
    }

    // Same as above, named differently so ATTICUS keeps it where it is placed
    // in a node's action list instead of moving it to the node arrival.
    void resetNow() { applyImmediateCorrectionAuto(); }

    // ---------------- continuous tracking
    void startContinuous() {
        if (contStarted_) return;
        contStarted_ = true;
        pros::Task([this] { runContinuous(); }, "wall-track");
    }
    void pauseContinuous(bool p) { contPaused_ = p; }

    // ---------------- bumper squaring. backwards = push the back bumper.
    void beginSquare(bool backwards) {
        pauseContinuous(true);
        chassis.cancelAllMotions();
        const int mv = backwards ? -kSquareMv : kSquareMv;
        leftDrive.move_voltage(mv);
        rightDrive.move_voltage(mv);
    }
    // wallHeadingDeg is the robot heading when square (0 = facing the red wall,
    // 180 = back on the red wall). Snaps heading, and the axis normal to that
    // wall if the bumper offset is measured.
    void endSquare(double wallHeadingDeg, bool backwards) {
        const arc::Pose v = arc::getSpeed();   // while still pushing
        leftDrive.move_voltage(0);
        rightDrive.move_voltage(0);
        pros::delay(20);
        const arc::Pose p = chassis.getPose();
        const double skew = std::remainder(p.theta - wallHeadingDeg, 360.0);
        const double speed = std::hypot(v.x, v.y);
        if (!(std::fabs(skew) <= kSquareMaxSkewDeg) || !(speed <= kSquareMaxSpeedIn)) {
            pauseContinuous(false);
            std::printf("[square] SKIPPED, heading %.1f is %.1f deg off the wall, speed %.1f in/s\n", p.theta, skew, speed);
            return;
        }
        double x = p.x, y = p.y;
        const double bumper = backwards ? backBumperIn() : kFrontBumperIn;
        // direction the bumper faces (toward the wall)
        const double wallDir = std::fmod(wallHeadingDeg + (backwards ? 180.0 : 0.0), 360.0);
        const int k = static_cast<int>(std::lround(wallDir / 90.0)) & 3;
        if (!std::isnan(bumper)) {
            const double c = field::kHalf - bumper;
            if (k == 0) y = c; else if (k == 1) x = c; else if (k == 2) y = -c; else x = -c;
        }
        chassis.setPose(x, y, wallHeadingDeg);
        pauseContinuous(false);
        if (verbose) std::printf("[square] heading %.1f -> %.1f, pose (%.2f, %.2f)\n", p.theta, wallHeadingDeg, x, y);
    }

    // ---------------- close on a wall by the front sensor
    // Robot facing a wall, within 8 in of it. Reads the front sensor, then creeps on the
    // tracking wheel until the part frontIn ahead of the robot origin is gapIn from the
    // wall. The wall itself sets the distance, so odometry drift and the turn before it
    // drop out. Progress is counted at the sensor, which sits beside the Toggle paddle,
    // so a small yaw while creeping drops out too. A reading that is already too close
    // backs the robot out. An uncalibrated sensor is used raw. With no reading at all it
    // uses the odometry distance to the wall. Returns false if it ended over 0.25 in off.
    int kCloseMv = 3000;          // creep voltage (TUNE: must start the robot rolling)
    int kCloseSlowMv = 1800;      // voltage over the last inch. It is raised by itself if the robot stalls
    double kCloseLeadIn = 0.20;   // cut the motors this far early, the robot rolls on a little (TUNE from the [close] line)
    double kCloseFixIn = 0.08;    // ended further than this from the target: a short nudge back (or on)
    int kCloseFixes = 3;          // at most this many nudges. With one, 12 of 63 simulated runs still ended 0.09 to 0.12 in off
                                  // and knew it, and the one that was also read 0.2 in wrong missed the Toggle (80 points)
    int kCloseSamples = 12;       // front sensor readings for the wall distance (median), 0.36 s. With 6 the reading itself was
                                  // 0.2 in off in 2 of 64 simulated runs (3 mm of sensor noise) and the Toggle was missed in 1.
                                  // With 12 and kCloseFixes 3: 128 of 128 runs strike twice, gap at the paddle 0.95 to 1.28 in
    bool closeToWall(double frontIn, double gapIn, int timeoutMs = 1500) {
        pauseContinuous(true);
        chassis.cancelAllMotions();
        Sample s[3];
        sampleAll(s, kCloseSamples);
        const arc::Pose p0 = chassis.getPose();
        const DistanceMount& m = mounts[0];
        const bool uncal = std::isnan(m.biasIn);
        const double d = s[0].med + (uncal ? 0.0 : m.biasIn);
        double range = NAN;
        const char* how = uncal ? "front sensor UNCALIBRATED (bias 0)" : "front sensor";
        if (std::isfinite(p0.x) && std::isfinite(p0.y) && std::isfinite(p0.theta) && !std::isnan(m.forwardIn)) {
            if (m.dev && s[0].spread <= kMaxSpreadIn && d <= kTrustRangeIn) {
                range = d;
            } else if (!std::isnan(m.rightIn)) {
                DistanceMount g = m;
                g.biasIn = 0.0;
                double cs, sign, ux, uy; int axis;
                if (beamToWall(p0, g, kMaxSkewDeg, kMaxRangeIn, range, cs, axis, sign, ux, uy)) how = "ODOMETRY, no front sensor reading";
                else range = NAN;
            }
        }
        if (!std::isfinite(range)) {
            pauseContinuous(false);
            std::printf("[close] SKIPPED, no wall in front by sensor or odometry\n");
            return false;
        }
        const double travel = range - (frontIn - m.forwardIn) - gapIn;   // + = forward
        const double dir = travel >= 0.0 ? 1.0 : -1.0, th = p0.theta * M_PI / 180.0;
        const double side = std::isnan(m.rightIn) ? 0.0 : m.rightIn;
        // forward progress of the sensor point: clockwise yaw moves a point on the right backwards
        auto progress = [&](const arc::Pose& p) {
            return ((p.x - p0.x) * std::sin(th) + (p.y - p0.y) * std::cos(th) -
                    side * std::sin((p.theta - p0.theta) * M_PI / 180.0)) * dir;
        };
        const auto was = leftDrive.get_brake_mode();
        leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        const auto end = pros::millis() + timeoutMs;
        double gone = 0.0, lastGone = 0.0;
        auto lastMove = pros::millis();
        int boost = 0;
        while (pros::millis() < end) {
            gone = progress(chassis.getPose());
            const double left = std::fabs(travel) - kCloseLeadIn - gone;
            if (left <= 0.0) break;
            if (gone > lastGone + 0.02) { lastGone = gone; lastMove = pros::millis(); }
            else if (pros::millis() - lastMove > 100) { boost += 200; lastMove = pros::millis(); }   // stalled, push harder
            const int mv = static_cast<int>(dir * std::min(6000, (left > 1.0 ? kCloseMv : kCloseSlowMv) + boost));
            leftDrive.move_voltage(mv);
            rightDrive.move_voltage(mv);
            pros::delay(5);
        }
        leftDrive.brake();
        rightDrive.brake();
        pros::delay(120);
        gone = progress(chassis.getPose());
        double off = travel - gone * dir;   // + = stopped short of the target
        // One nudge when it ended more than kCloseFixIn off. The robot rolls 0.1 to 0.25 in past the
        // cut-off, and the Toggle paddle's window is only 0.6 in wide. Simulated: 1 run in 30 ended
        // so close that both paddle ends struck and the Toggle finished on the wrong colour.
        int fboost = 0;   // kept from one nudge to the next: a nudge that never got the robot rolling starts the next one harder
        for (int fix = 0; fix < kCloseFixes && std::fabs(off) > kCloseFixIn; ++fix) {
            const double fdir = off > 0.0 ? 1.0 : -1.0, from = gone * dir, need = std::fabs(off) - 0.05;
            const auto fixEnd = pros::millis() + 300;
            double lastAt = from;
            auto lastT = pros::millis();
            while (pros::millis() < fixEnd) {
                const double at = progress(chassis.getPose()) * dir;   // forward travel since the start
                if ((at - from) * fdir >= need) break;
                if (std::fabs(at - lastAt) > 0.02) { lastAt = at; lastT = pros::millis(); }
                else if (pros::millis() - lastT > 60) { fboost += 200; lastT = pros::millis(); }
                const int mv = static_cast<int>(fdir * std::min(4000, kCloseSlowMv + fboost));
                leftDrive.move_voltage(mv);
                rightDrive.move_voltage(mv);
                pros::delay(5);
            }
            leftDrive.brake();
            rightDrive.brake();
            pros::delay(80);
            gone = progress(chassis.getPose());
            off = travel - gone * dir;
        }
        leftDrive.set_brake_mode_all(was);
        rightDrive.set_brake_mode_all(was);
        leftDrive.brake();   // again, so the restored mode (HOLD during a Toggle visit) takes hold
        rightDrive.brake();
        const bool ok = std::fabs(off) <= 0.25;
        pauseContinuous(false);
        std::printf("[close] %s read %.2f in, moved %.2f of %.2f in, gap now about %.2f%s\n", how, range, gone * dir, travel,
                    gapIn + off, ok ? "" : ", OFF TARGET");
        return ok;
    }

    // Median raw reading of one mount, in, NAN without enough good samples (auto_calib.hpp).
    double medianIn(int mount, int samples) {
        Sample s[3];
        sampleAll(s, samples);
        return s[mount].med;
    }

    // Robot parked square at a known distance from a wall. Prints raw medians.
    void printReadings() {
        Sample s[3];
        sampleAll(s, 8);
        for (int i = 0; i < 3; ++i)
            if (mounts[i].dev) std::printf("[dist] mount %d median %.2f in spread %.2f\n", i, s[i].med, s[i].spread);
    }

  private:
    struct Sample { double med, spread; };
    std::atomic<bool> busy_{false};
    bool contStarted_ = false;
    std::atomic<bool> contPaused_{false};

    static bool mountReady(const DistanceMount& m) {
        return m.dev && !std::isnan(m.rightIn) && !std::isnan(m.forwardIn) && !std::isnan(m.biasIn);
    }

    // Geometry for one mount at pose p. Returns false if the beam is not square
    // to a wall or the predicted beam is blocked. axis 0 = x, 1 = y. sign
    // converts (measured - predicted) into the pose correction.
    bool beamToWall(const arc::Pose& p, const DistanceMount& m, double maxSkew, double maxRange,
                    double& predRange, double& cosSkew, int& axis, double& sign, double& ux, double& uy) const {
        const double th = p.theta * M_PI / 180.0;
        const double sx = p.x + std::sin(th) * m.forwardIn + std::cos(th) * m.rightIn;
        const double sy = p.y + std::cos(th) * m.forwardIn - std::sin(th) * m.rightIn;
        double hs = std::fmod(p.theta + m.angleDeg, 360.0); if (hs < 0) hs += 360.0;
        const long q = std::lround(hs / 90.0);
        const double skewDeg = hs - 90.0 * q;
        if (std::fabs(skewDeg) > maxSkew) return false;
        cosSkew = std::cos(skewDeg * M_PI / 180.0);
        ux = std::sin(hs * M_PI / 180.0); uy = std::cos(hs * M_PI / 180.0);
        double predAlong;
        switch (static_cast<int>(q & 3)) {
            case 0: predAlong = field::kHalf - sy; axis = 1; sign = -1; break;   // +Y wall (red)
            case 1: predAlong = field::kHalf - sx; axis = 0; sign = -1; break;   // +X wall
            case 2: predAlong = sy + field::kHalf; axis = 1; sign = 1; break;    // -Y wall (blue)
            default: predAlong = sx + field::kHalf; axis = 0; sign = 1; break;   // -X wall
        }
        predRange = predAlong / cosSkew;
        if (predRange <= 0 || predRange > maxRange) return false;
        const double tc = std::tan(kConeHalfDeg * M_PI / 180.0);
        for (const auto& o : field::kObstacles) {
            const double t = (o.x - sx) * ux + (o.y - sy) * uy;
            if (t <= 0 || t >= predRange) continue;
            if (std::fabs((o.x - sx) * uy - (o.y - sy) * ux) < o.r + t * tc + 0.5) return false;
        }
        return true;
    }

    void runStopReset() {
        const uint32_t tw = pros::millis();
        while (pros::millis() - tw < static_cast<uint32_t>(kStillWaitMs)) {
            const arc::Pose v = arc::getSpeed(false);
            if (std::hypot(v.x, v.y) < kStillSpeedIn && std::fabs(v.theta) < kStillTurnDeg) break;
            pros::delay(10);
        }
        const arc::Pose p0 = chassis.getPose();
        if (!std::isfinite(p0.x) || !std::isfinite(p0.y) || !std::isfinite(p0.theta)) {
            if (verbose) std::printf("[reset] skipped, odometry pose is not finite\n");
            return;
        }
        Sample s[3];
        sampleAll(s, kSamples);
        const arc::Pose p1 = chassis.getPose();
        if (std::hypot(p1.x - p0.x, p1.y - p0.y) > kMoveTolIn || std::fabs(p1.theta - p0.theta) > kMoveTolDeg) {
            if (verbose) std::printf("[reset] skipped, robot moved while sampling\n");
            return;
        }
        // the readings span the sampling window, so compare them with its middle pose
        arc::Pose pm = p0;
        pm.x = 0.5f * (p0.x + p1.x); pm.y = 0.5f * (p0.y + p1.y);
        double c[2] = {0, 0}; int n[2] = {0, 0};
        for (int i = 0; i < 3; ++i) {
            const auto& m = mounts[i];
            if (!mountReady(m)) continue;
            double predRange, cs, sign, ux, uy; int axis;
            const char* why = "ok";
            if (!beamToWall(pm, m, kMaxSkewDeg, kMaxRangeIn + kRecoveryGateIn, predRange, cs, axis, sign, ux, uy)) why = "skip, no clear square wall in range";
            else if (std::isnan(s[i].med)) why = "skip, no valid reading";
            else if (s[i].spread > kMaxSpreadIn) why = "skip, readings noisy";
            else {
                const double d = s[i].med + m.biasIn;
                const double err = (d - predRange) * cs;
                const double gate = d < kTrustRangeIn ? kRecoveryGateIn : kGateIn;
                if (d > kMaxRangeIn) why = "skip, reading too long";
                else if (d < kMinRangeIn) why = "skip, reading too short to trust";
                else if (std::fabs(err) > gate) why = "REJECT, disagrees with odometry beyond the gate";
                else { c[axis] += sign * err; ++n[axis]; }
            }
            if (verbose) std::printf("[reset] mount %d read %.2f pred %.2f -> %s\n", i, s[i].med + m.biasIn, predRange, why);
        }
        if (!n[0] && !n[1]) return;
        const double dx = n[0] ? c[0] / n[0] : 0.0, dy = n[1] ? c[1] / n[1] : 0.0;
        arc::nudgePose(dx, dy);
        if (verbose) std::printf("[reset] applied dx %.2f dy %.2f\n", dx, dy);
    }

    void runContinuous() {
        double hist[3][3]; int fill[3] = {0, 0, 0}, head = 0;
        arc::Pose prev = chassis.getPose();
        while (true) {
            pros::delay(kContPeriodMs);
            const arc::Pose p = chassis.getPose();
            const double dt = kContPeriodMs / 1000.0;
            const double vx = (p.x - prev.x) / dt, vy = (p.y - prev.y) / dt;
            const double w = std::fabs(std::remainder(p.theta - prev.theta, 360.0)) / dt;
            prev = p;
            for (int i = 0; i < 3; ++i) {
                if (!mounts[i].dev) continue;
                const int mm = mounts[i].dev->get();
                hist[i][head] = (mm > 0 && mm < 9999) ? mm / 25.4 : NAN;
                fill[i] = std::min(3, fill[i] + 1);
            }
            head = (head + 1) % 3;
            if (contPaused_ || busy_ || w > kContMaxTurnRate) continue;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) continue;
            double c[2] = {0, 0}; int n[2] = {0, 0};
            for (int i = 0; i < 3; ++i) {
                const auto& m = mounts[i];
                if (!mountReady(m) || fill[i] < 3) continue;
                double r[3] = {hist[i][0], hist[i][1], hist[i][2]};
                if (std::isnan(r[0]) || std::isnan(r[1]) || std::isnan(r[2])) continue;
                std::sort(r, r + 3);
                if (r[2] - r[0] > kMaxSpreadIn) continue;
                double predRange, cs, sign, ux, uy; int axis;
                if (!beamToWall(p, m, kContSkewDeg, kMaxRangeIn, predRange, cs, axis, sign, ux, uy)) continue;
                if (std::fabs(vx * ux + vy * uy) > kContMaxBeamSpeed) continue;   // sensor latency would lie
                if (r[1] + m.biasIn < kMinRangeIn) continue;
                const double err = (r[1] + m.biasIn - predRange) * cs;
                if (std::fabs(err) > kContGateIn) continue;
                c[axis] += sign * err; ++n[axis];
            }
            if (!n[0] && !n[1]) continue;
            auto step = [this](double v) { return std::clamp(v * kContAlpha, -kContMaxStepIn, kContMaxStepIn); };
            const double dx = n[0] ? step(c[0] / n[0]) : 0.0, dy = n[1] ? step(c[1] / n[1]) : 0.0;
            arc::nudgePose(dx, dy);   // not setPose(p + d): p is up to a tick old by now
        }
    }

    void sampleAll(Sample out[3], int samples) {
        double r[3][16]; int n[3] = {0, 0, 0};
        for (int k = 0; k < samples && k < 16; ++k) {
            for (int i = 0; i < 3; ++i) {
                if (!mounts[i].dev) continue;
                const int mm = mounts[i].dev->get();
                if (mm > 0 && mm < 9999) r[i][n[i]++] = mm / 25.4;
            }
            pros::delay(kSamplePeriodMs);
        }
        for (int i = 0; i < 3; ++i) {
            if (n[i] < 3) { out[i] = {NAN, NAN}; continue; }
            std::sort(r[i], r[i] + n[i]);
            const int c = n[i];
            out[i] = {(c % 2) ? r[i][c / 2] : 0.5 * (r[i][c / 2 - 1] + r[i][c / 2]), r[i][c - 1] - r[i][0]};
        }
    }
};


inline WallLocalizer localizer;

namespace skills {
// Out of time for the next piece. Back straight off the Goal to (bx, by), lower the
// lift, drive through (vx, vy) and stop on (px, py), which touches the Midfield.
// The points come from gen_route.py with the rest of the route.
inline void parkMidfield(float bx, float by, float vx, float vy, float px, float py) {
    std::printf("[t %5.2f] out of time, parking in the Midfield\n", elapsedMs() / 1000.0);
    chassis.moveToPoint(bx, by, {.timeout = 600, .forwards = false});
    dr4b::to(dr4b::kCarry);
    chassis.turnToPoint(vx, vy, {.timeout = 900});
    chassis.moveToPoint(vx, vy, {.timeout = 1500});
    chassis.turnToPoint(px, py, {.timeout = 700});
    chassis.moveToPoint(px, py, {.timeout = 2500});
    rclaw::hold();
}

// Drive forward until the robot origin is reachIn from (x, y), along the line from where
// the robot is NOW. Call it right after a turnToPoint at the same (x, y). The claw then
// lands on that point even when the robot started a little off the planned line. A move
// to a fixed stop point instead swings the claw about 0.8 in sideways for every inch the
// start was off (the first Loader grab missed that way in the simulator).
//
// Re-aim (route v21). moveToPoint steers at its stop point and stops steering 6 in from it. A robot
// that is 0.2 in to the side there ends up pointing 2 deg past the Goal, and the claw is 7 to 9 in
// ahead of the robot. Measured in the simulator: the robot believed it was aimed 1 to 3.6 deg off
// at most releases, which was most of the sideways miss on every Goal. So kReaimIn before the stop
// the drive is started again from where the robot is now. The new stop point lies on the line
// from the robot through (x, y), and the steering that follows turns the robot onto it.
// Costs no time. Only for a drive that is waited for (not async), and only when it starts at
// least 4 in further out (after a chained move or a long diagonal). 0 = off.
// Keep it at 16 or more. The steering needs about 10 in to settle before it switches off 6 in
// from the stop. Started later it is still turning at that moment and the robot swings 4 to 10 deg
// past (simulated at 12: a front rail hit the Goal base in 2 of 16 runs).
inline double kReaimIn = 18.0;
// roll: end still rolling (within 2 in, under 25 in/s) for a drive that carries straight on, like
// the one to the lift gate.
// Turn toward a point and drive to it, ending loosely because a turn follows. The route's branch without
// the red Midfield Pin S uses it (gen_route.py).
inline void hop(float x, float y, int turnMs, int moveMs) {
    chassis.turnToPoint(x, y, {.timeout = turnMs, .velocityExit = 60.0f, .errorExit = 3.0f, .settleTimeMs = 0});
    chassis.moveToPoint(x, y, {.timeout = moveMs, .velocityExit = 20.0f, .errorExit = 1.5f});
}
inline void reachTo(float x, float y, float reachIn, int timeoutMs, bool async = false, bool roll = false) {
    auto drive = [&](int timeout, bool as) {
        const arc::Pose p = chassis.getPose();
        const float d = std::hypot(x - p.x, y - p.y);
        const float k = d > reachIn ? (d - reachIn) / d : 0.0f;   // already inside reach: stay put
        arc::MoveToPointParams prm{.timeout = timeout, .async = as};
        if (roll) { prm.velocityExit = 25.0f; prm.errorExit = 2.0f; }
        chassis.moveToPoint(p.x + (x - p.x) * k, p.y + (y - p.y) * k, prm);
        return d - reachIn;
    };
    auto left = [&] { const arc::Pose p = chassis.getPose(); return std::hypot(x - p.x, y - p.y) - reachIn; };
    if (async || kReaimIn <= 0.0 || left() < kReaimIn + 4.0) { drive(timeoutMs, async); return; }
    const auto t0 = pros::millis();
    drive(timeoutMs, true);
    pros::delay(10);
    while (chassis.isInMotion() && left() > kReaimIn) pros::delay(10);
    if (!chassis.isInMotion()) return;   // timed out or cancelled before the re-aim point
    chassis.cancelMotion();
    chassis.waitUntilDone();             // the first drive has let go of the chassis
    drive(std::max(300, timeoutMs - static_cast<int>(pros::millis() - t0)), false);
}

// Aim fix, at the placing stop just before the release. If the robot believes it is pointing more
// than kAimTolDeg to the side of (x, y) it turns the last bit. The claw is 5 to 9 in ahead of the
// robot, so 3 deg is 0.3 to 0.5 in at the claw. Simulated: about 1 release in 6 on the Loader lanes
// arrives that far off, and those were the ones that missed the post. A missed post costs more
// than its own Pin, because the robot counts it as placed and every later release on that Goal
// is then too high. Does nothing (and takes no time) inside the tolerance.
inline double kAimTolDeg = 3.0;
inline int kAimTimeoutMs = 351;   // the simulator knows this turn by this number, keep it odd
// tolDeg: tighter for a release that others are stacked on. The first Pin on the Tall Goal uses 1.5:
// it came in 0.8 deg to one side on average (chained approach) and missed in 4 of 63 simulated runs,
// and the two units placed on it afterwards are lost with it.
inline void aimAt(float x, float y, double tolDeg = kAimTolDeg) {
    const arc::Pose p = chassis.getPose();
    const double off = std::remainder(std::atan2(x - p.x, y - p.y) * 180.0 / M_PI - p.theta, 360.0);
    if (!(std::fabs(off) > tolDeg) || std::hypot(x - p.x, y - p.y) < 3.0) return;
    std::printf("[t %5.2f] aim %.1f deg off, turning\n", elapsedMs() / 1000.0, off);
    chassis.turnToPoint(x, y, {.timeout = kAimTimeoutMs, .velocityExit = 10.0f, .errorExit = static_cast<float>(std::min(1.0, tolDeg / 2.0)), .settleTimeMs = 40});
}

// ---- Placing on a Goal, from what is REALLY on it ----
// The route used to release at heights and stops fixed in advance. After one failed grab every
// later release on that Goal was a whole object too high. Now the robot counts what it has put
// on each Goal and works out the lift height and the stop from that count when it gets there.
// It can only know about grabs the claw sensor did not confirm. An object that was released
// but missed the post still counts as placed.
inline int onGoal[4] = {0, 0, 0, 0}; // objects on the Short, Alliance and Tall Goal (dr4b::Goal order), then slot 3:
                                     // the second red Alliance Goal (48,24). A slot of -1 below means the Goal's own.
inline bool holding = true;          // false after a grab the claw sensor did not confirm. The preload starts in the claw
inline double kClawReachIn = 6.5;    // robot origin to the held object's centre, lift down. DERIVED from the CAD carriage (front face 3.42 in
                                     // ahead of the origin), the claw is not in the CAD. MEASURE. Same as S_LOAD_IN in gen_route.py
inline double kPlaceCarryIn = 0.4;   // how far past the claw centre a released object lands, the rollers throw it forward.
                                     // ASSUMED from the simulator. MEASURE: place 5 times, see how far past the post it lands
inline double kGoalMinIn = 5.0;      // closest the origin may come to a Goal centre (4.72 seats the base in the notch)
inline double kEmptyStopIn = 14.0;   // with nothing in the claw, stop this far out so the low claw stays off the stack

inline void resetGoals() { onGoal[0] = onGoal[1] = onGoal[2] = onGoal[3] = 0; holding = true; }   // start of a run
// Call with the result of every rclaw::confirmGrip(). With no sensor value yet it assumes the grab worked.
inline void noteGrip(bool seen) {
    holding = seen || std::isnan(rclaw::kSeesObjectMm);
    if (!holding) std::printf("[t %5.2f] nothing in the claw, the next placement is skipped\n", elapsedMs() / 1000.0);
}
// units: 1 for a bare Pin, 2 for a Cup+Pin
inline double goalAngle(dr4b::Goal g, int units, int slot = -1) {
    return dr4b::levelAngle(g, onGoal[slot < 0 ? g : slot], units == 2 ? dr4b::kCupPin : dr4b::kPin);
}
inline void liftForGoal(dr4b::Goal g, int units, int slot = -1) {
    if (holding) dr4b::to(goalAngle(g, units, slot));
}
// How far from the Goal centre the robot origin stops for this release
inline float goalStopIn(dr4b::Goal g, int units, int slot = -1) {
    const double a = goalAngle(g, units, slot);
    if (!holding || std::isnan(a)) return static_cast<float>(kEmptyStopIn);
    return static_cast<float>(std::max(kGoalMinIn, kClawReachIn + kPlaceCarryIn - dr4b::clawBackIn(a)));
}
inline void releaseOn(dr4b::Goal g, int units, int slot = -1) {
    if (!holding) return;
    const int s = slot < 0 ? g : slot;
    rclaw::release();
    onGoal[s] += units;
    holding = false;
    std::printf("[t %5.2f] Goal %d now holds %d\n", elapsedMs() / 1000.0, s, onGoal[s]);
}

// Rolling lift gate, 6 in short of the placing stop on a tall stack. Up to here the claw is still
// clear of the stack. With the lift up already the robot keeps rolling in. With the lift still more
// than kGateTolDeg low it brakes here and waits, as the route did every time before (0.5 s each).
inline double kGateTolDeg = 1.5;   // 1.5 deg is 0.63 in of height near the bottom, inside kDropClearIn
inline void liftGate(int timeoutMs) {
    const double t = dr4b::target();
    if (!holding || std::isnan(t) || dr4b::angle() > t - kGateTolDeg) return;
    leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    leftDrive.brake();
    rightDrive.brake();
    std::printf("[t %5.2f] lift gate, waiting for the lift at %.1f of %.1f deg\n", elapsedMs() / 1000.0, dr4b::angle(), t);
    dr4b::waitSettled(timeoutMs);
    leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

// Toggle visit, with the robot facing the wall and the front rails a few inches off it.
// Fixes the pose off the wall, closes to gapIn with the drive held (the paddle would push
// a coasting robot back out of reach), then spins the paddle. Skipped when the lift is
// not up at kToggleUp, because the held Pin would be pressed on the wall.
inline void toggleVisit(double frontIn, double gapIn) {
    if (std::isnan(dr4b::kToggleUp) || dr4b::target().load() != dr4b::kToggleUp || !dr4b::waitSettled(400)) {
        std::printf("[toggle] SKIPPED, the lift is not up at kToggleUp\n");
        return;
    }
    localizer.resetNow();
    pros::delay(300);
    leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    localizer.closeToWall(frontIn, gapIn);
    toggle::flip();
    leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}
}  // namespace skills

// Every unset value switches a feature off without an error (a NAN lift target is
// skipped, a NAN bias means no wall fix, ...). Lists them all. Returns how many.
inline int preflight() {
    int n = 0;
    auto need = [&n](bool missing, const char* what) {
        if (missing) { std::printf("[preflight] MISSING %s\n", what); ++n; }
    };
    auto anyNan = [](const double* v, int count) {
        for (int i = 0; i < count; ++i) if (std::isnan(v[i])) return true;
        return false;
    };
    need(std::isnan(dr4b::kP) || std::isnan(dr4b::kD) || std::isnan(dr4b::kG), "dr4b kP kD kG, the lift never moves in auton");
    need(std::isnan(dr4b::kTolDeg), "dr4b::kTolDeg, every lift wait is skipped and releases happen at any height");
    need(std::isnan(dr4b::kCarry) || std::isnan(dr4b::kToggleUp), "dr4b kCarry / kToggleUp");
    need(anyNan(dr4b::kShort, 4), "dr4b::kShort[0..3] (stacking model)");
    need(anyNan(dr4b::kAlliance, 5), "dr4b::kAlliance[0..4] (stacking model)");
    need(anyNan(dr4b::kTall, 3), "dr4b::kTall[0..2] (Tall Goal seat and base height)");
    need(std::isnan(rclaw::kSeesObjectMm), "rclaw::kSeesObjectMm, every grip waits its full timeout and floor creeps never stop early");
    const char* mount[3] = {"front distance biasIn, no wall fixes from it", "left distance biasIn, no wall fixes from it",
                            "right distance biasIn, no wall fixes from it"};
    for (int i = 0; i < 3; ++i) need(std::isnan(localizer.mounts[i].biasIn), mount[i]);
    need(std::isnan(localizer.kBackBumperIn), "kBackBumperIn, the back square fixes heading only");
    std::printf("[preflight] %d missing\n", n);
    return n;
}

// The ATTICUS-generated skills routine (src/skills_dr4b.cpp).
void skillsDr4b();
