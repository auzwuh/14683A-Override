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
inline uint32_t kSkipALegAfterMs = 54000;   // A, (96,96), about 4.6 s
inline uint32_t kLastLegStartMs = 53500;    // B, (120,120), about 5.1 s
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
// PLACEHOLDER ports (negative = reversed). Free of main.cpp's 2 4 5 11 12 16 19 20.
// Replace with the real wiring before running.
namespace skills_ports {
constexpr int kLiftLeft = 3;       // PLACEHOLDER  DR4B motor, left tower
constexpr int kLiftRight = -6;     // PLACEHOLDER  DR4B motor, right tower. The two are mirrored on one shaft, so one is
                                   // reversed. CHECK which: R1 must raise the lift. If it drives down, move the minus
constexpr int kClawRollers = 10;   // PLACEHOLDER  vertical roller claw motor
constexpr int kLiftRotation = 7;   // PLACEHOLDER  rotation sensor on the stage-1 bottom pivot shaft
constexpr int kClawSensor = 8;     // PLACEHOLDER  distance sensor looking across the claw throat
constexpr int kDistFront = 9;      // PLACEHOLDER  wall sensors, see README for where they go
constexpr int kDistLeft = 13;      // PLACEHOLDER
constexpr int kDistRight = 14;     // PLACEHOLDER
constexpr int kToggleLeft = 15;    // PLACEHOLDER  5.5 W Toggle spinner motor, left brace
constexpr int kToggleRight = -17;  // PLACEHOLDER  5.5 W Toggle spinner motor, right brace (mirrored, so reversed)
constexpr bool kPortsSet = true;
static_assert(kPortsSet, "skills_support.hpp: set the ports above, then kPortsSet = true");
}  // namespace skills_ports

// ====================================================================== DR4B
namespace dr4b {
// Lift angle (deg) = stage 1 rotation from the rest pose (lift down on its stop
// at power-on). Read from the lift motor encoders through the 12T to 48T crank
// (kUseMotorEncoders) or from a rotation sensor on a stage 1 pivot.
// NAN means not tuned, and dr4b::to() ignores NAN targets.
inline bool kUseMotorEncoders = true;
// CHECK. The CAD shows 12T to 48T only, not how the 48T turns stage 1, and the 100 rpm cartridge in
// motors() is assumed. Raise the lift by hand until stage 1 is level: angle() must print about 51.
inline double kMotorDegPerLiftDeg = 4.0;
inline double kLoader = 0.0;      // pickup at the rest pose, object standing on the tiles (ASSUMED)
inline double kFloor = 0.0;       // same pickup pose for floor Cup+Pins (ASSUMED)
inline double kCarry = NAN;       // travel height with a held object. Keep it at 20 deg or less: higher, the back of
                                  // the lift sticks out past the frame and hits the wall on the Loader lane turns
inline double kToggleUp = NAN;    // claw and held Pin clear above the wall and Toggle while the paddle spins
                                  // (Toggle top 14.3 in up, 14.4 as it turns, paddle sweeps to 14.9 on the right)
// Goal placements used by the skills route, in route order. Filled by
// computeRouteTargets() from the stacking model below, do not type them in.
inline double kShort[6] = {NAN, NAN, NAN, NAN, NAN, NAN};     // Short Goal, placement 1..
inline double kAlliance[6] = {NAN, NAN, NAN, NAN, NAN, NAN};  // Alliance Goal, placement 1..
inline double kTall[3] = {NAN, NAN, NAN};                     // Tall Goal, placement 1..
// Controller (volts per deg, volts per deg/s, gravity volts at the heaviest lift
// angle, see gravityShare)
inline double kP = NAN, kD = NAN, kG = NAN;
inline double kTolDeg = NAN;
// Integral, volts per deg second. 0 = off. Tune kP, kD and kG first, then add a little if the
// lift stops a degree or two off target. It only acts inside kIZoneDeg and is capped at kIMaxV.
inline double kI = 0.0, kIZoneDeg = 3.0, kIMaxV = 2.0;
inline int kSettleMs = 60;   // waitSettled wants the lift inside kTolDeg for this long, not just passing through

inline arc::MotorGroup& motors() {
    static arc::MotorGroup m({skills_ports::kLiftLeft, skills_ports::kLiftRight}, 100.0, 1.0);
    return m;
}
inline pros::Rotation& sensor() { static pros::Rotation r(skills_ports::kLiftRotation); return r; }
// NAN when nothing can be read. An unplugged motor reports infinity and is left out, so the
// other motor's encoder still gives the angle (both are on one shaft).
inline double angle() {
    if (kUseMotorEncoders) return lift_ctrl::meanFinite(motors().get_position_all()) / kMotorDegPerLiftDeg;
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
                                                    kG * gravityShare(a), -12.0, 12.0);
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
    target() = deg;
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
inline std::vector<std::pair<double, double>> kLiftTable{};  // {angle deg, Pin bottom in}
// dr4b.step side view. Directions in deg, 0 = forward, 90 = up.
// Final CAD, dr4b (2).step. The CAD pose is NOT the rest pose (stage 1 is drawn at
// -130.0, 0.22 in above its stop). At rest the stage 1 lower bars sit on the two
// 5.5 W Toggle motors, which gives -128.7. Stage 2 is drawn at -25.1, and its real
// rest is somewhere from -24.3 to -26.4 (set by which 60T tooth is meshed), so
// MEASURE both with an angle gauge on the robot, or fill kLiftTable instead.
inline double kL1In = 11.0, kL2In = 13.5;     // stage lengths, hole to hole (22 and 27 holes)
inline double kStage1RestDir = -128.7;        // tower pivot to middle pivot, lift down on the 5.5 W motors
inline double kStage2RestDir = -25.0;         // middle pivot to front pivot, lift down
// Goal and object numbers (game object and Goal CAD, Tall Goal not known yet)
inline double kGoalTopIn[3] = {3.25, 3.25, NAN};   // top of the Goal base
inline double kSeatIn[3] = {0.085, 0.085, NAN};    // object bottom when seated in an empty Goal (3.25 - 3.165 nest)
inline double kPitchIn = 3.32;                     // 6.49 tall, 3.165 nest. Check with a tape
inline double kObjTallIn = 6.49;                   // height of one Pin or Cup standing alone
inline double kUnitBelowPinIn = 0.0;   // both picked standing on the tiles at the same pose
inline double kClawBelowObjIn = 0.25;  // lowest claw part below the held object's bottom (ASSUMED)
inline double kDropClearIn = 0.4;      // gap between the claw underside and the stack top / Goal base at release.
                                       // Must be more than the lift settle error (kTolDeg x about 0.45 in per deg)
inline double kTopDeg = 100.0;         // top of the usable stroke. Stage 2's own bars meet near 105 (final CAD). MEASURE

// Rise of the held object's bottom above its pickup height at lift angle th.
inline double riseIn(double th) {
    auto z = [](double t) {
        return kL1In * std::sin((kStage1RestDir - t) * M_PI / 180.0) +
               kL2In * std::sin((kStage2RestDir + t) * M_PI / 180.0);
    };
    return z(th) - z(0.0);
}
// Gravity load on the lift motors at lift angle a, as a share of the heaviest
// angle. The load follows how far the claw rises per lift degree, which both
// stages set (stage 1 rests 51 deg below level, stage 2 25, and L2 > L1), so the
// peak is between where the two stages are level, not where stage 1 is level.
// Assumes the claw end carries most of the weight.
inline double gravityShare(double a) {
    auto slope = [](double t) { return riseIn(t + 0.5) - riseIn(t - 0.5); };
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
    const std::pair<int, Held> ta[3] = {{0, kPin}, {1, kCupPin}, {3, kCupPin}};
    for (int i = 0; i < 4; ++i) kShort[i] = levelAngle(kShortGoal, sh[i].first, sh[i].second);
    for (int i = 0; i < 5; ++i) kAlliance[i] = levelAngle(kAllianceGoal, al[i].first, al[i].second);
    for (int i = 0; i < 3; ++i) kTall[i] = levelAngle(kTallGoal, ta[i].first, ta[i].second);
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
    // to the front rail ends, 8.55 at the back (1x1 angles on the frame; the lowered lift no
    // longer sticks out behind). The front can NOT square at the Toggle, the spinner hits it first.
    double kFrontBumperIn = NAN, kBackBumperIn = NAN;
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
        // Sides 1.0 in BEHIND the traction axle (0.5 in ahead of the frame centre). Further forward
        // than 0.1 the beam to the red wall passes too close to the wall stack Cups at all 7 lane
        // Goal stops and those resets are rejected. Works from -2.25 to 0.1.
        // Final CAD: the new 45 deg braces (right +-4.3 to 4.8) cross this spot at 7.1 to 7.8 in up,
        // and on the right the Toggle chain (right 5.13 to 5.70) runs at 6.0 to 6.4 and 7.5 to 8.1 in up.
        // The right sensor must sit outboard of the chain (right > 5.7) or clear of both runs.
        {&distLeft(), -4.96, -1.0, -90.0, NAN},  // left tower outer face, 7 in up
        {&distRight(), 4.96, -1.0, 90.0, NAN},   // right tower outer face, 7 in up
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
        const double bumper = backwards ? kBackBumperIn : kFrontBumperIn;
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
    bool closeToWall(double frontIn, double gapIn, int timeoutMs = 1500) {
        pauseContinuous(true);
        chassis.cancelAllMotions();
        Sample s[3];
        sampleAll(s, kSamples);
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
        leftDrive.set_brake_mode_all(was);
        rightDrive.set_brake_mode_all(was);
        leftDrive.brake();   // again, so the restored mode (HOLD during a Toggle visit) takes hold
        rightDrive.brake();
        gone = progress(chassis.getPose());
        const double off = travel - gone * dir;   // + = stopped short of the target
        const bool ok = std::fabs(off) <= 0.25;
        pauseContinuous(false);
        std::printf("[close] %s read %.2f in, moved %.2f of %.2f in, gap now about %.2f%s\n", how, range, gone * dir, travel,
                    gapIn + off, ok ? "" : ", OFF TARGET");
        return ok;
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
