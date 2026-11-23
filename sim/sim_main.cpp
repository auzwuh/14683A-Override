// sim_main.cpp - competition flow, robot constants, motion tracking, outputs,
// driver scenario and Monte Carlo for the 14683A host simulator.
#include "main.h"
#include "drive_mix.hpp"
#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "skills_support.hpp"
#include "auto_calib.hpp"
#include "sim_app.hpp"
#include "sim_config.hpp"
#include <algorithm>
#include <atomic>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
#include <thread>

extern arc::Chassis chassis;
extern pros::Rotation verticalEncoder;

namespace sim {
Options opt;
}
namespace simapp {
std::vector<TraceRow> trace;
bool traceOn = false;
int motionTimeouts = 0, motionsEnded = 0;
}  // namespace simapp
using namespace sim;
using namespace simapp;

namespace {
constexpr double PI = 3.14159265358979323846;

// ------------------------------------------------------------- --tuned-drive gains
// Tuned IN THE SIM against this plant model (see the selftest closed-loop rows). Exits,
// timeouts and everything else stay as main.cpp has them.
// PLANT-ASSUMPTION DEPENDENT: the plant's friction and turning scrub are assumed, not measured,
// so these are not gains for the real robot. They were picked to behave across the assumed range
// (scrub lever 0.25 to 2.0 in, drive drag 0.95 to 2.85 N/side): the integral terms are what
// push through static friction whatever its size.
arc::PIDConfig kTunedLateral{   // 24 in in 0.88 s, 5 in in 0.47 s, no overshoot, rests 0.0 to 0.3 in short over that friction range
    .proportional = {.initial = 14.0f, .final = 14.0f, .scale = 12.0f, .power = 1.0f},
    .kI = 40.0f,            // only inside 3 in
    .kD = 1.2f,
    .integralRange = 3.0f,
    .integralSignReset = true,
};
arc::PIDConfig kTunedAngular{   // 90 deg in 0.71 to 0.95 s, 180 deg in 0.91 to 1.21 s over that scrub range, about 1 deg overshoot
    .proportional = {.initial = 6.0f, .final = 3.0f, .scale = 28.0f, .power = 1.5f},
    .scheduleMode = arc::GainScheduleMode::FilteredCurrentError,
    .scheduleAlpha = 0.15f,
    .derivativeAlpha = 1.0f,
    .kI = 30.0f,            // only inside 12 deg
    .kD = 0.35f,
    .integralRange = 12.0f,
    .integralSignReset = true,
};

// ------------------------------------------------------------- robot constants (see sim_config.hpp table)
void applyWiring() {
    const auto l = leftDrive.get_port_all(), r = rightDrive.get_port_all();
    for (int i = 0; i < 2; ++i) { wiring.left[i] = std::abs(l.at(i)); wiring.right[i] = std::abs(r.at(i)); }
    // absolute ports: a negative port is the PROS reversed flag, which motorRegister() takes from the robot's own constructor
    wiring.lift[0] = std::abs(skills_ports::kLiftLeft); wiring.lift[1] = std::abs(skills_ports::kLiftRight);
    wiring.toggle[0] = std::abs(skills_ports::kToggleLeft); wiring.toggle[1] = std::abs(skills_ports::kToggleRight);
    wiring.claw = skills_ports::kClawRollers;
    wiring.rot = verticalEncoder.get_port();
    wiring.imu[0] = 10; wiring.imu[1] = 20;   // main.cpp kImuPortA / kImuPortB (file-local constants there)
    wiring.dist[0] = skills_ports::kDistFront; wiring.dist[1] = skills_ports::kDistLeft;
    wiring.dist[2] = skills_ports::kDistRight; wiring.dist[3] = skills_ports::kClawSensor;
}
void applyRobotConstants() {
    // the robot code has lift gains now: only fill them if it leaves them unset
    if (std::isnan(dr4b::kP)) dr4b::kP = 0.8;
    if (std::isnan(dr4b::kD)) dr4b::kD = 0.012;
    // "raise the volts until a Cup+Pin holds at 35 deg" on this seed's plant, with the error a hand tuning leaves
    (void)dr4b::motors();   // registers the lift motors, so liftHoldVolts knows their cartridge
    dr4b::kG = liftHoldVolts(35.0, true) * (1.0 + truth.kGErr);
    if (std::isnan(dr4b::kTolDeg)) dr4b::kTolDeg = 1.5;   // the robot code has its own now
    if (opt.liftKp >= 0) {   // --lift-gains kP,kD,kI[,kG scale]
        dr4b::kP = opt.liftKp; dr4b::kD = opt.liftKd; dr4b::kI = opt.liftKi; dr4b::kG *= opt.liftKgScale;
    }
    // kToggleUp = "claw and held Pin clear above the wall and Toggle while the paddle spins". A team raises the lift
    // until the claw clears the turning Toggle by about an inch, so it is solved on THIS seed's lift: the encoder angle
    // (arm angle + slop) that puts the claw underside (0.25 in below the held Pin) kToggleClearIn over the top of the
    // turning Toggle, 14.43 in.
    if (opt.clockShiftMs > 0) {   // what-if: every clock limit later by this much (use with --auton-ms), as if the robot were that much faster
        for (uint32_t* p : {&skills::kSkipALegAfterMs, &skills::kLastLegStartMs, &skills::kMidWStartMs, &skills::kMidSStartMs, &skills::kD24StartMs, &skills::kLoaderCycleStartMs})
            *p += static_cast<uint32_t>(opt.clockShiftMs);
    }
    const auto encoderDegForRise = [](double riseIn) {
        double lo = 0.0, hi = simcfg::kLiftTopDeg;
        for (int i = 0; i < 50; ++i) { const double m = 0.5 * (lo + hi); (liftRiseIn(m) < riseIn ? lo : hi) = m; }
        return 0.5 * (lo + hi) + truth.liftSlopDeg;
    };
    const double toggleUpRise = simcfg::kToggleTopTurningIn + simcfg::kToggleClearIn + dr4b::kClawBelowObjIn;
    dr4b::kToggleUp = encoderDegForRise(toggleUpRise);
    if (opt.flipTurns >= 0) {   // sweep only: which kFlipTurns would end red
        toggle::kFlipTurns = opt.flipTurns;
        logLine(fmt("[sim] EXPERIMENT: toggle::kFlipTurns set to %.2f from the sim side (the code has 2.0)", opt.flipTurns));
    }
    if (opt.closeLeadIn >= 0 || opt.closeMv >= 0 || opt.closeSlowMv >= 0) {   // sweep only: which closeToWall() values would do better
        if (opt.closeLeadIn >= 0) localizer.kCloseLeadIn = opt.closeLeadIn;
        if (opt.closeMv >= 0) localizer.kCloseMv = static_cast<int>(opt.closeMv);
        if (opt.closeSlowMv >= 0) localizer.kCloseSlowMv = static_cast<int>(opt.closeSlowMv);
        logLine("[sim] EXPERIMENT: WallLocalizer closeToWall() constants set from the sim side (the code has 3000 / 1800 mV, lead 0.10 in)");
    }
    dr4b::kGoalTopIn[dr4b::kTallGoal] = 8.77;
    dr4b::kSeatIn[dr4b::kTallGoal] = 8.77 - 3.165;
    if (!opt.selfCal) {   // with --self-cal the robot has to find these itself
        rclaw::kSeesObjectMm = 70.0;
        for (int i = 0; i < 3; ++i) localizer.mounts[i].biasIn = truth.calBias[i];
    }
    // measured from the robot origin (traction axle), as the header asks: 6.50 to the front rails, 8.55 to the back
    localizer.kFrontBumperIn = simcfg::kFrontIn - simcfg::kAxleFwdIn;
    localizer.kBackBumperIn = simcfg::kRouteRearIn + simcfg::kAxleFwdIn;   // the same belief the route's start pose is built on
    if (opt.noContTrack) {   // experiment only: every continuous-tracking reading fails its gate
        localizer.kContGateIn = -1.0;
        logLine("[sim] EXPERIMENT: continuous wall tracking corrections disabled (kContGateIn = -1)");
    }
    if (opt.tunedDrive) {
        chassis.lateralPID = arc::PID(kTunedLateral);
        chassis.angularPID = arc::PID(kTunedAngular);
        logLine(fmt("[sim] --tuned-drive: lateral Kp %.2f kI %.2f kD %.2f iRange %.1f | angular Kp %.2f..%.2f kI %.1f kD %.3f iRange %.0f dAlpha %.2f "
                    "(sim-tuned against an ASSUMED plant, replaces main.cpp gains; not gains for the real robot)",
                    kTunedLateral.proportional.initial, kTunedLateral.kI, kTunedLateral.kD, kTunedLateral.integralRange, kTunedAngular.proportional.initial,
                    kTunedAngular.proportional.final, kTunedAngular.kI, kTunedAngular.kD, kTunedAngular.integralRange, kTunedAngular.derivativeAlpha));
    } else logLine("[sim] drive PID gains: as in src/main.cpp (lateral Kp 5.73 kI 0.88 kD 0.01, angular Kp 2.6..4.6 kD 0.157)");
    logLine(fmt("[sim] robot constants: kP %.2f kD %.3f kG %.2f kTol %.1f kCarry %.0f kToggleUp %.1f (Pin bottom %.2f in up, claw underside %.2f) "
                "TallTop %.2f TallSeat %.3f kSeesObjectMm %.0f bias {%.3f %.3f %.3f} bumpers %.2f/%.2f | as the code has them: kFlipTurns %.2f, "
                "kCloseMv %d, kCloseSlowMv %d, kCloseLeadIn %.2f",
                dr4b::kP, dr4b::kD, dr4b::kG, dr4b::kTolDeg, dr4b::kCarry, dr4b::kToggleUp, toggleUpRise, toggleUpRise - dr4b::kClawBelowObjIn,
                dr4b::kGoalTopIn[2], dr4b::kSeatIn[2], rclaw::kSeesObjectMm, truth.calBias[0], truth.calBias[1],
                truth.calBias[2], localizer.kFrontBumperIn, localizer.kBackBumperIn, toggle::kFlipTurns, localizer.kCloseMv, localizer.kCloseSlowMv,
                localizer.kCloseLeadIn));
}

// ------------------------------------------------------------- planned motions (labels parsed from the route)
struct Planned { std::string kind; double x, y; int timeout; int line; double reach = 0; };   // reach: skills::reachTo only
std::vector<Planned> planned;
const int kAimTimeoutMs = 351;   // skills::kAimTimeoutMs: how the optional aim turn is told from the next route motion
struct Check { size_t motionsBefore; std::string limit; int line; };
std::vector<Check> checks;
void parseRoute() {
    std::ifstream f(opt.repoDir + "/src/skills_dr4b.cpp");
    std::string s;
    int ln = 0;
    const std::regex mv(R"(chassis\.(moveToPoint|turnToPoint|turnToHeading)\(\s*([-0-9.]+)f?(?:,\s*([-0-9.]+)f?)?(.*)\);)");
    const std::regex to(R"(\.timeout\s*=\s*(\d+))");
    const std::regex rt(R"(skills::reachTo\(\s*([-0-9.]+)f?,\s*([-0-9.]+)f?,\s*(.+),\s*(\d+)(?:,\s*(?:true|false))*\);)");   // reach: a number, or worked out at run time
    const std::regex am(R"(skills::aimAt\(\s*([-0-9.]+)f?,\s*([-0-9.]+)f?(?:,\s*[-0-9.]+f?)?\);)");   // a turn that only happens when the aim is off
    const std::regex ck(R"(skills::elapsedMs\(\)\s*>\s*skills::(\w+))");
    while (std::getline(f, s)) {
        ++ln;
        std::smatch m, t;
        if (std::regex_search(s, m, mv)) {
            const std::string rest = m[4];
            int timeout = m[1] == "moveToPoint" ? 2200 : 2400;
            if (std::regex_search(rest, t, to)) timeout = std::stoi(t[1]);
            planned.push_back({m[1], std::stod(m[2]), m[3].matched ? std::stod(m[3]) : 0.0, timeout, ln});   // turnToHeading: x = heading
        } else if (std::regex_search(s, m, rt)) {   // one moveToPoint inside the helper, to a point worked out at run time
            const std::string reach = m[3];
            const bool number = reach.find_first_not_of("-0123456789.f") == std::string::npos;
            planned.push_back({"reachTo", std::stod(m[1]), std::stod(m[2]), std::stoi(m[4]), ln, number ? std::stod(reach) : 0.0});   // 0: end error is the distance left to the point
        } else if (std::regex_search(s, m, am)) {
            planned.push_back({"aimAt", std::stod(m[1]), std::stod(m[2]), kAimTimeoutMs, ln});
        } else if (std::regex_search(s, m, ck)) {
            checks.push_back({planned.size(), m[1], ln});
        }
    }
    logLine(fmt("[sim] parsed %zu motions and %zu clock checks from src/skills_dr4b.cpp", planned.size(), checks.size()));
}
struct MotionRec { size_t idx; uint64_t start, end = 0; bool cancelled = false, timedOut = false, route = false, park = false; double endErr = 0; bool matched = false; };
std::vector<MotionRec> motions;
size_t routeMotions = 0, parkMotions = 0;   // parkMotions: started inside skills::parkMidfield, not in the route file
int routeTimedOut = 0, routeCancelled = 0;
thread_local bool tlTimedOut = false;
size_t reaimOf = static_cast<size_t>(-1);

enum Phase { kInit, kSelftest, kAuton, kCoast, kGap, kDriver };   // kCoast: disabled after the 60 s cut, rolling to rest
Phase phase = kInit;

arc::Pose odomPose() { return arc::getPose(false); }
double trueDeg() { return body.th * 180.0 / PI; }
double headErr(const arc::Pose& p) { return std::remainder(p.theta - trueDeg(), 360.0); }
}  // namespace

// ------------------------------------------------------------- link-time wraps (-Wl,--wrap)
extern "C" {
void __real__ZN3arc7Chassis18requestMotionStartEv(arc::Chassis*);
void __real__ZN3arc7Chassis9endMotionEv(arc::Chassis*);
void __real__ZN3arc7Chassis7setPoseEfffb(arc::Chassis*, float, float, float, bool);
void __real__ZN3arc9nudgePoseEdd(double, double);
bool __real__ZN3arc13ExitCondition4timeEji(std::uint32_t, int);

void __wrap__ZN3arc7Chassis18requestMotionStartEv(arc::Chassis* c) {
    __real__ZN3arc7Chassis18requestMotionStartEv(c);
    tlTimedOut = false;
    MotionRec r;
    r.route = phase == kAuton;
    r.park = r.route && counters.parkAtMs;
    // skills::reachTo() re-aims: it cancels its drive part way and starts it again at once. That second
    // start is the same route line, so it keeps the label of the first.
    const bool reaim = r.route && !r.park && !motions.empty() && motions.back().route && !motions.back().park && motions.back().cancelled
                       && motions.back().idx < planned.size() && planned[motions.back().idx].kind == "reachTo" && now() - motions.back().end <= 40
                       && reaimOf != motions.back().idx;
    if (reaim) reaimOf = motions.back().idx;   // one re-aim per reachTo
    r.idx = !r.route ? 0 : r.park ? parkMotions++ : reaim ? motions.back().idx : routeMotions++;
    r.start = now();
    motions.push_back(r);
}
bool __wrap__ZN3arc13ExitCondition4timeEji(std::uint32_t start, int timeout) {
    // skills::aimAt() lines are in the route but its turn only runs when the aim is off. The first
    // timeout check of a motion tells which it is: step the label past aim turns that did not happen.
    if (!motions.empty() && motions.back().end == 0 && !motions.back().matched && motions.back().route && !motions.back().park) {
        MotionRec& r = motions.back();
        r.matched = true;
        while (r.idx < planned.size() && planned[r.idx].kind == "aimAt" && timeout != kAimTimeoutMs) { ++r.idx; ++routeMotions; }
    }
    const bool running = __real__ZN3arc13ExitCondition4timeEji(start, timeout);
    if (!running) tlTimedOut = true;
    return running;
}
void __wrap__ZN3arc7Chassis9endMotionEv(arc::Chassis* c) {
    if (!motions.empty() && motions.back().end == 0) {
        MotionRec& r = motions.back();
        r.end = now();
        r.cancelled = !c->isInMotion();
        r.timedOut = tlTimedOut && !r.cancelled;
        tlTimedOut = false;
        ++motionsEnded;
        if (r.timedOut) ++motionTimeouts;
        const Planned* p = r.route && !r.park && r.idx < planned.size() ? &planned[r.idx] : nullptr;
        if (r.route) { routeTimedOut += r.timedOut; routeCancelled += r.cancelled; }
        const int dur = static_cast<int>(r.end - r.start);
        if (p) {
            // true distance from the target at the end (moves) or true heading error (turns)
            if (p->kind == "moveToPoint") r.endErr = std::hypot(body.x - p->x, body.y - p->y);
            else if (p->kind == "reachTo") r.endErr = std::hypot(body.x - p->x, body.y - p->y) - p->reach;
            else if (p->kind == "turnToHeading") r.endErr = std::remainder(p->x - trueDeg(), 360.0);
            else {
                const double b = std::atan2(p->x - body.x, p->y - body.y) * 180.0 / PI;
                r.endErr = std::remainder(b - trueDeg(), 180.0);   // mod 180: forwards or backwards facing
            }
            event(fmt("\"type\": \"motion\", \"i\": %zu, \"kind\": \"%s\", \"target\": [%.2f, %.2f], \"line\": %d, \"timeout_ms\": %d, "
                      "\"dur_ms\": %d, \"timed_out\": %s, \"cancelled\": %s, \"true_end_err\": %.3f, \"true_speed_in_s\": %.1f",
                      r.idx, p->kind.c_str(), p->x, p->y, p->line, p->timeout, dur, r.timedOut ? "true" : "false",
                      r.cancelled ? "true" : "false", r.endErr, body.v / simcfg::IN));
            if (r.timedOut)
                logLine(fmt("[motion] TIMEOUT line %d %s(%.2f, %.2f) after %d ms: true %s error %.2f, still moving %.1f in/s %.0f deg/s",
                            p->line, p->kind.c_str(), p->x, p->y, dur, p->kind == "moveToPoint" || p->kind == "reachTo" ? "distance" : "heading", r.endErr,
                            body.v / simcfg::IN, body.w * 180.0 / PI));
        } else if (r.park && r.timedOut)
            logLine(fmt("[motion] TIMEOUT parkMidfield motion %zu of 5 after %d ms", r.idx + 1, dur));
    }
    __real__ZN3arc7Chassis9endMotionEv(c);
}
void notePoseWrite(float x, float y, float th) {
    if (phase == kAuton) {
        const arc::Pose before = odomPose();
        const std::string task = currentTaskName();
        const char* src = task == "wall-track" ? "track" : task == "wall-reset" ? "stop_reset" : counters.setPoseRoute == 0 ? "route" : "square";
        const double errBefore = std::hypot(before.x - body.x, before.y - body.y);
        const double errAfter = std::hypot(x - body.x, y - body.y);
        if (!std::strcmp(src, "track")) ++counters.setPoseTrack;
        else if (!std::strcmp(src, "stop_reset")) ++counters.setPoseReset;
        else if (!std::strcmp(src, "square")) ++counters.setPoseSquare;
        else ++counters.setPoseRoute;
        if (std::strcmp(src, "track")) {
            event(fmt("\"type\": \"set_pose\", \"source\": \"%s\", \"odom_before\": [%.3f, %.3f, %.2f], \"new\": [%.3f, %.3f, %.2f], "
                      "\"true\": [%.3f, %.3f, %.2f], \"xy_err_before\": %.3f, \"xy_err_after\": %.3f",
                      src, before.x, before.y, before.theta, x, y, th, body.x, body.y, trueDeg(), errBefore, errAfter));
            logLine(fmt("[pose] %s: odom (%.2f, %.2f, %.1f) -> (%.2f, %.2f, %.1f), true (%.2f, %.2f, %.1f), xy error %.2f -> %.2f in",
                        src, before.x, before.y, before.theta, x, y, th, body.x, body.y, trueDeg(), errBefore, errAfter));
        }
    }
}
void __wrap__ZN3arc7Chassis7setPoseEfffb(arc::Chassis* c, float x, float y, float th, bool rad) {
    notePoseWrite(x, y, th);
    __real__ZN3arc7Chassis7setPoseEfffb(c, x, y, th, rad);
}
// wall fixes (stop resets, continuous tracking) shift the pose with arc::nudgePose
void __wrap__ZN3arc9nudgePoseEdd(double dx, double dy) {
    const arc::Pose before = odomPose();
    notePoseWrite(before.x + dx, before.y + dy, before.theta);
    __real__ZN3arc9nudgePoseEdd(dx, dy);
}
}

namespace {
// ------------------------------------------------------------- run state
int initId = -1, autonId = -1, driverId = -1;
uint64_t autonStart = 0, autonReturned = 0, driverStart = 0, gapStart = 0, initDoneAt = 0, cutAt = 0, restAt = 0;
double speedAtCut = 0, yawRateAtCut = 0, cutX = 0, cutY = 0, cutTh = 0;
bool midfieldAtCut = false;
std::string knobTag, knobJson;   // truth knobs given on the command line
FILE* csv = nullptr;
double odomErrMax = 0, odomErrFinal = 0, headErrFinal = 0, headErrMax = 0, odomErrSum = 0;
long odomErrN = 0;
bool midfieldAtEnd = false, autonCut = false;
int pinsAtEnd = 0, pinsOwnedAtEnd = 0, pointsAtEnd = 0, pointsIfOwned = 0;
double endX = 0, endY = 0, endTh = 0;

// ---- driver scenario (requirement 9)
struct Seg { int ms, ly, rx; bool r1, r2, l1, l2; const char* name; };
const Seg kSegs[] = {
    {400, 0, 0, 0, 0, 0, 0, "idle (leftover auton motion must be cancelled)"},
    {600, 127, 0, 0, 0, 0, 0, "full forward"},
    {600, 127, 64, 0, 0, 0, 0, "forward + half right"},
    {500, 127, 127, 0, 0, 0, 0, "forward + full right"},
    {600, 0, 0, 0, 0, 0, 0, "idle"},
    {600, 0, 64, 0, 0, 0, 0, "half right, standing"},
    {500, 0, 127, 0, 0, 0, 0, "point turn full right"},
    {600, 0, 0, 0, 0, 0, 0, "idle"},
    {500, -127, 0, 0, 0, 0, 0, "full reverse"},
    {600, 0, 0, 0, 0, 0, 0, "idle"},
    {3000, 0, 0, 1, 0, 0, 0, "R1 lift up into the top"},
    {3000, 0, 0, 0, 1, 0, 0, "R2 lift down into the bottom"},
    {800, 0, 0, 0, 0, 1, 0, "L1 claw intake"},
    {800, 0, 0, 0, 0, 0, 1, "L2 claw outtake"},
    {1000, 0, 0, 0, 0, 0, 0, "idle, claw hold"},
};
constexpr int kNSeg = sizeof(kSegs) / sizeof(kSegs[0]);
struct SegStat { int n = 0, nLate = 0; double v = 0, w = 0, vLate = 0, wLate = 0, vmin = 1e9, liftMin = 1e9, liftMax = -1e9; int cmdL = 0, cmdR = 0; double voltL = 0, voltR = 0, claw = 0; };
SegStat segStat[kNSeg];
double driverLiftMin = 1e9, driverLiftMax = -1e9;
bool leftoverStarted = false, leftoverCancelled = false;
int motionsEndedAtGap = 0;
double leftoverSpeedAtIdleEnd = 0;

double liftMeasured() { return 0.5 * (motorReadPos(wiring.lift[0]) + motorReadPos(wiring.lift[1])) / dr4b::kMotorDegPerLiftDeg; }
double sideVolt(const int* ports) { const MotorDev& m = motor(ports[1]); return m.mount * m.volt; }

void writeCsv(uint64_t t) {
    if (traceOn) trace.push_back({t / 1000.0, body.x, body.y, trueDeg(), body.v / simcfg::IN, body.w * 180.0 / PI, lift.a * 180.0 / PI});
    if (!csv || t % 10) return;
    const arc::Pose p = odomPose();
    const char* ph = phase == kInit ? "init" : phase == kAuton ? "auton" : phase == kSelftest ? "selftest" :
                     phase == kCoast || phase == kGap ? "disabled" : "driver";
    std::fprintf(csv, "%.3f,%s,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d\n", t / 1000.0, ph,
                 body.x, body.y, trueDeg(), p.x, p.y, p.theta, body.v / simcfg::IN, body.w * 180.0 / PI,
                 lift.a * 180.0 / PI, liftMeasured(), sideVolt(wiring.left), sideVolt(wiring.right),
                 motor(wiring.claw).mount * motor(wiring.claw).volt, body.contact, holdingObject() ? 1 : 0);
    if (phase == kAuton) {
        const double e = std::hypot(p.x - body.x, p.y - body.y);
        odomErrMax = std::max(odomErrMax, e);
        odomErrSum += e; ++odomErrN;
        headErrMax = std::max(headErrMax, std::fabs(headErr(p)));
    }
}

std::string jstr(const std::string& s) {
    std::string o;
    for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
    return o;
}
const char* B(bool b) { return b ? "true" : "false"; }

[[noreturn]] void finish(const char* why) {
    logLine(std::string("[sim] finished: ") + why);
    const std::string base = opt.outDir + "/";
    {
        std::ofstream f(base + "events_" + opt.tag + ".json");
        f << "[\n";
        const auto& ev = allEvents();
        for (size_t i = 0; i < ev.size(); ++i) f << "  " << ev[i] << (i + 1 < ev.size() ? ",\n" : "\n");
        f << "]\n";
    }
    const auto goals = goalResults();
    std::ostringstream js, kv;
    js.setf(std::ios::fixed);
    js.precision(3);
    js << "{\n  \"tag\": \"" << opt.tag << "\", \"seed\": " << opt.seed << ", \"noise\": " << B(opt.noise)
       << ", \"drive_gains\": \"" << (opt.tunedDrive ? "sim-tuned (--tuned-drive)" : "main.cpp as written") << "\",\n";
    js << "  \"scenario\": {\"goal_start_pins\": " << B(opt.goalPins) << ", \"draw_heights\": " << B(opt.drawHeights)
       << ", \"clusters_pushable\": " << B(opt.clusters) << ", \"rear_contact_in\": " << truth.rearIn << ", \"track_wheel_dia_in\": "
       << truth.wheelDia << ", \"imu_gain\": [" << sim::imu(wiring.imu[0]).gain << ", " << sim::imu(wiring.imu[1]).gain << "], \"loader_x_in\": "
       << truth.loaderX << ", \"continuous_tracking\": " << B(!opt.noContTrack) << ", \"knobs\": {" << knobJson << "}},\n";
    js << "  \"plant_assumed_not_measured\": {\"scrub_lever_in\": " << truth.scrubLeverIn << ", \"drive_drag_n_per_side\": " << truth.driveFricN
       << ", \"mass_kg\": " << truth.massKg << ", \"lift_rest_deg\": [" << truth.rest1Deg << ", " << truth.rest2Deg << "], \"lift_slop_deg\": "
       << truth.liftSlopDeg << ", \"mid_stage_kg\": " << truth.midKg << ", \"kG_tuning_error\": " << truth.kGErr << ", \"stack_pitch_in\": " << truth.pitch
       << ", \"capture_r_in\": " << truth.captureR << ", \"grab_lat_in\": " << truth.grabLatIn << ", \"grab_ahead_in\": " << truth.grabAheadIn
       << ", \"release_ms\": " << truth.releaseMs << ", \"release_fwd_in\": " << truth.releaseFwdIn << ", \"imu_jitter_ms\": " << truth.imuJitterMs
       << ", \"imu_phase_ms\": [" << truth.phaseImu[0] << ", " << truth.phaseImu[1] << "], \"motor_phase_ms\": " << truth.phaseMotor
       << ", \"rotation_phase_ms\": " << truth.phaseRot << ", \"auton_start_delay_ms\": " << truth.autonDelayMs
       << ", \"wiring\": \"motor reversal flags, port numbers and sensor signs are taken from the robot code, not checked against hardware\"},\n";
    js << "  \"at_60s_cut\": {\"true_pose\": [" << cutX << ", " << cutY << ", " << cutTh << "], \"true_speed_in_s\": " << speedAtCut
       << ", \"yaw_rate_deg_s\": " << yawRateAtCut << ", \"midfield_at_cut\": " << B(midfieldAtCut) << ", \"rolled_on_for_ms\": " << (restAt - cutAt)
       << ", \"note\": \"robot disabled at the cut (motors coast, ASSUMED); the field is scored once it is at rest\"},\n";
    js << "  \"imu_fusion\": {\"robot_reads_of_both_sensors\": " << counters.imuTicks << ", \"reads_over_the_disagreement_gate\": " << counters.imuDisagree
       << ", \"note\": \"sim-side copy of the 2 deg / 5 % gate in imu_fusion.hpp; over the gate the fuser keeps one sensor's change and discards the other\"},\n";
    js << "  \"scoring_rule\": \"README: 10 per Pin the robot seated on a Goal (each Cup hides one half, so n Pins show n halves; yellow up assumed), "
          "+8 if any part of the robot is over the Midfield diamond at 60 s. 'points' also requires the red-wall Toggle to be red "
          "for the two Goals in that Quadrant (G1, G2); Tall Goal Pins always count (ownership rule not in any source). 'points_if_owned' ignores the Toggle. "
          "Pins already on a Goal at the start are not counted. ASSUMPTIONS, not findings: any object seats on any stack or bare Goal base; an object must be "
          "released with its bottom at or above the top of what is on the Goal; every paddle end that passes through the Toggle moves it exactly one state "
          "(yellow, blue, red, yellow...); a stack that gets hit keeps its Pins. Scored with the robot at rest after the 60 s cut.\",\n";
    const char* toggleCol[3] = {"yellow", "blue", "red"};
    js << "  \"toggle\": {\"end_state\": \"" << toggleCol[toggleState()] << "\", \"paddle_strikes\": " << counters.toggleSweeps << ", \"flip_calls\": "
       << counters.toggleFlips << ", \"true_rail_gap_at_flip_in\": " << counters.flipGapIn << ", \"true_gap_at_the_paddle_in\": " << counters.flipGapPaddleIn
       << ", \"rail_gap_left_right_in\": [" << counters.flipGapLeftIn << ", " << counters.flipGapRightIn << "], \"true_heading_at_flip_deg\": "
       << counters.flipHeadDeg << ", \"claw_underside_at_flip_in\": " << counters.flipClawUnderIn << ", \"paddle_start_deg\": " << truth.paddleDeg0
       << ", \"paddle_deg_at_flip\": " << counters.flipPaddleDeg << ",\n    \"close_to_wall\": {\"by_front_sensor\": " << counters.closeSensor
       << ", \"by_odometry\": " << counters.closeOdom << ", \"skipped\": " << counters.closeSkipped << ", \"true_rail_gap_after_in\": " << counters.closeGapIn
       << "}, \"paddle_blocked_by_wall_ms\": " << counters.toggleJamMs << ", \"chain_or_plates_on_toggle_ms\": " << counters.toggleBumpMs
       << ", \"claw_or_object_on_wall_or_toggle_ms\": " << counters.clawWallMs << ", \"flip_ended_inside_toggle\": " << counters.toggleStoppedInside
       << ", \"idle_paddle_pushed_by_robot\": " << counters.togglePushedPaddle
       << ",\n    \"note\": \"gap -1 = never called. One paddle end reaches the Toggle from 0.92 to 1.41 in, both ends inside 0.92, the wall blocks the paddle inside 0.80\"},\n";
    js << "  \"points\": " << pointsAtEnd << ", \"points_if_owned\": " << pointsIfOwned << ", \"pins_seated\": " << pinsAtEnd
       << ", \"pins_owned\": " << pinsOwnedAtEnd << ", \"toggle_sweeps\": " << counters.toggleSweeps << ", \"midfield_at_end\": " << B(midfieldAtEnd) << ",\n";
    js << "  \"route_end_s\": " << (autonReturned ? (autonReturned - autonStart) / 1000.0 : -1.0)
       << ", \"route_completed_in_60s\": " << B(!autonCut)
       << ", \"skills_clock_at_return_s\": " << (autonReturned ? (autonReturned - skills::startMs()) / 1000.0 : -1.0) << ",\n";
    js << "  \"end_pose_true\": [" << endX << ", " << endY << ", " << endTh << "],\n";
    js << "  \"goals\": [";
    for (size_t i = 0; i < goals.size(); ++i)
        js << (i ? ", " : "") << "{\"name\": \"" << goals[i].name << "\", \"type\": \"" << goals[i].type << "\", \"pins_seated\": "
           << goals[i].pins << ", \"start_pins\": " << goals[i].startPins << ", \"objects\": " << goals[i].objects << "}";
    js << "],\n  \"placements\": [";
    for (size_t i = 0; i < placeRecs.size(); ++i) {
        const auto& p = placeRecs[i];
        js << (i ? ",\n    " : "\n    ") << "{\"n\": " << p.n << ", \"t\": " << p.t << ", \"object\": \"" << jstr(p.object) << "\", \"goal\": \""
           << jstr(p.goal) << "\", \"ok\": " << B(p.ok) << ", \"why\": \"" << jstr(p.why) << "\", \"radial_in\": " << p.radial
           << ", \"along_in\": " << p.along << ", \"across_in\": " << p.across << ", \"bottom_in\": " << p.bottom
           << ", \"stack_top_in\": " << p.top << ", \"lift_deg\": " << p.liftDeg << "}";
    }
    js << "],\n";
    js << "  \"clock_checks\": [";
    for (size_t i = 0; i < checks.size(); ++i) {
        const auto& c = checks[i];
        std::string res = "not reached";
        double el = -1;
        const MotionRec* next = nullptr;
        for (const auto& m : motions) if (m.route && !m.park && m.idx == c.motionsBefore) next = &m;
        if (next) { res = "continued"; el = (next->start - skills::startMs()) / 1000.0; }
        else if ((autonReturned || counters.parkAtMs) && routeMotions == c.motionsBefore) { res = "stopped the route here"; el = ((counters.parkAtMs ? counters.parkAtMs : autonReturned) - skills::startMs()) / 1000.0; }
        js << (i ? ", " : "") << "{\"limit\": \"" << c.limit << "\", \"route_line\": " << c.line << ", \"result\": \"" << res << "\", \"elapsed_s\": " << el << "}";
        kv << "check_" << i << " " << (res == "continued" ? 1 : res == "stopped the route here" ? 0 : -1) << "\n";
    }
    js << "],\n";
    js << "  \"odometry\": {\"xy_err_max_in\": " << odomErrMax << ", \"xy_err_mean_in\": " << (odomErrN ? odomErrSum / odomErrN : 0.0)
       << ", \"xy_err_final_in\": " << odomErrFinal << ", \"heading_err_max_deg\": " << headErrMax << ", \"heading_err_final_deg\": " << headErrFinal << "},\n";
    js << "  \"resets\": {\"stop_resets_applied\": " << counters.resetApplied << ", \"stop_resets_skipped\": " << counters.resetSkipped
       << ", \"mount_readings_ok\": " << counters.resetMountOk << ", \"mount_readings_rejected\": " << counters.resetMountReject
       << ", \"mount_readings_skipped\": " << counters.resetMountSkip << ", \"squares\": " << counters.squares
       << ", \"continuous_track_corrections\": " << counters.setPoseTrack << "},\n";
    js << "  \"motions\": {\"count\": " << routeMotions + parkMotions << ", \"planned\": " << planned.size() << ", \"timed_out\": " << routeTimedOut
       << ", \"cancelled\": " << routeCancelled << "},\n";
    js << "  \"grabs\": {\"ok\": " << counters.grabs << ", \"miss\": " << counters.grabMiss << ", \"tipped\": " << counters.tipped
       << ", \"grip_ok_lines\": " << counters.gripOk << ", \"grip_not_seen_lines\": " << counters.gripNotSeen << "},\n";
    js << "  \"loose_objects_pushed\": " << looseSummary() << ",\n";
    js << "  \"held_object_hit_goal_or_stack\": " << counters.stackHits << ", \"of_those_scored_stacks\": " << counters.scoredStackHits
       << ", \"objects_left_on_the_tiles\": " << counters.dropped << ", \"ms_over_rough_tip_threshold\": " << counters.tipRiskMs << ",\n";
    js << "  \"lift_not_settled\": " << counters.liftNotSettled
       << ", \"untuned_lift_targets\": " << counters.untunedTargets << ", \"preflight_missing\": " << counters.preflightMissing << ",\n";
    js << "  \"lift_true_max_deg\": " << lift.maxDeg;
    if (opt.driver) {
        const auto late = [](const SegStat& s, bool yaw) { return s.nLate ? (yaw ? s.wLate : s.vLate) / s.nLate : 0.0; };
        const double wDrive = late(segStat[2], true), wStand = late(segStat[5], true);
        const bool liftOk = driverLiftMin >= -0.01 && driverLiftMax <= dr4b::kTopDeg + 2.0;
        const bool fwdOk = segStat[2].vmin > 5.0;
        const bool rateOk = std::fabs(wDrive - wStand) <= 0.15 * std::fabs(wStand);
        const bool spinOk = std::fabs(late(segStat[3], false)) < 6.0;
        js << ",\n  \"driver\": {\n    \"checks\": {\"lift_stays_within_0_and_kTopDeg_plus_2\": " << B(liftOk) << ", \"lift_true_min_deg\": " << driverLiftMin
           << ", \"lift_true_max_deg\": " << driverLiftMax << ",\n      \"partial_turn_keeps_forward_motion\": " << B(fwdOk) << ", \"min_forward_in_s_with_half_turn\": "
           << segStat[2].vmin << ",\n      \"turn_rate_same_driving_and_standing\": " << B(rateOk) << ", \"yaw_deg_s_half_stick_driving\": " << wDrive
           << ", \"yaw_deg_s_half_stick_standing\": " << wStand << ",\n      \"full_forward_plus_full_turn_is_a_point_turn\": " << B(spinOk)
           << ", \"forward_in_s_then\": " << late(segStat[3], false) << ",\n      \"opcontrol_cancels_leftover_auton_motion\": " << B(leftoverCancelled)
           << ", \"leftover_motion\": \"" << (leftoverStarted ? "test moveToPoint started by the sim" : "motion autonomous() was cut in at 60 s") << "\"" << ", \"true_speed_in_s_at_end_of_idle\": " << leftoverSpeedAtIdleEnd << "},\n    \"segments\": [";
        for (int i = 0; i < kNSeg; ++i) {
            const auto& s = segStat[i];
            const double n = std::max(1, s.n);
            js << (i ? ",\n      " : "\n      ") << "{\"name\": \"" << kSegs[i].name << "\", \"mix_L\": " << s.cmdL << ", \"mix_R\": " << s.cmdR
               << ", \"steady_v_in_s\": " << late(s, false) << ", \"steady_w_deg_s\": " << late(s, true)
               << ", \"lift_true_min\": " << s.liftMin << ", \"lift_true_max\": " << s.liftMax
               << ", \"mean_drive_volts_L\": " << s.voltL / n << ", \"mean_drive_volts_R\": " << s.voltR / n << ", \"mean_claw_volts\": " << s.claw / n << "}";
        }
        js << "]\n  }";
        kv << "driver_fwd_turning " << fwdOk << "\ndriver_lift_ok " << liftOk << "\ndriver_rate_ok " << rateOk << "\ndriver_cancel_ok " << leftoverCancelled << "\n";
        std::fprintf(stderr, "[sim] driver checks: lift in range %s (%.2f..%.2f deg) | forward kept with half turn %s (min %.1f in/s) | "
                             "turn rate driving %.0f vs standing %.0f deg/s %s | full fwd + full turn spins in place %s | leftover auton motion cancelled %s\n",
                     liftOk ? "PASS" : "FAIL", driverLiftMin, driverLiftMax, fwdOk ? "PASS" : "FAIL", segStat[2].vmin, wDrive, wStand,
                     rateOk ? "PASS" : "FAIL", spinOk ? "PASS" : "FAIL", leftoverCancelled ? "PASS" : "FAIL");
    }
    js << "\n}\n";
    std::ofstream(base + "summary_" + opt.tag + ".json") << js.str();
    kv << "points " << pointsAtEnd << "\npoints_if_owned " << pointsIfOwned << "\npins " << pinsAtEnd << "\nmidfield " << midfieldAtEnd << "\nroute_end_s "
       << (autonReturned ? (autonReturned - autonStart) / 1000.0 : 60.0) << "\ncompleted " << !autonCut << "\nodom_err_max " << odomErrMax
       << "\nodom_err_mean " << (odomErrN ? odomErrSum / odomErrN : 0.0) << "\nodom_err_final " << odomErrFinal << "\nhead_err_final " << std::fabs(headErrFinal)
       << "\nresets_applied " << counters.resetApplied << "\nmount_reject " << counters.resetMountReject << "\nmount_ok " << counters.resetMountOk
       << "\ntrack_corrections " << counters.setPoseTrack << "\nmotions_timed_out " << routeTimedOut << "\ngrab_ok " << counters.grabs
       << "\ngrab_miss " << counters.grabMiss << "\ntipped " << counters.tipped << "\ntoggle_sweeps " << counters.toggleSweeps
       << "\nlift_not_settled " << counters.liftNotSettled << "\nloose_pushed " << counters.loosePushed << "\nspeed_at_cut " << speedAtCut
       << "\nmidfield_at_cut " << midfieldAtCut << "\nimu_reads_over_gate " << counters.imuDisagree << "\nlost_heading_over_45 " << (std::fabs(headErrFinal) > 45.0)
       << "\nstack_hits " << counters.stackHits << "\ndropped " << counters.dropped << "\ntip_risk_ms " << counters.tipRiskMs
       << "\nscrub_in " << truth.scrubLeverIn << "\nplacements " << placeRecs.size()
       << "\ntoggle_state " << toggleState() << "\ntoggle_red " << (toggleState() == 2) << "\ntoggle_flips " << counters.toggleFlips
       << "\nclose_sensor " << counters.closeSensor << "\nclose_odom " << counters.closeOdom << "\nclose_skipped " << counters.closeSkipped
       << "\ntoggle_jam_ms " << counters.toggleJamMs << "\ntoggle_bump_ms " << counters.toggleBumpMs << "\nclaw_wall_ms " << counters.clawWallMs
       << "\nflip_inside " << counters.toggleStoppedInside << "\n";
    if (counters.toggleFlips)   // only runs that reached flip(), so the aggregate's min / median / max are over those
        kv << "flip_gap " << counters.flipGapIn << "\nflip_gap_paddle " << counters.flipGapPaddleIn << "\nflip_head_deg " << counters.flipHeadDeg
           << "\nflip_gap_in_window " << (counters.flipGapPaddleIn >= 0.92 && counters.flipGapPaddleIn <= 1.41)
           << "\nflip_claw_under " << counters.flipClawUnderIn << "\n";
    if (counters.closeGapIn >= 0) kv << "close_gap " << counters.closeGapIn << "\n";
    for (const auto& p : placeRecs) kv << "place_" << p.n << " " << p.ok << " " << p.radial << " " << (p.goal == "-" ? std::string("empty_claw") : p.goal.substr(0, p.goal.find(" "))) << "\n";
    std::ofstream(base + "summary_" + opt.tag + ".kv") << kv.str();
    if (csv) std::fclose(csv);
    std::fprintf(stderr, "[sim] %s (%s gains): %d points (%d Pins seated, %d owned, toggle strikes %d, midfield %d at rest / %d at the cut, %.1f in/s at the cut), "
                         "route end %.2f s, %d/%zu motions timed out, odom err max %.2f final %.2f in, heading err %.1f deg, IMU reads over the gate %d/%d -> %ssummary_%s.json\n",
                 opt.tag.c_str(), opt.tunedDrive ? "tuned" : "code", pointsAtEnd, pinsAtEnd, pinsOwnedAtEnd, counters.toggleSweeps, midfieldAtEnd, midfieldAtCut, speedAtCut,
                 autonReturned ? (autonReturned - autonStart) / 1000.0 : -1.0, routeTimedOut, routeMotions + parkMotions, odomErrMax, odomErrFinal, headErrFinal, counters.imuDisagree,
                 counters.imuTicks, base.c_str(), opt.tag.c_str());
    closeLog();
    exitNow(0);
}

// The 60 s cut: field control disables the robot. Odometry is compared here; the field is scored
// by scoreAtRest() once the robot has stopped rolling.
void endOfAuton(uint64_t t) {
    const arc::Pose p = odomPose();
    odomErrFinal = std::hypot(p.x - body.x, p.y - body.y);
    headErrFinal = headErr(p);
    midfieldAtCut = touchingMidfield();
    speedAtCut = std::hypot(body.v, body.vl) / simcfg::IN;
    yawRateAtCut = body.w * 180.0 / PI;
    cutX = body.x; cutY = body.y; cutTh = trueDeg();
    cutAt = t;
    if (!taskDone(autonId)) {
        autonCut = true;
        killTask(autonId);
        event("\"type\": \"auton_cut\", \"detail\": \"autonomous() still running at 60 s, task deleted\"");
        logLine("[sim] end of auton: autonomous() was still running, task deleted (route did not finish)");
    }
    event(fmt("\"type\": \"auton_cut_state\", \"midfield\": %s, \"true\": [%.2f, %.2f, %.1f], \"odom\": [%.2f, %.2f, %.1f], \"true_speed_in_s\": %.1f, "
              "\"chassis_in_motion\": %s",
              B(midfieldAtCut), body.x, body.y, trueDeg(), p.x, p.y, p.theta, speedAtCut, B(chassis.isInMotion())));
    logLine(fmt("[sim] %.0f s cut: robot disabled at true (%.2f, %.2f, %.1f) moving %.1f in/s, %.0f deg/s, lift %.1f deg, Midfield %s; odom (%.2f, %.2f, %.1f), "
                "xy err %.2f in, heading err %.2f deg. Physics runs on until it is at rest, then the field is scored",
                opt.autonMs / 1000.0, body.x, body.y, trueDeg(), speedAtCut, yawRateAtCut, lift.a * 180.0 / PI, midfieldAtCut ? "touching" : "NOT touching",
                p.x, p.y, p.theta, odomErrFinal, headErrFinal));
    setDisabled(true);   // field control: motors off and coasting, later commands ignored
}
void scoreAtRest(uint64_t t) {
    restAt = t;
    midfieldAtEnd = touchingMidfield();
    pinsAtEnd = pinsScored(false);
    pinsOwnedAtEnd = pinsScored(true);
    pointsAtEnd = 10 * pinsOwnedAtEnd + 5 * fivesScored() + (midfieldAtEnd ? 8 : 0);
    pointsIfOwned = 10 * pinsAtEnd + 5 * fivesScored() + (midfieldAtEnd ? 8 : 0);
    endX = body.x; endY = body.y; endTh = trueDeg();
    event(fmt("\"type\": \"end_of_auton\", \"points\": %d, \"pins\": %d, \"midfield\": %s, \"midfield_at_cut\": %s, \"true\": [%.2f, %.2f, %.1f], "
              "\"rolled_on_ms\": %d, \"true_speed_at_cut_in_s\": %.1f",
              pointsAtEnd, pinsAtEnd, B(midfieldAtEnd), B(midfieldAtCut), body.x, body.y, trueDeg(), static_cast<int>(restAt - cutAt), speedAtCut));
    logLine(fmt("[sim] at rest %d ms after the cut (%.1f in further), true (%.2f, %.2f, %.1f), lift %.1f deg: %d points (%d Pins seated, %d owned, Midfield %s, was %s at the cut)",
                static_cast<int>(restAt - cutAt), std::hypot(body.x - cutX, body.y - cutY), body.x, body.y, trueDeg(), lift.a * 180.0 / PI, pointsAtEnd,
                pinsAtEnd, pinsOwnedAtEnd, midfieldAtEnd ? "touching" : "NOT touching", midfieldAtCut ? "touching" : "not touching"));
}

void driverHook(uint64_t t) {
    const int rel = static_cast<int>(t - driverStart);
    int si = -1, t0 = 0;
    for (int i = 0, acc = 0; i < kNSeg; acc += kSegs[i].ms, ++i)
        if (rel >= acc && rel < acc + kSegs[i].ms) { si = i; t0 = acc; }
    ControllerScript s;
    if (si < 0) finish("end of the scripted driver scenario");
    const Seg& g = kSegs[si];
    s.analog[pros::E_CONTROLLER_ANALOG_LEFT_Y] = g.ly;
    s.analog[pros::E_CONTROLLER_ANALOG_RIGHT_X] = g.rx;
    s.digital[pros::E_CONTROLLER_DIGITAL_R1] = g.r1;
    s.digital[pros::E_CONTROLLER_DIGITAL_R2] = g.r2;
    s.digital[pros::E_CONTROLLER_DIGITAL_L1] = g.l1;
    s.digital[pros::E_CONTROLLER_DIGITAL_L2] = g.l2;
    if (rel == t0) logLine(fmt("[driver] segment: %s", g.name));
    SegStat& st = segStat[si];
    const auto mix = drive_mix::mix(g.ly, g.rx);
    st.cmdL = mix.first; st.cmdR = mix.second;
    const double v = body.v / simcfg::IN, w = body.w * 180.0 / PI, la = lift.a * 180.0 / PI;
    if (rel - t0 > 250) st.vmin = std::min(st.vmin, v);                               // after the response time
    if (rel - t0 >= g.ms - 200) { ++st.nLate; st.vLate += v; st.wLate += w; }         // steady state = last 200 ms
    st.n++; st.v += v; st.w += w;
    st.liftMin = std::min(st.liftMin, la); st.liftMax = std::max(st.liftMax, la);
    st.voltL += sideVolt(wiring.left); st.voltR += sideVolt(wiring.right); st.claw += motor(wiring.claw).mount * motor(wiring.claw).volt;
    driverLiftMin = std::min(driverLiftMin, la); driverLiftMax = std::max(driverLiftMax, la);
    if (si == 0 && rel == kSegs[0].ms - 1) {
        leftoverSpeedAtIdleEnd = std::hypot(body.v, body.vl) / simcfg::IN;
        // A test motion the sim started must also have run its endMotion(). A motion autonomous() was cut inside
        // (its task was deleted while holding the chassis mutex) can only be checked by the flag and the drive.
        leftoverCancelled = !chassis.isInMotion() && leftoverSpeedAtIdleEnd < 3.0 && (!leftoverStarted || motionsEnded > motionsEndedAtGap);
        logLine(fmt("[driver] leftover auton motion (%s): chassis.isInMotion() %d, motion task ended %d, true speed %.1f in/s -> %s",
                    leftoverStarted ? "test moveToPoint started by the sim" : "the motion autonomous() was cut in at 60 s", chassis.isInMotion(),
                    motionsEnded > motionsEndedAtGap, leftoverSpeedAtIdleEnd, leftoverCancelled ? "cancelled" : "NOT cancelled"));
    }
    if (rel % 100 == 0)
        logLine(fmt("[driver] t+%5d %-30.30s stick LY %4d RX %4d -> mix L %4d R %4d | drive V L %+5.1f R %+5.1f | v %6.1f in/s w %6.1f deg/s | lift true %6.2f meas %6.2f | claw V %+5.1f",
                    rel, g.name, g.ly, g.rx, mix.first, mix.second, sideVolt(wiring.left), sideVolt(wiring.right), v, w, la, liftMeasured(),
                    motor(wiring.claw).mount * motor(wiring.claw).volt));
    script = s;
}

void hook(uint64_t t) {
    worldSample(t);
    writeCsv(t);
    switch (phase) {
        case kInit:
            if (taskDone(initId)) {
                if (opt.selftest) {
                    phase = kSelftest;
                    logLine("[sim] initialize() returned, selftest starts");
                    spawn([] { runSelftest(); }, "selftest");
                    break;
                }
                // autonomous() does not start on a fixed tick after initialize() returns (per seed, 0 with --no-noise)
                if (!initDoneAt) initDoneAt = t;
                if (t < initDoneAt + static_cast<uint64_t>(truth.autonDelayMs)) break;
                if (opt.selfCal) {   // the robot's own calibration first, on the wall it starts on
                    static int calId = -1;
                    static bool calLogged = false;
                    static Body startBody;
                    if (calId < 0) { startBody = body; calId = spawn([] { calib::run(); }, "calib"); logLine("[sim] calib::run() starts"); break; }
                    if (!taskDone(calId)) { if (t > 90000) finish("calib::run() did not return"); break; }
                    if (!calLogged) {
                        calLogged = true;
                        logLine(fmt("[calwheel] tracking wheel offset %.3f (exact %.3f, code had 0.364)", calib::wheelOffset, truth.wheelOffset));
                        logLine(fmt("[calcheck] took %.1f s | IMU scale A %.4f (exact %.4f) B %.4f (exact %.4f) | bias front %.2f left %.2f right %.2f "
                                    "(parked at 6 in: %.2f %.2f %.2f) | true pose after (%.2f, %.2f, %.2f)",
                                    (t - initDoneAt) / 1000.0, calib::imuScale[0], 1.0 / truth.imuGain[0], calib::imuScale[1], 1.0 / truth.imuGain[1],
                                    localizer.mounts[0].biasIn, localizer.mounts[1].biasIn, localizer.mounts[2].biasIn,
                                    truth.calBias[0], truth.calBias[1], truth.calBias[2], body.x, body.y, trueDeg()));
                        body = startBody;   // set down on the start spot again for the run
                    }
                }
                phase = kAuton;
                autonStart = t;
                logLine(fmt("[sim] initialize() returned, autonomous() starts (%.0f s)", opt.autonMs / 1000.0));
                event("\"type\": \"auton_start\"");
                autonId = spawn([] { autonomous(); }, "auton");
            } else if (t > 30000) finish("initialize() did not return within 30 s");
            break;
        case kSelftest:
            if (t > 600000) finish("selftest did not finish within 600 s of virtual time");
            break;
        case kAuton:
            if (!autonReturned && taskDone(autonId)) {
                autonReturned = t;
                event(fmt("\"type\": \"route_end\", \"auton_s\": %.3f, \"skills_clock_s\": %.3f", (t - autonStart) / 1000.0,
                          (t - skills::startMs()) / 1000.0));
                logLine(fmt("[sim] skillsDr4b() returned at %.3f s of autonomous", (t - autonStart) / 1000.0));
            }
            if (t >= autonStart + static_cast<uint64_t>(opt.autonMs)) {
                endOfAuton(t);
                phase = kCoast;
            }
            break;
        case kCoast:
            // disabled: the robot coasts (and the lift sinks) until everything is at rest, 3 s at most
            if (t < cutAt + 100 || (!worldAtRest() && t < cutAt + 3000)) break;
            {
                scoreAtRest(t);
                if (!opt.driver) finish("end of the 60 s autonomous skills period");
                phase = kGap;
                gapStart = t;
                // Driver test on open floor (field reset between periods; odometry is not told and is not
                // used in driver control). A drive motion is left running to prove opcontrol() cancels it:
                // on a V5 an async motion task survives the switch from autonomous to opcontrol.
                openFloor = true;
                worldTeleport(0.0, 0.0, 0.0);
                motionsEndedAtGap = motionsEnded;
                if (!chassis.isInMotion()) {
                    leftoverStarted = true;
                    spawn([] { const arc::Pose p = chassis.getPose();
                               chassis.moveToPoint(p.x + 60.0f * std::sin(p.theta * PI / 180.0), p.y + 60.0f * std::cos(p.theta * PI / 180.0), {.timeout = 8000, .async = true}); },
                          "leftover");
                    logLine("[sim] driver test: robot on open floor, a leftover async moveToPoint (60 in ahead, 8 s timeout) is running");
                } else logLine("[sim] driver test: autonomous() was cut inside a motion (chassis mutex stays taken by the deleted task), no extra leftover motion started");
            }
            break;
        case kGap:
            if (t >= gapStart + 150) {
                phase = kDriver;
                driverStart = t;
                setDisabled(false);
                logLine("[sim] opcontrol() starts with the scripted driver inputs");
                driverId = spawn([] { opcontrol(); }, "opcontrol");
            }
            break;
        case kDriver:
            driverHook(t);
            break;
    }
}

// ------------------------------------------------------------- Monte Carlo
int runMany(int n, const std::string& exe, const std::string& passthrough) {
    const unsigned par = std::max(1u, std::min(8u, std::thread::hardware_concurrency()));
    std::vector<std::thread> th;
    std::atomic<int> next{1};
    const std::string mc = "mc" + (opt.tag.empty() ? std::string() : "_" + opt.tag) + "_seed";
    for (unsigned k = 0; k < par; ++k)
        th.emplace_back([&] {
            for (int s = next++; s <= n; s = next++) {
                // a summary left by an earlier run must not be read as this run's result if the child crashes
                std::remove((opt.outDir + "/summary_" + mc + std::to_string(s) + ".kv").c_str());
                const std::string cmd = "\"\"" + exe + "\" --seed " + std::to_string(s) + " --quiet --tag " + mc + std::to_string(s) +
                                        " --out \"" + opt.outDir + "\"" + (opt.repoDir.empty() ? "" : " --repo \"" + opt.repoDir + "\"") +
                                        " --watchdog-ms " + std::to_string(opt.watchdogMs) + passthrough + " > NUL 2>&1\"";
                std::system(cmd.c_str());
            }
        });
    for (auto& t : th) t.join();
    std::map<std::string, std::vector<double>> num;
    std::map<int, std::pair<int, int>> place;   // n -> ok, total
    std::map<int, std::string> placeGoal;
    std::map<int, std::vector<double>> placeRad;
    std::map<int, int> placeEmpty;
    int got = 0;
    std::vector<int> crashed;
    for (int s = 1; s <= n; ++s) {
        std::ifstream f(opt.outDir + "/summary_" + mc + std::to_string(s) + ".kv");
        if (!f) { crashed.push_back(s); continue; }
        ++got;
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream is(line);
            std::string k;
            is >> k;
            if (k.rfind("place_", 0) == 0) {
                int idx = std::stoi(k.substr(6)), ok;
                double rad;
                std::string g;
                is >> ok >> rad >> g;
                place[idx].first += ok;
                place[idx].second += 1;
                if (g == "empty_claw") placeEmpty[idx]++;
                else { placeGoal[idx] = g; placeRad[idx].push_back(rad); }
            } else {
                double v;
                if (is >> v) num[k].push_back(v);
            }
        }
    }
    auto stats = [](const std::vector<double>& v) {
        if (v.empty()) return std::string("null");
        double s = 0, mn = 1e18, mx = -1e18;
        for (double x : v) { s += x; mn = std::min(mn, x); mx = std::max(mx, x); }
        const double m = s / v.size();
        double var = 0;
        for (double x : v) var += (x - m) * (x - m);
        std::vector<double> w = v;
        std::sort(w.begin(), w.end());
        return fmt("{\"mean\": %.3f, \"sd\": %.3f, \"min\": %.3f, \"median\": %.3f, \"max\": %.3f}", m, std::sqrt(var / v.size()), mn,
                   w[w.size() / 2], mx);
    };
    std::ostringstream js;
    js << "{\n  \"runs\": " << n << ", \"completed_runs\": " << got << ", \"crashed_or_watchdog_seeds\": [";
    for (size_t i = 0; i < crashed.size(); ++i) js << (i ? ", " : "") << crashed[i];
    js << "],\n  \"drive_gains\": \"" << (opt.tunedDrive ? "sim-tuned (--tuned-drive)" : "main.cpp as written") << "\", \"flags\": \"" << passthrough << "\",\n  \"metrics\": {";
    bool first = true;
    for (const auto& [k, v] : num) { js << (first ? "\n    " : ",\n    ") << "\"" << k << "\": " << stats(v); first = false; }
    js << "\n  },\n  \"points_distribution\": {";
    std::map<int, int> hist;
    for (double p : num["points"]) hist[static_cast<int>(p)]++;
    first = true;
    for (const auto& [p, c] : hist) { js << (first ? "" : ", ") << "\"" << p << "\": " << c; first = false; }
    js << "},\n  \"placements\": [";
    first = true;
    for (const auto& [idx, oc] : place) {
        js << (first ? "\n    " : ",\n    ") << "{\"release\": " << idx << ", \"goal\": \"" << (placeGoal.count(idx) ? placeGoal[idx] : std::string("?")) << "\", \"success_rate\": "
           << fmt("%.3f", oc.first / double(oc.second)) << ", \"releases\": " << oc.second << ", \"runs_that_got_this_far\": " << fmt("%.3f", oc.second / double(std::max(1, got)))
           << ", \"claw_empty\": " << placeEmpty[idx] << ", \"radial_err_in_when_holding\": " << stats(placeRad[idx]) << "}";
        first = false;
    }
    js << "\n  ]\n}\n";
    const std::string path = opt.outDir + "/aggregate_" + std::to_string(n) + (opt.tag.empty() ? "" : "_" + opt.tag) + ".json";
    std::ofstream(path) << js.str();
    std::fputs(js.str().c_str(), stdout);
    std::fprintf(stderr, "[sim] aggregate written to %s\n", path.c_str());
    return crashed.empty() ? 0 : 1;
}
}  // namespace

namespace simapp {
[[noreturn]] void exitNow(int code) {
    closeLog();
    std::fflush(stdout);
    std::fflush(stderr);
    std::_Exit(code);
}
}  // namespace simapp

namespace {
// Truth knobs: robot_sim FLAG VALUE. Each one fixes a value the sim otherwise assumes (sim_config.hpp).
struct Knob { const char* flag; double Options::*v; const char* tag; };
const Knob kKnobs[] = {
    {"--scrub", &Options::scrubIn, "scrub"},             {"--drive-fric", &Options::driveFricN, "dfric"},
    {"--mass", &Options::massKg, "mass"},                {"--yaw-j", &Options::yawJ, "yawj"},
    {"--carriage", &Options::carriageKg, "carriage"},    {"--lift-fric", &Options::liftFricNm, "liftfric"},
    {"--imu-jitter", &Options::imuJitterMs, "imujit"},   {"--imu-b-drop", &Options::imuBDropS, "imubdrop"},
    {"--imu-phase-b", &Options::imuPhaseB, "imuphb"},
    {"--pitch", &Options::pitchIn, "pitch"},             {"--lift-err", &Options::liftErrDeg, "lifterr"},
    {"--lift-slop", &Options::liftSlopDeg, "liftslop"},  {"--lift-bands", &Options::liftBandNm, "bands"}, {"--band-count", &Options::bandCount, "bandn"}, {"--rigid-string", &Options::rigidString, "rigid"}, {"--drive-torque", &Options::driveTorque, "dtq"}, {"--drive-gear", &Options::driveGear, "dgear"}, {"--clock-shift", &Options::clockShiftMs, "cshift"},
    {"--grab-ahead", &Options::grabAheadIn, "grabahead"}, {"--grab-lat", &Options::grabLatIn, "grablat"},
    {"--release-ms", &Options::releaseMs, "relms"},      {"--release-fwd", &Options::releaseFwdIn, "relfwd"},
    {"--capture-r", &Options::captureR, "capr"},
    {"--pivot-fwd", &Options::pivotFwdIn, "pivot"},      {"--paddle-deg", &Options::paddleDeg, "paddle"},
    {"--cg-fwd", &Options::cgFwdIn, "cg"},
    {"--flip-turns", &Options::flipTurns, "flipturns"}, {"--close-lead", &Options::closeLeadIn, "closelead"},
    {"--close-mv", &Options::closeMv, "closemv"},       {"--close-slow-mv", &Options::closeSlowMv, "closeslowmv"},
};
const Knob* knobFor(const std::string& flag) {
    for (const Knob& k : kKnobs) if (flag == k.flag) return &k;
    return nullptr;
}
}  // namespace

int main(int argc, char** argv) {
    int runs = 0;
    bool watchdogDemo = false;
    std::string passthrough;
#ifdef SIM_REPO_DIR
    opt.repoDir = SIM_REPO_DIR;
#endif
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto val = [&] { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
        auto pass = [&](const std::string& v = "") { passthrough += " " + a + (v.empty() ? "" : " " + v); };
        if (a == "--seed") opt.seed = std::stoi(val());
        else if (a == "--runs") runs = std::stoi(val());
        else if (a == "--no-noise") { opt.noise = false; pass(); }
        else if (a == "--driver") { opt.driver = true; pass(); }
        else if (a == "--selftest") { opt.selftest = true; opt.noise = false; }
        else if (a == "--tuned-drive") { opt.tunedDrive = true; pass(); }
        else if (a == "--quiet") opt.quiet = true;
        else if (a == "--goal-pins") { opt.goalPins = true; pass(); }
        else if (a == "--calibrated") { opt.calibrated = true; pass(); }
        else if (a == "--self-cal") { opt.selfCal = true; pass(); }
        else if (a == "--draw-heights") { opt.drawHeights = true; pass(); }
        else if (a == "--no-clusters") { opt.clusters = false; pass(); }
        else if (a == "--no-cont-track") { opt.noContTrack = true; pass(); }
        else if (a == "--no-claw-wall") { opt.clawWall = false; pass(); }
        else if (a == "--rear") { const auto v = val(); opt.rearIn = std::stod(v); pass(v); }
        else if (a == "--wheel-dia") { const auto v = val(); opt.wheelDia = std::stod(v); pass(v); }
        else if (a == "--imu-gain") { const auto v = val(); opt.imuGain = std::stod(v); pass(v); }
        else if (a == "--loader-x") { const auto v = val(); opt.loaderX = std::stod(v); pass(v); }
        else if (a == "--imu-b-missing") { opt.imuBMissing = true; pass(); }
        else if (const Knob* k = knobFor(a)) {
            const auto v = val();
            opt.*(k->v) = std::stod(v); pass(v);
            knobTag += std::string("_") + k->tag + v;
            knobJson += std::string(knobJson.empty() ? "" : ", ") + "\"" + (k->flag + 2) + "\": " + v;
        }
        else if (a == "--auton-ms") { const auto v = val(); opt.autonMs = std::stoi(v); pass(v); }
        else if (a == "--lat-gains") {   // tuning knob: kp,ki,kd,integralRange (implies --tuned-drive)
            const auto v = val();
            std::sscanf(v.c_str(), "%f,%f,%f,%f", &kTunedLateral.proportional.initial, &kTunedLateral.kI, &kTunedLateral.kD, &kTunedLateral.integralRange);
            kTunedLateral.proportional.final = kTunedLateral.proportional.initial;
            opt.tunedDrive = true; passthrough += " --lat-gains " + v + " --tuned-drive";
        } else if (a == "--ang-gains") {   // tuning knob: kpSmall,kpLarge,kd,derivativeAlpha[,kI[,integralRange]] (implies --tuned-drive)
            const auto v = val();
            std::sscanf(v.c_str(), "%f,%f,%f,%f,%f,%f", &kTunedAngular.proportional.initial, &kTunedAngular.proportional.final, &kTunedAngular.kD,
                        &kTunedAngular.derivativeAlpha, &kTunedAngular.kI, &kTunedAngular.integralRange);
            opt.tunedDrive = true; passthrough += " --ang-gains " + v + " --tuned-drive";
        }
        else if (a == "--lift-gains") {   // kP,kD,kI[,kG scale]: the robot's lift gains, set from the sim side
            const auto v = val();
            std::sscanf(v.c_str(), "%f,%f,%f,%f", &opt.liftKp, &opt.liftKd, &opt.liftKi, &opt.liftKgScale);
            passthrough += " --lift-gains " + v;
        }
        else if (a == "--lat-full") {   // every lateral gain: initial,final,scale,power,kI,kD,integralRange
            const auto v = val();
            std::sscanf(v.c_str(), "%f,%f,%f,%f,%f,%f,%f", &kTunedLateral.proportional.initial, &kTunedLateral.proportional.final,
                        &kTunedLateral.proportional.scale, &kTunedLateral.proportional.power, &kTunedLateral.kI, &kTunedLateral.kD,
                        &kTunedLateral.integralRange);
            opt.tunedDrive = true; passthrough += " --lat-full " + v + " --tuned-drive";
        } else if (a == "--ang-full") {   // every angular gain: initial,final,scale,power,scheduleAlpha,derivativeAlpha,kI,kD,integralRange
            const auto v = val();
            std::sscanf(v.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f", &kTunedAngular.proportional.initial, &kTunedAngular.proportional.final,
                        &kTunedAngular.proportional.scale, &kTunedAngular.proportional.power, &kTunedAngular.scheduleAlpha,
                        &kTunedAngular.derivativeAlpha, &kTunedAngular.kI, &kTunedAngular.kD, &kTunedAngular.integralRange);
            opt.tunedDrive = true; passthrough += " --ang-full " + v + " --tuned-drive";
        }
        else if (a == "--watchdog-demo") watchdogDemo = true;
        else if (a == "--watchdog-ms") opt.watchdogMs = std::stoi(val());
        else if (a == "--tag") opt.tag = val();
        else if (a == "--out") opt.outDir = val();
        else if (a == "--repo") opt.repoDir = val();
        else {
            std::fprintf(stderr,
                         "usage: robot_sim [--selftest] [--seed N] [--runs N] [--no-noise] [--driver] [--tuned-drive] [--quiet] [--tag NAME]\n"
                         "  scenarios: [--goal-pins] [--draw-heights] [--no-clusters] [--no-cont-track] [--no-claw-wall] [--rear IN] [--wheel-dia IN]\n"
                         "             [--imu-gain G] [--loader-x IN] [--auton-ms MS] [--imu-b-missing]\n"
                         "  truth knobs (FLAG VALUE, see sim_config.hpp): --scrub IN  --drive-fric N  --imu-jitter MS  --imu-phase-b MS  --imu-b-drop S\n"
                         "             --pitch IN  --lift-err DEG  --lift-slop DEG  --lift-bands NM  --band-count N  --rigid-string 1  --grab-ahead IN  --grab-lat IN\n"
                         "             --release-ms MS  --release-fwd IN  --capture-r IN  --pivot-fwd IN  --cg-fwd IN  --paddle-deg DEG\n"
                         "  robot constants, sweep only: --flip-turns N (toggle::kFlipTurns)  --close-lead IN  --close-mv MV  --close-slow-mv MV\n"
                         "  other:     [--out DIR] [--repo DIR] [--watchdog-ms MS] [--watchdog-demo]\n");
            return 2;
        }
    }
    if (opt.outDir.empty()) opt.outDir = opt.repoDir + "/sim/out";
    if (runs > 0) return runMany(runs, argv[0], passthrough);
    if (opt.tag.empty()) {
        opt.tag = opt.selftest ? "selftest" : opt.noise ? "seed" + std::to_string(opt.seed) : std::string("nonoise");
        if (opt.tunedDrive) opt.tag += "_tuned";
        if (opt.goalPins) opt.tag += "_goalpins";
        if (opt.drawHeights) opt.tag += "_drawh";
        if (!opt.clusters) opt.tag += "_noclusters";
        if (opt.noContTrack) opt.tag += "_nocont";
        if (!opt.clawWall) opt.tag += "_noclawwall";
        if (opt.rearIn > 0) opt.tag += "_rear";
        if (opt.wheelDia > 0) opt.tag += "_wheel";
        if (opt.imuGain > 0) opt.tag += "_imugain";
        if (opt.loaderX > 0) opt.tag += "_loaderx";
        if (opt.imuBMissing) opt.tag += "_imubmissing";
        opt.tag += knobTag;
        if (opt.driver) opt.tag += "_driver";
    }
    openLog(opt.outDir + "/log_" + opt.tag + ".txt");
    csv = std::fopen((opt.outDir + "/pose_" + opt.tag + ".csv").c_str(), "w");
    if (csv) std::fputs("t,phase,x,y,theta,odom_x,odom_y,odom_theta,v_in_s,w_deg_s,lift_true_deg,lift_meas_deg,left_V,right_V,claw_V,contact_N,holding\n", csv);
    applyWiring();
    worldInit();
    applyRobotConstants();
    parseRoute();
    worldSample(0);
    initId = spawn([] { initialize(); }, "init");
    if (watchdogDemo) {   // proves the watchdog: a task that never calls delay()
        opt.watchdogMs = 1500;
        spawn([] { volatile unsigned long long n = 0; while (true) ++n; }, "busy-loop-demo");
    }
    kernelRun([](uint64_t t) { worldStep(t); }, hook);
}
