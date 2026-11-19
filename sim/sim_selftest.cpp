// sim_selftest.cpp - the validation gate (robot_sim --selftest).
// Every check drives the REAL robot API (leftDrive.move, chassis.moveToPoint, dr4b::to,
// localizer...) and compares against the TRUE physics state. Runs after the real
// initialize(), noise off. Rows marked INFO are measurements, not pass/fail.
#include "main.h"
#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "skills_support.hpp"
#include "sim_app.hpp"
#include "sim_config.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>

extern arc::Chassis chassis;
using namespace sim;
using namespace simapp;

namespace {
constexpr double PI = 3.14159265358979323846;
struct Row { std::string id, what, measured, limit; int pass; };   // 1 PASS, 0 FAIL, -1 INFO
std::vector<Row> rows;
void row(const std::string& id, const std::string& what, const std::string& measured, const std::string& limit, int pass) {
    rows.push_back({id, what, measured, limit, pass});
    logLine("[selftest] " + id + " " + (pass == 1 ? "PASS" : pass == 0 ? "FAIL" : "INFO") + " | " + what + " | " + measured + " | " + limit);
}
double trueDeg() { return body.th * 180.0 / PI; }
double speedIn() { return std::hypot(body.v, body.vl) / simcfg::IN; }
void driveRaw(int l, int r) { leftDrive.move(l); rightDrive.move(r); }
void settle(int maxMs = 6000) {
    for (int t = 0; t < maxMs; t += 10) {
        if (speedIn() < 0.02 && std::fabs(body.w) < 0.005) break;
        pros::delay(10);
    }
    pros::delay(60);
}
void place(double x, double y, double h) {
    driveRaw(0, 0);
    worldTeleport(x, y, h);
    pros::delay(40);            // odometry swallows the IMU jump of the teleport
    chassis.setPose(x, y, h);
    pros::delay(40);
}
void traceStart() { trace.clear(); traceOn = true; }
void traceStop() { traceOn = false; }
double meanLast(double TraceRow::*f, double seconds) {
    if (trace.empty()) return 0;
    const double t1 = trace.back().t;
    double s = 0; int n = 0;
    for (const auto& r : trace) if (r.t >= t1 - seconds) { s += r.*f; ++n; }
    return n ? s / n : 0;
}
struct OdomErr { double xy, head; };
OdomErr odomErr() {
    const arc::Pose p = chassis.getPose();
    return {std::hypot(p.x - body.x, p.y - body.y), std::remainder(p.theta - trueDeg(), 360.0)};
}
// why a motion that timed out never exited, from the robot's own (odometry) view
std::string exitState(bool lateral, double errOdom) {
    const arc::Pose v = arc::getSpeed(false);
    if (lateral)
        return fmt("odom error %.2f in (errorExit 0.6 %s), odom speed %.1f in/s (velocityExit 6 %s)", errOdom, errOdom <= 0.6 ? "met" : "NOT met",
                   std::hypot(v.x, v.y), std::hypot(v.x, v.y) <= 6.0 ? "met" : "NOT met");
    return fmt("odom error %.2f deg (errorExit 4 %s), odom rate %.1f deg/s (velocityExit 3 %s)", errOdom, std::fabs(errOdom) <= 4.0 ? "met" : "NOT met",
               std::fabs(v.theta), std::fabs(v.theta) <= 3.0 ? "met" : "NOT met");
}

void openLoop() {
    // ---- (a) forward
    openFloor = true;
    place(0, -60, 0);
    traceStart();
    driveRaw(127, 127);
    pros::delay(2000);
    traceStop();
    const double vs = meanLast(&TraceRow::v, 0.3);
    double t90 = -1;
    for (const auto& r : trace) if (r.v >= 0.9 * vs) { t90 = (r.t - trace.front().t) * 1000; break; }
    row("a1", "open loop: move(127) both sides for 2 s, steady TRUE speed", fmt("%.1f in/s (90%% after %.0f ms, %.1f in travelled)", vs, t90, body.y + 60),
        "55 to 68 in/s", vs >= 55 && vs <= 68);
    const double yStop = body.y;
    driveRaw(0, 0);
    settle(8000);
    row("a1c", "coast after move(0) from full speed (drive brake mode never set = COAST). Depends on ASSUMED drivetrain friction",
        fmt("%.1f in before stopping (drive drag %.2f N/side, crr %.3f)", body.y - yStop, truth.driveFricN, truth.rollResist), "measure on the robot (also robot finding F6)", -1);
    // ---- (b) odometry after the open-loop drive
    OdomErr e = odomErr();
    const arc::Pose p = chassis.getPose();
    row("b1", "odometry after that drive: arc pose vs TRUE pose", fmt("xy error %.3f in, heading error %.3f deg (true y %.2f, odom y %.2f)", e.xy, e.head, body.y, p.y),
        "within 0.5 in and 1 deg", e.xy <= 0.5 && std::fabs(e.head) <= 1.0);
    // ---- static friction: smallest move() that starts the robot from rest
    {
        const auto breakaway = [](int sl, int sr) {
            for (int c = 1; c <= 60; ++c) {
                place(0, 0, 0);
                driveRaw(sl * c, sr * c);
                pros::delay(500);
                const bool moved = std::hypot(body.x, body.y) > 0.25 || std::fabs(trueDeg()) > 2.0;
                driveRaw(0, 0);
                if (moved) return c;
            }
            return -1;
        };
        const int bs = breakaway(1, 1), bt = breakaway(1, -1);
        row("a3", "smallest move() that starts the robot from rest. Set by ASSUMED drivetrain friction, and it decides where a P loop stalls",
            fmt("straight move(%d) = %.2f V, point turn move(+-%d) = %.2f V. With main.cpp's lateral Kp 5.73 the P term alone stalls %.2f in from the target "
                "(errorExit is 0.6 in); each extra move(1) of breakaway adds 0.17 in", bs, bs * 12.0 / 127, bt, bt * 12.0 / 127, bs / 5.73),
            "measure on the robot, then set kDriveFricN / --drive-fric", -1);
    }
    // ---- (a) spin
    place(0, 0, 0);
    traceStart();
    driveRaw(127, -127);
    pros::delay(2000);
    traceStop();
    const double ws = meanLast(&TraceRow::wDeg, 0.3);
    double tw = -1;
    for (const auto& r : trace) if (r.wDeg >= 0.9 * ws) { tw = (r.t - trace.front().t) * 1000; break; }
    row("a2", "open loop: move(+127) left, move(-127) right for 2 s, steady TRUE yaw rate", fmt("%.0f deg/s clockwise (90%% after %.0f ms)", ws, tw),
        "350 to 720 deg/s, clockwise (free-speed ideal 698). Too wide to pin the turning scrub: see a2c", ws >= 350 && ws <= 720);
    const double thCut = trueDeg();
    const uint32_t tCut = pros::millis();
    driveRaw(0, 0);
    settle(8000);
    row("a2c", "spin-down after move(0) from that spin (COAST). This is the measurement that calibrates the turning scrub; turn overshoot and --tuned-drive hinge on it",
        fmt("%.0f deg more in %.2f s with scrub lever %.2f in (ASSUMED)", trueDeg() - thCut, (pros::millis() - tCut - 60) / 1000.0, truth.scrubLeverIn),
        "measure on the robot, then set kScrubLeverIn / --scrub", -1);
    e = odomErr();
    row("b2", "odometry after that spin: arc heading vs TRUE heading", fmt("heading error %.3f deg after %.0f deg turned, xy error %.3f in", e.head, trueDeg(), e.xy),
        "within 1.5 deg", std::fabs(e.head) <= 1.5);
    // ---- curved drive (tracking wheel offset, sideways slip)
    place(-40, -40, 0);
    driveRaw(127, 70);
    pros::delay(1200);
    driveRaw(0, 0);
    settle(8000);
    e = odomErr();
    row("b3", "odometry after a fast arc (move 127 / 70 for 1.2 s)", fmt("xy error %.3f in, heading error %.3f deg", e.xy, e.head),
        "no horizontal wheel: sideways slip is invisible to odometry", -1);
}

void setPoseLoss() {
    // The continuous wall tracker calls chassis.setPose() every 25 ms. arc::setPose() invalidates
    // the odometry tick in progress (odom.cpp:79-84 -> 59-63, 119-123), so the distance travelled
    // in that 10 ms tick is never added to the pose.
    static volatile bool run;
    openFloor = true;
    place(0, -50, 0);
    run = true;
    pros::Task([] { while (run) { const arc::Pose p = chassis.getPose(); chassis.setPose(p.x, p.y, p.theta); pros::delay(25); } }, "setpose-25ms");
    driveRaw(60, 60);
    pros::delay(1500);
    driveRaw(0, 0);
    settle(8000);
    run = false;
    pros::delay(60);
    const double trueTravel = body.y + 50, odomTravel = chassis.getPose().y + 50;
    row("g1", "ROBOT FINDING: drive 1.5 s at move(60) while a task calls chassis.setPose(getPose()) every 25 ms (what localizer.startContinuous() does)",
        fmt("true travel %.1f in, odometry %.1f in: %.0f%% of the motion lost", trueTravel, odomTravel, 100.0 * (1 - odomTravel / trueTravel)),
        "odom.cpp drops one tick per setPose()", -1);
}

void closedLoop() {
    openFloor = true;
    const char* gains = opt.tunedDrive ? "TUNED gains" : "main.cpp gains";
    // ---- (c1) moveToPoint 24 in
    {
        place(0, 0, 0);
        traceStart();
        const uint32_t t0 = pros::millis();
        const int to0 = motionTimeouts;
        chassis.moveToPoint(0, 24, {.timeout = 2000});
        const uint32_t dur = pros::millis() - t0;
        const bool timedOut = motionTimeouts > to0;
        const arc::Pose po = chassis.getPose();
        const std::string why = exitState(true, std::hypot(po.x, po.y - 24));
        const double errExit = std::hypot(body.x, body.y - 24), vExit = body.v / simcfg::IN;
        pros::delay(700);
        traceStop();
        const double errFinal = std::hypot(body.x, body.y - 24);
        double t90 = -1, peak = -1e9;
        for (const auto& r : trace) { if (t90 < 0 && r.y >= 21.6) t90 = (r.t - trace.front().t) * 1000; peak = std::max(peak, r.y); }
        row("c1", fmt("closed loop, %s: chassis.moveToPoint 24 in straight ahead, timeout 2000", gains),
            fmt("%s after %u ms; TRUE error %.2f in at exit (still %.1f in/s), %.2f in at rest; rise to 90%% %.0f ms; overshoot %.2f in%s", timedOut ? "TIMED OUT" : "exited",
                dur, errExit, vExit, errFinal, t90, std::max(0.0, peak - 24), timedOut ? ("; at timeout: " + why).c_str() : ""),
            "within 1.5 in true, before the 2 s timeout", errFinal <= 1.5 && !timedOut);
    }
    // ---- (c2) turnToHeading 90
    {
        place(0, 0, 0);
        traceStart();
        const uint32_t t0 = pros::millis();
        const int to0 = motionTimeouts;
        chassis.turnToHeading(90, {.timeout = 1500});
        const uint32_t dur = pros::millis() - t0;
        const bool timedOut = motionTimeouts > to0;
        const std::string why = exitState(false, 90 - chassis.getPose().theta);
        const double errExit = 90 - trueDeg(), wExit = body.w * 180 / PI;
        pros::delay(500);
        traceStop();
        const double errFinal = 90 - trueDeg();
        double t90 = -1, peak = -1e9;
        for (const auto& r : trace) { if (t90 < 0 && r.thDeg >= 81) t90 = (r.t - trace.front().t) * 1000; peak = std::max(peak, r.thDeg); }
        row("c2", fmt("closed loop, %s: chassis.turnToHeading(90), timeout 1500", gains),
            fmt("%s after %u ms; TRUE error %.2f deg at exit (still %.0f deg/s), %.2f deg at rest; rise to 90%% %.0f ms; overshoot %.1f deg%s", timedOut ? "TIMED OUT" : "exited",
                dur, errExit, wExit, errFinal, t90, std::max(0.0, peak - 90), timedOut ? ("; at timeout: " + why).c_str() : ""),
            "within 3 deg true, before the 1.5 s timeout", std::fabs(errFinal) <= 3.0 && !timedOut);
    }
    // ---- other sizes (the route has 3 to 48 in moves and 20 to 180 deg turns, with 350 to 2100 ms timeouts)
    int k = 0;
    for (double d : {5.0, 11.0, 48.0}) {
        place(0, -30, 0);
        traceStart();
        const uint32_t t0 = pros::millis();
        const int to0 = motionTimeouts;
        chassis.moveToPoint(0, -30 + d, {.timeout = 3000});
        const uint32_t dur = pros::millis() - t0;
        const bool timedOut = motionTimeouts > to0;
        pros::delay(600);
        traceStop();
        double peak = -1e9, tIn = -1;
        for (const auto& r : trace) { peak = std::max(peak, r.y + 30); if (tIn < 0 && std::fabs(r.y + 30 - d) <= 1.0) tIn = (r.t - trace.front().t) * 1000; }
        row(fmt("c1%c", 'b' + k++), fmt("closed loop, %s: chassis.moveToPoint %.0f in, timeout 3000", gains, d),
            fmt("%s after %u ms; within 1 in after %.0f ms; TRUE error %.2f in at rest; overshoot %.2f in", timedOut ? "TIMED OUT" : "exited", dur, tIn,
                d - (body.y + 30), std::max(0.0, peak - d)), "informational", -1);
    }
    k = 0;
    for (double a : {30.0, 180.0}) {
        place(0, 0, 0);
        traceStart();
        const uint32_t t0 = pros::millis();
        const int to0 = motionTimeouts;
        chassis.turnToHeading(a, {.timeout = 3000});
        const uint32_t dur = pros::millis() - t0;
        const bool timedOut = motionTimeouts > to0;
        pros::delay(400);
        traceStop();
        double peak = -1e9, tIn = -1;
        for (const auto& r : trace) { peak = std::max(peak, r.thDeg); if (tIn < 0 && std::fabs(r.thDeg - a) <= 4.0) tIn = (r.t - trace.front().t) * 1000; }
        row(fmt("c2%c", 'b' + k++), fmt("closed loop, %s: chassis.turnToHeading(%.0f), timeout 3000", gains, a),
            fmt("%s after %u ms; first within 4 deg after %.0f ms; TRUE error %.2f deg at rest; overshoot %.1f deg", timedOut ? "TIMED OUT" : "exited", dur, tIn,
                a - trueDeg(), std::max(0.0, peak - a)), "informational", -1);
    }
    // ---- (c3) 24 in square
    {
        place(-12, -12, 0);
        const uint32_t t0 = pros::millis();
        const int to0 = motionTimeouts;
        const double px[4] = {-12, 12, 12, -12}, py[4] = {12, 12, -12, -12}, ph[4] = {90, 180, 270, 0};
        std::string legs;
        for (int i = 0; i < 4; ++i) {
            chassis.moveToPoint(px[i], py[i]);
            legs += fmt("%s%.2f", i ? ", " : "", std::hypot(body.x - px[i], body.y - py[i]));
            chassis.turnToHeading(ph[i]);
        }
        const uint32_t dur = pros::millis() - t0;
        pros::delay(600);
        const double exy = std::hypot(body.x + 12, body.y + 12), eh = std::remainder(trueDeg(), 360.0);
        const OdomErr oe = odomErr();
        row("c3", fmt("closed loop, %s: 24 in square, 4 x (moveToPoint + turnToHeading), default timeouts", gains),
            fmt("ends %.2f in and %.2f deg from the start (TRUE); %u ms; %d of 8 motions timed out; true error at each corner %s in; odometry error %.2f in",
                exy, eh, dur, motionTimeouts - to0, legs.c_str(), oe.xy),
            "within 2 in and 3 deg true", exy <= 2.0 && std::fabs(eh) <= 3.0);
    }
    driveRaw(0, 0);
}

void liftChecks() {
    traceStart();
    const uint32_t t0 = pros::millis();
    dr4b::to(60);
    const bool ok = dr4b::waitSettled(3000);
    const uint32_t ts = pros::millis() - t0;
    pros::delay(1500);
    traceStop();
    double peak = 0;
    for (const auto& r : trace) peak = std::max(peak, r.liftDeg);
    const double a = lift.a * 180 / PI;
    row("d1", fmt("lift: dr4b::to(60) with kP %.2f kD %.3f kG %.2f, dr4b::waitSettled(3000)", dr4b::kP, dr4b::kD, dr4b::kG),
        fmt("%s after %u ms; overshoot %.2f deg; TRUE angle %.2f deg 1.5 s later (encoder reads %.2f)", ok ? "settled" : "NOT settled", ts, std::max(0.0, peak - 60), a, dr4b::angle()),
        fmt("inside kTolDeg %.1f deg", dr4b::kTolDeg), ok && std::fabs(a - 60) <= dr4b::kTolDeg);
    // Steady state at every angle the route releases at. The plant is not the robot's own model:
    // mid-stage mass (gravity curve differs from gravityShare()), gear slop, kG from a hand tuning.
    std::vector<double> targets;
    const auto add = [&](const double* v, int n) {
        for (int i = 0; i < n; ++i) {
            bool dup = std::isnan(v[i]);
            for (double t : targets) dup = dup || std::fabs(t - v[i]) < 0.05;
            if (!dup) targets.push_back(v[i]);
        }
    };
    add(dr4b::kShort, 4); add(dr4b::kAlliance, 5); add(dr4b::kTall, 3);
    std::string out;
    double worstDeg = 0, worstIn = 0, worstDrift = 0;
    for (double target : targets) {
        dr4b::to(target);
        pros::delay(2000);
        const double a0 = lift.a * 180 / PI;
        pros::delay(500);
        const double a = lift.a * 180 / PI, eDeg = a - target, eIn = liftRiseIn(a) - dr4b::riseIn(target);
        worstDeg = std::max(worstDeg, std::fabs(eDeg)); worstIn = std::max(worstIn, std::fabs(eIn)); worstDrift = std::max(worstDrift, std::fabs(a - a0));
        out += fmt("%s%.1f: %+.2f deg (%+.2f in)", out.empty() ? "" : ", ", target, eDeg, eIn);
    }
    row("d2", fmt("lift steady state at the route's %zu release angles, holding the preload Pin: TRUE arm angle minus target (height error of the object). "
                  "Plant: mid-stage mass %.2f kg, slop %.1f deg, kG tuning error %+.0f %%", targets.size(), truth.midKg, truth.liftSlopDeg, 100 * truth.kGErr),
        fmt("%s; worst %.2f deg / %.2f in, worst drift in 0.5 s %.3f deg", out.c_str(), worstDeg, worstIn, worstDrift),
        fmt("compare with kTolDeg %.1f deg and the code's drop clearance %.1f in; lift masses, slop and friction are ASSUMED", dr4b::kTolDeg, dr4b::kDropClearIn), -1);
    dr4b::to(0);
    pros::delay(1500);

    // Step responses for gain tuning. Settled = encoder inside 1.0 deg of the target and staying there.
    const double steps[][2] = {{0, 11}, {11, 0}, {0, 36}, {36, 52}, {52, 0}, {0, 69}};
    int n = 0;
    for (const auto& st : steps) {
        dr4b::to(st[0]);
        pros::delay(2000);
        dr4b::to(st[1]);
        const double dir = st[1] > st[0] ? 1.0 : -1.0;
        int lastOutMs = 0;
        double over = 0;
        for (int ms = 10; ms <= 2500; ms += 10) {
            pros::delay(10);
            const double e = dr4b::angle() - st[1];
            if (std::fabs(e) > 1.0) lastOutMs = ms;
            over = std::max(over, e * dir);
        }
        const bool ok = lastOutMs < 2400;
        row(fmt("l%d", ++n), fmt("lift step %.0f -> %.0f deg, kP %.2f kD %.3f kI %.2f kG %.2f", st[0], st[1], dr4b::kP, dr4b::kD, dr4b::kI, dr4b::kG),
            fmt("%s after %d ms; encoder %+.2f deg at rest; TRUE arm %+.2f deg off; overshoot %.2f deg", ok ? "settled" : "NOT SETTLED", lastOutMs + 10,
                dr4b::angle() - st[1], lift.a * 180 / PI - st[1], std::max(0.0, over)),
            "informational", -1);
    }
    dr4b::to(0);
    pros::delay(1500);
}

void distanceChecks() {
    openFloor = true;   // walls only: these three poses are compared with plain wall geometry
    const double poses[3][3] = {{0, 55, 0}, {55, 0, 90}, {-40, -50, 180}};
    pros::Distance* dev[3] = {&distFront(), &distLeft(), &distRight()};
    const char* name[3] = {"front", "left", "right"};
    int k = 1;
    for (const auto& ps : poses) {
        worldTeleport(ps[0], ps[1], ps[2]);
        pros::delay(150);
        std::string out;
        bool pass = true;
        for (int i = 0; i < 3; ++i) {
            const auto& m = localizer.mounts[i];
            const double th = ps[2] * PI / 180, hs = (ps[2] + m.angleDeg) * PI / 180;
            const double sx = ps[0] + std::sin(th) * m.forwardIn + std::cos(th) * m.rightIn;
            const double sy = ps[1] + std::cos(th) * m.forwardIn - std::sin(th) * m.rightIn;
            const double ux = std::sin(hs), uy = std::cos(hs);
            double d = 1e9;
            if (ux > 1e-9) d = std::min(d, (simcfg::kHalf - sx) / ux);
            if (ux < -1e-9) d = std::min(d, (-simcfg::kHalf - sx) / ux);
            if (uy > 1e-9) d = std::min(d, (simcfg::kHalf - sy) / uy);
            if (uy < -1e-9) d = std::min(d, (-simcfg::kHalf - sy) / uy);
            const double mmTrue = d * 25.4;
            const int mm = dev[i]->get();
            const bool ok = mmTrue > 2000 ? mm == 9999 : std::fabs(mm - mmTrue) <= (mmTrue < 200 ? 15.0 : 0.05 * mmTrue);
            pass = pass && ok;
            out += fmt("%s%s %d mm (true %.0f)%s", i ? ", " : "", name[i], mm, mmTrue, ok ? "" : " BAD");
        }
        row(fmt("e%d", k++), fmt("distance sensors at TRUE pose (%.0f, %.0f, %.0f deg)", ps[0], ps[1], ps[2]), out,
            "+-15 mm under 200 mm, 5% beyond, 9999 past 2000 mm", pass);
    }
}

// Final CAD: the true robot must turn about the middle of its traction axle (1.5 in ahead of the frame centre), which is
// also the point the robot code calls its pose. A slow spin (grip holds) and a full-power one (it may not).
void pivotCheck() {
    openFloor = true;
    int k = 0;
    for (int cmd : {40, 127}) {
        place(0, 0, 0);
        double fx0, fy0; frameCentre(fx0, fy0);
        traceStart();
        driveRaw(cmd, -cmd);
        for (int t = 0; t < 8000 && trueDeg() < 360.0 - (cmd == 127 ? 60.0 : 2.0); t += 2) pros::delay(2);
        driveRaw(0, 0);
        settle(8000);
        traceStop();
        double axleMax = 0, frameMax = 0, wMax = 0;
        for (const auto& r : trace) {
            const double th = r.thDeg * PI / 180.0;
            axleMax = std::max(axleMax, std::hypot(r.x, r.y));
            frameMax = std::max(frameMax, std::hypot(r.x - simcfg::kAxleFwdIn * std::sin(th) - fx0, r.y - simcfg::kAxleFwdIn * std::cos(th) - fy0));
            wMax = std::max(wMax, r.wDeg);
        }
        double fx, fy; frameCentre(fx, fy);
        const arc::Pose p = chassis.getPose();
        const bool slow = cmd == 40;
        row(fmt("p%d", ++k), fmt("turning centre: spin in place, move(+%d) / move(-%d), about one turn. How far the TRUE traction axle centre and the TRUE frame "
                                 "centre moved, and what odometry says%s", cmd, cmd, slow ? "" : " (full power)"),
            fmt("turned %.0f deg, peak %.0f deg/s | traction axle centre: at most %.2f in from where it started, ends %.2f in away | frame centre: at most %.2f in "
                "away (a %.2f in radius circle), ends %.2f in away | odometry says it moved %.2f in, heading error %.2f deg",
                trueDeg(), wMax, axleMax, std::hypot(body.x, body.y), frameMax, frameMax / 2, std::hypot(fx - fx0, fy - fy0), std::hypot(p.x, p.y),
                std::remainder(p.theta - trueDeg(), 360.0)),
            slow ? "axle centre and odometry within 0.25 in, frame centre on a 1.5 in radius circle (2.7 to 3.3 in across)" : "informational",
            slow ? (axleMax <= 0.25 && std::hypot(p.x, p.y) <= 0.25 && frameMax >= 2.7 && frameMax <= 3.3) : -1);
    }
}

// Toggle approach and spinner, at the route's own numbers: pre-gap pose (0, 60.71, 0) = front rails 3.0 in from the red
// wall, localizer.closeToWall(6.50, 1.15), toggle::flip(). Lift at kToggleUp holding the Pin, as in the route.
void toggleChecks() {
    openFloor = false;
    const double preY = 60.71, railIn = 6.50, gapIn = 1.15;
    const char* col[3] = {"yellow", "blue", "RED"};
    dr4b::to(dr4b::kToggleUp);
    dr4b::waitSettled(3000);
    pros::delay(500);
    const double bias = localizer.mounts[0].biasIn;
    struct Case { const char* id; bool sensor; double odomErr; int judge; };   // odomErr: odometry y minus true y at the pre-gap pose
    const Case cases[] = {{"t1", true, 0.0, 1}, {"t1b", true, 0.75, 1}, {"t1c", true, -0.75, 1},
                          {"t2", false, 0.0, 1}, {"t2b", false, 0.75, -1}, {"t2c", false, -0.75, -1}};
    for (const Case& c : cases) {
        localizer.mounts[0].biasIn = c.sensor ? bias : NAN;   // NAN = a front sensor nobody calibrated: the code falls back to odometry
        driveRaw(0, 0);
        worldTeleport(0, preY, 0);
        pros::delay(40);
        chassis.setPose(0, preY + c.odomErr, 0);
        pros::delay(150);
        toggleReset(simcfg::kPaddleParkDeg);
        const int bump0 = counters.toggleBumpMs, claw0 = counters.clawWallMs, s0 = counters.closeSensor, o0 = counters.closeOdom;
        const double gap0 = railGapIn();
        const uint32_t t0 = pros::millis();
        const bool ok = localizer.closeToWall(railIn, gapIn);
        const uint32_t dur = pros::millis() - t0;
        pros::delay(150);
        const double gap = railGapIn();
        const bool in = gap >= 0.92 && gap <= 1.41;
        std::string flipTxt;
        int state = -1;
        if (c.odomErr == 0.0) {   // then the route spins the paddle
            const uint32_t f0 = pros::millis();
            toggle::flip();
            pros::delay(200);
            state = toggleState();
            flipTxt = fmt(" | toggle::flip() from there: %d strike(s), Toggle ends %s, %u ms, paddle blocked by the wall %d ms, claw underside %.2f in up (Toggle top %.2f)",
                          counters.toggleSweeps, col[state], pros::millis() - f0, counters.toggleJamMs, counters.flipClawUnderIn, simcfg::kToggleTopTurningIn);
        }
        row(c.id, fmt("localizer.closeToWall(%.2f, %.2f) from the route's pre-gap pose (0, %.2f, 0), TRUE rail gap %.2f in; %s; odometry %s", railIn, gapIn, preY,
                      gap0, c.sensor ? "front sensor calibrated" : "NO usable front sensor (biasIn NAN)",
                      c.odomErr == 0.0 ? "exact" : fmt("%.2f in %s the wall than the truth", std::fabs(c.odomErr), c.odomErr > 0 ? "nearer" : "further from").c_str()),
            fmt("%s, used %s, %u ms; TRUE rail gap now %.2f in (%s), heading %.2f deg; chain / plates on the Toggle %d ms, claw on the wall or Toggle %d ms%s",
                ok ? "returned true" : "SKIPPED", counters.closeSensor > s0 ? "the front sensor" : counters.closeOdom > o0 ? "ODOMETRY" : "nothing", dur, gap,
                in ? "inside 0.92 to 1.41" : "OUTSIDE 0.92 to 1.41", std::remainder(trueDeg(), 360.0), counters.toggleBumpMs - bump0, counters.clawWallMs - claw0,
                flipTxt.c_str()),
            c.judge == 1 ? (c.odomErr == 0.0 ? "gap inside the single-strike window 0.92 to 1.41 in, and the Toggle ends red" : "gap inside 0.92 to 1.41 in")
                         : "informational: without the sensor the odometry error goes straight into the gap",
            c.judge == 1 ? (in && (state < 0 || state == 2)) : -1);
    }
    localizer.mounts[0].biasIn = bias;
    // what flip() does at each true gap, and from each paddle start angle (ASSUMED: one strike = one state)
    std::string out;
    // (the chain stops the rails at 0.78 in inside the Toggle's span, so the sweep starts there)
    for (int g = 80; g <= 150; g += 5) {
        driveRaw(0, 0);
        worldTeleport(0, simcfg::kHalf - railIn - g / 100.0, 0);
        pros::delay(100);
        toggleReset(simcfg::kPaddleParkDeg);
        toggle::flip();
        pros::delay(150);
        out += fmt("%s%.2f: %d -> %s%s", out.empty() ? "" : ", ", counters.flipGapIn, counters.toggleSweeps, col[toggleState()],
                   counters.toggleJamMs ? " (paddle BLOCKED by the wall)" : counters.toggleStoppedInside ? " (ended inside the Toggle)" : "");
    }
    row("t3", fmt("toggle::flip() (kFlipTurns %.1f) with the robot standing at each TRUE rail gap, paddle parked back: strikes -> Toggle end state. "
                  "ASSUMED: every paddle end through the Toggle = one state", toggle::kFlipTurns), out,
        "red needs 2 strikes (or 5): one end per turn. CAD arithmetic: 0.92 to 1.41 in", -1);
    out.clear();
    for (double a : {0.0, 9.2, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0}) {
        driveRaw(0, 0);
        worldTeleport(0, simcfg::kHalf - railIn - gapIn, 0);
        pros::delay(100);
        toggleReset(a);
        pros::delay(50);
        toggle::flip();
        pros::delay(150);
        out += fmt("%s%.0f deg: %d -> %s%s", out.empty() ? "" : ", ", a, counters.toggleSweeps, col[toggleState()],
                   counters.toggleStoppedInside ? " (the robot pushed the idle paddle against the Toggle, and flip() ended with it inside the Toggle)" : "");
    }
    row("t4", "toggle::flip() at a TRUE rail gap of 1.15 in from each paddle start angle (0 = long end forward, 9 = the CAD pose, 180 = parked back as the header asks). "
              "Nothing in the code homes the paddle", out, "informational", -1);
    driveRaw(0, 0);
    worldTeleport(0, 40, 0);
    pros::delay(40);
    dr4b::to(0);
    pros::delay(1500);
}

void localizerCheck() {
    openFloor = false;   // real field: Loader, wall stacks, Goals
    worldTeleport(58.80, 59.12, 0.0);
    pros::delay(40);
    chassis.setPose(58.80f + 1.06f, 59.12f - 1.06f, 0.0f);
    pros::delay(250);
    const double before = odomErr().xy;
    const int a0 = counters.resetApplied;
    localizer.applyImmediateCorrectionAuto();
    pros::delay(800);
    const double after = odomErr().xy;
    const arc::Pose p = chassis.getPose();
    row("f1", "localizer.applyImmediateCorrectionAuto() at the top Loader stop (58.80, 59.12, 0) with odometry offset 1.5 in",
        fmt("odometry error %.2f in -> %.2f in (odom now %.2f, %.2f), reset applied %d", before, after, p.x, p.y, counters.resetApplied - a0),
        "within 0.5 in after the reset", after <= 0.5);
}
// The two IMUs are on separate ports and are not sampled at the same instant. Same spin as a2/b2,
// with each sensor's sample arriving +-1 ms around the moment odometry reads it, so a read can see
// no new sample from one sensor and two from the other. Runs last: the fuser may drop a sensor.
void imuJitterCheck() {
    openFloor = true;
    const int d0 = counters.imuDisagree, n0 = counters.imuTicks;
    imuTiming(1.0, 0, 0);
    place(0, 0, 0);
    driveRaw(127, -127);
    pros::delay(2000);
    driveRaw(0, 0);
    settle(8000);
    const OdomErr e = odomErr();
    row("b2j", "ROBOT FINDING if V5 sample timing is like this: the b2 spin again, each IMU sample arriving +-1 ms around the odometry read instead of both together",
        fmt("heading error %.2f deg after %.0f deg turned; %d of %d odometry reads had the two sensors more than the 2 deg gate apart", e.head, trueDeg(),
            counters.imuDisagree - d0, counters.imuTicks - n0),
        "imu_fusion.hpp keeps one sensor's change and discards the other on such a read. Real timing unknown: log both raw per-tick changes in a fast spin", -1);
    imuTiming(0.0, 3, 3);
}
}  // namespace

void simapp::runSelftest() {
    row("init", "real initialize(): chassis.calibrate() (both IMUs), preflight()", fmt("preflight missing %d, dual IMU in use %d of 2", counters.preflightMissing,
        static_cast<int>(imu(wiring.imu[0]).everCal) + static_cast<int>(imu(wiring.imu[1]).everCal)), "0 missing, 2 sensors", counters.preflightMissing == 0 &&
        imu(wiring.imu[0]).everCal && imu(wiring.imu[1]).everCal);
    openLoop();
    pivotCheck();
    closedLoop();
    setPoseLoss();
    liftChecks();
    distanceChecks();
    localizerCheck();
    toggleChecks();
    imuJitterCheck();

    int fails = 0;
    std::string out = fmt("SELFTEST (%s drive gains, noise off)\n", opt.tunedDrive ? "sim-tuned --tuned-drive" : "src/main.cpp");
    out += "| id | check | measured | limit | result |\n|---|---|---|---|---|\n";
    for (const auto& r : rows) {
        out += "| " + r.id + " | " + r.what + " | " + r.measured + " | " + r.limit + " | " + (r.pass == 1 ? "PASS" : r.pass == 0 ? "FAIL" : "INFO") + " |\n";
        fails += r.pass == 0;
    }
    out += fmt("\n%d FAIL of %zu rows\n", fails, rows.size());
    const std::string path = opt.outDir + "/" + opt.tag + ".md";
    std::ofstream(path) << out;
    std::fputs(("\n" + out).c_str(), stderr);
    std::fprintf(stderr, "[sim] selftest table written to %s\n", path.c_str());
    exitNow(fails ? 1 : 0);
}
