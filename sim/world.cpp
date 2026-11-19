// world.cpp - physics, devices, field, game objects and logging for the 14683A sim.
// All physics and scoring use the TRUE pose. The robot only ever sees it through
// the fake sensors below. Distance and odometry sensor models are written here
// from the hardware description, not from the robot's WallLocalizer geometry.
#include "world.hpp"
#include "sim_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <random>

using namespace simcfg;

namespace sim {
namespace {
constexpr double PI = 3.14159265358979323846;
inline double d2r(double d) { return d * PI / 180.0; }
inline double r2d(double r) { return r * 180.0 / PI; }
inline double towardZero(double v, double d) { return std::fabs(v) <= d ? 0.0 : v - std::copysign(d, v); }

std::mt19937_64 rng;
std::mt19937_64 trng;   // sample timing (IMU jitter): drawn whether or not --no-noise is set
double gauss(double mean, double sd) {
    if (!opt.noise || sd <= 0) return mean;
    return std::normal_distribution<double>(mean, sd)(rng);
}
double unif(double a, double b) {
    if (!opt.noise) return 0.5 * (a + b);
    return std::uniform_real_distribution<double>(a, b)(rng);
}

FILE* logf = nullptr;
int heldObj = -1;   // index into objs of the object in the claw
int heldNear = -1, heldHit = -1;   // Goal the held object is over / hit from the side on the way in
bool disabled = false;             // field control disable: motors coast, commands ignored
bool imuBDead = false;
int64_t imuNominal[2] = {3, 3};    // each IMU's own sample clock (ms)
uint64_t imuNext[2] = {3, 3};
double sway = 0, swayV = 0;        // fore-aft deflection of the raised claw, in and in/s
std::vector<std::string> events;
bool worldReady = false;
// Toggle spinner paddle: a = direction of its long end in the robot's side view (rad, 0 forward, 90 up),
// in[e] = end e (0 long, 1 short) went into the Toggle while the paddle was turning.
struct Paddle { double a = 0, w = 0; bool in[2] = {false, false}; double aIn[2] = {0, 0}, dirIn[2] = {0, 0}; bool jamLogged = false; } paddle;
}  // namespace
static double heldBottom();
static void noteFlipStart();

Truth truth;
Body body;
LiftState lift;
Counters counters;
Wiring wiring;
bool openFloor = false;
std::vector<PlaceRec> placeRecs;

// ================================================================ logging
std::string fmt(const char* f, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}
static void emit(const std::string& line) {
    if (logf) { std::fputs(line.c_str(), logf); std::fputc('\n', logf); }
    if (!opt.quiet) { std::fputs(line.c_str(), stdout); std::fputc('\n', stdout); }
}
void logLine(const std::string& s) { emit(fmt("[%9.3f sim       ] ", now() / 1000.0) + s); }
void event(const std::string& body_) { events.push_back(fmt("{\"t\": %.3f, ", now() / 1000.0) + body_ + "}"); }
const std::vector<std::string>& allEvents() { return events; }
void openLog(const std::string& path) { logf = std::fopen(path.c_str(), "w"); }
void closeLog() { if (logf) std::fclose(logf); logf = nullptr; }

void robotLine(const std::string& task, const std::string& s) {
    emit(fmt("[%9.3f %-10.10s] ", now() / 1000.0, task.c_str()) + s);
    auto has = [&](const char* k) { return s.find(k) != std::string::npos; };
    if (has("[reset] applied")) { ++counters.resetApplied; event("\"type\": \"reset_applied\", \"line\": \"" + s + "\""); }
    if (has("[reset] mount")) {
        if (has("REJECT")) ++counters.resetMountReject;
        else if (has("-> ok")) ++counters.resetMountOk;
        else ++counters.resetMountSkip;
    }
    if (has("[reset] skipped")) ++counters.resetSkipped;
    if (has("[square]")) { ++counters.squares; event("\"type\": \"square\", \"line\": \"" + s + "\""); }
    if (has("NOT settled")) { ++counters.liftNotSettled; event("\"type\": \"lift_not_settled\", \"line\": \"" + s + "\""); }
    if (has("grip NOT SEEN")) { ++counters.gripNotSeen; event("\"type\": \"grip_not_seen\""); }
    if (has("grip ok")) ++counters.gripOk;
    if (has("parking in the Midfield")) counters.parkAtMs = static_cast<int>(now());
    if (has("target not tuned")) ++counters.untunedTargets;
    if (has("[preflight] MISSING")) ++counters.preflightMissing;
    if (has("[close] ")) {
        const char* how = has("SKIPPED") ? "skipped" : has("ODOMETRY") ? "odometry" : "front sensor";
        ++(has("SKIPPED") ? counters.closeSkipped : has("ODOMETRY") ? counters.closeOdom : counters.closeSensor);
        if (counters.closeGapIn < 0) counters.closeGapIn = railGapIn();
        event(fmt("\"type\": \"close_to_wall\", \"by\": \"%s\", \"true_rail_gap_in\": %.3f, \"true\": [%.3f, %.3f, %.2f], \"line\": \"%s\"", how,
                  railGapIn(), body.x, body.y, r2d(body.th), s.c_str()));
        logLine(fmt("[close] closeToWall() ended (%s): TRUE front rail gap %.2f in at the centre line, heading %.2f deg", how, railGapIn(),
                    std::remainder(r2d(body.th), 360.0)));
    }
    if (has("toggle spun")) {
        const char* col[3] = {"yellow", "blue", "RED"};
        const bool inside = paddle.in[0] || paddle.in[1];
        counters.toggleStoppedInside += inside;
        event(fmt("\"type\": \"toggle_flip_end\", \"strikes_so_far\": %d, \"state\": \"%s\", \"paddle_deg\": %.1f, \"stopped_inside_toggle\": %s, "
                  "\"jam_ms\": %d, \"rail_gap_in\": %.3f", counters.toggleSweeps, col[toggleState()], paddleDeg(), inside ? "true" : "false",
                  counters.toggleJamMs, railGapIn()));
        logLine(fmt("[toggle] flip() done: %d strike(s) so far, Toggle is %s, paddle at %.0f deg%s, blocked by the wall for %d ms, rail gap now %.2f in",
                    counters.toggleSweeps, col[toggleState()], paddleDeg(), inside ? " and STOPPED INSIDE the Toggle (left part-turned)" : "",
                    counters.toggleJamMs, railGapIn()));
    }
    if (s.size() >= 9 && s.compare(s.size() - 9, 9, "] release") == 0) {
        ++counters.releases;
        if (heldObj < 0) {
            placeRecs.push_back({counters.releases, "nothing held", "-", false, "claw empty at release", 0, 0, 0, 0, 0, 0, now() / 1000.0});
            event(fmt("\"type\": \"placement\", \"n\": %d, \"object\": \"nothing held\", \"goal\": \"-\", \"ok\": false, \"why\": \"claw empty at release\"", counters.releases));
            logLine(fmt("[place] release #%d with the claw empty", counters.releases));
        }
    }
}
void logRumble(const char* p) { logLine(std::string("[rumble] \"") + (p ? p : "") + "\""); }

// ================================================================ lift geometry (truth)
static double zIn(double deg) {
    return kL1In * std::sin(d2r(truth.rest1Deg - deg)) + kL2In * std::sin(d2r(truth.rest2Deg + deg));
}
static double xIn(double deg) {
    return kL1In * std::cos(d2r(truth.rest1Deg - deg)) + kL2In * std::cos(d2r(truth.rest2Deg + deg));
}
// TRUE rise: the owner's table, plus this seed's linkage error (the difference between this seed's
// linkage and the nominal one), so seeds still differ a little.
static double tableRiseIn(double deg) {
    const int n = static_cast<int>(sizeof(kLiftTableDeg) / sizeof(kLiftTableDeg[0]));
    int i = 0;
    while (i < n - 2 && deg > kLiftTableDeg[i + 1]) ++i;
    const double f = (deg - kLiftTableDeg[i]) / (kLiftTableDeg[i + 1] - kLiftTableDeg[i]);   // extrapolates past both ends
    return kLiftTableIn[i] + f * (kLiftTableIn[i + 1] - kLiftTableIn[i]);
}
static double zNomIn(double deg) { return kL1In * std::sin(d2r(kRest1Deg - deg)) + kL2In * std::sin(d2r(kRest2Deg + deg)); }
double liftRiseIn(double deg) { return tableRiseIn(deg) + (zIn(deg) - zIn(0.0)) - (zNomIn(deg) - zNomIn(0.0)); }
double liftDxIn(double deg) { return xIn(deg) - xIn(0.0); }
// Torque at the lift shaft (N m) that gravity, less the bands, puts on the lift at TRUE angle deg.
// Two masses: the carriage (and object) at the claw end, which rises with both stages, and the
// mid-stage mass at the middle pivot, which rises with stage 1 only. The robot's gravityShare()
// assumes everything is at the claw end, so its feedforward does not match this curve.
static double liftLoadNm(double deg, double objKg) {
    const double dz = (liftRiseIn(deg + 0.5) - liftRiseIn(deg - 0.5)) / d2r(1.0) * IN;   // claw end, m per rad
    const double dzMid = -kL1In * IN * std::cos(d2r(truth.rest1Deg - deg));              // middle pivot, m per rad
    return ((truth.carriageKg + objKg) * dz + truth.midKg * dzMid) * G - truth.liftBandNm;
}
MotorDev& motor(int p) {
    static MotorDev m[32];
    return m[std::clamp(p, 0, 31)];
}
// Volts on both lift motors that balance that load at `deg` on this seed's plant (centre of the friction band).
double liftHoldVolts(double deg, bool withObject) {
    // stall torque of the cartridge the robot code gave its lift motors (red 100 rpm, green 200, blue 600)
    const double stall = kStallNm[std::clamp(motor(wiring.lift[0]).gear, 0, 2)];
    return 12.0 * liftLoadNm(deg, withObject ? kObjectKg : 0.0) / (2.0 * kCrank * kGearEff * kTorqueLineGain * stall);
}

// ================================================================ devices
static int mechOf(int p) {
    for (int i = 0; i < 2; ++i) {
        if (p == wiring.left[i]) return 1;
        if (p == wiring.right[i]) return 2;
        if (p == wiring.lift[i]) return 3;
    }
    if (p == wiring.toggle[1]) return 5;   // right 5.5 W motor: the paddle. The left one (toggle[0]) drives nothing in the CAD: free rotor
    return p == wiring.claw ? 4 : 0;
}
static void assignMotorTruth(MotorDev& m) {
    const double spread = unif(-kMotorSpread, kMotorSpread);
    m.tauLim = kStallNm[m.gear] * (1.0 + spread);
    m.wFree = kFreeRpm[m.gear] * 2 * PI / 60.0 * (1.0 + 0.6 * spread);
    m.mech = mechOf(m.port);
    if (m.mech == 4) m.rotorJ = kRollerInertia;
    if (m.mech == 5 || m.port == wiring.toggle[0]) m.tauLim = kToggleMotorStallNm * (1.0 + spread);   // 5.5 W motor, 200 rpm (the default green numbers otherwise)
}
void motorRegister(int sp, int gear, int units) {
    const int p = std::abs(sp);
    MotorDev& m = motor(p);
    if (!m.exists) {
        m.exists = true;
        m.port = p;
        m.mount = sp < 0 ? -1 : 1;   // hardware is assumed wired so the configured sign is correct
        m.reversed = sp < 0;
        m.gear = gear >= 0 && gear <= 2 ? gear : 1;
        if (worldReady) assignMotorTruth(m);
    } else {
        m.reversed = sp < 0;
        if (gear >= 0 && gear <= 2 && gear != m.gear) { m.gear = gear; if (worldReady) assignMotorTruth(m); }
    }
    if (units >= 0 && units <= 2) m.units = units;
}
void motorCommand(int p, int kind, double v) {
    MotorDev& m = motor(p);
    m.q.push_back({now(), kind, (m.reversed ? -1.0 : 1.0) * v});
}
static double unitScale(const MotorDev& m);
static double quantRad(const MotorDev& m, double rad);
// move_relative: the target is the position the brain last saw plus delta (ASSUMED position loop, see kMovePosKp)
void motorCommandRel(int p, double delta, double rpm) {
    MotorDev& m = motor(p);
    m.q.push_back({now(), kPos, quantRad(m, m.sPos) + (m.reversed ? -1.0 : 1.0) * delta / unitScale(m), std::fabs(rpm) * 2 * PI / 60.0});
    if (p == wiring.toggle[0]) noteFlipStart();
}
static double unitScale(const MotorDev& m) {   // units per physical rad
    switch (m.units) {
        case 1: return 1.0 / (2 * PI);
        case 2: return kTicksPerRev[m.gear] / (2 * PI);
        default: return 180.0 / PI;
    }
}
static double quantRad(const MotorDev& m, double rad) {
    const double tick = 2 * PI / kTicksPerRev[m.gear];
    return std::round(rad / tick) * tick;
}
double motorReadPos(int p) {
    MotorDev& m = motor(p);
    return (m.reversed ? -1.0 : 1.0) * (quantRad(m, m.sPos) - m.zero) * unitScale(m);
}
double motorReadRpm(int p) {
    MotorDev& m = motor(p);
    return (m.reversed ? -1.0 : 1.0) * m.sVel * 60.0 / (2 * PI);
}
void motorTare(int p, double robotPos) {
    MotorDev& m = motor(p);
    m.zero = quantRad(m, m.sPos) - (m.reversed ? -1.0 : 1.0) * robotPos / unitScale(m);
}

RotationDev& rotation(int p) {
    static RotationDev r[32];
    return r[std::clamp(p, 0, 31)];
}
void rotationRegister(int sp) {
    RotationDev& r = rotation(std::abs(sp));
    if (!r.exists) { r.exists = true; r.mount = sp < 0 ? -1 : 1; }
    r.reversed = sp < 0;
}

ImuDev& imu(int p) {
    static ImuDev d[32];
    ImuDev& x = d[std::clamp(p, 0, 31)];
    x.exists = p == wiring.imu[0] || (p == wiring.imu[1] && !imuBDead);   // the sensors plugged in
    return x;
}
// Sim-side copy of the fusion gate (imu_fusion.hpp: 2 deg or 5 % per update): counts the robot's
// reads where the two sensors' changes since the previous read differ by more than that.
void imuReadNote(int p, double v) {
    static double last[2] = {NAN, NAN}, delta[2] = {0, 0};
    static uint64_t at[2] = {~0ull, ~0ull};
    const int i = p == wiring.imu[0] ? 0 : p == wiring.imu[1] ? 1 : -1;
    if (i < 0) return;
    delta[i] = std::isnan(last[i]) ? 0.0 : v - last[i];
    last[i] = v; at[i] = now();
    if (i == 1 && at[0] == at[1]) {
        ++counters.imuTicks;
        if (std::fabs(delta[0] - delta[1]) > std::max(2.0, 0.05 * std::max(std::fabs(delta[0]), std::fabs(delta[1])))) ++counters.imuDisagree;
    }
}
void imuTiming(double jitterMs, int phaseA, int phaseB) {
    truth.imuJitterMs = jitterMs;
    const int ph[2] = {phaseA, phaseB};
    for (int i = 0; i < 2; ++i) {
        truth.phaseImu[i] = ph[i];
        imuNominal[i] = static_cast<int64_t>(now() / kImuPeriodMs + 1) * kImuPeriodMs + ph[i];
        imuNext[i] = static_cast<uint64_t>(imuNominal[i]);
    }
}
void imuReset(int p) {
    ImuDev& d = imu(p);
    if (!d.exists) return;
    d.calibrating = true;
    d.calEnd = now() + kImuCalMs + static_cast<uint64_t>(unif(0, 120));
    logLine(fmt("[imu] port %d calibrating until %.3f s", p, d.calEnd / 1000.0));
}

struct DistDev { bool exists = false; int32_t value = 9999; int phase = 0; };
static DistDev& dist(int p) {
    static DistDev d[32];
    return d[std::clamp(p, 0, 31)];
}
void distanceRegister(int p) {
    DistDev& d = dist(p);
    if (!d.exists) { d.exists = true; d.phase = (p * 7) % kDistPeriodMs; }
}
int32_t distanceGet(int p) { return dist(p).value; }

// ================================================================ controller script
ControllerScript script;
int controllerAnalog(int ch) { return (ch >= 0 && ch < 4) ? script.analog[ch] : 0; }
int controllerDigital(int b) { return (b >= 0 && b < 32) ? script.digital[b] : 0; }

// ================================================================ field
namespace {
struct Goal { const char* name; int type; double x, y; bool needsToggle; int objects = 0; int pins = 0; int startPins = 0; };
std::vector<Goal> goals;
struct Circle { double x, y, r, h; };
std::vector<Circle> staticCups;   // wall stacks and diamond vertex Cups (beam targets only, the route stays clear of them)
struct Box { double x0, y0, x1, y1; };
std::vector<Box> loaderBoxes;
// F7: cluster objects the robot plows. Pushed kinematically out of the robot footprint, stopped by
// walls / Goals / Loaders; one that cannot get out of the way pushes back on the robot (jam).
// ponytail: discs do not collide with each other and a flat Pin is a 1.58 in disc, not a 6.5 in
// cylinder. Upgrade to rigid bodies if cluster outcomes start to decide the result.
struct Loose { std::string kind; double x, y, r, h, x0, y0, pushed = 0; bool reported = false; };
std::vector<Loose> loose;

enum ObjState { kStanding = 0, kHeld = 1, kOnGoal = 2, kLost = 3, kTipped = 4, kQueued = 5 };
struct Obj {
    std::string name;
    int units;        // 1 bare Pin, 2 Cup+Pin
    double x, y;
    int state;
    double pushed = 0;
    int loader = -1;  // which Loader queue it belongs to
    double dz = 0;      // held: object bottom minus lift rise (minus the claw rise it was gripped at)
    double latOff = 0;  // held: sideways offset from the claw centre line
};
std::vector<Obj> objs;
struct Loader { double x, y; std::vector<int> queue; size_t next = 0; uint64_t presentAt = 0; int presented = -1; };
std::vector<Loader> loaders;
int outMs = 0;
bool intakeSession = false, sessionGrabbed = false;
uint64_t sessionStart = 0;
int placements = 0;

double goalTop(int type) { return opt.drawHeights ? kGoalTopDraw[type] : kGoalTop[type]; }
const char* typeName(int t) { return t == 0 ? "Short" : t == 1 ? "Alliance" : "Tall"; }

void P(double xa, double ya, double& x, double& y) { x = (ya - 72.0) * kScale; y = (72.0 - xa) * kScale; }

void buildField() {
    // G1 and G2 lie in the red-wall Quadrant: their Pins are Owned only with that Toggle red (2 sweeps)
    goals = {{"Short G1 (24,96)", 0, 23.40, 46.80, true},  {"Short (48,120)", 0, 46.80, 23.40, false},
             {"Short (120,48)", 0, -46.80, -23.40, false},  {"Short (96,24)", 0, -23.40, -46.80, false},
             {"Alliance G2 (24,48)", 1, -23.40, 46.80, true}, {"Alliance (48,24)", 1, -46.80, 23.40, false},
             {"Alliance blue (120,96)", 1, 46.80, -23.40, false}, {"Alliance blue (96,120)", 1, 23.40, -46.80, false},
             {"Tall (72,72)", 2, 0.0, 0.0, false}};
    for (auto& g : goals)
        if (opt.goalPins && g.type != 1) { g.objects = 1; g.startPins = 1; }
    // wall stacks: 3 Cups side by side, middle one holds a Pin (taller)
    const double along[3] = {44.84, 48.0, 51.16}, along2[3] = {92.84, 96.0, 99.16};
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < 3; ++i) {
            const double a = s ? along2[i] : along[i];
            const double h = i == 1 ? kCupPinH : kCupH;
            double x, y;
            P(1.58, a, x, y); staticCups.push_back({x, y, kCupR, h});
            P(142.42, a, x, y); staticCups.push_back({x, y, kCupR, h});
            P(a, 1.58, x, y); staticCups.push_back({x, y, kCupR, h});
            P(a, 142.42, x, y); staticCups.push_back({x, y, kCupR, h});
        }
    for (auto c : {std::pair{72.0, 96.0}, {96.0, 72.0}, {72.0, 48.0}, {48.0, 72.0}}) {     // diamond vertex Cups + Pins
        double x, y; P(c.first, c.second, x, y); staticCups.push_back({x, y, kCupR, kCupPinH});
    }
    if (opt.clusters)
        for (auto c : {std::pair{24.0, 120.0}, {48.0, 96.0}, {96.0, 48.0}, {120.0, 24.0}}) {   // clusters: clear Cup + 4 flat Pins
            double x, y; P(c.first, c.second, x, y);
            loose.push_back({"cluster Cup", x, y, kCupR, kCupH, x, y});
            for (auto d : {std::pair{4.83, 0.0}, {-4.83, 0.0}, {0.0, 4.83}, {0.0, -4.83}})
                loose.push_back({"flat Pin", x + d.first, y + d.second, kFlatPinR, kFlatPinH, x + d.first, y + d.second});
        }
    // Loaders (chutes): 4.84 wide, face 3.74 off the wall
    for (double sx : {-truth.loaderX, truth.loaderX})
        for (double sy : {-1.0, 1.0}) {
            const double y0 = sy > 0 ? kLoaderFaceY : -kHalf, y1 = sy > 0 ? kHalf : -kLoaderFaceY;
            loaderBoxes.push_back({sx - kLoaderHalfW, y0, sx + kLoaderHalfW, y1});
        }
    // diagonal floor Cup+Pins
    for (auto c : {std::pair{24.0, 24.0}, {48.0, 48.0}, {96.0, 96.0}, {120.0, 120.0}}) {
        Obj o; P(c.first, c.second, o.x, o.y);
        o.name = fmt("diag Cup+Pin (%g,%g)", c.first, c.second); o.units = 2; o.state = kStanding;
        objs.push_back(o);
    }
    // preload, held
    { Obj o; o.name = "preload Pin"; o.units = 1; o.x = o.y = 0; o.state = kHeld; o.dz = truth.preloadDzIn; objs.push_back(o); heldObj = (int)objs.size() - 1; }
    // red Loaders: top (+x) 3 Cup+Pin then a bare Pin, bottom (-x) 2 Cup+Pin then a bare Pin
    const std::vector<int> topQ = {2, 2, 2, 1}, botQ = {2, 2, 1};
    for (int L = 0; L < 2; ++L) {
        Loader ld; ld.x = L == 0 ? truth.loaderX : -truth.loaderX; ld.y = kLoaderY;
        const auto& q = L == 0 ? topQ : botQ;
        for (size_t i = 0; i < q.size(); ++i) {
            Obj o; o.name = fmt("%s Loader load %zu (%s)", L == 0 ? "top" : "bottom", i + 1, q[i] == 2 ? "Cup+Pin" : "Pin");
            o.units = q[i]; o.state = kQueued; o.loader = L; o.x = ld.x; o.y = ld.y;
            objs.push_back(o); ld.queue.push_back((int)objs.size() - 1);
        }
        ld.presentAt = 0;
        loaders.push_back(ld);
    }
}
}  // namespace

// ================================================================ init
void worldInit() {
    rng.seed(0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(opt.seed) * 1315423911ull);
    trng.seed(0xD1B54A32D192ED03ull ^ static_cast<uint64_t>(opt.seed) * 2654435761ull);
    // unif() returns the midpoint and gauss() the mean with --no-noise, so every line below gives
    // the nominal value there. A knob (>= 0) replaces the draw but the draw is still made, so the
    // same seed keeps every other value when one knob is set.
    const auto knob = [](double o, double drawn) { return o >= 0 ? o : drawn; };
    const auto spread = [](double f) { return 1 + unif(-f, f); };
    truth.mu = kMu * spread(kMuSpread);
    truth.rollResist = kRollResist * spread(kRollSpread);
    const double cal = opt.calibrated ? 0.2 : 1.0;   // what is left of each calibration error once it has been measured
    truth.wheelDia = (opt.wheelDia > 0 ? opt.wheelDia : kTrackWheelDiaIn) * (1 + cal * gauss(kWheelDiaErrMean, kWheelDiaErrSd));
    truth.wheelOffset = kTrackWheelOffsetIn + cal * gauss(0, kWheelOffsetSd);
    truth.rearIn = opt.rearIn > 0 ? opt.rearIn : kRearIn;
    truth.loaderX = opt.loaderX > 0 ? opt.loaderX : kLoaderX;
    truth.scrubLeverIn = knob(opt.scrubIn, opt.noise ? std::exp(unif(std::log(kScrubLo), std::log(kScrubHi))) : kScrubLeverIn);
    truth.massKg = knob(opt.massKg, kMassKg * spread(0.08));
    truth.yawJ = knob(opt.yawJ, kChassisYawJ * spread(0.25));
    truth.driveFricN = knob(opt.driveFricN, kDriveFricN * spread(0.50));
    truth.rest1Deg = kRest1Deg + opt.liftErrDeg + unif(-kLiftAngleErrDeg, kLiftAngleErrDeg);
    truth.rest2Deg = kRest2Deg + opt.liftErrDeg + unif(-kLiftAngleErrDeg, kLiftAngleErrDeg);
    truth.carriageKg = knob(opt.carriageKg, kCarriageKg * spread(0.10));
    truth.midKg = kMidStageKg * spread(0.30);
    truth.liftFricNm = knob(opt.liftFricNm, kLiftFricNm * spread(0.30));
    truth.liftBandNm = knob(opt.liftBandNm, kLiftBandNm);
    truth.liftSlopDeg = knob(opt.liftSlopDeg, unif(0.0, 2 * kLiftSlopDeg));
    truth.kGErr = unif(-kKgTuneErr, kKgTuneErr);
    truth.pitch = knob(opt.pitchIn, opt.noise ? kPitch + cal * (unif(kPitch, kPitchOld) - kPitch) : kPitch);
    truth.preloadDzIn = gauss(0, kPreloadDzSdIn);
    truth.grabAheadIn = knob(opt.grabAheadIn, kGrabAheadIn);
    truth.grabLatIn = knob(opt.grabLatIn, kGrabLatIn);
    truth.releaseFwdIn = knob(opt.releaseFwdIn, kReleaseFwdIn);
    truth.releaseMs = knob(opt.releaseMs, kReleaseMs);
    truth.captureR = knob(opt.captureR, kCaptureR);
    truth.imuJitterMs = knob(opt.imuJitterMs, opt.noise ? kImuJitterMs : 0.0);
    truth.pivotFwdIn = knob(opt.pivotFwdIn, kPivotFwdIn);
    truth.cgFwdIn = knob(opt.cgFwdIn, kCgFwdIn);
    if (opt.noise) {   // where in the 10 ms tick each sensor's sample lands, and when autonomous() starts
        const auto ph = [] { return std::min(9, static_cast<int>(unif(0.0, 10.0))); };
        truth.phaseImu[0] = ph(); truth.phaseImu[1] = ph(); truth.phaseMotor = ph(); truth.phaseRot = ph(); truth.autonDelayMs = ph();
    }
    if (opt.imuPhaseB >= 0) truth.phaseImu[1] = static_cast<int>(opt.imuPhaseB) % kImuPeriodMs;
    imuBDead = opt.imuBMissing;
    const double commonGainErr = (opt.calibrated ? 0.2 : 1.0) * gauss(0, kImuCommonGainSd);   // both sensors uncalibrated the same way
    for (int i = 0; i < 2; ++i) {
        ImuDev& d = imu(wiring.imu[i]);
        d.gain = (opt.imuGain > 0 ? opt.imuGain : 1.0 + commonGainErr) * (1 + (opt.calibrated ? 0.2 : 1.0) * gauss(0, kImuGainSd));
        truth.imuGain[i] = d.gain;
        d.driftDegPerMs = gauss(0, kImuDriftDegPerMinSd) / 60000.0;
        imuNominal[i] = truth.phaseImu[i];
        imuNext[i] = truth.phaseImu[i];
    }
    const double nominal[3][4] = {{5.71, 7.75, 0.0, kFrontSensorHIn}, {-4.96, 0.5, -90.0, kSideSensorHIn}, {4.96, 0.5, 90.0, kSideSensorHIn}};
    for (int i = 0; i < 3; ++i) {
        const double er = gauss(0, kMountErrSdIn), ef = gauss(0, kMountErrSdIn);
        truth.mount[i][0] = nominal[i][0] + er;
        truth.mount[i][1] = nominal[i][1] + ef;
        truth.mount[i][2] = nominal[i][2];
        truth.mount[i][3] = nominal[i][3];
        const double a = d2r(nominal[i][2]);
        truth.distOffMm[i] = gauss(0, kDistOffSdMm);
        truth.distScale[i] = gauss(0, kDistScaleSd);
        // printReadings calibration parked kDistCalIn from a wall: bias = true distance (from the
        // PLANNED mount position) minus the reading. Absorbs the along-beam mount error and the
        // sensor's offset and scale error at that one distance.
        truth.calBias[i] = ef * std::cos(a) + er * std::sin(a) -
                           (truth.distOffMm[i] + truth.distScale[i] * kDistCalIn * 25.4) / 25.4;
    }
    for (int p = 0; p < 32; ++p) if (motor(p).exists) assignMotorTruth(motor(p));
    worldReady = true;
    buildField();
    for (auto& ld : loaders)   // jitter where each load stands
        for (int id : ld.queue) { objs[id].x = ld.x + unif(-kLoaderJitterIn, kLoaderJitterIn); objs[id].y = ld.y + unif(-kLoaderJitterIn, kLoaderJitterIn); }
    // start pose: rearmost part on the red wall, heading 180, placement error. The frame centre is rearIn off the wall,
    // the true origin (traction axle) kAxleFwdIn further: 8.55 in with the CAD numbers, the route's (37.05, 61.65).
    const double th = 180.0 + unif(-kStartDeg, kStartDeg);
    const double sd = std::fabs(std::sin(d2r(th)));
    const double cy = kHalf - truth.rearIn * std::fabs(std::cos(d2r(th))) - kHalfWidthIn * sd - 0.01 - (opt.noise ? unif(0, kStartGapIn) : 0.0);
    body.x = 37.05 + unif(-kStartXYIn, kStartXYIn);
    body.y = cy + kAxleFwdIn * std::cos(d2r(th));
    body.th = d2r(th);
    // drawn last, so every other per-seed value is the same as before the Toggle spinner existed
    truth.paddleDeg0 = knob(opt.paddleDeg, kPaddleParkDeg + unif(-kPaddleParkErrDeg, kPaddleParkErrDeg));
    paddle = Paddle{};
    paddle.a = d2r(truth.paddleDeg0);
    logLine(fmt("[world] final CAD: true origin = traction axle, %.2f in ahead of the frame centre (front rails %.2f ahead of it, back of the frame %.2f behind); "
                "sideways grip acts %.2f in ahead of the frame centre%s, CG %.2f in ahead of it (ASSUMED) | rear contact up to %.2f behind the frame centre with the lift raised | "
                "Toggle paddle starts at %.0f deg (0 = long end forward, 180 = parked back), %s",
                kAxleFwdIn, kFrontIn - kAxleFwdIn, truth.rearIn + kAxleFwdIn, truth.pivotFwdIn,
                std::fabs(truth.pivotFwdIn - kAxleFwdIn) < 1e-9 ? " (it turns about its origin)" : " (--pivot-fwd: it does NOT turn about its origin)", truth.cgFwdIn,
                kL1In - kTowerPivotFwdIn + kMidLinkBehindIn, truth.paddleDeg0,
                opt.paddleDeg >= 0 ? "--paddle-deg" : opt.noise ? "parked by hand" : "parked exactly"));
    logLine(fmt("[world] seed %d noise %d: start true (%.2f, %.2f, %.2f) | mu %.3f crr %.3f | track wheel dia %.4f off %.3f | "
                "imu gain %.5f / %.5f drift %.2f / %.2f deg/min | rear contact %.2f | loader x %.3f | goal start Pins %d | clusters %d",
                opt.seed, opt.noise, body.x, body.y, th, truth.mu, truth.rollResist, truth.wheelDia, truth.wheelOffset,
                imu(wiring.imu[0]).gain, imu(wiring.imu[1]).gain, imu(wiring.imu[0]).driftDegPerMs * 60000.0,
                imu(wiring.imu[1]).driftDegPerMs * 60000.0, truth.rearIn, truth.loaderX, opt.goalPins, opt.clusters));
    logLine(fmt("[world] plant (ASSUMED, not measured): mass %.2f kg yawJ %.3f | scrub lever %.2f in | drive drag %.2f N/side | "
                "lift: rest angles %.1f / %.1f deg, carriage %.2f kg, mid-stage %.2f kg, friction %.2f N m, bands %.2f N m, slop %.2f deg, kG tuning error %+.0f %% | "
                "stack pitch %.3f in, preload dz %+.2f in | claw: grab window lat %.2f ahead %.2f in, release %.0f ms fwd %.2f in | capture radius %.2f in",
                truth.massKg, truth.yawJ, truth.scrubLeverIn, truth.driveFricN, truth.rest1Deg, truth.rest2Deg, truth.carriageKg, truth.midKg,
                truth.liftFricNm, truth.liftBandNm, truth.liftSlopDeg, 100 * truth.kGErr, truth.pitch, truth.preloadDzIn, truth.grabLatIn,
                truth.grabAheadIn, truth.releaseMs, truth.releaseFwdIn, truth.captureR));
    logLine(fmt("[world] timing: IMU A phase %d, IMU B phase %d (ms inside the 10 ms tick, odometry reads at 0), jitter +-%.0f ms%s | motor telemetry phase %d | "
                "rotation sensor phase %d | autonomous() starts %d ms after initialize() returns",
                truth.phaseImu[0], truth.phaseImu[1], truth.imuJitterMs, opt.imuBMissing ? ", IMU B UNPLUGGED" : "", truth.phaseMotor,
                truth.phaseRot, truth.autonDelayMs));
}

void worldTeleport(double x, double y, double headingDeg) {
    body.x = x; body.y = y; body.th = d2r(headingDeg);
    body.v = body.w = body.vl = 0;
}
void frameCentre(double& x, double& y) {
    x = body.x - kAxleFwdIn * std::sin(body.th);
    y = body.y - kAxleFwdIn * std::cos(body.th);
}

// ================================================================ motor model
// Torque at the cartridge output for physical voltage V at output speed w.
static double motorTorque(MotorDev& m, double w) {
    m.vel = w;
    const bool braking = m.kind == kBrake || (m.kind == kVolt && m.val == 0) || (m.kind == kVel && m.val == 0);
    double V;
    if (braking) {
        if (m.brake == 0 || disabled) {   // coast: open circuit, only gearbox drag
            m.holdLatched = false; m.volt = 0;
            m.torque = -kMotorViscFrac * m.tauLim * w / m.wFree;
            return m.torque;
        }
        if (m.brake == 1) { m.holdLatched = false; V = 0; }   // brake: windings shorted
        else {
            if (!m.holdLatched) { m.holdLatched = true; m.holdPos = m.pos; }
            V = std::clamp(kHoldKpVPerDeg * r2d(m.holdPos - m.pos) - 6.0 * w / m.wFree, -12.0, 12.0);
        }
    } else {
        m.holdLatched = false;
        if (m.kind == kVolt) V = std::clamp(m.val / 1000.0, -12.0, 12.0);
        else {
            const double target = m.kind == kPos ? std::clamp(kMovePosKp * (m.val - m.pos), -m.lim, m.lim) : m.val * 2 * PI / 60.0;
            V = std::clamp(12.0 * target / m.wFree + kVelKpFrac * 12.0 * (target - w) / m.wFree, -12.0, 12.0);
        }
    }
    m.volt = V;
    m.torque = std::clamp(kTorqueLineGain * m.tauLim * (V / 12.0 - w / m.wFree), -m.tauLim, m.tauLim) -
               kMotorViscFrac * m.tauLim * w / m.wFree;
    return m.torque;
}
static double motorFric(const MotorDev& m) { return kMotorFricFrac * m.tauLim; }

// Field control disable. ASSUMED: a disabled V5 motor coasts whatever its brake mode. worldStep()
// throws away every command while this is set, so tasks that survive the period change nothing.
void setDisabled(bool on) { disabled = on; }
bool worldAtRest() {
    return std::hypot(body.v, body.vl) / IN < 0.5 && std::fabs(body.w) < d2r(2.0) && std::fabs(lift.w) < d2r(1.0);
}

// ================================================================ contacts
namespace {
struct Wrench { double fx = 0, fy = 0, tcw = 0; };
void applyPoint(Wrench& w, double px, double py, double nx, double ny, double pen) {
    // px,py: contact point (in, world); n: unit push direction on the robot; pen in inches. Lever arm from the CG
    const double gx = body.x - (kAxleFwdIn - truth.cgFwdIn) * std::sin(body.th), gy = body.y - (kAxleFwdIn - truth.cgFwdIn) * std::cos(body.th);
    const double rx = (px - gx) * IN, ry = (py - gy) * IN;
    const double vx = body.v * std::sin(body.th) + body.vl * std::cos(body.th) + body.w * ry;
    const double vy = body.v * std::cos(body.th) - body.vl * std::sin(body.th) - body.w * rx;
    const double vn = vx * nx + vy * ny;
    const double f = std::max(0.0, kContactK * pen * IN + kContactC * std::max(0.0, -vn));
    const double fx = f * nx, fy = f * ny;
    w.fx += fx; w.fy += fy;
    w.tcw += -(rx * fy - ry * fx);
}
// Robot coordinates here are the CAD's: forward and right of the FRAME centre, which is kAxleFwdIn behind the true origin.
void robotToWorld(double fwd, double right, double& x, double& y) {
    x = body.x + std::sin(body.th) * (fwd - kAxleFwdIn) + std::cos(body.th) * right;
    y = body.y + std::cos(body.th) * (fwd - kAxleFwdIn) - std::sin(body.th) * right;
}
void worldToRobot(double x, double y, double& fwd, double& right) {
    const double dx = x - body.x, dy = y - body.y;
    fwd = dx * std::sin(body.th) + dy * std::cos(body.th) + kAxleFwdIn;
    right = dx * std::cos(body.th) - dy * std::sin(body.th);
}
// Penetration of a circle (robot frame coords cf, cr, radius r) into the box f0..f1 x -hw..hw.
// Returns pen > 0 and the unit normal (robot frame) that pushes the CIRCLE out of the box.
double boxPen(double cf, double cr, double r, double f0, double f1, double hw, double& nf, double& nr) {
    if (std::fabs(cr) > hw + r || cf > f1 + r || cf < f0 - r) return 0;
    const double qf = std::clamp(cf, f0, f1), qr = std::clamp(cr, -hw, hw);
    nf = cf - qf; nr = cr - qr;
    const double d = std::hypot(nf, nr);
    if (d < 1e-9) {   // centre inside the box: leave by the shallowest side
        const double toF = f1 - cf, toB = cf - f0, toR = hw - cr, toL = cr + hw;
        const double mn = std::min({toF, toB, toR, toL});
        nf = mn == toF ? 1 : mn == toB ? -1 : 0;
        nr = nf != 0 ? 0 : mn == toR ? 1 : -1;
        return r + mn;
    }
    nf /= d; nr /= d;
    return r - d;
}
// A circle against one box of the robot: fore-aft f0..f1, sideways rc - hw .. rc + hw (robot frame, frame centre).
void boxContact(Wrench& w, double cx, double cy, double r, double f0, double f1, double rc, double hw, double slack = 0.0) {
    double cf, cr, nf, nr;
    worldToRobot(cx, cy, cf, cr);
    const double pen = boxPen(cf, cr - rc, r, f0, f1, hw, nf, nr) - slack;
    if (pen <= 0) return;
    double px, py; robotToWorld(std::clamp(cf, f0, f1), std::clamp(cr, rc - hw, rc + hw), px, py);
    // force on the robot is opposite to the direction that frees the circle
    const double nx = -(std::sin(body.th) * nf + std::cos(body.th) * nr);
    const double ny = -(std::cos(body.th) * nf - std::sin(body.th) * nr);
    applyPoint(w, px, py, nx, ny, pen);
}
void circleContact(Wrench& w, double cx, double cy, double r, double slack = 0.0) {
    boxContact(w, cx, cy, r, -truth.rearIn, kFrontIn, 0.0, kHalfWidthIn, slack);
}
// A Goal base meets the frame behind the notch and the two drive rails beside it, not a solid front.
void goalContact(Wrench& w, double gx, double gy) {
    const double railHw = 0.5 * (kHalfWidthIn - kNotchHalfWIn), railC = kNotchHalfWIn + railHw;
    boxContact(w, gx, gy, kGoalR, -truth.rearIn, kNotchFlatIn, 0.0, kHalfWidthIn);
    boxContact(w, gx, gy, kGoalR, kNotchFlatIn, kFrontIn, railC, railHw);
    boxContact(w, gx, gy, kGoalR, kNotchFlatIn, kFrontIn, -railC, railHw);
}
// Rear-most part: the frame, or the raised lift's middle link swinging out behind it (final CAD). For a wall the
// link only counts while it is still below the wall top (stage 1 short of level).
double liftRearIn(bool forWall) {
    const double dir = d2r(truth.rest1Deg) - lift.a;
    if (forWall && kTowerPivotUpIn + kL1In * std::sin(dir) - 0.5 >= kWallHeightIn) return truth.rearIn;
    return std::max(truth.rearIn, -(kTowerPivotFwdIn + kL1In * std::cos(dir)) + kMidLinkBehindIn);
}
double clawReach();
// The near edge of the Toggle prism, as a y coordinate (0.06 in beyond the red wall's inner face)
const double kToggleEdgeY = kHalf + kToggleAxisBeyondIn - kToggleCircumRIn;
Wrench contacts() {
    Wrench w;
    if (openFloor) return w;
    const auto walls = [&w](double px, double py) {
        const double f0 = w.fx * w.fx + w.fy * w.fy;
        if (py > kHalf) applyPoint(w, px, py, 0, -1, py - kHalf);
        if (py < -kHalf) applyPoint(w, px, py, 0, 1, -kHalf - py);
        if (px > kHalf) applyPoint(w, px, py, -1, 0, px - kHalf);
        if (px < -kHalf) applyPoint(w, px, py, 1, 0, -kHalf - px);
        return w.fx * w.fx + w.fy * w.fy != f0;
    };
    const double lr = liftRearIn(true);
    const double cf[6] = {kFrontIn, kFrontIn, -truth.rearIn, -truth.rearIn, -lr, -lr};
    const double cr[6] = {kHalfWidthIn, -kHalfWidthIn, kHalfWidthIn, -kHalfWidthIn, kLiftHalfWIn, -kLiftHalfWIn};
    for (int i = 0; i < (lr > truth.rearIn ? 6 : 4); ++i) {
        double px, py; robotToWorld(cf[i], cr[i], px, py);
        walls(px, py);
    }
    // Final CAD: the chain on the 18T and the tower plates stand ahead of the front rails at Toggle height (right side
    // only) and meet the Toggle's near edge before the rails meet the wall, so inside the Toggle's span the rails stop
    // about 0.78 in short and the robot is pushed on its right side.
    if (std::cos(body.th) > 0.5) {
        const double dz = kToggleAxisUpIn - kPaddleAxisUpIn;
        bool pressed = false;
        for (const auto& b : kToggleBumpers) {
            double px, py; robotToWorld(kPaddleAxisFwdIn + std::sqrt(b[0] * b[0] - dz * dz), b[1], px, py);
            if (std::fabs(px) > 0.5 * kToggleLenIn || py <= kToggleEdgeY) continue;
            applyPoint(w, px, py, 0, -1, py - kToggleEdgeY);
            pressed = true;
        }
        if (pressed && !counters.toggleBumpMs++) {
            event(fmt("\"type\": \"toggle_bumped\", \"rail_gap_in\": %.3f", railGapIn()));
            logLine(fmt("[toggle] the chain / tower plates ran into the Toggle with the front rails %.2f in from the wall", railGapIn()));
        }
    }
    // The claw (and an object it carries) cannot go through a wall it is not above, nor through the Toggle on the red
    // wall. Claw front: 1 in ahead of the seat, or the far side of a carried object (at pickup height the object can
    // still sit shallower in the rollers, so only the claw body counts there). Underside 0.25 in below the object.
    // ponytail: the Toggle is a vertical face at its near edge up to its turning height, not a prism.
    if (opt.clawWall) {
        const double rise = liftRiseIn(r2d(lift.a));
        const double low = (heldObj >= 0 ? std::min(rise, heldBottom()) : rise) - 0.25;
        const double front = clawReach() + (heldObj >= 0 && rise > kGrabMaxRiseIn ? kCupR : 1.0);
        bool pressed = false;
        for (double side : {-kClawHalfWIn, kClawHalfWIn}) {
            double px, py; robotToWorld(front, side, px, py);
            if (low < kWallHeightIn) pressed = walls(px, py) || pressed;
            else if (low < kToggleTopTurningIn && std::fabs(px) <= 0.5 * kToggleLenIn && py > kToggleEdgeY) {
                applyPoint(w, px, py, 0, -1, py - kToggleEdgeY);
                pressed = true;
            }
        }
        if (pressed && !counters.clawWallMs++) {
            event(fmt("\"type\": \"claw_hit_wall\", \"claw_underside_in\": %.2f, \"lift_deg\": %.1f, \"true\": [%.2f, %.2f, %.1f], \"holding\": %s",
                      low, r2d(lift.a), body.x, body.y, r2d(body.th), heldObj >= 0 ? "true" : "false"));
            logLine(fmt("[claw] the %s met %s with its underside %.2f in up (wall top %.2f, Toggle top %.2f), lift %.1f deg, robot at (%.2f, %.2f, %.1f)",
                        heldObj >= 0 ? "claw / held object" : "claw", low < kWallHeightIn ? "a wall" : "the Toggle", low, kWallHeightIn,
                        kToggleTopTurningIn, r2d(lift.a), body.x, body.y, r2d(body.th)));
        }
    }
    for (const auto& g : goals) goalContact(w, g.x, g.y);
    for (const auto& b : loaderBoxes) {   // chute as a circle touching its face
        const double cx = 0.5 * (b.x0 + b.x1);
        const double cy = b.y0 > 0 ? b.y0 + kLoaderHalfW : b.y1 - kLoaderHalfW;
        circleContact(w, cx, cy, kLoaderHalfW);
    }
    for (const auto& l : loose) circleContact(w, l.x, l.y, l.r, 0.05);   // only bites when the object is jammed
    return w;
}
double clawReach() { return kClawReachIn + liftDxIn(r2d(lift.a)) + sway; }

// Loose discs are moved kinematically, so one must never be put down inside the frame box: the penalty contact would
// see inches of penetration appear in one step and throw the robot across the field. The pre-CAD sim did that in a few
// runs (claw box shoving a disc back into the frame, a dropped object pushed out of a Goal into the frame).
double framePen(double x, double y, double r) {
    double cf, cr, nf, nr;
    worldToRobot(x, y, cf, cr);
    return boxPen(cf, cr, r, -truth.rearIn, kFrontIn, kHalfWidthIn, nf, nr);
}
// A disc squeezed between a Goal base and the frame pops out sideways: round the base to the nearest spot clear of the frame.
void slideRoundGoal(double gx, double gy, double need, double r, double& x, double& y) {
    if (framePen(x, y, r) <= 0) return;
    const double a0 = std::atan2(y - gy, x - gx);
    for (int k = 1; k <= 90; ++k)
        for (double sgn : {1.0, -1.0}) {
            const double px = gx + need * std::cos(a0 + sgn * d2r(2.0 * k)), py = gy + need * std::sin(a0 + sgn * d2r(2.0 * k));
            if (framePen(px, py, r) <= 0) { x = px; y = py; return; }
        }
}
void looseStep() {
    const double rise = liftRiseIn(r2d(lift.a));
    for (auto& l : loose) {
        double cf, cr, nf, nr;
        worldToRobot(l.x, l.y, cf, cr);
        double moved = 0;
        // frame, then the claw body ahead of it while its underside is below the object's top. The claw box runs back
        // through the frame so it can only shove a disc forward or sideways, never back into the frame.
        for (int pass = 0; pass < 3; ++pass) {
            double m = 0;
            for (int k = 0; k < 2; ++k) {
                if (k == 1 && rise - 0.25 >= l.h) break;
                const double pen = k == 0 ? boxPen(cf, cr, l.r, -truth.rearIn, kFrontIn, kHalfWidthIn, nf, nr)
                                          : boxPen(cf, cr, l.r, -truth.rearIn, clawReach() + 1.0, kClawHalfWIn, nf, nr);
                if (pen <= 0) continue;
                cf += nf * pen; cr += nr * pen; m += pen;
            }
            moved += m;
            if (m <= 0) break;
        }
        if (moved <= 0) continue;
        robotToWorld(cf, cr, l.x, l.y);
        l.pushed += moved;
        // statics stop it
        l.x = std::clamp(l.x, -kHalf + l.r, kHalf - l.r);
        l.y = std::clamp(l.y, -kHalf + l.r, kHalf - l.r);
        for (const auto& g : goals) {
            const double d = std::hypot(l.x - g.x, l.y - g.y), need = kGoalR + l.r;
            if (d < need && d > 1e-9) {
                l.x = g.x + (l.x - g.x) / d * need; l.y = g.y + (l.y - g.y) / d * need;
                slideRoundGoal(g.x, g.y, need, l.r, l.x, l.y);
            }
        }
        if (!l.reported) {
            l.reported = true; ++counters.loosePushed;
            event(fmt("\"type\": \"loose_pushed\", \"object\": \"%s\", \"from\": [%.2f, %.2f]", l.kind.c_str(), l.x0, l.y0));
            logLine(fmt("[plow] robot hit the %s that started at (%.2f, %.2f)", l.kind.c_str(), l.x0, l.y0));
        }
    }
}
}  // namespace

std::string looseSummary() {
    std::string s = "[";
    bool first = true;
    for (const auto& l : loose)
        if (l.pushed > 0) {
            s += fmt("%s{\"object\": \"%s\", \"from\": [%.2f, %.2f], \"to\": [%.2f, %.2f], \"pushed_in\": %.1f}", first ? "" : ", ",
                     l.kind.c_str(), l.x0, l.y0, l.x, l.y, l.pushed);
            first = false;
        }
    return s + "]";
}

// ================================================================ Toggle spinner and the red wall Toggle
// Side view at the paddle (all in): s = distance beyond the red wall's inner face, z = up.
namespace {
bool inWall(double s, double z) { return s >= 0 && z <= kWallHeightIn; }
bool inPrism(double s, double z) {   // the Toggle at rest: one edge toward the field, faces 30 deg off level
    const double s0 = s - (kToggleAxisBeyondIn - kToggleCircumRIn);
    return s0 >= 0 && s <= kToggleAxisBeyondIn + 0.5 * kToggleCircumRIn && std::fabs(z - kToggleAxisUpIn) <= s0 * 0.57735;
}
struct PaddleView { double d, c; bool wall, prism; };   // paddle axis to the wall face, cos(heading off the wall), near the wall, inside the Toggle's span
PaddleView paddleView() {
    double px, py; robotToWorld(kPaddleAxisFwdIn, 0.5 * (kPaddleRightLoIn + kPaddleRightHiIn), px, py);
    const double c = std::cos(body.th);
    return {kHalf - py, c, !openFloor && c > 0.5 && kHalf - py < 3.0, std::fabs(px) <= 0.5 * kToggleLenIn};
}
// What end e (0 long, 1 short) of the paddle overlaps at paddle angle a: bit 1 the wall, bit 2 the Toggle
int endHits(const PaddleView& v, double a, int e) {
    if (!v.wall) return 0;
    const double R = e ? kPaddleShortIn : kPaddleLongIn, dir = a + (e ? PI : 0.0);
    int hit = 0;
    for (double rho = R; rho > 0.3; rho -= 0.05) {
        const double s = rho * std::cos(dir) * v.c - v.d, z = kPaddleAxisUpIn + rho * std::sin(dir);
        if (inWall(s, z)) hit |= 1;
        if (v.prism && inPrism(s, z)) hit |= 2;
    }
    return hit;
}
double gearLoss(double tau, double w) { return tau * (tau * w >= 0 ? kGearEff : 1.0 / kGearEff); }
// ASSUMED, not from any source: every paddle end that goes all the way through the Toggle while the paddle is
// turning moves it exactly one state (yellow -> blue -> red -> yellow), whatever the overlap. A paddle that the
// ROBOT pushes into the Toggle or the wall (not turning: its motors coast) is turned out of the way and moves nothing.
// An end that stops inside the Toggle and is then taken away leaves the Toggle in the state it had.
void paddleStep(double dt) {
    double Tm = 0, fr = 0, Jp = kPaddleJ;
    for (int p = 0; p < 32; ++p) {   // the motor(s) chained to the paddle: the right one only, as in the CAD
        MotorDev& m = motor(p);
        if (!m.exists || m.mech != 5) continue;
        const double wm = m.mount * paddle.w * kPaddleRatio;
        Tm += m.mount * gearLoss(motorTorque(m, wm), wm) * kPaddleRatio;
        fr += motorFric(m) * kPaddleRatio;
        Jp += m.rotorJ * kPaddleRatio * kPaddleRatio;
    }
    if (paddle.in[0] || paddle.in[1]) fr += kToggleResistNm;
    paddle.w = towardZero(paddle.w + Tm / Jp * dt, fr / Jp * dt);
    const PaddleView v = paddleView();
    const bool turning = paddle.w != 0;
    double a1 = paddle.a + paddle.w * dt;
    int h[2] = {endHits(v, a1, 0), endHits(v, a1, 1)};
    if (turning && ((h[0] | h[1]) & 1)) {   // an end would go into the wall: it stops on the wall's top edge, motors stalled
        a1 = paddle.a; paddle.w = 0;
        h[0] = endHits(v, a1, 0); h[1] = endHits(v, a1, 1);
        if (!counters.toggleJamMs++ || !paddle.jamLogged) {
            paddle.jamLogged = true;
            event(fmt("\"type\": \"paddle_blocked\", \"rail_gap_in\": %.3f, \"paddle_deg\": %.1f", railGapIn(), paddleDeg()));
            logLine(fmt("[toggle] paddle BLOCKED by the top edge of the wall at %.0f deg, front rails %.2f in from the wall (needs 0.80)", paddleDeg(), railGapIn()));
        }
    }
    paddle.a = a1;
    if (turning) {
        for (int e = 0; e < 2; ++e) {
            const bool now = h[e] & 2;
            if (now && !paddle.in[e]) { paddle.in[e] = true; paddle.aIn[e] = paddle.a; paddle.dirIn[e] = paddle.w > 0 ? 1 : -1; }
            else if (!now && paddle.in[e]) {
                paddle.in[e] = false;
                if ((paddle.a - paddle.aIn[e]) * paddle.dirIn[e] < d2r(1.0)) continue;   // backed out the way it came in: no strike
                const char* col[3] = {"yellow", "blue", "RED"};
                const int before = toggleState();
                ++counters.toggleSweeps;
                event(fmt("\"type\": \"toggle_strike\", \"n\": %d, \"end\": \"%s\", \"state\": \"%s\", \"rail_gap_in\": %.3f", counters.toggleSweeps,
                          e ? "short" : "long", col[toggleState()], railGapIn()));
                logLine(fmt("[toggle] strike %d, %s end of the paddle went through the Toggle: %s -> %s", counters.toggleSweeps, e ? "short" : "long",
                            col[before], col[toggleState()]));
            }
        }
        return;
    }
    // not turning: whatever now overlaps got there because the robot moved
    bool push = (h[0] | h[1]) & 1;
    for (int e = 0; e < 2; ++e) {
        if (paddle.in[e] && !(h[e] & 2)) {   // the robot took a stopped paddle out of a part-turned Toggle
            paddle.in[e] = false;
            ++counters.togglePartTurned;
            logLine("[toggle] the paddle had stopped inside the Toggle and the robot has now left: the Toggle was left part-turned (state kept, ASSUMED)");
        }
        push = push || ((h[e] & 2) && !paddle.in[e]);
    }
    if (!push) return;
    for (int k = 1; k <= 360; ++k)
        for (double sgn : {1.0, -1.0}) {
            const double a = paddle.a + sgn * d2r(0.5 * k);
            if (endHits(v, a, 0) || endHits(v, a, 1)) continue;
            if (!counters.togglePushedPaddle++)
                logLine(fmt("[toggle] the robot drove the idle paddle into the %s; it was turned %.1f deg out of the way (motors coast)",
                            (h[0] | h[1]) & 1 ? "wall" : "Toggle", sgn * 0.5 * k));
            paddle.a = a; paddle.in[0] = paddle.in[1] = false;
            return;
        }
}
}  // namespace
static void noteFlipStart() {
    double lx, ly, rx, ry;
    robotToWorld(kFrontIn, -kHalfWidthIn, lx, ly);
    robotToWorld(kFrontIn, kHalfWidthIn, rx, ry);
    const double gapPaddle = paddleView().d - (kFrontIn - kPaddleAxisFwdIn);
    const double under = (heldObj >= 0 ? std::min(liftRiseIn(r2d(lift.a)), heldBottom()) : liftRiseIn(r2d(lift.a))) - 0.25;
    if (!counters.toggleFlips++) {
        counters.flipGapIn = railGapIn(); counters.flipGapLeftIn = kHalf - ly; counters.flipGapRightIn = kHalf - ry;
        counters.flipGapPaddleIn = gapPaddle; counters.flipHeadDeg = std::remainder(r2d(body.th), 360.0);
        counters.flipClawUnderIn = under; counters.flipPaddleDeg = paddleDeg();
    }
    event(fmt("\"type\": \"toggle_flip_start\", \"true\": [%.3f, %.3f, %.2f], \"rail_gap_in\": %.3f, \"rail_gap_left_in\": %.3f, \"rail_gap_right_in\": %.3f, "
              "\"gap_at_paddle_in\": %.3f, \"paddle_deg\": %.1f, \"lift_deg\": %.1f, \"claw_underside_in\": %.2f, \"speed_in_s\": %.1f",
              body.x, body.y, r2d(body.th), railGapIn(), kHalf - ly, kHalf - ry, gapPaddle, paddleDeg(), r2d(lift.a), under, std::hypot(body.v, body.vl) / IN));
    logLine(fmt("[toggle] flip() starts: true (%.2f, %.2f, %.1f), front rails %.2f in from the red wall (left end %.2f, right end %.2f, at the paddle %.2f; "
                "the paddle works from 0.80 to 1.40), paddle at %.0f deg, lift %.1f deg, claw underside %.2f in up (Toggle top %.2f), still moving %.1f in/s",
                body.x, body.y, r2d(body.th), railGapIn(), kHalf - ly, kHalf - ry, gapPaddle, paddleDeg(), r2d(lift.a), under, kToggleTopTurningIn,
                std::hypot(body.v, body.vl) / IN));
}
void toggleReset(double deg) {
    paddle = Paddle{};
    paddle.a = d2r(deg);
    counters.toggleSweeps = counters.toggleFlips = counters.toggleJamMs = counters.toggleStoppedInside = counters.togglePartTurned = 0;
    counters.togglePushedPaddle = counters.toggleBumpMs = 0;
}
int toggleState() { return counters.toggleSweeps % 3; }
double paddleDeg() { const double a = std::fmod(r2d(paddle.a), 360.0); return a < 0 ? a + 360.0 : a; }
double paddleRpm() { return paddle.w * 60.0 / (2 * PI); }
double railGapIn() { double x, y; robotToWorld(kFrontIn, 0.0, x, y); return kHalf - y; }

// ================================================================ game objects
static void clawPoint(double& x, double& y) { robotToWorld(clawReach(), 0.0, x, y); }
// Top of what is on a Goal: the base, or the top object of the stack (also what the beams see).
static double stackTop(const Goal& g) {
    return g.objects == 0 ? goalTop(g.type) : goalTop(g.type) - kNest + (g.objects - 1) * truth.pitch + kObjLen;
}
static double heldBottom() { return liftRiseIn(r2d(lift.a)) + objs[heldObj].dz; }
// A dropped or knocked-over object stays on the tiles as a disc that the frame, the claw and the
// beams meet (same simplification as the cluster objects: one 1.58 in disc, 3.16 in high).
static void addDebris(const Obj& o, const char* how) {
    double x = o.x, y = o.y;
    {   // it cannot lie inside the robot frame (see framePen)
        double cf, cr, nf, nr;
        worldToRobot(x, y, cf, cr);
        const double pen = boxPen(cf, cr, kCupR, -truth.rearIn, kFrontIn, kHalfWidthIn, nf, nr);
        if (pen > 0) robotToWorld(cf + nf * pen, cr + nr * pen, x, y);
    }
    for (const auto& g : goals) {   // nor inside a Goal base
        const double d = std::hypot(x - g.x, y - g.y), need = kGoalR + kCupR;
        if (d >= need) continue;
        const double ux = d > 1e-6 ? (x - g.x) / d : -std::sin(body.th), uy = d > 1e-6 ? (y - g.y) / d : -std::cos(body.th);
        x = g.x + ux * need; y = g.y + uy * need;
        slideRoundGoal(g.x, g.y, need, kCupR, x, y);
    }
    loose.push_back({o.name + " (" + how + ")", x, y, kCupR, kFlatPinH, x, y});
    ++counters.dropped;
    logLine(fmt("[debris] %s now lies at (%.2f, %.2f)", loose.back().kind.c_str(), x, y));
}
static void releaseHeld() {
    Obj& o = objs[heldObj];
    o.x += std::sin(body.th) * truth.releaseFwdIn;
    o.y += std::cos(body.th) * truth.releaseFwdIn;
    const double b = heldBottom();
    int gi = -1; double best = 1e9;
    for (size_t i = 0; i < goals.size(); ++i) {
        const double d = std::hypot(o.x - goals[i].x, o.y - goals[i].y);
        if (d < best) { best = d; gi = (int)i; }
    }
    Goal& g = goals[gi];
    const double H = stackTop(g);
    const char* onGoal = g.objects == 0 ? "Goal base" : "stack";
    std::string why;
    bool ok = false;
    if (heldHit == gi) why = fmt("ran into the %s from the side on the way in (object bottom below its top, %.2f in)", onGoal, H);
    else if (best > truth.captureR) why = fmt("missed the post by %.2f in (capture radius %.2f)", best - truth.captureR, truth.captureR);
    else if (b < H - kPlaceBelowTol) why = fmt("too low, object bottom %.2f in is below the top of the %s, %.2f in", b, onGoal, H);
    else if (b > H + kPlaceAboveMax) why = fmt("too high, object bottom %.2f in more than %.1f above the top of the %s, %.2f in", b, kPlaceAboveMax, onGoal, H);
    else { ok = true; why = "seated"; }
    placements = counters.releases;
    const int before = g.objects;
    if (ok) { g.objects += o.units; g.pins += 1; o.state = kOnGoal; }
    else { o.state = kLost; addDebris(o, "dropped"); }
    heldNear = heldHit = -1;
    // where the object was relative to the Goal, along/across the heading
    const double ex = o.x - g.x, ey = o.y - g.y;
    const double along = ex * std::sin(body.th) + ey * std::cos(body.th);
    const double across = ex * std::cos(body.th) - ey * std::sin(body.th);
    event(fmt("\"type\": \"placement\", \"n\": %d, \"object\": \"%s\", \"goal\": \"%s\", \"ok\": %s, \"why\": \"%s\", "
              "\"radial_err_in\": %.3f, \"along_err_in\": %.3f, \"across_err_in\": %.3f, \"bottom_in\": %.3f, \"stack_top_in\": %.3f, "
              "\"objects_before\": %d, \"lift_deg\": %.2f, \"x\": %.2f, \"y\": %.2f",
              placements, o.name.c_str(), g.name, ok ? "true" : "false", why.c_str(), best, along, across, b, H, before,
              r2d(lift.a), o.x, o.y));
    logLine(fmt("[place] #%d %s -> %s: %s (radial %.2f in, along %+.2f across %+.2f, bottom %.2f vs top %.2f, lift %.1f deg)",
                placements, o.name.c_str(), g.name, ok ? "OK" : ("FAIL, " + why).c_str(), best, along, across, b, H, r2d(lift.a)));
    placeRecs.push_back({placements, o.name, g.name, ok, why, best, along, across, b, H, r2d(lift.a), now() / 1000.0});
    heldObj = -1;
}

static void objectsStep(uint64_t t) {
    MotorDev& claw = motor(wiring.claw);
    // what the robot commanded (not the HOLD loop's own correction voltage)
    const bool intaking = claw.exists && claw.kind == kVolt && claw.val * claw.mount > 4000;
    const bool outtaking = claw.exists && claw.kind == kVolt && claw.val * claw.mount < -4000;
    const double rise = liftRiseIn(r2d(lift.a));
    double cx, cy; clawPoint(cx, cy);
    const double fx = std::sin(body.th), fy = std::cos(body.th), rx = std::cos(body.th), ry = -std::sin(body.th);

    // Loader refills
    for (size_t L = 0; L < loaders.size(); ++L) {
        Loader& ld = loaders[L];
        if (ld.presented < 0 && ld.next < ld.queue.size() && t >= ld.presentAt) {
            // the human does not put a load down while the robot (frame or claw) is over the spot
            const Obj& nx = objs[ld.queue[ld.next]];
            double lf, lr, nf, nr;
            worldToRobot(nx.x, nx.y, lf, lr);
            if (boxPen(lf, lr, kCupR, -truth.rearIn, clawReach() + 1.0, kHalfWidthIn, nf, nr) > 0) continue;
            ld.presented = ld.queue[ld.next++];
            objs[ld.presented].state = kStanding;
            logLine(fmt("[loader] %s presented at (%.2f, %.2f)", objs[ld.presented].name.c_str(), objs[ld.presented].x, objs[ld.presented].y));
        }
    }
    // intake sessions (for grab-miss diagnostics)
    if (intaking && !intakeSession) { intakeSession = true; sessionGrabbed = false; sessionStart = t; }
    if (!intaking && intakeSession) {
        intakeSession = false;
        // Rollers stopping (HOLD) with an object already between them pinch it, even if it is not
        // drawn all the way back to the seat yet.
        if (heldObj < 0 && !outtaking && rise <= kGrabMaxRiseIn)
            for (size_t i = 0; i < objs.size(); ++i) {
                Obj& o = objs[i];
                if (o.state != kStanding) continue;
                const double along = (o.x - cx) * fx + (o.y - cy) * fy;
                const double lat = (o.x - cx) * rx + (o.y - cy) * ry;
                if (std::fabs(lat) > truth.grabLatIn || along > truth.grabAheadIn || along < -kGrabBehindIn) continue;
                o.state = kHeld; heldObj = (int)i; sessionGrabbed = true;
                o.dz = -rise; o.latOff = kGrabKeepFrac * lat;
                for (auto& ld : loaders)
                    if (ld.presented == (int)i) { ld.presented = -1; ld.presentAt = t + kRefillMs; }
                ++counters.grabs;
                event(fmt("\"type\": \"grab\", \"object\": \"%s\", \"lat_in\": %.3f, \"pushed_in\": %.3f, \"short_of_seat_in\": %.2f", o.name.c_str(), lat, o.pushed, along));
                logLine(fmt("[grab] %s pinched by the stopped rollers %.2f in short of the seat (lat %+.2f in, pushed %.2f in first)", o.name.c_str(), along, lat, o.pushed));
                break;
            }
        if (!sessionGrabbed && heldObj < 0) {
            int bi = -1; double bd = 1e9;
            for (size_t i = 0; i < objs.size(); ++i)
                if (objs[i].state == kStanding || objs[i].state == kTipped) {
                    const double d = std::hypot(objs[i].x - cx, objs[i].y - cy);
                    if (d < bd) { bd = d; bi = (int)i; }
                }
            std::string desc = "nothing near";
            if (bi >= 0) {
                const double along = (objs[bi].x - cx) * fx + (objs[bi].y - cy) * fy;
                const double lat = (objs[bi].x - cx) * rx + (objs[bi].y - cy) * ry;
                desc = fmt("nearest %s%s: along %+.2f lat %+.2f in, claw rise %.2f in", objs[bi].name.c_str(),
                           objs[bi].state == kTipped ? " (TIPPED)" : "", along, lat, rise);
            }
            ++counters.grabMiss;
            event(fmt("\"type\": \"grab_miss\", \"intake_ms\": %llu, \"detail\": \"%s\"", (unsigned long long)(t - sessionStart), desc.c_str()));
            logLine("[grab] MISS after " + std::to_string(t - sessionStart) + " ms of intake, " + desc);
        }
    }
    auto tip = [&](Obj& o, const char* by) {
        o.state = kTipped; ++counters.tipped;
        bool atLoader = false;   // the human takes a knocked-over load away; anywhere else it stays on the tiles
        for (const auto& ld : loaders) atLoader = atLoader || ld.presented == (int)(&o - objs.data());
        if (!atLoader) addDebris(o, "tipped");
        for (auto& ld : loaders)   // a knocked-over load is in the way; the human replaces it
            if (ld.presented == (int)(&o - objs.data())) { ld.presented = -1; ld.presentAt = t + kRefillMs; }
        event(fmt("\"type\": \"tipped\", \"object\": \"%s\", \"by\": \"%s\", \"pushed_in\": %.2f", o.name.c_str(), by, o.pushed));
        logLine(fmt("[grab] %s TIPPED by %s after %.2f in", o.name.c_str(), by, o.pushed));
    };

    if (heldObj >= 0) {
        Obj& h = objs[heldObj];
        h.x = cx + rx * h.latOff; h.y = cy + ry * h.latOff;
        // Arriving over a Goal from the side with the object's bottom below the top of what is there:
        // it hits the base or the stack instead of going over it.
        // ponytail: the hit only fails the release that follows; the robot is not held back and the
        // stack is not knocked over. Add a contact force and stack damage if stacking ever gets that far.
        int nearG = -1;
        for (size_t i = 0; i < goals.size(); ++i)
            if (std::hypot(h.x - goals[i].x, h.y - goals[i].y) < kCupR + (goals[i].objects ? kCupR : kGoalR)) nearG = (int)i;
        if (nearG >= 0 && nearG != heldNear && heldBottom() < stackTop(goals[nearG]) - kPlaceBelowTol) {
            const Goal& g = goals[nearG];
            heldHit = nearG; ++counters.stackHits;
            event(fmt("\"type\": \"stack_hit\", \"object\": \"%s\", \"goal\": \"%s\", \"bottom_in\": %.2f, \"top_in\": %.2f, \"pins_on_goal\": %d",
                      h.name.c_str(), g.name, heldBottom(), stackTop(g), g.pins));
            logLine(fmt("[place] %s reaches %s with its bottom at %.2f in, below the top of the %s at %.2f in: it hits it from the side",
                        h.name.c_str(), g.name, heldBottom(), g.objects ? "stack" : "Goal base", stackTop(g)));
            if (g.pins > 0) {
                ++counters.scoredStackHits;
                logLine(fmt("[place] WARNING a stack with %d scored Pin(s) was hit (knocking Pins off is not modelled, points are kept)", g.pins));
            }
        }
        heldNear = nearG;
        if (outtaking) { if (++outMs >= truth.releaseMs) releaseHeld(); }
        else outMs = 0;
    } else outMs = 0;
    for (size_t i = 0; i < objs.size(); ++i) {
        Obj& o = objs[i];
        if (o.state != kStanding) continue;
        // the frame itself running into a standing object knocks it over
        double of, orr, nf, nr;
        worldToRobot(o.x, o.y, of, orr);
        if (boxPen(of, orr, kCupR, -truth.rearIn, kFrontIn, kHalfWidthIn, nf, nr) > 0.3) { tip(o, "the robot frame"); continue; }
        const double along = (o.x - cx) * fx + (o.y - cy) * fy;
        const double lat = (o.x - cx) * rx + (o.y - cy) * ry;
        if (std::fabs(lat) > truth.grabLatIn) {
            // beside the throat: the claw body (side plates, rollers, a held object) shoves it, and a long shove tips it
            const double pen = rise - 0.25 < (o.units == 2 ? kCupPinH : kObjLen)
                                   ? boxPen(of, orr, kCupR, -truth.rearIn, clawReach() + 1.0, kClawHalfWIn, nf, nr) : 0.0;   // forward or sideways only
            if (pen > 0) {
                robotToWorld(of + nf * pen, orr + nr * pen, o.x, o.y);
                o.pushed += pen;
                if (o.pushed > kTipPushIn) tip(o, "the claw body");
            }
            continue;
        }
        const bool canGrip = heldObj < 0 && rise <= kGrabMaxRiseIn;
        if (canGrip && intaking && along <= truth.grabAheadIn && along >= -kGrabBehindIn) {
            const double step = kPullInPerS * std::min(1.0, std::fabs(claw.val) / 12000.0) * 0.001;
            if (along <= step) {
                o.state = kHeld; heldObj = (int)i; sessionGrabbed = true;
                o.dz = -rise; o.latOff = kGrabKeepFrac * lat;
                for (auto& ld : loaders)
                    if (ld.presented == (int)i) { ld.presented = -1; ld.presentAt = t + kRefillMs; }
                ++counters.grabs;
                event(fmt("\"type\": \"grab\", \"object\": \"%s\", \"lat_in\": %.3f, \"pushed_in\": %.3f", o.name.c_str(), lat, o.pushed));
                logLine(fmt("[grab] %s seated (lat %+.2f in, pushed %.2f in first)", o.name.c_str(), lat, o.pushed));
                continue;
            }
            o.x -= fx * step; o.y -= fy * step;   // rollers pull it in
        } else if (along < -kGrabBehindIn && along > -kGrabBehindIn - 1.5 && rise - 0.25 < kCupPinH) {
            // claw (or the object it holds) drove into it without the rollers taking it
            const double push = -kGrabBehindIn - along;
            o.x += fx * push; o.y += fy * push;
            o.pushed += push;
            if (o.pushed > kTipPushIn) tip(o, heldObj >= 0 ? "the held object" : rise > kGrabMaxRiseIn ? "the raised claw" : "the claw");
        }
    }
}

// ================================================================ physics step
void worldStep(uint64_t t) {
    const double dt = 0.001;
    for (int p = 0; p < 32; ++p) {
        MotorDev& m = motor(p);
        if (!m.exists) continue;
        while (!m.q.empty() && m.q.front().t + kCmdLatencyMs <= t) {
            m.kind = m.q.front().kind; m.val = m.q.front().val; m.lim = m.q.front().lim;
            m.q.pop_front();
        }
        if (disabled) { m.q.clear(); m.kind = kVolt; m.val = 0; }
    }
    // ---- drive: two sides, each geared to its motors; body v (forward), vl (rightward), w (clockwise)
    const double W = kTrackIn * IN, r = 0.5 * kWheelDiaIn * IN, Gr = kDriveRatio, hw = W / 2;
    const double reach = clawReach() * IN;
    const double objKg = heldObj >= 0 ? kObjectKg : 0.0, v0 = body.v;
    const double M = truth.massKg, J = truth.yawJ + (truth.carriageKg + objKg) * reach * reach;
    // gear loss: the motor driving the mechanism loses torque, the mechanism driving the motor meets more
    const auto geared = gearLoss;
    const double vL = body.v + body.w * hw, vR = body.v - body.w * hw;
    double FL = 0, FR = 0, frL = 0, frR = 0;
    for (int p = 0; p < 32; ++p) {
        MotorDev& m = motor(p);
        if (!m.exists || (m.mech != 1 && m.mech != 2)) continue;
        const double vs = m.mech == 1 ? vL : vR;
        const double wm = m.mount * vs / r * Gr;
        const double F = m.mount * geared(motorTorque(m, wm), wm) * Gr / r;
        (m.mech == 1 ? FL : FR) += F;
        (m.mech == 1 ? frL : frR) += motorFric(m) * Gr / r;
    }
    const double Ftr = truth.mu * M * G / 2;   // traction limit per side (wheel spin itself is not modelled)
    FL = std::clamp(FL, -Ftr, Ftr);
    FR = std::clamp(FR, -Ftr, Ftr);
    const Wrench c = contacts();
    const double ff = c.fx * std::sin(body.th) + c.fy * std::cos(body.th);
    const double fl = c.fx * std::cos(body.th) - c.fy * std::sin(body.th);
    double v1 = body.v + ((FL + FR + ff) / M + body.vl * body.w) * dt;
    double w1 = body.w + ((FL - FR) * hw + c.tcw) / J * dt;
    {   // Coulomb friction of each side (motor + rolling + constant drivetrain drag), exact stick/slip for the coupled pair
        const double fricL = frL + truth.rollResist * M * G / 2 + truth.driveFricN, fricR = frR + truth.rollResist * M * G / 2 + truth.driveFricN;
        const double self = 1 / M + hw * hw / J, cross = 1 / M - hw * hw / J, det = self * self - cross * cross;
        const double vL1 = v1 + w1 * hw, vR1 = v1 - w1 * hw;
        double fL = -(self * vL1 - cross * vR1) / (det * dt), fR = -(self * vR1 - cross * vL1) / (det * dt);
        if (std::fabs(fL) > fricL) {
            fL = std::copysign(fricL, fL);
            fR = std::clamp(-(vR1 / dt + cross * fL) / self, -fricR, fricR);
        } else if (std::fabs(fR) > fricR) {
            fR = std::copysign(fricR, fR);
            fL = std::clamp(-(vL1 / dt + cross * fR) / self, -fricL, fricL);
        }
        v1 += (fL + fR) / M * dt;
        w1 += (fL - fR) * hw / J * dt;
    }
    w1 = towardZero(w1, truth.mu * M * G * truth.scrubLeverIn * IN / J * dt);   // omnis rolling sideways, tyre twist
    // sideways: the two centre traction wheels hold the robot AT THE TRACTION AXLE, truth.pivotFwdIn ahead of the
    // frame centre (final CAD 1.5 in), so that is the point the robot turns about; the omni rollers add a little drag.
    // The friction impulse j acts at the axle: it stops the sideways speed there (vl + w * pivot) up to the traction
    // limit. With pivot 0 this is the pre-CAD sim: towardZero(u1, muLat * G * dt), turning about the frame centre.
    double u1 = body.vl + (fl / M - body.v * body.w) * dt;
    {
        const double pv = (truth.pivotFwdIn - truth.cgFwdIn) * IN, jmax = truth.mu * kTractionShare * M * G * dt;   // from the CG
        const double j = std::clamp(-(u1 + w1 * pv) / (1 / M + pv * pv / J), -jmax, jmax);
        u1 += j / M;
        w1 += j * pv / J;
    }
    u1 = towardZero(u1, kOmniLatMu * (1 - kTractionShare) * G * dt);
    body.v = v1; body.w = w1; body.vl = u1;
    const double thm = body.th + body.w * dt / 2;
    body.th += body.w * dt;
    // body.x, body.y is the traction axle centre, (kAxleFwdIn - cg) ahead of the CG: its sideways speed is vl + w * that
    const double uo = body.vl + body.w * (kAxleFwdIn - truth.cgFwdIn) * IN;
    body.x += (body.v * std::sin(thm) + uo * std::cos(thm)) * dt / IN;
    body.y += (body.v * std::cos(thm) - uo * std::sin(thm)) * dt / IN;
    body.contact = std::hypot(c.fx, c.fy);
    {   // claw sway and tip risk, both from the chassis' forward acceleration (pitch itself is not modelled)
        const double acc = std::clamp((v1 - v0) / dt, -G, G);
        const double rise = liftRiseIn(r2d(lift.a));
        const double wn = 2 * PI * kSwayHz, frac = std::clamp(rise / kSwayFullRiseIn, 0.0, 1.0);
        swayV += (-acc / IN * frac - 2 * kSwayZeta * wn * swayV - wn * wn * sway) * dt;
        sway = std::clamp(sway + swayV * dt, -1.5, 1.5);
        const double mc = truth.carriageKg + objKg;
        const double xcg = mc * clawReach() / M, hcg = ((M - mc) * kChassisCgHIn + mc * (rise + kCarriageCgUpIn)) / M;
        const double tipG = (kFrontAxleIn - xcg) / hcg;
        if (!openFloor && body.contact == 0 && -acc > G * tipG && !counters.tipRiskMs++) {
            event(fmt("\"type\": \"tip_risk\", \"decel_g\": %.2f, \"threshold_g\": %.2f, \"lift_deg\": %.1f", -acc / G, tipG, r2d(lift.a)));
            logLine(fmt("[tip] braking at %.2f g with the lift at %.1f deg is over the rough tip threshold %.2f g (CG %.1f in up, %.1f in behind the front axle). "
                        "Pitch is not modelled: the sim robot stays flat", -acc / G, r2d(lift.a), tipG, hcg, kFrontAxleIn - xcg));
        }
    }
    // tracking wheel rolls with the ground at its lateral offset (forward component only); no slip
    truth.trackS += (body.v - body.w * truth.wheelOffset * IN) * dt / IN;
    // ---- lift
    {
        const double deg = r2d(lift.a);
        const double dz = (liftRiseIn(deg + 0.5) - liftRiseIn(deg - 0.5)) / d2r(1.0) * IN;   // m/rad
        const double dx = (liftDxIn(deg + 0.5) - liftDxIn(deg - 0.5)) / d2r(1.0) * IN;
        const double mass = truth.carriageKg + objKg;
        double Tm = 0, fr = truth.liftFricNm;
        for (int p = 0; p < 32; ++p) {
            MotorDev& m = motor(p);
            if (!m.exists || m.mech != 3) continue;
            const double wm = m.mount * lift.w * kCrank;
            Tm += m.mount * geared(motorTorque(m, wm), wm) * kCrank;
            fr += motorFric(m) * kCrank;
        }
        const double Jl = mass * (dz * dz + dx * dx) + truth.midKg * (kL1In * IN) * (kL1In * IN) + kLiftArmsInertia;
        const double net = Tm - liftLoadNm(deg, objKg);
        if (lift.a <= 0 && lift.w <= 0 && net <= fr) { lift.a = 0; lift.w = 0; }   // resting on the bottom stop
        else {
            lift.w += net / Jl * dt;
            lift.w = towardZero(lift.w, fr / Jl * dt);
            lift.a += lift.w * dt;
            if (lift.a < 0) { lift.a = 0; lift.w = std::max(0.0, lift.w); }
            if (lift.a > d2r(kLiftTopDeg)) { lift.a = d2r(kLiftTopDeg); lift.w = std::min(0.0, lift.w); }
        }
        lift.maxDeg = std::max(lift.maxDeg, r2d(lift.a));
    }
    paddleStep(dt);
    // ---- claw rollers and unused motors (free rotors)
    for (int p = 0; p < 32; ++p) {
        MotorDev& m = motor(p);
        if (!m.exists || (m.mech != 0 && m.mech != 4)) continue;
        const double tau = motorTorque(m, m.vel);
        m.vel += tau / m.rotorJ * dt;
        m.vel = towardZero(m.vel, motorFric(m) / m.rotorJ * dt);
    }
    // ---- motor shaft positions from the mechanisms
    const double vL2 = body.v + body.w * hw, vR2 = body.v - body.w * hw;
    for (int p = 0; p < 32; ++p) {
        MotorDev& m = motor(p);
        if (!m.exists) continue;
        if (m.mech == 1) m.vel = m.mount * vL2 / r * Gr;
        else if (m.mech == 2) m.vel = m.mount * vR2 / r * Gr;
        else if (m.mech == 3) {
            // gear slop: free while the lift rests on its stop (where it is tared), taken up over the
            // first half degree of travel, so under load the motor is ahead of the arm by the slop
            m.vel = m.mount * lift.w * kCrank;
            m.pos = m.mount * (lift.a + d2r(truth.liftSlopDeg) * std::min(1.0, lift.a / d2r(0.5))) * kCrank;
            continue;
        } else if (m.mech == 5) m.vel = m.mount * paddle.w * kPaddleRatio;
        m.pos += m.vel * dt;
    }
    if (!openFloor) { looseStep(); objectsStep(t); }
}

// ================================================================ sensors
// Nearest hit along one ray from (sx, sy) at compass heading hdeg, beam height h (in).
static double rayHit(double sx, double sy, double hdeg, double h, bool chutes) {
    const double ux = std::sin(d2r(hdeg)), uy = std::cos(d2r(hdeg));
    double best = 1e9;
    if (h <= kWallHeightIn) {
        if (ux > 1e-9) best = std::min(best, (kHalf - sx) / ux);
        if (ux < -1e-9) best = std::min(best, (-kHalf - sx) / ux);
        if (uy > 1e-9) best = std::min(best, (kHalf - sy) / uy);
        if (uy < -1e-9) best = std::min(best, (-kHalf - sy) / uy);
    }
    if (openFloor) return best;
    const double tanV = std::tan(d2r(kDistConeHalfDeg));
    auto circ = [&](double cx, double cy, double r, double top) {   // top: height of the object
        const double ox = cx - sx, oy = cy - sy;
        const double tca = ox * ux + oy * uy;
        const double d2 = ox * ox + oy * oy - tca * tca;
        if (d2 > r * r) return;
        const double thc = std::sqrt(r * r - d2);
        double tt = tca - thc;
        if (tt < 0) { if (tca + thc < 0) return; tt = 0; }
        if (h - tt * tanV > top) return;   // lower than the sensor and the cone has not dropped to it yet
        best = std::min(best, tt);
    };
    for (const auto& g : goals) {
        circ(g.x, g.y, kGoalR, goalTop(g.type));
        if (g.objects > 0) circ(g.x, g.y, kCupR, stackTop(g));   // the stack above the base
    }
    for (const auto& c : staticCups) circ(c.x, c.y, c.r, c.h);
    for (const auto& l : loose) circ(l.x, l.y, l.r, l.h);
    for (const auto& o : objs)
        if (o.state == kStanding) circ(o.x, o.y, kCupR, o.units == 2 ? kCupPinH : kObjLen);
    if (chutes)
        for (const auto& b : loaderBoxes) {   // slab test
            double t0 = 0, t1 = 1e9;
            const double lo[2] = {b.x0, b.y0}, hi[2] = {b.x1, b.y1}, o[2] = {sx, sy}, u[2] = {ux, uy};
            bool hit = true;
            for (int k = 0; k < 2 && hit; ++k) {
                if (std::fabs(u[k]) < 1e-12) { if (o[k] < lo[k] || o[k] > hi[k]) hit = false; continue; }
                double a = (lo[k] - o[k]) / u[k], c2 = (hi[k] - o[k]) / u[k];
                if (a > c2) std::swap(a, c2);
                t0 = std::max(t0, a); t1 = std::min(t1, c2);
                if (t0 > t1) hit = false;
            }
            if (hit) best = std::min(best, t0);
        }
    return best;
}
// V5 distance sensor: +-15 mm below 200 mm, about 5 % beyond. Modelled as a per-sensor offset
// and scale error (systematic) plus sample jitter (3 mm, 1 % beyond 200 mm). 9999 = nothing seen.
static int32_t noisyMm(double in, int sensor) {
    double mm = in * 25.4;
    if (mm > kDistMaxMm) return 9999;
    if (sensor >= 0) mm = mm * (1.0 + truth.distScale[sensor]) + truth.distOffMm[sensor];
    const double sd = mm < 200 ? kDistJitterMm : kDistJitterFrac * mm;
    return static_cast<int32_t>(std::max(0.0, std::round(mm + gauss(0, sd))));
}
static int32_t wallSensor(int i) {
    const auto& mt = truth.mount[i];
    double sx, sy; robotToWorld(mt[1], mt[0], sx, sy);
    const double hc = r2d(body.th) + mt[2];
    double best = 1e9;
    for (int k = 0; k < kDistRays; ++k) {
        const double off = -kDistConeHalfDeg + 2 * kDistConeHalfDeg * k / (kDistRays - 1);
        best = std::min(best, rayHit(sx, sy, hc + off, mt[3], true));
    }
    return noisyMm(best, i);
}
static int32_t clawSensor() {
    if (heldObj >= 0) return static_cast<int32_t>(std::round(kSeatedMm + gauss(0, 3.0)));
    double sx, sy; robotToWorld(clawReach() - kThroatBackIn, 0.0, sx, sy);
    // looks along the throat; a Loader chute is open toward the robot, so it does not block the beam
    return noisyMm(rayHit(sx, sy, r2d(body.th), liftRiseIn(r2d(lift.a)) + 1.0, false), -1);
}

void worldSample(uint64_t t) {
    // motor telemetry
    if (t % kMotorSamplePeriodMs == static_cast<uint64_t>(truth.phaseMotor))
        for (int p = 0; p < 32; ++p) {
            MotorDev& m = motor(p);
            if (m.exists) { m.sPos = m.pos; m.sVel = m.vel; }
        }
    // tracking wheel rotation sensor
    if (t % kRotationPeriodMs == static_cast<uint64_t>(truth.phaseRot)) {
        RotationDev& r = rotation(wiring.rot);
        const double cdeg = truth.trackS / (PI * truth.wheelDia) * 36000.0;
        const double q = 36000.0 / kRotationCpr;
        const double phys = r.mount * std::round(cdeg / q) * q;
        r.sampleVel = (phys - r.sample) / (kRotationPeriodMs / 1000.0);
        r.sample = phys;
    }
    // The two IMUs: independent gain, drift and noise, both measuring the TRUE yaw, each on its own
    // 10 ms clock (own phase, arrival jitter, now and then a lost packet). The robot reads whatever
    // value arrived last, so one odometry tick can see 0, 1 or 2 new samples from a sensor.
    if (!imuBDead && opt.imuBDropS >= 0 && t >= static_cast<uint64_t>(opt.imuBDropS * 1000.0)) {
        imuBDead = true;
        logLine(fmt("[imu] port %d (IMU B) unplugged (--imu-b-drop)", wiring.imu[1]));
        event("\"type\": \"imu_b_dropped\"");
    }
    for (int i = 0; i < 2; ++i) {
        if (t < imuNext[i] || (i == 1 && imuBDead)) continue;
        const int J = static_cast<int>(std::lround(truth.imuJitterMs));
        imuNominal[i] += kImuPeriodMs;
        imuNext[i] = static_cast<uint64_t>(std::max<int64_t>(static_cast<int64_t>(t) + 1,
                                                             imuNominal[i] + (J > 0 ? std::uniform_int_distribution<int>(-J, J)(trng) : 0)));
        if (J > 0 && std::uniform_real_distribution<double>(0, 1)(trng) < kImuMissProb) continue;   // lost packet
        ImuDev& d = imu(wiring.imu[i]);
        if (d.calibrating && t >= d.calEnd) {
            d.calibrating = false; d.everCal = true;
            d.base = body.th; d.raw = 0; d.offset = 0; d.calDone = t;
            logLine(fmt("[imu] port %d calibrated", wiring.imu[i]));
        }
        if (!d.calibrating && d.everCal) {
            d.raw = r2d(body.th - d.base) * d.gain + d.driftDegPerMs * (t - d.calDone) + gauss(0, kImuNoiseDeg);
            d.roll = gauss(0, 0.05); d.pitch = gauss(0, 0.05);
        }
    }
    // distance sensors
    for (int k = 0; k < 4; ++k) {
        DistDev& d = dist(wiring.dist[k]);
        if (!d.exists || t % kDistPeriodMs != static_cast<uint64_t>(d.phase)) continue;
        d.value = k < 3 ? wallSensor(k) : clawSensor();
    }
}

// ================================================================ results
bool touchingMidfield() {
    // frame outline, then the raised lift's rear overhang (final CAD); the claw and the paddle are not counted
    const double lr = liftRearIn(false);
    const double cf[6] = {kFrontIn, kFrontIn, -truth.rearIn, -truth.rearIn, -lr, -lr};
    const double cr[6] = {kHalfWidthIn, -kHalfWidthIn, -kHalfWidthIn, kHalfWidthIn, kLiftHalfWIn, -kLiftHalfWIn};
    const int from[5] = {0, 1, 2, 3, 4}, to[5] = {1, 2, 3, 0, 5};
    for (int e = 0; e < 5; ++e)
        for (int s = 0; s <= 40; ++s) {
            const double a = s / 40.0;
            double x, y;
            robotToWorld(cf[from[e]] + (cf[to[e]] - cf[from[e]]) * a, cr[from[e]] + (cr[to[e]] - cr[from[e]]) * a, x, y);
            if (std::fabs(x) + std::fabs(y) <= kMidfieldHalfDiag) return true;
        }
    return std::fabs(body.x) + std::fabs(body.y) <= kMidfieldHalfDiag;
}
std::vector<GoalResult> goalResults() {
    std::vector<GoalResult> r;
    for (const auto& g : goals) r.push_back({g.name, typeName(g.type), g.pins, g.startPins, g.objects, g.needsToggle});
    return r;
}
// Pins the robot seated. applyToggle: Pins on the two red-wall Quadrant Goals only count when
// that Toggle ends red (yellow -> blue -> red, one state per paddle strike, ASSUMED).
int pinsScored(bool applyToggle) {
    int n = 0;
    const bool red = counters.toggleSweeps % 3 == 2;
    for (const auto& g : goals)
        if (!applyToggle || !g.needsToggle || red) n += g.pins;
    return n;
}
bool holdingObject() { return heldObj >= 0; }
}  // namespace sim
