// sim_core.hpp - interfaces shared by the fake PROS layer (pros_sim.cpp), the
// world model (world.cpp) and the drivers (sim_main.cpp, sim_selftest.cpp).
// Not seen by robot code.
#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace sim {

// ------------------------------------------------------------- options
struct Options {
    int seed = 1;
    bool noise = true;
    bool driver = false;
    bool quiet = false;
    bool selftest = false;
    bool tunedDrive = false;     // override the two drive PID gain sets with sim-tuned values
    bool selfCal = false;        // scenario: the robot runs calib::run() (auto_calib.hpp) before autonomous() and nothing is filled in for it
    bool calibrated = false;     // scenario: tracking wheel size and offset, IMU scales and stack pitch measured (a fifth of the error left)
    bool goalPins = false;       // F2 scenario: Short and Tall Goals start holding a yellow Pin
    bool drawHeights = false;    // F3 scenario: Short Goal base 5.77 in (draw.py) instead of the code's 3.25
    bool clusters = true;        // F7: cluster Cup + 4 flat Pins are pushable objects (--no-clusters removes them)
    bool noContTrack = false;    // experiment: localizer.startContinuous() corrections gated off
    bool clawWall = true;        // the claw / held object cannot pass through a wall or the Toggle (--no-claw-wall: the pre-CAD sim, ablation only)
    double rearIn = -1;          // F5 scenario: true rear contact (default sim_config kRearIn)
    double wheelDia = -1;        // F8 scenario: true tracking wheel diameter
    double imuGain = -1;         // F9 scenario: true raw gain of both IMUs
    double loaderX = -1;         // F10 scenario: true |x| of the red Loaders
    double scrubIn = -1;         // sensitivity knob: turning scrub lever (default sim_config kScrubLeverIn)
    // further truth knobs, all "unset" below 0 (see kKnobs in sim_main.cpp and sim_config.hpp)
    double imuJitterMs = -1;     // +- ms arrival jitter of each IMU sample
    double imuBDropS = -1;       // IMU B dies at this sim time, s (autonomous() starts near 2.1 s)
    double imuPhaseB = -1;       // IMU B's sample phase inside the 10 ms tick (odometry reads at 0)
    bool imuBMissing = false;    // IMU B unplugged from boot
    double pitchIn = -1, liftSlopDeg = -1, liftBandNm = -1, driveFricN = -1;
    double massKg = -1, yawJ = -1;   // tuning sweeps: true robot mass (kg) and chassis yaw inertia (kg m^2)
    double carriageKg = -1, liftFricNm = -1;   // tuning sweeps: true mass at the claw end, lift shaft friction
    float liftKp = -1, liftKd = -1, liftKi = -1, liftKgScale = 1;   // --lift-gains: sets the ROBOT's dr4b gains (sweep only)
    double grabAheadIn = -1, grabLatIn = -1, releaseMs = -1, releaseFwdIn = -1, captureR = -1;
    double liftErrDeg = 0;       // shifts both linkage rest angles (truth)
    double pivotFwdIn = -1;      // where sideways grip acts, ahead of the frame centre (default sim_config kPivotFwdIn = the axle, 0 = frame centre)
    double cgFwdIn = -1;         // CG ahead of the frame centre (default sim_config kCgFwdIn = on the traction axle)
    double paddleDeg = -1;       // Toggle paddle start angle, 0 to 360 (default: parked long end back, 180, +- 30 per seed)
    double flipTurns = -1;       // sets the ROBOT constant toggle::kFlipTurns from the sim side (sweep only; default: code value)
    double closeLeadIn = -1, closeMv = -1, closeSlowMv = -1;   // same for WallLocalizer::kCloseLeadIn / kCloseMv / kCloseSlowMv (sweep only)
    int watchdogMs = 5000;       // real ms a task may run without yielding
    int autonMs = 60000;
    std::string tag, outDir, repoDir;
};
extern Options opt;

// Which port is which mechanism. Filled by sim_main.cpp from the robot's own objects.
struct Wiring {
    int left[2] = {4, 2}, right[2] = {11, 12}, lift[2] = {3, 6}, claw = 10, rot = 16;
    int imu[2] = {20, 19}, dist[4] = {9, 13, 14, 8};   // front, left, right, claw
    int toggle[2] = {15, 17};                          // the two 5.5 W Toggle motors: [0] left (drives nothing in the CAD), [1] right (the paddle)
};
extern Wiring wiring;

// ------------------------------------------------------------- scheduler (pros_sim.cpp)
uint64_t now();                                   // virtual ms
int spawn(std::function<void()> fn, const std::string& name);
void killTask(int id);
bool taskDone(int id);
std::string currentTaskName();
// Runs tasks and time forever. hook(t) is called after every 1 ms physics step.
[[noreturn]] void kernelRun(const std::function<void(uint64_t)>& stepMs, const std::function<void(uint64_t)>& hook);

// ------------------------------------------------------------- devices (world.cpp)
enum CmdKind { kVolt = 0, kVel = 1, kBrake = 2, kPos = 3 };   // kPos: move_relative, val = target (physical rad), lim = rad/s
struct MotorDev {
    bool exists = false;
    int port = 0;
    int mount = 1;           // hardware coupling sign (set by the port sign at first construction)
    bool reversed = false;   // PROS reversed flag
    int gear = 1;            // 0 red, 1 green, 2 blue
    int units = 0;           // 0 deg, 1 rot, 2 counts
    int brake = 0;           // 0 coast, 1 brake, 2 hold
    double tauLim = 1, wFree = 1;   // N m, rad/s at the cartridge output (truth, with spread)
    struct Cmd { uint64_t t; int kind; double val; double lim = 0; };   // val in PHYSICAL sign
    std::deque<Cmd> q;
    int kind = kVolt; double val = 0, lim = 0;           // active command (physical sign)
    bool holdLatched = false; double holdPos = 0;
    double pos = 0, vel = 0;           // physical rad, rad/s
    double zero = 0;                   // physical rad that reads as 0
    double sPos = 0, sVel = 0;         // last telemetry sample (physical)
    double volt = 0, torque = 0;       // last applied (physical)
    int mech = 0;                      // 0 free, 1 drive L, 2 drive R, 3 lift, 4 claw, 5 Toggle paddle
    double rotorJ = 0.0005;            // free mechanism inertia
};
MotorDev& motor(int absPort);
void motorRegister(int signedPort, int gear /*-1 keep*/, int units /*-1 keep*/);
void motorCommand(int absPort, int kind, double robotVal);   // mV, rpm or 0
void motorCommandRel(int absPort, double robotDelta, double rpm);   // move_relative, delta in the configured units
double motorReadPos(int absPort);      // configured units, robot sign
double motorReadRpm(int absPort);      // robot sign
void motorTare(int absPort, double robotPosUnits);

struct RotationDev { bool exists = false; int mount = 1; bool reversed = false; double zero = 0, sample = 0, sampleVel = 0; };
RotationDev& rotation(int absPort);
void rotationRegister(int signedPort);

struct ImuDev {
    bool exists = false, calibrating = false, everCal = false;
    uint64_t calEnd = 0, calDone = 0;
    double raw = 0, offset = 0, roll = 0, pitch = 0;
    double gain = 1.0, driftDegPerMs = 0, base = 0;   // truth
};
ImuDev& imu(int port);
void imuReset(int port);
void imuReadNote(int port, double value);   // every raw read the robot makes (disagreement counter)

int32_t distanceGet(int port);
void distanceRegister(int port);

int controllerAnalog(int channel);
int controllerDigital(int button);
void logRumble(const char* pattern);

// ------------------------------------------------------------- logging (world.cpp)
void logLine(const std::string& s);                 // timestamped line to the run log
void robotLine(const std::string& task, const std::string& s);   // a line the robot printed
void event(const std::string& json);                // one events.json entry (object body without braces)
std::string fmt(const char* f, ...);
}  // namespace sim
