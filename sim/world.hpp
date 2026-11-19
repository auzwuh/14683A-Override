// world.hpp - world model state shared between world.cpp, sim_main.cpp and sim_selftest.cpp.
#pragma once
#include "sim_core.hpp"

namespace sim {
struct Truth {
    double mu = 0.9, rollResist = 0.03;
    double wheelDia = 2.0, wheelOffset = 0.364;  // tracking wheel (truth)
    double rearIn = 7.05, loaderX = 58.80, scrubLeverIn = 0.25, pivotFwdIn = 1.5, cgFwdIn = 1.5, paddleDeg0 = 180.0;
    double mount[3][4] = {};                     // right, forward, angle, height (truth)
    double calBias[3] = {};                      // biasIn a printReadings calibration gives
    double imuGain[2] = {1.0, 1.0};              // raw degrees each IMU reports per real degree (truth)
    double distOffMm[3] = {}, distScale[3] = {}; // systematic distance sensor error (truth)
    double trackS = 0;                           // tracking wheel ground travel, in
    double massKg = 5.67, yawJ = 0.10, driveFricN = 0;                       // chassis
    double rest1Deg = -128.7, rest2Deg = -25.0, carriageKg = 1.02, midKg = 0.45, liftFricNm = 0.5, liftBandNm = 0,
           liftSlopDeg = 0, kGErr = 0;                                       // lift
    double pitch = 3.32, preloadDzIn = 0;                                    // stacking
    double grabAheadIn = 1.75, grabLatIn = 0.77, releaseFwdIn = 0.5, captureR = 1.0, releaseMs = 80;   // claw / Goal
    double imuJitterMs = 0;
    int phaseImu[2] = {3, 3}, phaseMotor = 5, phaseRot = 7, autonDelayMs = 0;   // sample phases inside the 10 ms tick
};
// x, y: the TRUE robot origin = middle of the traction axle (the point the robot code calls its pose), in. th rad CW.
// v, vl: forward and rightward speed of the CG (truth.cgFwdIn ahead of the frame centre: on the axle by default), m/s. w rad/s.
struct Body { double x = 0, y = 0, th = 0, v = 0, w = 0, vl = 0, contact = 0; };
struct LiftState { double a = 0, w = 0, maxDeg = 0; };                              // rad, rad/s
struct Counters {
    int resetApplied = 0, resetSkipped = 0, resetMountOk = 0, resetMountReject = 0, resetMountSkip = 0;
    int squares = 0, liftNotSettled = 0, gripNotSeen = 0, gripOk = 0, untunedTargets = 0, preflightMissing = 0;
    int toggleSweeps = 0, parkAtMs = 0;   // parkAtMs: sim time of the "parking in the Midfield" line, 0 = never
    // Toggle spinner. toggleSweeps = paddle ends that went through the Toggle (one state each, ASSUMED)
    int toggleFlips = 0, toggleJamMs = 0, toggleStoppedInside = 0, togglePartTurned = 0, togglePushedPaddle = 0;
    int toggleBumpMs = 0, clawWallMs = 0;   // ms the chain / plates pressed on the Toggle, ms the claw or held object pressed on a wall or the Toggle
    int closeSensor = 0, closeOdom = 0, closeSkipped = 0;   // localizer.closeToWall(): measured by the front sensor / by odometry / gave up
    double closeGapIn = -1;                 // TRUE rail gap (centre line) when the first closeToWall() printed its result
    double flipGapIn = -1, flipGapLeftIn = -1, flipGapRightIn = -1, flipGapPaddleIn = -1, flipHeadDeg = 0, flipClawUnderIn = 0,
           flipPaddleDeg = 0;             // TRUE state when the first toggle::flip() started (gap -1 = never called)
    int setPoseRoute = 0, setPoseReset = 0, setPoseTrack = 0, setPoseSquare = 0;
    int grabs = 0, grabMiss = 0, tipped = 0, releases = 0, loosePushed = 0;
    int imuTicks = 0, imuDisagree = 0;   // robot reads of both IMUs, and how many of them differ by more than the fusion gate
    int stackHits = 0, scoredStackHits = 0, dropped = 0, tipRiskMs = 0;
};
struct PlaceRec { int n; std::string object, goal; bool ok; std::string why; double radial, along, across, bottom, top, liftDeg, t; };
extern std::vector<PlaceRec> placeRecs;
struct ControllerScript { int analog[4] = {}; int digital[32] = {}; };
struct GoalResult { std::string name, type; int pins, startPins, objects; bool needsToggle; };

extern Truth truth;
extern Body body;
extern LiftState lift;
extern Counters counters;
extern ControllerScript script;
extern bool openFloor;      // selftest: no walls, Goals or objects

void worldInit();
void worldStep(uint64_t t);
void worldSample(uint64_t t);
void worldTeleport(double x, double y, double headingDeg);   // sets the TRUE pose (traction axle centre), zero velocity
void frameCentre(double& x, double& y);                      // TRUE position of the frame centre (the CAD origin)
void setDisabled(bool on);     // field control: every motor off and coasting, commands ignored
bool worldAtRest();
void imuTiming(double jitterMs, int phaseA, int phaseB);   // selftest: change the IMU sample clocks
void openLog(const std::string& path);
void closeLog();
const std::vector<std::string>& allEvents();
double liftRiseIn(double deg);
double liftDxIn(double deg);
double liftHoldVolts(double deg, bool withObject);   // nominal plant, centre of the friction band
bool touchingMidfield();
std::vector<GoalResult> goalResults();
int pinsScored(bool applyToggle);
bool holdingObject();
void toggleReset(double paddleDeg);   // selftest: Toggle back to yellow, paddle at this angle, counters cleared
int toggleState();                    // 0 yellow, 1 blue, 2 red
double paddleDeg();                   // long end direction, 0 = forward, 90 = up
double paddleRpm();
double railGapIn();                   // TRUE gap, front rail ends (centre line) to the red wall
std::string looseSummary();
}  // namespace sim
