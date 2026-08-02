#include "main.h"
#include "autons.hpp"
#include "GenSelector/selector.hpp"
#include "gen/electronics.h"
#include "gen/setup.hpp"
#include "pros/distance.hpp"

using DriveMode = gen::Controller::DriveMode;
using Button = gen::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

gen::Controller controller(gen::Controller::DriveMode::Arcade2Stick, 3, 10.0, false);
gen::MotorGroup leftDrive({18, 17}, 600.0, 1.33);
gen::MotorGroup rightDrive({20, 19}, 600.0, 1.33);
gen::MotorGroup intake({12}, 600.0, 1.0);
// Scoring mechanisms used by the Override AWP routines in src/autons.cpp.
// PORTS ARE UNVERIFIED - these come from the robot description, not from this
// project's existing wiring.  Port 1 is the IMU here, so the intake stays on
// 12 rather than moving to 1.  Check these before running on a real robot.
gen::MotorGroup lift({2, 3}, 600.0, 1.0);     // cascade lift
gen::MotorGroup claw({4}, 600.0, 1.0);        // claw grip
gen::MotorGroup clawRot({5}, 600.0, 1.0);     // claw rotation for stacking
pros::Rotation horizontalEncoder(-15);
pros::Rotation verticalEncoder(-16);
gen::CustomIMU imu(1, 1.01123595506);

// Measure these signed offsets from the robot's tracking center.
// Left/back offsets are negative; right/front offsets are positive.
gen::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0, -1.0);
gen::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.75, -2.469176);

// gen::Piston wingPiston('A', false, "wing");
// gen::PistonGroup wings({{"wing", &wingPiston}});

constexpr gen::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 10.6f,
    .wheelDiameterIn = gen::Omniwheel::NEW_275,
    .wheelRpm = 450.0f,
    .horizontalDrift = 8.0f,
};

constexpr gen::Motion::ControllerProfile lateralProfile{
    .gains = {
        .proportional = {.initial = 5.73f, .final = 5.73f, .scale = 12.0f, .power = 1.0f},
        .kI = 0.88f,
        .kD = 0.01f,
        .integralRange = 6.0f,
        .integralSignReset = true,
    },
    .exits = {
        .timeout = 2000,
        .velocityExit = 6.0f,
        .errorExit = 0.6f,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
    },
};

constexpr gen::Motion::ControllerProfile angularProfile{
    .gains = {
        .proportional = {.initial = 4.75f, .final = 2.33f, .scale = 28.0f, .power = 1.5f},
        .scheduleMode = gen::GainScheduleMode::FilteredCurrentError,
        .scheduleAlpha = 0.15f,
        .derivativeAlpha = 0.15f,
        .kI = 0.0f,  // Reintroduce only if testing shows a consistent steady-state error.
        .kD = 0.157f,
        .integralRange = 5.0f,
        .integralSignReset = true,
    },
    .correctionGains = {
        .proportional = {.initial = 0.0f, .final = 0.0f, .scale = 1.0f, .power = 1.0f},
        .kD = 0.0f,
    },
    .exits = {
        .timeout = 2000,
        .velocityExit = 3.0f,
        .errorExit = 4.0f,
        .halfPlaneExit = false,
        .settleTimeMs = 150,
    },
};

constexpr gen::Motion::OdomProfile odomProfile{
    .vertical1 = nullptr,
    .vertical2 = nullptr,
    .horizontal1 = nullptr,
    .horizontal2 = nullptr,
    .imu = &imu,
};
// ======================= End User Template =======================

gen::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
gen::ControllerSettings lateralController = lateralProfile.toGen();
gen::ControllerSettings angularController = angularProfile.toGen();
gen::OdomSensors odomSensor = odomProfile.toGen();
gen::Chassis chassis(drivetrain, lateralController, angularController, odomSensor);

double selectorX() { return chassis.getPose().x; }
double selectorY() { return chassis.getPose().y; }
double selectorTheta() { return chassis.getPose().theta; }

namespace Auton {

void currentTest();
void doNothing() {}

}

robot::AutonRoutineList autonRoutines = {
    {"Override AWP Red", static_cast<robot::AutonFunc>(Auton::overrideAwpRed)},
    {"Override AWP Blue", static_cast<robot::AutonFunc>(Auton::overrideAwpBlue)},
    {"Current Test", static_cast<robot::AutonFunc>(Auton::currentTest)},
    {"Do Nothing", static_cast<robot::AutonFunc>(Auton::doNothing)},
};

const robot::SelectorConfig autonSelectorConfig{
    .input = {
        .type = robot::SelectorInputType::BrainScreen,
    },
    .menu = {
        .teamNumber = "14683A",
    },
    .devices = robot::SelectorDevicesConfig(
        {"Left Drive", &leftDrive, 0},
        {"Right Drive", &rightDrive, 0},
        {"Intake", &intake, 0}
    ),
    .terminal = {
        .fields = {
            {"X", selectorX, 2},
            {"Y", selectorY, 2},
            {"Theta", selectorTheta, 2},
        },
        .refreshMs = 50,
    },
    .lcdLine = 4,
    .pollDelayMs = 20,
};

robot::AutonSelector autonSelector(autonSelectorConfig, autonRoutines);

void initialize() {
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    autonSelector.start();
    // controller.raw().rumble(".");
}

void disabled() {}

void competition_initialize() {}

void Auton::currentTest() {
    // chassis.moveToPose(24, 24, 90, {.timeout = 2000, .velocityExit = -1, .errorExit = -1, .halfPlaneExit = false, .halfPlaneTolerance = 2});
    // chassis.turnToHeading(10,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1}); 
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1}); 
    // chassis.turnToHeading(20,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(30,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(45,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(60,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(75,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(90,  {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(120, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(150, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(180, {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.turnToHeading(0,   {.timeout = 1500, .velocityExit = -1, .errorExit = -1});
    // chassis.moveToPoint(0, 24);
    // chassis.moveToPoint(0, 0);
    // chassis.moveToPoint(24, 24);
    // chassis.moveToPoint(0, 0);

    // chassis.moveToPoint(24, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(0, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(24, 24, {.timeout = 2000, .velocityExit = 6, .errorExit = 0.0, .forwards = true, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});
    // chassis.moveToPoint(0, 0, {.timeout = 2000, .velocityExit = 0, .errorExit = 0.0, .forwards = false, .maxSpeed = 100, .halfPlaneExit = true, .halfPlaneTolerance = 2.4f});

    // chassis.moveToPoint(24, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(0, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(24, 24, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPoint(0, 0, {
    //     .timeout = 2500,
    //     .velocityExit = 6.0f,
    //     .errorExit = 0.6f,
    //     .forwards = false,
    //     .maxSpeed = 100,
    //     .minSpeed = 0,
    //     .halfPlaneExit = false,
    //     .settle = true,
    // });




    // chassis.moveToPose(24, 24, 45, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(0, 24, 270, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // chassis.moveToPose(24, 24, 90, {
    //     .timeout = 2000,
    //     .velocityExit = -1,
    //     .errorExit = -1,
    //     .forwards = true,
    //     .maxSpeed = 100,
    //     .minSpeed = 30,
    //     .halfPlaneExit = true,
    //     .halfPlaneTolerance = 2.4f,
    //     .settle = false,
    // });

    // // Drive backward southwest while the robot's front faces northeast.
    // chassis.moveToPose(0, 0, 0, {
    //     .timeout = 2500,
    //     .velocityExit = 6.0f,
    //     .errorExit = 0.6f,
    //     .forwards = false,
    //     .maxSpeed = 100,
    //     .minSpeed = 0,
    //     .halfPlaneExit = false,
    //     .settle = true,
    // });


    // Assumes the robot starts at (0, 0, 0).

    // Assumes the robot starts at (0, 0, 0).

    chassis.moveToPose(24, 24, 45, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .dLead = 8.0f,
        .gLead = 0.35f,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });

    chassis.moveToPose(0, 24, 245, {
        .timeout = 2000,
        .velocityExit = -1,
        .errorExit = -1,
        .forwards = true,
        .dLead = 8.0f,
        .gLead = 0.35f,
        .maxSpeed = 100,
        .minSpeed = 30,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
        .settle = false,
    });

    chassis.moveToPose(0, 0, 180, {
        .timeout = 2500,
        .velocityExit = 6.0f,
        .errorExit = 0.6f,
        .forwards = true,
        .dLead = 3.0f,
        .gLead = 0.0f,
        .maxSpeed = 100,
        .minSpeed = 0,
        .halfPlaneExit = false,
        .settle = true,
    });

    //  chassis.turnToHeading(0, {.timeout = 1500, .velocityExit = 3, .errorExit = 4, .lockedSide = gen::LockedSide::RIGHT});
                                     
}

void autonomous() {
    autonSelector.runSelected(Auton::currentTest);
}

void opcontrol() {
    while (true) {
        const auto [leftOutput, rightOutput] = controller.arcade_two_stick();
        chassis.tank(leftOutput, rightOutput);

        // Currently holding R1: intake spins; otherwise it stops.
        controller.holding(Button::R1) ? intake.move(127) : intake.move(0);

        pros::delay(10);
    }
}
