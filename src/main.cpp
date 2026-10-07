#include "main.h"
#include "autons.hpp"
#include "GenSelector/selector.hpp"
#include "gen/electronics.h"
#include "gen/features.hpp"
#include "gen/setup.hpp"


using DriveMode = arc::Controller::DriveMode;
using Button = arc::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

arc::Controller controller(arc::Controller::DriveMode::Arcade2Stick, 3, 10.0, false);
arc::MotorGroup leftDrive({-9, 10}, 600.0, 1.33);
arc::MotorGroup rightDrive({-1, 2}, 600.0, 1.33);
arc::MotorGroup intake({12}, 600.0, 1.0);

arc::MotorGroup lift({6, 3}, 600.0, 1.0);
arc::MotorGroup pinRollers({4}, 200.0, 0.25); 
arc::MotorGroup clawRot({5}, 200.0, 0.25);    
// pros::Rotation horizontalEncoder(-15);
// pros::Rotation verticalEncoder(-16);
arc::CustomIMU imu(20, 1.01123595506); // change the port to match


// arc::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0, -1.0);
// arc::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.0, -2.469176);

// arc::Piston wingPiston('A', false, "wing");
// arc::PistonGroup wings({{"wing", &wingPiston}});

constexpr arc::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 10.63f,
    .wheelDiameterIn = arc::Omniwheel::NEW_275,
    .wheelRpm = 450.0f,
    .horizontalDrift = 8.0f,
};

constexpr arc::Motion::ControllerProfile lateralProfile{
    .gains = {
        .proportional = {.initial = 5.73f, .final = 5.73f, .scale = 12.0f, .power = 1.0f},
        .kI = 0.88f,
        .kD = 0.01f,
        .integralRange = 6.0f,
        .integralSignReset = true,
    },
    .exits = {
        .timeout = 2200,
        .velocityExit = 6.0f,
        .errorExit = 0.6f,
        .halfPlaneExit = true,
        .halfPlaneTolerance = 2.4f,
    },
};

constexpr arc::Motion::ControllerProfile angularProfile{
    .gains = {
        .proportional = {.initial = 2.6f, .final = 4.6f, .scale = 28.0f, .power = 1.5f},
        .scheduleMode = arc::GainScheduleMode::FilteredCurrentError,
        .scheduleAlpha = 0.15f,
        .derivativeAlpha = 0.15f,
        .kI = 0.0f, 
        .kD = 0.157f,
        .integralRange = 5.0f,
        .integralSignReset = true,
    },
    .correctionGains = {
        .proportional = {.initial = 0.0f, .final = 0.0f, .scale = 1.0f, .power = 1.0f},
        .kD = 0.0f,
    },
    .exits = {
        .timeout = 2400,
        .velocityExit = 3.0f,
        .errorExit = 4.0f,
        .halfPlaneExit = false,
        .settleTimeMs = 150,
    },
};

constexpr arc::Motion::OdomProfile odomProfile{
    .vertical1 = nullptr,
    .vertical2 = nullptr,
    .horizontal1 = nullptr,
    .horizontal2 = nullptr,
    .imu = &imu,
};
// ======================= End User Template =======================

arc::Drivetrain drivetrain = drivetrainProfile.toGen(&leftDrive, &rightDrive);
arc::ControllerSettings lateralController = lateralProfile.toGen();
arc::ControllerSettings angularController = angularProfile.toGen();
arc::OdomSensors odomSensor = odomProfile.toGen();
arc::Chassis chassis(drivetrain, lateralController, angularController, odomSensor);

double selectorX() { return chassis.getPose().x; }
double selectorY() { return chassis.getPose().y; }
double selectorTheta() { return chassis.getPose().theta; }


namespace Auton {

void boomerangTest();
void pidTest();
void doNothing() {}

}


robot::AutonRoutineList autonRoutines = {
    {"Do Nothing", static_cast<robot::AutonFunc>(Auton::doNothing)},
#if ARC_RAMSETE_LQR_ENABLED
    {"RAMSETE-LQR Test", static_cast<robot::AutonFunc>(Auton::ramseteLqrTestRoutine)},
    {"RAMSETE-LQR Tau Step", static_cast<robot::AutonFunc>(Auton::ramseteLqrTauTest)},
#endif
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

    // controller.raw().rumble(".");
}

namespace {

void pidTestPause() { pros::delay(500); }

}

void disabled() {}

void competition_initialize() {}

void Auton::boomerangTest() {
    // chassis.moveToPose(24, 24, 90, {.timeout = 2000, .velocityExit = -1, .errorExit = -1, .halfPlaneExit = false, .halfPlaneTolerance = 2});
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

    //  chassis.turnToHeading(0, {.timeout = 1500, .velocityExit = 3, .errorExit = 4, .lockedSide = arc::LockedSide::RIGHT});

}


void Auton::pidTest() {
    chassis.setPose(0, 0, 0);
    for (float heading : {10.0f, 20.0f, 30.0f, 45.0f, 60.0f, 75.0f, 90.0f, 120.0f, 135.0f, 150.0f, 180.0f}) {
        chassis.turnToHeading(heading);
        pidTestPause();
        chassis.turnToHeading(0);
        pidTestPause();
    }

    chassis.setPose(0, 0, 0);
    for (float distance : {6.0f, 12.0f, 24.0f, 36.0f, 48.0f}) {
        chassis.moveDistance(distance);
        pidTestPause();
        chassis.moveDistance(-distance);
        pidTestPause();
    }
}

void autonomous() {
    Auton::skills924();
}

void opcontrol() {
    controller.raw().rumble("."); 
    while (true) {
        const auto [leftOutput, rightOutput] = controller.arcade_two_stick();
        chassis.tank(leftOutput, rightOutput);

        pros::delay(10);
    }
}
