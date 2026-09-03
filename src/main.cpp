#include "main.h"
#include "autons.hpp"
#include "macros.hpp"
#include "GenSelector/selector.hpp"
#include "gen/asset.hpp"
#include "gen/chassis/odom.hpp"
#include "gen/electronics.h"
#include "gen/setup.hpp"
#include "pros/distance.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

using DriveMode = arc::Controller::DriveMode;
using Button = arc::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

arc::Controller controller(arc::Controller::DriveMode::Arcade2Stick, 3, 10.0, false);
arc::MotorGroup leftDrive({18, 17}, 600.0, 1.33);
arc::MotorGroup rightDrive({20, 19}, 600.0, 1.33);
arc::MotorGroup intake({12}, 600.0, 1.0);

arc::MotorGroup lift({2, 3}, 600.0, 1.0);     // cascade lift
arc::MotorGroup claw({4}, 600.0, 1.0);        // claw grip
arc::MotorGroup clawRot({5}, 600.0, 1.0);     // claw rotation for stacking
pros::Rotation horizontalEncoder(-15);
pros::Rotation verticalEncoder(-16);
arc::CustomIMU imu(1, 1.01123595506);


arc::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0, -1.0);
arc::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.0, -2.469176);

// arc::Piston wingPiston('A', false, "wing");
// arc::PistonGroup wings({{"wing", &wingPiston}});

constexpr arc::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 10.6f,
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
    .vertical1 = &verticalTrackingWheel,
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

ASSET(autonomous_seg0_path_txt);
ASSET(autonomous_seg1_path_txt);
ASSET(autonomous_seg2_path_txt);

namespace Auton {

void boomerangTest();
void ramseteLqrPathTest();
void ramseteLqrTestRoutine();
void ramseteLqrTauTest();
void pidTest();
void doNothing() {}

}


robot::AutonRoutineList autonRoutines = {
    {"Red - Left", static_cast<robot::AutonFunc>(Auton::overrideRedLeft)},
    {"Red - Bottom", static_cast<robot::AutonFunc>(Auton::overrideRedBottom)},
    {"Blue - Right", static_cast<robot::AutonFunc>(Auton::overrideBlueRight)},
    {"Blue - Top", static_cast<robot::AutonFunc>(Auton::overrideBlueTop)},
    {"Boomerang Test", static_cast<robot::AutonFunc>(Auton::boomerangTest)},
    {"RAMSETE-LQR Test Routine", static_cast<robot::AutonFunc>(Auton::ramseteLqrTestRoutine)},
    {"RAMSETE-LQR Tau Step Test", static_cast<robot::AutonFunc>(Auton::ramseteLqrTauTest)},
    {"PID Test", static_cast<robot::AutonFunc>(Auton::pidTest)},
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

    robot::mech.init();
    autonSelector.start();
    // controller.raw().rumble(".");
}

namespace {

void pidTestPause() { pros::delay(500); }

void showSelectionWhileDisabled() {
    char last[24] = {};
    while (true) {
        const std::size_t index = autonSelector.selectedIndex();
        char buf[24];
        std::snprintf(buf, sizeof(buf), "%-14.14s",
                      index < autonRoutines.size() ? autonRoutines[index].first.c_str() : "?");
        if (std::strcmp(buf, last) != 0) {
            std::snprintf(last, sizeof(last), "%s", buf);
            controller.raw().set_text(0, 0, buf);
        }
        pros::delay(100);
    }
}

}

void disabled() { showSelectionWhileDisabled(); }

void competition_initialize() { showSelectionWhileDisabled(); }

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

// Open-loop velocity step response, straight line only. Drives at a fixed
// duty cycle and logs (t_ms, in_per_sec) to the SD card so the drivetrain's
// true first-order velocity time constant can be fit from real data instead
// of assumed - see RamseteLQRParams::velocityTimeConstant and
// docs/path_following_notebook.md (RAMSETE + LQR section) for what this
// number feeds into and why 0.10s was only ever a placeholder.
//
// Reading the result: tau is the time from the step starting to the
// velocity first crossing 63.2% of its steady-state (settled) value. Pick
// the settled value from the last ~20 rows of the CSV (avg them - there
// will be some V5 sensor noise), find 0.632 * that, and read off the
// matching t_ms.
void Auton::ramseteLqrTauTest() {
    constexpr float kStepDuty = 80.0f;   // out of 127, matches typical RAMSETE-LQR maxSpeed
    constexpr int kDurationMs = 1500;    // long enough to settle at kStepDuty on this drivetrain
    constexpr int kSampleMs = 10;

    chassis.setPose(0, 0, 0);
    FILE* log = std::fopen("/usd/ramsete_tau_step.csv", "w");
    if (log != nullptr) std::fprintf(log, "t_ms,in_per_sec\n");

    const std::uint32_t start = pros::millis();
    std::uint32_t t = 0;
    while (t <= static_cast<std::uint32_t>(kDurationMs)) {
        chassis.tank(static_cast<int>(kStepDuty), static_cast<int>(kStepDuty));
        const arc::Pose velocity = arc::getSpeed(false);
        const float speed = std::hypot(velocity.x, velocity.y);
        if (log != nullptr) std::fprintf(log, "%lu,%.4f\n", static_cast<unsigned long>(t), speed);
        pros::delay(kSampleMs);
        t = pros::millis() - start;
    }
    chassis.tank(0, 0);
    if (log != nullptr) std::fclose(log);
}

void Auton::ramseteLqrTestRoutine() {
    chassis.setPose(-60.0, -0.0, 0.0);

    chassis.turnToPoint(-24.0, -24.0, {.timeout = 868});
    arc::followRamseteLQR(chassis, autonomous_seg0_path_txt, {
        .timeout = 2173,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
    pros::delay(100);
    arc::followRamseteLQR(chassis, autonomous_seg1_path_txt, {
        .timeout = 2874,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
    pros::delay(100);
    chassis.turnToPoint(48.0, -0.0, {.timeout = 591});
    arc::followRamseteLQR(chassis, autonomous_seg2_path_txt, {
        .timeout = 2192,
        .velocityExit = 6,
        .errorExit = -1,
        .maxSpeed = 100,
    });
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
    autonSelector.runSelected(Auton::doNothing);
}

void opcontrol() {
    Auton::ramseteLqrTestRoutine(); //test ramsete lqr path following
    while (true) {
        const auto [leftOutput, rightOutput] = controller.arcade_two_stick();
        chassis.tank(leftOutput, rightOutput);
        robot::mech.driverTick();

        pros::delay(10);
    }
}
