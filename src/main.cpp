#include "main.h"
#include "autons.hpp"
#include "drive_mix.hpp"
#include "dual_imu.hpp"
#include "GenSelector/selector.hpp"
#include "gen/electronics.h"
#include "gen/features.hpp"
#include "gen/setup.hpp"
#include "skills_support.hpp"


using DriveMode = arc::Controller::DriveMode;
using Button = arc::Controller::Button;

// ========================= User Template =========================
// Edit this section first when reusing the project on a new robot.
// 1) Update motor/sensor ports.
// 2) Update drivetrain geometry and wheel RPM.
// 3) Update PID constants after tuning.

// auton 7.87 inches center of bot

arc::Controller controller(arc::Controller::DriveMode::Arcade2Stick, 3, 10.0, false);

// Negative = reversed. Left: 2 forward, 4 reversed. Right: 11 forward, 12 reversed.
arc::MotorGroup leftDrive({-4, 2}, 600.0, 1.33);
arc::MotorGroup rightDrive({11, -12}, 600.0, 1.33);

arc::MotorGroup clawRot({5}, 200.0, 0.25);    
// pros::Rotation horizontalEncoder(-15);
pros::Rotation verticalEncoder(-16);  

// Final CAD: both flat under the cross member, 2.0 in behind the traction axle (the robot origin)
constexpr double kImuARightIn = -2.59, kImuAForwardIn = -2.00;
constexpr double kImuBRightIn = 2.09, kImuBForwardIn = -2.00;
constexpr int kImuPortA = 20;
constexpr int kImuPortB = 19;      // PLACEHOLDER, set to the second sensor's real port
constexpr double kImuScaleA = 1.0; // MEASURE as mounted: 3600 / raw after 10 turns (was 1.0112 on the old robot)
constexpr double kImuScaleB = 1.0; // MEASURE
arc::DualIMU imu(kImuPortA, kImuPortB, {.scale = {kImuScaleA, kImuScaleB}});


// Final CAD: the two 2 in omnis sit 0.364 in RIGHT of the centre line (was -1.0, a guess from the old
// robot). CHECK on the robot: spin 10 turns in place, the odometry should not creep forward or back.
arc::TrackingWheel verticalTrackingWheel(&verticalEncoder, 2.0, 0.364);
// arc::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, 2.0, -2.469176);

// arc::Piston wingPiston('A', false, "wing");
// arc::PistonGroup wings({{"wing", &wingPiston}});

constexpr arc::Motion::DrivetrainProfile drivetrainProfile{
    .trackWidthIn = 10.63f,
    .wheelDiameterIn = 2.75f,  // 6 wheels, 2.75 in: omni, traction (middle), omni per side
    .wheelRpm = 450.0f,
    .horizontalDrift = 8.0f,   // 8 = centre traction wheels hold the robot sideways (2 is for all omni)
};

// STARTING VALUES from the simulator (17 Nov), not tuned on the robot. Swept on 9 modelled
// robots (6.1 kg +-10 %, 3x range of drive friction). Useful ranges: kP 14 to 18, kI 30 to 60,
// kD 0.8 to 1.2. Stops short or creeps at the end: raise kI, then kP. Overshoots or shakes
// at the end: raise kD, then lower kP. moveToPoint always reads the kP curve at 12, so with
// scale 12 only the average of initial and final matters. Keep them equal.
// Before: kP 5.73, kI 0.88, kD 0.01, integralRange 6.
constexpr arc::Motion::ControllerProfile lateralProfile{
    .gains = {
        .proportional = {.initial = 16.0f, .final = 16.0f, .scale = 12.0f, .power = 1.0f},
        .kI = 40.0f,
        .kD = 0.9f,
        .integralRange = 2.5f,
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

// STARTING VALUES from the simulator (17 Nov), not tuned on the robot. Swept on 9 modelled
// robots (8x range of turning scrub, +-30 % yaw inertia). initial is the gain near the target,
// final the gain far from it. Useful ranges: initial 8 to 12, final 3 to 4, kI 30 to 60,
// kD 0.35 to 0.6. Stops a degree or two short: raise kI. Overshoots: raise kD or lower final.
// Buzzes when stopped: lower initial. Keep derivativeAlpha at 0.5 or more, 0.15 made these
// gains overshoot badly in the simulator.
// Before: initial 2.6, final 4.6, kI 0, kD 0.157, integralRange 5, derivativeAlpha 0.15.
constexpr arc::Motion::ControllerProfile angularProfile{
    .gains = {
        .proportional = {.initial = 10.0f, .final = 3.5f, .scale = 28.0f, .power = 1.5f},
        .scheduleMode = arc::GainScheduleMode::FilteredCurrentError,
        .scheduleAlpha = 0.15f,
        .derivativeAlpha = 1.0f,
        .kI = 60.0f,
        .kD = 0.45f,
        .integralRange = 8.0f,
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
double selectorImuA() { return imu.rawRotation(0); }
double selectorImuB() { return imu.rawRotation(1); }


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
        {"Lift", &dr4b::motors(), 0}
    ),
    .terminal = {
        .fields = {
            {"X", selectorX, 2},
            {"Y", selectorY, 2},
            {"Theta", selectorTheta, 2},
            {"IMU A raw", selectorImuA, 1},
            {"IMU B raw", selectorImuB, 1},
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
    dr4b::zeroAtRest();  // lift must be resting on its bottom stop at power-on
    dr4b::computeRouteTargets();   // fills the Goal angles from the stacking model
    if (preflight() > 0) controller.raw().rumble("---");  // long buzz = auton is missing tuned values, read the terminal
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
    // Skills only for now. Runs the ATTICUS skills route directly, no selector.
    skillsDr4b();
}

void opcontrol() {
    controller.raw().rumble(".");
    chassis.cancelAllMotions();  // an auton move cut off mid-run would otherwise keep fighting the sticks
    dr4b::target() = NAN;  // driver owns the lift, stops the auton hold loop writing to it
    dr4b::motors().set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);  // lift stays put when R1/R2 are released
    // Lift stages. Up / Down pick the stage (shown on the controller), R2 sends the lift
    // to it, R1 brings it back down to the pickup height. X / B still move it by hand.
    const auto& stages = dr4b::driverStages();
    const int lastStage = static_cast<int>(stages.size()) - 1;
    int stage = 0, shownStage = -1;
    bool goingDown = false;
    std::uint32_t lastPrint = 0;
    while (true) {
        if (controller.pressed(Button::Up)) stage = std::min(stage + 1, lastStage);
        if (controller.pressed(Button::Down)) stage = std::max(stage - 1, 0);
        if (controller.pressed(Button::R2) && lastStage >= 0) {
            dr4b::to(stages[stage]);
            goingDown = false;
        }
        if (controller.pressed(Button::R1)) {
            dr4b::to(dr4b::kLoader);
            goingDown = true;
        }
        // Down and resting on the stop: let go, so the motors do not push into it all match
        if (goingDown && dr4b::angle() < dr4b::kLoader + 1.5) {
            dr4b::target() = NAN;
            goingDown = false;
        }
        // Manual, both towers together. X up, B down, slowing near the bottom stop and kTopDeg.
        // Touching either one takes the lift back from the macro.
        const int liftCmd = controller.holding(Button::X) ? 127 : controller.holding(Button::B) ? -127 : 0;
        if (liftCmd != 0) {
            dr4b::target() = NAN;
            goingDown = false;
        }
        if (std::isnan(dr4b::target())) {   // nobody is driving the lift to a target
            const int liftOut = drive_mix::liftPower(liftCmd, dr4b::angle(), dr4b::kTopDeg);
            if (liftOut == 0) dr4b::motors().brake();
            else dr4b::motors().move(liftOut);
        }
        // The controller screen takes one line at a time, at most every 50 ms
        if (stage != shownStage && pros::millis() - lastPrint > 60) {
            controller.raw().print(0, 0, "Lift stage %d of %d   ", stage + 1, lastStage + 1);
            shownStage = stage;
            lastPrint = pros::millis();
        }

        // Roller claw. L1 intake, L2 outtake, otherwise hold the grip
        if (controller.holding(Button::L1)) rclaw::intake();
        else if (controller.holding(Button::L2)) rclaw::motor().move_voltage(-12000);
        else rclaw::hold();

        // Toggle spinner, hold A
        toggle::spin(controller.holding(Button::A));

        const auto [leftOutput, rightOutput] = drive_mix::mix(
            controller.raw().get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            controller.raw().get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        chassis.tank(leftOutput, rightOutput);

        pros::delay(10);
    }
}
