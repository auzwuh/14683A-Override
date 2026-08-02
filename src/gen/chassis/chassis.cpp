#include <cmath>
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "gen/util.hpp"
#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "gen/chassis/trackingWheel.hpp"
#include "pros/rtos.hpp"

gen::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu) {}

gen::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

gen::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                      OdomSensors sensors)
    : drivetrain(drivetrain),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      lateralPID(linearSettings.gains),
      angularPID(angularSettings.gains),
      headingPID(angularSettings.correctionGains) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(gen::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (!std::isnan(sensors.imu->get_heading()) && !std::isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
    }
}

void gen::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new gen::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new gen::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void gen::Chassis::setPose(float x, float y, float theta, bool radians) {
    gen::setPose(gen::Pose(x, y, theta), radians);
    headingTarget = radians ? radToDeg(theta) : theta;
}

void gen::Chassis::setPose(Pose pose, bool radians) {
    gen::setPose(pose, radians);
    headingTarget = radians ? radToDeg(pose.theta) : pose.theta;
}

gen::Pose gen::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = gen::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void gen::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void gen::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void gen::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void gen::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void gen::Chassis::cancelMotion() {
    this->motionRunning = false;
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    pros::delay(10); // give time for motion to stop
}

void gen::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    pros::delay(10); // give time for motion to stop
}

bool gen::Chassis::isInMotion() const { return this->motionRunning; }

void gen::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    gen::setPose(gen::Pose(0, 0, theta), false);
}

void gen::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}
