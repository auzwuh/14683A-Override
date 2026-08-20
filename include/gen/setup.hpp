#pragma once

#include "gen/api.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace arc::Motion {

struct ControllerProfile {
    PIDConfig gains{};
    PIDConfig correctionGains{};
    ExitSettings exits{};

    arc::ControllerSettings toGen() const;
};

struct DrivetrainProfile {
    float trackWidthIn = 0.0f;
    float wheelDiameterIn = arc::Omniwheel::NEW_325;
    float wheelRpm = 0.0f;
    float horizontalDrift = 2.0f;

    arc::Drivetrain toGen(pros::MotorGroup* left, pros::MotorGroup* right) const;
};

struct OdomProfile {
    arc::TrackingWheel* vertical1 = nullptr;
    arc::TrackingWheel* vertical2 = nullptr;
    arc::TrackingWheel* horizontal1 = nullptr;
    arc::TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;

    arc::OdomSensors toGen() const;
};

} // namespace arc::Motion
