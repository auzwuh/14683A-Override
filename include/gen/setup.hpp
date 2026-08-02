#pragma once

#include "gen/api.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

namespace gen::Motion {

struct ControllerProfile {
    PIDConfig gains{};
    PIDConfig correctionGains{};
    ExitSettings exits{};

    gen::ControllerSettings toGen() const;
};

struct DrivetrainProfile {
    float trackWidthIn = 0.0f;
    float wheelDiameterIn = gen::Omniwheel::NEW_325;
    float wheelRpm = 0.0f;
    float horizontalDrift = 2.0f;

    gen::Drivetrain toGen(pros::MotorGroup* left, pros::MotorGroup* right) const;
};

struct OdomProfile {
    gen::TrackingWheel* vertical1 = nullptr;
    gen::TrackingWheel* vertical2 = nullptr;
    gen::TrackingWheel* horizontal1 = nullptr;
    gen::TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;

    gen::OdomSensors toGen() const;
};

} // namespace gen::Motion
