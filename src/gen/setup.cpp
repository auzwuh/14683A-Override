#include "gen/setup.hpp"

namespace arc::Motion {

arc::ControllerSettings ControllerProfile::toGen() const {
    return arc::ControllerSettings(gains, correctionGains, exits);
}

arc::Drivetrain DrivetrainProfile::toGen(pros::MotorGroup* left, pros::MotorGroup* right) const {
    return arc::Drivetrain(left, right, trackWidthIn, wheelDiameterIn, wheelRpm, horizontalDrift);
}

arc::OdomSensors OdomProfile::toGen() const {
    return arc::OdomSensors(vertical1, vertical2, horizontal1, horizontal2, imu);
}

} // namespace arc::Motion
