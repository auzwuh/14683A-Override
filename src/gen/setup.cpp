#include "gen/setup.hpp"

namespace gen::Motion {

gen::ControllerSettings ControllerProfile::toGen() const {
    return gen::ControllerSettings(gains, correctionGains, exits);
}

gen::Drivetrain DrivetrainProfile::toGen(pros::MotorGroup* left, pros::MotorGroup* right) const {
    return gen::Drivetrain(left, right, trackWidthIn, wheelDiameterIn, wheelRpm, horizontalDrift);
}

gen::OdomSensors OdomProfile::toGen() const {
    return gen::OdomSensors(vertical1, vertical2, horizontal1, horizontal2, imu);
}

} // namespace gen::Motion
