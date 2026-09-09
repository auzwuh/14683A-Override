// Midpoint/chord odometry follows the Pilons tracking-wheel construction.
#include <cmath>
#include <cerrno>
#include <mutex>
#include "pros/rtos.hpp"
#include "gen/util.hpp"
#include "gen/chassis/odom.hpp"
#include "gen/chassis/trackingWheel.hpp"
#include "gen/damp/measurement.hpp"

namespace {
pros::Task* trackingTask = nullptr;
pros::Mutex odomMutex;
arc::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr);
arc::Pose odomPose(0, 0, 0), odomSpeed(0, 0, 0), odomLocalSpeed(0, 0, 0);
arc::OdomSnapshot snapshot;
arc::damp::measurement::Tracker measurement;
struct Readings {
    double v1 = 0, v2 = 0, h1 = 0, h2 = 0, imu = 0;
    std::uint32_t time = 0;
    bool valid = true;
};
Readings previous;
bool initialized = false;
bool unpowered(arc::TrackingWheel* wheel) { return wheel && wheel->getType() == 0; }
double readWheel(arc::TrackingWheel* wheel, bool& valid) {
    if (!wheel) return 0;
    // Integer PROS_ERR is converted to inches inside TrackingWheel. Check errno
    // at the call boundary, before another device can overwrite the error.
    errno = 0;
    const double distance = wheel->getDistanceTraveled();
    valid = valid && errno == 0 && std::isfinite(distance) && std::isfinite(wheel->getOffset());
    return distance;
}
Readings readSensors() {
    Readings result;
    result.v1 = readWheel(odomSensors.vertical1, result.valid);
    result.v2 = readWheel(odomSensors.vertical2, result.valid);
    result.h1 = readWheel(odomSensors.horizontal1, result.valid);
    result.h2 = readWheel(odomSensors.horizontal2, result.valid);
    if (odomSensors.imu) {
        errno = 0;
        const double rotation = odomSensors.imu->get_rotation();
        // PROS_ERR_F is infinity; isfinite also rejects NaN.
        const bool rotationValid = errno == 0 && std::isfinite(rotation);
        errno = 0;
        const bool calibrating = odomSensors.imu->is_calibrating();
        result.valid = result.valid && rotationValid && errno == 0 && !calibrating;
        result.imu = rotation * 3.14159265358979323846 / 180;
    }
    result.time = pros::millis();
    return result;
}
void invalidate(std::uint32_t time) {
    snapshot = {};
    snapshot.pose = odomPose;
    snapshot.timestampMs = time;
    odomSpeed = arc::Pose(0, 0, 0);
    odomLocalSpeed = arc::Pose(0, 0, 0);
    measurement.reset();
    initialized = false;
}
arc::Pose inUnits(arc::Pose pose, bool radians) {
    if (!radians) pose.theta = arc::radToDeg(pose.theta);
    return pose;
}
}
void arc::setSensors(arc::OdomSensors sensors, arc::Drivetrain drivetrain) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    (void)drivetrain;
    odomSensors = sensors;
    invalidate(pros::millis());
}
arc::Pose arc::getPose(bool radians) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    return inUnits(odomPose, radians);
}
arc::OdomSnapshot arc::getOdomSnapshot() {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    return snapshot;
}
void arc::setPose(arc::Pose pose, bool radians) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    if (!radians) pose.theta = degToRad(pose.theta);
    odomPose = pose;
    invalidate(pros::millis());
}
arc::Pose arc::getSpeed(bool radians) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    return inUnits(odomSpeed, radians);
}
arc::Pose arc::getLocalSpeed(bool radians) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    return inUnits(odomLocalSpeed, radians);
}
arc::Pose arc::estimatePose(float time, bool radians) {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    const Pose delta = odomLocalSpeed * time;
    const float midpoint = odomPose.theta + delta.theta / 2;
    Pose future = odomPose;
    future.x += delta.y * std::sin(midpoint) - delta.x * std::cos(midpoint);
    future.y += delta.y * std::cos(midpoint) + delta.x * std::sin(midpoint);
    return inUnits(future, radians);
}
void arc::update() {
    std::lock_guard<pros::Mutex> lock(odomMutex);
    const Readings current = readSensors();
    if (!current.valid || !std::isfinite(odomPose.x) || !std::isfinite(odomPose.y) ||
        !std::isfinite(odomPose.theta)) {
        invalidate(current.time);
        return;
    }
    auto* vertical = unpowered(odomSensors.vertical1) ? odomSensors.vertical1 :
                     unpowered(odomSensors.vertical2) ? odomSensors.vertical2 :
                     odomSensors.vertical1 ? odomSensors.vertical1 : odomSensors.vertical2;
    auto* horizontal = unpowered(odomSensors.horizontal1) ? odomSensors.horizontal1 :
                       unpowered(odomSensors.horizontal2) ? odomSensors.horizontal2 :
                       odomSensors.horizontal1 ? odomSensors.horizontal1 : odomSensors.horizontal2;
    const double v = vertical == odomSensors.vertical1 ? current.v1 : current.v2;
    const double h = horizontal == odomSensors.horizontal1 ? current.h1 : current.h2;
    const double vo = vertical ? vertical->getOffset() : 0;
    const double ho = horizontal ? horizontal->getOffset() : 0;
    const auto raw = measurement.update({v, h, current.imu, current.time,
                                         unpowered(vertical) && unpowered(horizontal) && odomSensors.imu}, vo, ho);
    snapshot = {};
    snapshot.pose = odomPose;
    snapshot.timestampMs = current.time;
    const auto elapsed = current.time - previous.time;
    if (!initialized || elapsed == 0 || elapsed > 0x7fffffffu) {
        previous = current;
        initialized = true;
        return;
    }
    const double dv1 = current.v1 - previous.v1, dv2 = current.v2 - previous.v2;
    const double dh1 = current.h1 - previous.h1, dh2 = current.h2 - previous.h2;
    double turn = 0;
    // DAMP pose and raw body velocity must share the same IMU turn increment.
    // Otherwise preserve legacy heading priority.
    if (unpowered(vertical) && unpowered(horizontal) && odomSensors.imu) {
        turn = current.imu - previous.imu;
    } else if (odomSensors.horizontal1 && odomSensors.horizontal2 &&
        odomSensors.horizontal1->getOffset() != odomSensors.horizontal2->getOffset()) {
        turn = -(dh1 - dh2) / (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    } else if (unpowered(odomSensors.vertical1) && unpowered(odomSensors.vertical2) &&
               odomSensors.vertical1->getOffset() != odomSensors.vertical2->getOffset()) {
        turn = -(dv1 - dv2) / (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    } else if (odomSensors.imu) {
        turn = current.imu - previous.imu;
    } else if (odomSensors.vertical1 && odomSensors.vertical2 &&
               odomSensors.vertical1->getOffset() != odomSensors.vertical2->getOffset()) {
        turn = -(dv1 - dv2) / (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    }
    const double forward = vertical ? (vertical == odomSensors.vertical1 ? dv1 : dv2) + vo * turn : 0;
    const double left = horizontal ? (horizontal == odomSensors.horizontal1 ? dh1 : dh2) + ho * turn : 0;
    const auto field = damp::measurement::fieldDisplacement(forward, left, odomPose.theta, turn);
    const double chord = std::abs(turn) < 1e-9 ? 1 : 2 * std::sin(turn / 2) / turn;
    const double dt = elapsed * .001;
    if (!std::isfinite(field.x) || !std::isfinite(field.y) || !std::isfinite(turn)) {
        invalidate(current.time);
        return;
    }
    odomPose.x += field.x;
    odomPose.y += field.y;
    odomPose.theta += turn;
    // Preserve EMA weighting and local (left, forward) field order.
    odomSpeed.x = ema(field.x / dt, odomSpeed.x, .95);
    odomSpeed.y = ema(field.y / dt, odomSpeed.y, .95);
    odomSpeed.theta = ema(turn / dt, odomSpeed.theta, .95);
    odomLocalSpeed.x = ema(left * chord / dt, odomLocalSpeed.x, .95);
    odomLocalSpeed.y = ema(forward * chord / dt, odomLocalSpeed.y, .95);
    odomLocalSpeed.theta = ema(turn / dt, odomLocalSpeed.theta, .95);
    previous = current;
    snapshot.pose = odomPose;
    snapshot.forwardVelocity = raw.forwardVelocity;
    snapshot.leftVelocity = raw.leftVelocity;
    snapshot.clockwiseAngularVelocity = raw.clockwiseAngularVelocity;
    snapshot.dtSeconds = raw.dtSeconds;
    snapshot.valid = raw.valid;
}
void arc::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
