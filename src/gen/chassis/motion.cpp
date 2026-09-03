#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "gen/path_following/ramsete_lqr.hpp"
#include "gen/util.hpp"
#include "pros/rtos.hpp"

namespace {

constexpr float kMaxOutput = 127.0f;
constexpr float kChaseOutputScale = (127.0f * 127.0f) / 144.0f;
constexpr float kInchesPerFoot = 12.0f;

float clamp(float value, float low, float high) {
    return std::clamp(value, low, high);
}

float speedOutput(float speed) {
    return clamp(std::fabs(speed), 0.0f, kMaxOutput);
}

std::pair<float, float> reduceRatio(float maximum, float first, float second) {
    maximum = std::fabs(maximum);
    if (std::fabs(first) > maximum && std::fabs(first) >= std::fabs(second)) {
        second *= maximum / std::fabs(first);
        first = std::copysign(maximum, first);
    } else if (std::fabs(second) > maximum) {
        first *= maximum / std::fabs(second);
        second = std::copysign(maximum, second);
    }
    return {first, second};
}

std::pair<float, float> scaleToRatio(float factor, float first, float second) {
    const float largest = std::max(std::fabs(first), std::fabs(second));
    if (largest == 0.0f || factor == 0.0f) return {0.0f, 0.0f};
    const float scale = factor / largest;
    return {first * scale, second * scale};
}

void runOutput(const arc::Drivetrain& drivetrain, float left, float right, bool reverse = false) {
    if (reverse) {
        drivetrain.leftMotors->move(static_cast<int>(clamp(-right, -kMaxOutput, kMaxOutput)));
        drivetrain.rightMotors->move(static_cast<int>(clamp(-left, -kMaxOutput, kMaxOutput)));
    } else {
        drivetrain.leftMotors->move(static_cast<int>(clamp(left, -kMaxOutput, kMaxOutput)));
        drivetrain.rightMotors->move(static_cast<int>(clamp(right, -kMaxOutput, kMaxOutput)));
    }
}

void stop(const arc::Drivetrain& drivetrain) { runOutput(drivetrain, 0.0f, 0.0f); }

float translationSpeed() {
    const arc::Pose velocity = arc::getSpeed(false);
    return std::hypot(velocity.x, velocity.y);
}

float angularSpeed() { return std::fabs(arc::getSpeed(false).theta); }

// A negative exit threshold disables that exit. Disabled exits must not make a
// motion settle immediately; they remain pending until another enabled exit or
// the timeout ends the motion.
bool errorPending(float error, float threshold) {
    return threshold < 0.0f || arc::ExitCondition::error(error, threshold);
}

bool velocityPending(float velocity, float threshold) {
    return threshold < 0.0f || arc::ExitCondition::velocity(velocity, threshold);
}

bool anyEnabledExitPending(bool velocityEnabled, float velocity, float velocityThreshold,
                           bool errorEnabled, float error, float errorThreshold,
                           bool auxiliaryEnabled = false, bool auxiliaryPending = false) {
    const bool anyEnabled = velocityEnabled || errorEnabled || auxiliaryEnabled;
    return !anyEnabled ||
           (velocityEnabled && velocityPending(velocity, velocityThreshold)) ||
           (errorEnabled && errorPending(error, errorThreshold)) ||
           (auxiliaryEnabled && auxiliaryPending);
}

float reducedAngle(float degrees) { return std::remainder(degrees, 360.0f); }

arc::Pose motionPose(bool forwards) {
    arc::Pose pose = arc::getPose(false);
    if (!forwards) pose.theta = reducedAngle(pose.theta + 180.0f);
    return pose;
}

float bearing(const arc::Pose& from, const arc::Pose& to) {
    return reducedAngle(arc::radToDeg(std::atan2(to.x - from.x, to.y - from.y)));
}

float faceError(const arc::Pose& pose, const arc::Pose& target,
                arc::AngularDirection direction = arc::AngularDirection::AUTO) {
    return arc::angleError(bearing(pose, target), pose.theta, false, direction);
}

float parallelError(const arc::Pose& pose, float targetHeading,
                    arc::AngularDirection direction = arc::AngularDirection::AUTO) {
    return arc::angleError(targetHeading, pose.theta, false, direction);
}

float radiusBetween(const arc::Pose& first, const arc::Pose& second, float heading) {
    const float distance = first.distance(second);
    const float error = arc::degToRad(arc::angleError(bearing(first, second), heading, false));
    const float denominator = std::sqrt(std::max(0.0f, 2.0f - 2.0f * std::cos(2.0f * error)));
    if (denominator < 1e-6f) return std::numeric_limits<float>::infinity();
    return distance / denominator;
}

float pathCurvature(const arc::Pose& first, const arc::Pose& second, float heading) {
    const float radius = radiusBetween(first, second, heading);
    if (!std::isfinite(radius) || radius == 0.0f) return 0.0f;
    const float headingRad = arc::degToRad(heading);
    const float sideValue = std::cos(headingRad) * (second.x - first.x) -
                            std::sin(headingRad) * (second.y - first.y);
    return std::copysign(1.0f / radius, sideValue);
}

float pathOutput(float storedSpeed) {
    return clamp(storedSpeed, -kMaxOutput, kMaxOutput);
}

struct PathData {
    std::vector<arc::Pose> points;
    // Optional 4th column (Atticus export, math-standard CCW-positive
    // convention, 1/inch - see mod/pathing.py:calculate_path_curvature in
    // the Atticus repo). Empty for older 3-column (x, y, speed) paths.
    std::vector<float> curvature;

    bool valid() const { return points.size() >= 2; }
    bool hasCurvature() const { return curvature.size() == points.size(); }
    int lastIndex() const { return static_cast<int>(points.size()) - 1; }
    const arc::Pose& at(int index) const { return points.at(std::clamp(index, 0, lastIndex())); }
    float curvatureAt(int index) const { return curvature.at(std::clamp(index, 0, lastIndex())); }

    int closestIndex(const arc::Pose& pose) const {
        int result = 0;
        float closest = std::numeric_limits<float>::infinity();
        for (int index = 0; index <= lastIndex(); ++index) {
            const float distance = pose.distance(points[index]);
            if (distance < closest) {
                closest = distance;
                result = index;
            }
        }
        return result;
    }

    arc::Pose lookaheadPoint(const arc::Pose& pose, int startIndex, float lookahead) const {
        for (int index = std::max(0, startIndex); index <= lastIndex(); ++index) {
            if (pose.distance(points[index]) >= lookahead) return points[index];
        }
        return points.back();
    }

    bool nearEnd(int index) const { return lastIndex() - index <= 5; }
};

PathData readPath(const asset& path) {
    PathData result;
    const std::string data(reinterpret_cast<const char*>(path.buf), path.size);
    std::istringstream lines(data);
    std::string line;
    bool sawCurvatureColumn = false;
    while (std::getline(lines, line)) {
        if (line == "endData" || line == "endData\r") break;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream values(line);
        float x = 0.0f;
        float y = 0.0f;
        float speed = 0.0f;
        float curvature = 0.0f;
        if (values >> x >> y >> speed) {
            result.points.emplace_back(x, y, speed);
            // A 4th column is optional - present on paths exported by the
            // Atticus curvature update, absent on older 3-column paths.
            if (values >> curvature) sawCurvatureColumn = true;
            result.curvature.push_back(curvature);
        }
    }
    if (!sawCurvatureColumn) result.curvature.clear();
    return result;
}

bool continuePath(const PathData& path, int pathIndex, float velocityExit, float errorExit,
                  const arc::Pose& pose) {
    const bool notAtEnd = pathIndex != path.lastIndex();
    const bool stoppedEnd = pathOutput(path.points.back().theta) == 0.0f &&
                            arc::ExitCondition::velocity(translationSpeed(), velocityExit);
    const bool outsideError = arc::ExitCondition::error(pose.distance(path.points.back()), errorExit);
    return notAtEnd || stoppedEnd || outsideError;
}

template <typename Params>
void applyExitDefaults(Params& params, const arc::ExitSettings& defaults) {
    if (params.timeout == arc::useProfileTimeout) params.timeout = defaults.timeout;
    if (params.velocityExit == arc::useProfileExit) params.velocityExit = defaults.velocityExit;
    if (params.errorExit == arc::useProfileExit) params.errorExit = defaults.errorExit;
}

template <typename Params>
void applySettleDefaults(Params& params, const arc::ExitSettings& defaults) {
    if (params.settleTimeMs == arc::useProfileSettleTime) {
        params.settleTimeMs = defaults.settleTimeMs;
    }
}

class SettleTimer {
  public:
    bool pending(bool exitPending, int settleTimeMs) {
        if (exitPending) {
            timing_ = false;
            return true;
        }
        if (settleTimeMs <= 0) return false;
        if (!timing_) {
            timing_ = true;
            settledSince_ = pros::millis();
        }
        return pros::millis() - settledSince_ < static_cast<std::uint32_t>(settleTimeMs);
    }

  private:
    bool timing_ = false;
    std::uint32_t settledSince_ = 0;
};

int turnSides(arc::LockedSide lockedSide) {
    if (lockedSide == arc::LockedSide::LEFT) return 1;
    if (lockedSide == arc::LockedSide::RIGHT) return 0;
    return 2;
}

template <typename Params>
void applyHalfPlaneDefaults(Params& params, const arc::ExitSettings& defaults) {
    if (params.halfPlaneExit == arc::useProfileHalfPlane) {
        params.halfPlaneExit = defaults.halfPlaneExit ? 1 : 0;
    }
    if (params.halfPlaneTolerance == arc::useProfileExit) {
        params.halfPlaneTolerance = defaults.halfPlaneTolerance;
    }
}

} // namespace

namespace arc {

void Chassis::tank(int left, int right) {
    drivetrain.leftMotors->move(std::clamp(left, -127, 127));
    drivetrain.rightMotors->move(std::clamp(right, -127, 127));
}

void Chassis::arcade(int throttle, int turn, float desaturateBias) {
    if (std::abs(throttle) + std::abs(turn) > 127) {
        const int originalThrottle = throttle;
        const int originalTurn = turn;
        throttle = static_cast<int>(throttle * (1.0f - desaturateBias * std::fabs(originalTurn / 127.0f)));
        turn = static_cast<int>(turn * (1.0f - (1.0f - desaturateBias) * std::fabs(originalThrottle / 127.0f)));
    }
    const auto output = reduceRatio(127.0f, throttle + turn, throttle - turn);
    tank(static_cast<int>(output.first), static_cast<int>(output.second));
}

void Chassis::curvature(int throttle, int turn) {
    if (throttle == 0) {
        arcade(0, turn);
        return;
    }

    const float steering = clamp(turn * 90.0f / 127.0f, -90.0f, 90.0f);
    if (std::fabs(steering) >= 89.999f) {
        tank(static_cast<int>(std::copysign(std::fabs(throttle), steering)),
             static_cast<int>(std::copysign(std::fabs(throttle), -steering)));
        return;
    }

    const auto output = scaleToRatio(static_cast<float>(throttle),
                                     2.0f + std::tan(degToRad(steering)) / 2.0f,
                                     2.0f - std::tan(degToRad(steering)) / 2.0f);
    tank(static_cast<int>(output.first), static_cast<int>(output.second));
}

void Chassis::moveDistance(float distance, MoveDistanceParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, distance, params] { moveDistance(distance, params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const float startDistance = sensors.vertical1 == nullptr ? 0.0f : sensors.vertical1->getDistanceTraveled();
    const float target = startDistance + distance;
    float error = distance;

    lateralPID.reset(error);
    lateralPID.setTarget(error);
    const float initialHeadingError = angleError(headingTarget, getPose().theta, false);
    headingPID.reset(initialHeadingError);
    headingPID.setTarget(initialHeadingError);
    distTraveled = 0.0f;

    do {
        const float position = sensors.vertical1 == nullptr ? startDistance : sensors.vertical1->getDistanceTraveled();
        error = target - position;

        float lateralOutput = clamp(lateralPID.update(error), -speedOutput(params.maxSpeed), speedOutput(params.maxSpeed));
        const float minimum = speedOutput(params.minSpeed);
        if (minimum > 0.0f && std::fabs(lateralOutput) < minimum) lateralOutput = std::copysign(minimum, lateralOutput);

        const float headingError = angleError(headingTarget, getPose().theta, false);
        const float headingOutput = clamp(headingPID.update(headingError),
                                          -speedOutput(params.maxSpeed), speedOutput(params.maxSpeed));
        const auto output = reduceRatio(speedOutput(params.maxSpeed),
                                        lateralOutput + headingOutput, lateralOutput - headingOutput);
        runOutput(drivetrain, output.first, output.second);
        distTraveled = std::fabs(position - startDistance);
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             anyEnabledExitPending(params.velocityExit >= 0.0f, translationSpeed(), params.velocityExit,
                                   params.errorExit >= 0.0f, error, params.errorExit));

    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::push(float speed, PushParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, speed, params] { push(speed, params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const float initialHeadingError = angleError(headingTarget, getPose().theta, false);
    headingPID.reset(initialHeadingError);
    headingPID.setTarget(initialHeadingError);
    const float lateralOutput = std::copysign(speedOutput(speed), speed);
    distTraveled = 0.0f;
    Pose previous = getPose();

    do {
        const float headingOutput = headingPID.update(angleError(headingTarget, getPose().theta, false));
        const auto output = reduceRatio(std::fabs(lateralOutput),
                                        lateralOutput + headingOutput, lateralOutput - headingOutput);
        runOutput(drivetrain, output.first, output.second);
        const Pose current = getPose();
        distTraveled += current.distance(previous);
        previous = current;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             velocityPending(translationSpeed(), params.velocityExit) &&
             (sensors.imu == nullptr || std::fabs(sensors.imu->get_roll()) < 10.0f));

    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::pushTo(float coordinate, bool isY, float speed, PushParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, coordinate, isY, speed, params] { pushTo(coordinate, isY, speed, params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const Pose startPose = getPose();
    const bool startSide = (isY ? startPose.y : startPose.x) > coordinate;
    const float initialHeadingError = angleError(headingTarget, startPose.theta, false);
    headingPID.reset(initialHeadingError);
    headingPID.setTarget(initialHeadingError);
    const float lateralOutput = std::copysign(speedOutput(speed), speed);
    distTraveled = 0.0f;
    Pose previous = startPose;
    bool currentSide = startSide;
    float coordinateError = coordinate - (isY ? startPose.y : startPose.x);

    do {
        const Pose current = getPose();
        const float currentCoordinate = isY ? current.y : current.x;
        currentSide = currentCoordinate > coordinate;
        coordinateError = coordinate - currentCoordinate;

        const float headingOutput = headingPID.update(angleError(headingTarget, current.theta, false));
        const auto output = reduceRatio(std::fabs(lateralOutput),
                                        lateralOutput + headingOutput, lateralOutput - headingOutput);
        runOutput(drivetrain, output.first, output.second);
        distTraveled += current.distance(previous);
        previous = current;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             (((ExitCondition::velocity(translationSpeed(), params.velocityExit) &&
                (sensors.imu == nullptr || std::fabs(sensors.imu->get_roll()) < 10.0f)) ||
               currentSide == startSide) || ExitCondition::error(coordinateError, params.errorExit)));

    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::push(float coordinate, bool isY, float speed, PushParams params) {
    pushTo(coordinate, isY, speed, params);
}

void Chassis::crossBarrier(CrossBarrierParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, params] { crossBarrier(params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const float initialHeadingError = angleError(headingTarget, getPose().theta, false);
    headingPID.reset(initialHeadingError);
    headingPID.setTarget(initialHeadingError);
    bool reachedMinimum = false;
    bool reachedMaximum = false;
    float roll = 0.0f;
    const float lateralOutput = std::copysign(speedOutput(params.speed), params.speed);

    do {
        roll = sensors.imu == nullptr ? 0.0f : sensors.imu->get_roll();
        if (roll < -10.0f) reachedMinimum = true;
        if (roll > 10.0f) reachedMaximum = true;

        const float headingOutput = headingPID.update(angleError(headingTarget, getPose().theta, false));
        const auto output = reduceRatio(std::fabs(lateralOutput),
                                        lateralOutput + headingOutput, lateralOutput - headingOutput);
        runOutput(drivetrain, output.first, output.second);
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             (!reachedMinimum || !reachedMaximum || ExitCondition::error(roll + 5.0f, params.errorExit)));

    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::turnHeadingMotion(float theta, int sides, AngularDirection direction, float maxSpeed, float minSpeed,
                                int timeout, float velocityExit, float errorExit, int settleTimeMs) {
    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    float error = angleError(theta, getPose().theta, false, direction);
    angularPID.reset(error);
    angularPID.setTarget(sides == 2 ? error : error / 2.5f);
    float previousHeading = getPose().theta;
    SettleTimer settleTimer;
    distTraveled = 0.0f;

    do {
        const float heading = getPose().theta;
        error = angleError(theta, heading, false, direction);
        float output = clamp(angularPID.update(error), -speedOutput(maxSpeed), speedOutput(maxSpeed));
        const float minimum = speedOutput(minSpeed);
        if (minimum > 0.0f && std::fabs(output) < minimum) output = std::copysign(minimum, output);

        if (sides == 0) runOutput(drivetrain, output, 0.0f);
        else if (sides == 1) runOutput(drivetrain, 0.0f, -output);
        else runOutput(drivetrain, output, -output);

        distTraveled += std::fabs(angleError(heading, previousHeading, false));
        previousHeading = heading;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, timeout) &&
             settleTimer.pending(
                 anyEnabledExitPending(minSpeed == 0.0f && velocityExit >= 0.0f,
                                       angularSpeed(), velocityExit,
                                       errorExit >= 0.0f, error, errorExit),
                 settleTimeMs));

    headingTarget = reducedAngle(theta);
    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::turnPointMotion(float x, float y, bool forwards, int sides, AngularDirection direction,
                              float maxSpeed, float minSpeed, int timeout, float velocityExit, float errorExit,
                              int settleTimeMs) {
    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const Pose target(x, y);
    Pose pose = motionPose(forwards);
    float error = faceError(pose, target, direction);
    angularPID.reset(error);
    angularPID.setTarget(sides == 2 ? error : error / 2.5f);
    float previousHeading = pose.theta;
    SettleTimer settleTimer;
    distTraveled = 0.0f;

    do {
        pose = motionPose(forwards);
        error = faceError(pose, target, direction);
        float output = clamp(angularPID.update(error), -speedOutput(maxSpeed), speedOutput(maxSpeed));
        const float minimum = speedOutput(minSpeed);
        if (minimum > 0.0f && std::fabs(output) < minimum) output = std::copysign(minimum, output);

        if (sides == 0) runOutput(drivetrain, output, 0.0f);
        else if (sides == 1) runOutput(drivetrain, 0.0f, -output);
        else runOutput(drivetrain, output, -output);

        distTraveled += std::fabs(angleError(pose.theta, previousHeading, false));
        previousHeading = pose.theta;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, timeout) &&
             settleTimer.pending(
                 anyEnabledExitPending(minSpeed == 0.0f && velocityExit >= 0.0f,
                                       angularSpeed(), velocityExit,
                                       errorExit >= 0.0f, error, errorExit),
                 settleTimeMs));

    headingTarget = getPose().theta;
    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::turnToHeading(float theta, TurnToHeadingParams params) {
    applyExitDefaults(params, angularSettings.exits);
    applySettleDefaults(params, angularSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, theta, params] { turnToHeading(theta, params); });
        return;
    }
    turnHeadingMotion(theta, turnSides(params.lockedSide), params.direction, params.maxSpeed, params.minSpeed,
                      params.timeout, params.velocityExit, params.errorExit, params.settleTimeMs);
}

void Chassis::turnToPoint(float x, float y, TurnToPointParams params) {
    applyExitDefaults(params, angularSettings.exits);
    applySettleDefaults(params, angularSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, x, y, params] { turnToPoint(x, y, params); });
        return;
    }
    turnPointMotion(x, y, params.forwards, turnSides(params.lockedSide), params.direction,
                    params.maxSpeed, params.minSpeed,
                    params.timeout, params.velocityExit, params.errorExit, params.settleTimeMs);
}

void Chassis::turnToPointStep(float x, float y, TurnToPointParams params) {
    applyExitDefaults(params, angularSettings.exits);
    const Pose pose = motionPose(params.forwards);
    const Pose target(x, y);
    const float error = faceError(pose, target, params.direction);
    float output = clamp(angularPID.update(error), -speedOutput(params.maxSpeed), speedOutput(params.maxSpeed));
    const float minimum = speedOutput(params.minSpeed);
    if (minimum > 0.0f && std::fabs(output) < minimum) output = std::copysign(minimum, output);
    const int sides = turnSides(params.lockedSide);
    if (sides == 0) runOutput(drivetrain, output, 0.0f);
    else if (sides == 1) runOutput(drivetrain, 0.0f, -output);
    else runOutput(drivetrain, output, -output);
    headingTarget = getPose().theta;
}

void Chassis::moveToPoint(float x, float y, MoveToPointParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    applyHalfPlaneDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, x, y, params] { moveToPoint(x, y, params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const Pose target(x, y);
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    float distanceError = pose.distance(target);
    float angularError = faceError(pose, target);
    lateralPID.reset(distanceError);
    lateralPID.setTarget(kInchesPerFoot);
    angularPID.reset(angularError);
    angularPID.setTarget(180.0f);

    const float tolerance = params.halfPlaneTolerance >= 0.0f
                                ? params.halfPlaneTolerance
                                : (params.minSpeed == 0.0f ? 2.4f : 0.0f);
    bool lastHalfPlane = ExitCondition::halfPlane(pose, target, pose.theta, tolerance);
    bool crossed = params.errorExit >= 0.0f && distanceError <= params.errorExit;
    distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        const bool currentHalfPlane = ExitCondition::halfPlane(pose, target, pose.theta, tolerance);
        if (!currentHalfPlane && lastHalfPlane) crossed = true;
        lastHalfPlane = currentHalfPlane;

        distanceError = pose.distance(target);
        angularError = faceError(pose, target);

        float distanceOutput = lateralPID.update(distanceError);
        if (params.settle && distanceError < 8.4f) distanceOutput *= std::cos(degToRad(angularError));
        distanceOutput = std::copysign(clamp(std::fabs(distanceOutput), speedOutput(params.minSpeed), kMaxOutput),
                                       distanceOutput);

        float angularOutput = angularPID.update(angularError);
        if (distanceError < 6.0f) angularOutput = 0.0f;
        angularOutput = clamp(angularOutput, -kMaxOutput, kMaxOutput);

        const float excess = std::fabs(distanceOutput) + std::fabs(angularOutput) - kMaxOutput;
        if (excess > 0.0f) distanceOutput -= std::copysign(excess, distanceOutput);

        const auto output = reduceRatio(speedOutput(params.maxSpeed),
                                        distanceOutput + angularOutput, distanceOutput - angularOutput);
        runOutput(drivetrain, output.first, output.second, !params.forwards);

        distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             anyEnabledExitPending(params.minSpeed == 0.0f && params.velocityExit >= 0.0f,
                                   translationSpeed(), params.velocityExit,
                                   params.errorExit >= 0.0f, distanceError, params.errorExit,
                                   params.halfPlaneExit, !crossed));

    headingTarget = getPose().theta;
    if (params.minSpeed == 0.0f || !motionRunning) stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::moveToPose(float x, float y, float theta, MoveToPoseParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    applyHalfPlaneDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        pros::Task task([this, x, y, theta, params] { moveToPose(x, y, theta, params); });
        return;
    }

    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    const float requestedHeading = theta;
    if (!params.forwards) theta = reducedAngle(theta + 180.0f);

    const Pose end(x, y, theta);
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    Pose carrot(x - std::sin(degToRad(theta)) * params.dLead,
                y - std::cos(degToRad(theta)) * params.dLead);
    const Pose initialCarrot = carrot;
    Pose target = carrot;

    float minimumCarrotDistance = pose.distance(carrot);
    const float initialCarrotDistance = minimumCarrotDistance;
    float distanceError = pose.distance(end);
    float angularError = faceError(pose, carrot);
    lateralPID.reset(pose.distance(carrot));
    lateralPID.setTarget(kInchesPerFoot);
    angularPID.reset(angularError);
    angularPID.setTarget(180.0f);

    const float tolerance = params.halfPlaneTolerance >= 0.0f
                                ? params.halfPlaneTolerance
                                : (params.minSpeed == 0.0f ? 2.4f : 0.0f);
    bool lastHalfPlane = ExitCondition::halfPlane(pose, end, theta, tolerance);
    bool crossed = params.errorExit >= 0.0f && distanceError <= params.errorExit;
    bool closeEnd = false;
    bool closeGhost = false;
    distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        minimumCarrotDistance = std::min(minimumCarrotDistance, pose.distance(carrot));
        const float ratio = initialCarrotDistance < 1e-6f ? 0.0f : minimumCarrotDistance / initialCarrotDistance;
        carrot = Pose(x - ratio * std::sin(degToRad(theta)) * params.dLead,
                      y - ratio * std::cos(degToRad(theta)) * params.dLead);
        const Pose ghost(initialCarrot.x + (carrot.x - initialCarrot.x) * (1.0f - params.gLead),
                         initialCarrot.y + (carrot.y - initialCarrot.y) * (1.0f - params.gLead));

        const bool currentHalfPlane = ExitCondition::halfPlane(pose, end, theta, tolerance);
        if (!currentHalfPlane && lastHalfPlane) crossed = true;
        lastHalfPlane = currentHalfPlane;

        distanceError = pose.distance(end);
        if (pose.distance(carrot) < 8.4f || closeEnd) {
            closeEnd = true;
            if (distanceError < 8.4f) angularError = parallelError(pose, theta);
            else angularError = faceError(pose, end);
            target = end;
        } else if (pose.distance(ghost) / kInchesPerFoot <
                       2.0f * clamp(std::fabs(params.maxSpeed), 0.0f, 127.0f) / 127.0f || closeGhost) {
            closeGhost = true;
            angularError = faceError(pose, carrot);
            target = carrot;
        } else {
            angularError = faceError(pose, ghost);
            target = ghost;
        }

        float angularOutput = clamp(angularPID.update(angularError), -kMaxOutput, kMaxOutput);
        float distanceOutput = (closeEnd || distanceError > pose.distance(carrot))
                                   ? lateralPID.update(distanceError)
                                   : lateralPID.update(pose.distance(carrot));

        if (params.settle && closeEnd && distanceError < 8.4f) {
            distanceOutput *= std::cos(degToRad(faceError(pose, target)));
        }
        distanceOutput = std::copysign(clamp(std::fabs(distanceOutput), speedOutput(params.minSpeed), kMaxOutput),
                                       distanceOutput);

        if (params.chasePower >= 0.0f && (!closeEnd || distanceError >= 8.4f)) {
            const float radiusFeet = radiusBetween(pose, target, pose.theta) / kInchesPerFoot;
            const float distanceFeet = pose.distance(target) / kInchesPerFoot;
            const float normalizedRadius = radiusFeet / std::max(1e-6f, std::min(distanceFeet, 1.0f));
            const float maximumSlip = std::sqrt(std::max(0.0f,
                params.chasePower * normalizedRadius * kChaseOutputScale));
            distanceOutput = clamp(distanceOutput, -maximumSlip, maximumSlip);
        }

        const float excess = std::fabs(distanceOutput) + std::fabs(angularOutput) - kMaxOutput;
        if (excess > 0.0f) distanceOutput -= std::copysign(excess, distanceOutput);

        const auto output = reduceRatio(speedOutput(params.maxSpeed),
                                        distanceOutput + angularOutput, distanceOutput - angularOutput);
        runOutput(drivetrain, output.first, output.second, !params.forwards);

        distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             anyEnabledExitPending(params.minSpeed == 0.0f && params.velocityExit >= 0.0f,
                                   translationSpeed(), params.velocityExit,
                                   params.errorExit >= 0.0f, distanceError, params.errorExit,
                                   params.halfPlaneExit, !crossed));

    headingTarget = reducedAngle(requestedHeading);
    if (params.minSpeed == 0.0f || !motionRunning) stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::follow(const asset& path, PursuitParams params) { followPursuit(path, params); }

void Chassis::followPursuit(const asset& pathAsset, PursuitParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        const asset path = pathAsset;
        pros::Task task([this, path, params] { followPursuit(path, params); });
        return;
    }

    const PathData path = readPath(pathAsset);
    if (!path.valid()) return;
    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    int pathIndex = 0;
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        pathIndex = path.closestIndex(pose);
        const float crosstrackFeet = pose.distance(path.at(pathIndex)) / kInchesPerFoot;
        const float radiusFeet = radiusBetween(pose, path.at(pathIndex + params.radiusLookahead), pose.theta) /
                                 kInchesPerFoot;
        const float speedFeet = translationSpeed() / kInchesPerFoot;
        const float lookaheadFeet = clamp(speedFeet * params.velocityLookahead +
                                          radiusFeet * params.curvatureLookahead -
                                          crosstrackFeet * params.crosstrackLookahead,
                                          params.minLookahead / kInchesPerFoot,
                                          params.maxLookahead / kInchesPerFoot);
        const float lookahead = lookaheadFeet * kInchesPerFoot;
        const Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, lookahead);

        const float turnError = faceError(pose, lookaheadPoint);
        float outputScale = pathOutput(path.at(pathIndex).theta);
        if (std::fabs(turnError) >= 90.0f) {
            const float direction = std::copysign(1.0f, turnError);
            runOutput(drivetrain, outputScale * direction, outputScale * -direction, !params.forwards);
        } else {
            if (params.chasePower >= 0.0f && lookaheadFeet > 1e-6f) {
                const float turnRadiusFeet = radiusBetween(pose, lookaheadPoint, pose.theta) / kInchesPerFoot;
                outputScale = std::min(std::sqrt(std::max(0.0f,
                    params.chasePower * turnRadiusFeet / lookaheadFeet * kChaseOutputScale)), outputScale);
            }
            const float pursuitCurvature = pathCurvature(pose, lookaheadPoint, pose.theta);
            const auto output = scaleToRatio(outputScale,
                                             2.0f + pursuitCurvature * drivetrain.trackWidth,
                                             2.0f - pursuitCurvature * drivetrain.trackWidth);
            runOutput(drivetrain, output.first, output.second, !params.forwards);
        }

        distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             continuePath(path, pathIndex, params.velocityExit, params.errorExit, pose));

    headingTarget = getPose().theta;
    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::followStanley(const asset& pathAsset, StanleyParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        const asset path = pathAsset;
        pros::Task task([this, path, params] { followStanley(path, params); });
        return;
    }

    const PathData path = readPath(pathAsset);
    if (!path.valid()) return;
    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    const std::uint32_t startTime = pros::millis();
    int pathIndex = 0;
    int furthestIndex = 0;
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        pathIndex = path.closestIndex(pose);
        furthestIndex = std::max(furthestIndex, pathIndex);
        pathIndex = furthestIndex;

        const Pose current = path.at(pathIndex);
        const Pose next = path.at(pathIndex + 1);
        const Pose prior = path.at(pathIndex - 1);
        const float pathHeading = pathIndex == path.lastIndex() ? bearing(prior, current) : bearing(current, next);
        const float crossTrack = std::copysign(pose.distance(current),
                                               -pathCurvature(current, pose, pathHeading));
        const float headingError = parallelError(pose, pathHeading);
        float speedFeet = translationSpeed() / kInchesPerFoot;
        if (std::fabs(speedFeet) < 1e-3f) speedFeet = 1e-3f;
        const float crossTrackScale = radToDeg(std::atan(params.k * (crossTrack / kInchesPerFoot) / speedFeet));
        const float steering = 5.0f * headingError + crossTrackScale;
        float outputScale = pathOutput(current.theta);

        if (std::fabs(steering) >= 90.0f) {
            const float direction = std::copysign(1.0f, steering);
            runOutput(drivetrain, outputScale * direction, outputScale * -direction, !params.forwards);
        } else {
            if (params.chasePower >= 0.0f) {
                const float tangent = std::tan(degToRad(std::fabs(steering)));
                const float radiusFeet = std::fabs(tangent) < 1e-6f
                                             ? std::numeric_limits<float>::infinity()
                                             : drivetrain.trackWidth / kInchesPerFoot / tangent;
                outputScale = std::min(std::sqrt(std::max(0.0f,
                    params.chasePower * radiusFeet * kChaseOutputScale)), outputScale);
            }
            const auto output = scaleToRatio(outputScale,
                                             2.0f + std::tan(degToRad(steering)) / 2.0f,
                                             2.0f - std::tan(degToRad(steering)) / 2.0f);
            runOutput(drivetrain, output.first, output.second, !params.forwards);
        }

        distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             continuePath(path, pathIndex, params.velocityExit, params.errorExit, pose));

    headingTarget = getPose().theta;
    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

void Chassis::followAPS(const asset& pathAsset, APSParams params) {
    applyExitDefaults(params, lateralSettings.exits);
    if (params.async) {
        params.async = false;
        const asset path = pathAsset;
        pros::Task task([this, path, params] { followAPS(path, params); });
        return;
    }

    const PathData path = readPath(pathAsset);
    if (!path.valid()) return;
    requestMotionStart();
    if (!motionRunning) {
        endMotion();
        return;
    }

    angularPID.reset(0.0f);
    angularPID.setTarget(0.0f);
    const std::uint32_t startTime = pros::millis();
    int pathIndex = 0;
    int furthestIndex = 0;
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        pathIndex = path.closestIndex(pose);
        furthestIndex = std::max(furthestIndex, pathIndex);
        pathIndex = furthestIndex;

        const Pose current = path.at(pathIndex);
        const Pose next = path.at(pathIndex + 1);
        const Pose prior = path.at(pathIndex - 1);
        float pathHeading = 0.0f;
        if (pathIndex == 0) pathHeading = bearing(current, next);
        else if (pathIndex == path.lastIndex()) pathHeading = bearing(prior, current);
        else {
            const float priorHeading = bearing(prior, current);
            pathHeading = reducedAngle(priorHeading +
                angleError(bearing(current, next), priorHeading, false) / 2.0f);
        }

        const float crossTrackFeet = pose.distance(current) / kInchesPerFoot;
        float angularTerm = parallelError(pose, pathHeading);
        const float denominator = std::pow(25.0f * crossTrackFeet, 2.0f);
        if (denominator > 1e-6f) {
            angularTerm = std::copysign(std::min(std::fabs(angularTerm / denominator), std::fabs(angularTerm)),
                                        angularTerm);
        }
        const float crossTrackTerm = path.nearEnd(pathIndex)
                                         ? 0.0f
                                         : 10.0f * crossTrackFeet * std::copysign(1.0f, faceError(pose, current));
        const float headingOutput = clamp(angularPID.update(angularTerm + crossTrackTerm),
                                          -kMaxOutput, kMaxOutput);

        float lateralOutput = pathOutput(current.theta);
        if (params.chasePower >= 0.0f) {
            const Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, 1.8f * kInchesPerFoot);
            const float radiusFeet = radiusBetween(pose, lookaheadPoint, pose.theta) / kInchesPerFoot;
            lateralOutput = std::min(std::sqrt(std::max(0.0f,
                params.chasePower * radiusFeet * kChaseOutputScale)), lateralOutput);
        }

        const float excess = std::fabs(lateralOutput) + std::fabs(headingOutput) - kMaxOutput;
        if (excess > 0.0f) lateralOutput -= std::copysign(excess, lateralOutput);
        const auto output = reduceRatio(speedOutput(params.maxSpeed),
                                        lateralOutput + headingOutput, lateralOutput - headingOutput);
        runOutput(drivetrain, output.first, output.second, !params.forwards);

        distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (motionRunning && ExitCondition::time(startTime, params.timeout) &&
             continuePath(path, pathIndex, params.velocityExit, params.errorExit, pose));

    headingTarget = getPose().theta;
    stop(drivetrain);
    distTraveled = -1.0f;
    endMotion();
}

// Real implementation - declared as a friend of Chassis (chassis.hpp) so it
// can reach the same protected motion-queueing state (motionRunning,
// requestMotionStart()/endMotion(), distTraveled, headingTarget) and
// drivetrain/lateralSettings every other motion in this file uses.
// Chassis::followRamseteLQR() below just forwards here.
void followRamseteLQR(Chassis& chassis, const asset& pathAsset, RamseteLQRParams params) {
    applyExitDefaults(params, chassis.lateralSettings.exits);
    if (params.async) {
        params.async = false;
        const asset path = pathAsset;
        pros::Task task([&chassis, path, params] { followRamseteLQR(chassis, path, params); });
        return;
    }

    const PathData path = readPath(pathAsset);
    if (!path.valid()) return;
    chassis.requestMotionStart();
    if (!chassis.motionRunning) {
        chassis.endMotion();
        return;
    }

    // The exported path's speed column is a -127..127 duty value, same as
    // every other follow*() here. arc::path controllers work in real
    // inches/second and radians/second, so this is the conversion factor
    // between the two: the chassis's theoretical top wheel speed.
    const float maxLinearInPerSec = chassis.drivetrain.wheelDiameter * static_cast<float>(M_PI) *
                                    chassis.drivetrain.rpm / 60.0f;

    arc::path::Limits limits;
    limits.maxLinear = maxLinearInPerSec;
    limits.maxAngular = maxLinearInPerSec / std::max(1e-3f, chassis.drivetrain.trackWidth * 0.5f);
    // Generous - clampControl's job here is just to keep the math finite;
    // real acceleration limiting already happens in the drivetrain itself.
    limits.maxLinearAcceleration = limits.maxLinear * 6.0f;
    limits.maxAngularAcceleration = limits.maxAngular * 6.0f;

    arc::path::RamseteLQRConfig config;
    config.b = params.b;
    config.zeta = params.zeta;
    config.velocityTimeConstant = params.velocityTimeConstant;
    config.linearVelocityTolerance = params.linearVelocityTolerance;
    config.angularVelocityTolerance = params.angularVelocityTolerance;
    config.linearCommandTolerance = params.linearCommandTolerance;
    config.angularCommandTolerance = params.angularCommandTolerance;
    config.limits = limits;
    arc::path::RamseteLQRController controller(config);

    const std::uint32_t startTime = pros::millis();
    std::uint32_t previousTime = startTime;
    int pathIndex = 0;
    int furthestIndex = 0;
    Pose pose = motionPose(params.forwards);
    Pose previous = pose;
    chassis.distTraveled = 0.0f;

    do {
        pose = motionPose(params.forwards);
        pathIndex = path.closestIndex(pose);
        furthestIndex = std::max(furthestIndex, pathIndex);
        pathIndex = furthestIndex;

        const Pose current = path.at(pathIndex);
        const Pose next = path.at(pathIndex + 1);
        const Pose prior = path.at(pathIndex - 1);
        const float pathHeadingDeg = pathIndex == path.lastIndex() ? bearing(prior, current) : bearing(current, next);
        const float refSpeedDuty = current.theta; // stored -127..127 target speed at this waypoint
        const float refLinearInPerSec = refSpeedDuty / kMaxOutput * maxLinearInPerSec;
        // Prefer the path's own exported curvature column when present
        // (Atticus, math-standard CCW-positive - the same convention
        // arc::path already uses, so it plugs in directly with no sign
        // flip). Fall back to a two-point estimate for older 3-column
        // paths: pathCurvature is signed in this file's compass convention
        // (positive = clockwise/right, same as bearing()), the opposite of
        // arc::path's, so that estimate is negated to match.
        const float refAngularRadPerSec = path.hasCurvature()
            ? refLinearInPerSec * path.curvatureAt(pathIndex)
            : -refLinearInPerSec * pathCurvature(current, next, pathHeadingDeg);

        // --- Coordinate conversion -------------------------------------
        // arc::Chassis poses are compass-style: x = east, y = north/forward,
        // theta = degrees clockwise from +y (north). arc::path assumes the
        // standard unicycle frame: x = forward, y = left, theta = radians
        // counter-clockwise from +x. x/y stay the same field inches in both
        // frames; only theta and angular velocity need converting:
        //   theta_path = 90deg - theta_compass      (both in the same axes)
        //   omega_path = -omega_compass
        // Verified against both endpoints: compass 0 deg (facing +y) maps to
        // theta_path = 90 deg, i.e. motion along +y - matches; compass 90 deg
        // (facing +x) maps to theta_path = 0 deg, motion along +x - matches.
        const float poseThetaRad = degToRad(90.0f - pose.theta);
        const float refThetaRad = degToRad(90.0f - pathHeadingDeg);
        const Pose angularSpeedPose = getSpeed(false); // degrees/sec, compass convention
        const float poseOmegaRadPerSec = -degToRad(angularSpeedPose.theta);

        arc::path::State state;
        state.pose = {pose.x, pose.y, poseThetaRad};
        // translationSpeed() is an unsigned magnitude; !forwards already
        // flips pose.theta by 180 deg via motionPose() the same way every
        // other motion here handles reverse driving, so this stays positive.
        state.linearVelocity = translationSpeed();
        state.angularVelocity = poseOmegaRadPerSec;

        arc::path::Reference reference;
        reference.pose = {current.x, current.y, refThetaRad};
        reference.velocity = {refLinearInPerSec, refAngularRadPerSec};
        const arc::path::Horizon horizon(1, reference);

        const std::uint32_t now = pros::millis();
        const double dt = std::max(0.001, (now - previousTime) / 1000.0);
        previousTime = now;

        const arc::path::Control command = controller.calculate(state, horizon, dt);

        // No sign flip needed here: command.linear/angular are already in
        // arc::path's standard frame, which is exactly what this differential
        // -drive wheel-speed formula expects (positive angular = CCW = turn
        // left = right wheel faster).
        const float halfTrack = chassis.drivetrain.trackWidth * 0.5f;
        const float wheelOmegaInPerSec = static_cast<float>(command.angular) * halfTrack;
        const float leftInPerSec = static_cast<float>(command.linear) - wheelOmegaInPerSec;
        const float rightInPerSec = static_cast<float>(command.linear) + wheelOmegaInPerSec;

        const float leftOutput = leftInPerSec / maxLinearInPerSec * kMaxOutput;
        const float rightOutput = rightInPerSec / maxLinearInPerSec * kMaxOutput;
        const auto output = reduceRatio(speedOutput(params.maxSpeed), leftOutput, rightOutput);
        runOutput(chassis.drivetrain, output.first, output.second, !params.forwards);

        chassis.distTraveled += pose.distance(previous);
        previous = pose;
        pros::delay(10);
    } while (chassis.motionRunning && ExitCondition::time(startTime, params.timeout) &&
             continuePath(path, pathIndex, params.velocityExit, params.errorExit, pose));

    chassis.headingTarget = getPose().theta;
    stop(chassis.drivetrain);
    chassis.distTraveled = -1.0f;
    chassis.endMotion();
}

void Chassis::followRamseteLQR(const asset& path, RamseteLQRParams params) {
    arc::followRamseteLQR(*this, path, params);
}

} // namespace arc
