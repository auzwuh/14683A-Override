#pragma once

#include "gen/asset.hpp"
#include "gen/chassis/trackingWheel.hpp"
#include "gen/exitcondition.hpp"
#include "gen/pid.hpp"
#include "gen/pose.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

namespace arc {

class OdomSensors {
    public:
        OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                    TrackingWheel* horizontal2, pros::Imu* imu);

        TrackingWheel* vertical1;
        TrackingWheel* vertical2;
        TrackingWheel* horizontal1;
        TrackingWheel* horizontal2;
        pros::Imu* imu;
};

class ControllerSettings {
    public:
        ControllerSettings(PIDConfig gains = {}, PIDConfig correctionGains = {}, ExitSettings exits = {})
            : gains(gains), correctionGains(correctionGains), exits(exits) {}

        PIDConfig gains;
        PIDConfig correctionGains;
        ExitSettings exits;
};

class Drivetrain {
    public:
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                   float wheelDiameter, float rpm, float horizontalDrift);

        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors;
        float trackWidth;
        float wheelDiameter;
        float rpm;
        float horizontalDrift;
};

enum class AngularDirection {
    CW_CLOCKWISE,
    CCW_COUNTERCLOCKWISE,
    AUTO
};

enum class LockedSide {
    NONE,
    LEFT,
    RIGHT
};

// Linear coordinates and error thresholds are inches. Linear velocity exits are inches/second.
// Angular coordinates and exits are degrees and degrees/second. Speeds use the PROS -127..127 scale.
struct MoveDistanceParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    float maxSpeed = 127.0f;
    float minSpeed = 0.0f;
    bool async = false;
};

struct PushParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool async = false;
};

struct CrossBarrierParams {
    int timeout = useProfileTimeout;
    float velocityExit = -1.0f;
    float errorExit = 3.0f;
    float speed = 127.0f;
    bool async = false;
};

struct TurnToHeadingParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    int settleTimeMs = useProfileSettleTime;
    AngularDirection direction = AngularDirection::AUTO;
    LockedSide lockedSide = LockedSide::NONE;
    float maxSpeed = 127.0f;
    float minSpeed = 0.0f;
    bool async = false;
};

struct TurnToPointParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    int settleTimeMs = useProfileSettleTime;
    bool forwards = true;
    AngularDirection direction = AngularDirection::AUTO;
    LockedSide lockedSide = LockedSide::NONE;
    float maxSpeed = 127.0f;
    float minSpeed = 0.0f;
    bool async = false;
};

struct MoveToPointParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float maxSpeed = 127.0f;
    float minSpeed = 0.0f;
    std::int8_t halfPlaneExit = useProfileHalfPlane;
    float halfPlaneTolerance = useProfileExit;
    bool settle = true;
    bool async = false;
};

struct MoveToPoseParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float dLead = 0.0f;
    float gLead = 0.0f;
    float maxSpeed = 127.0f;
    float minSpeed = 0.0f;
    std::int8_t halfPlaneExit = useProfileHalfPlane;
    float halfPlaneTolerance = useProfileExit;
    float chasePower = -1.0f;
    bool settle = true;
    bool async = false;
};

struct PursuitParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float minLookahead = 12.0f;
    float maxLookahead = 12.0f;
    float velocityLookahead = 0.2f;
    float curvatureLookahead = 0.4f;
    float crosstrackLookahead = 0.0f;
    int radiusLookahead = 5;
    float chasePower = -1.0f;
    bool async = false;
};

struct StanleyParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float k = 1.0f;
    float chasePower = -1.0f;
    bool async = false;
};

struct APSParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float maxSpeed = 127.0f;
    float chasePower = -1.0f;
    bool async = false;
};

// Uses the same exported-path asset format as followPursuit/followStanley
// (x, y, speed[-127..127] rows). Internally builds a arc::path::Reference
// each tick and runs it through arc::path::RamseteLQRController - see
// docs/path_following_notebook.md for what b/zeta mean and why this
// controller was picked.
struct RamseteLQRParams {
    int timeout = useProfileTimeout;
    float velocityExit = useProfileExit;
    float errorExit = useProfileExit;
    bool forwards = true;
    float maxSpeed = 127.0f;
    float b = 0.00129032f;
    float zeta = 0.7f;
    bool async = false;
};

class Chassis {
    public:
        Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                OdomSensors sensors);

        void calibrate(bool calibrateIMU = true);
        void setPose(float x, float y, float theta, bool radians = false);
        void setPose(Pose pose, bool radians = false);
        Pose getPose(bool radians = false, bool standardPos = false);

        void waitUntil(float dist);
        void waitUntilDone();
        void setBrakeMode(pros::motor_brake_mode_e mode);

        void moveDistance(float distance, MoveDistanceParams params = {});
        void push(float speed, PushParams params = {});
        void push(float coordinate, bool isY, float speed, PushParams params = {});
        void pushTo(float coordinate, bool isY, float speed, PushParams params = {});
        void crossBarrier(CrossBarrierParams params = {});

        void turnToHeading(float theta, TurnToHeadingParams params = {});
        void turnToPoint(float x, float y, TurnToPointParams params = {});
        void turnToPointStep(float x, float y, TurnToPointParams params = {});

        void moveToPoint(float x, float y, MoveToPointParams params = {});
        void moveToPose(float x, float y, float theta, MoveToPoseParams params = {});

        void follow(const asset& path, PursuitParams params = {});
        void followPursuit(const asset& path, PursuitParams params = {});
        void followStanley(const asset& path, StanleyParams params = {});
        void followAPS(const asset& path, APSParams params = {});
        // Thin wrapper - see arc::followRamseteLQR() below, same relationship
        // as Chassis::getPose()/arc::getPose().
        void followRamseteLQR(const asset& path, RamseteLQRParams params = {});

        void tank(int left, int right);
        void arcade(int throttle, int turn, float desaturateBias = 0.5f);
        void curvature(int throttle, int turn);

        void cancelMotion();
        void cancelAllMotions();
        bool isInMotion() const;
        void resetLocalPosition();

        PID lateralPID;
        PID angularPID;
        PID headingPID;

    protected:
        void requestMotionStart();
        void endMotion();

        bool motionRunning = false;
        bool motionQueued = false;
        float distTraveled = 0.0f;
        float headingTarget = 0.0f;

        ControllerSettings lateralSettings;
        ControllerSettings angularSettings;
        Drivetrain drivetrain;
        OdomSensors sensors;

    private:
        void turnHeadingMotion(float theta, int sides, AngularDirection direction, float maxSpeed, float minSpeed,
                               int timeout, float velocityExit, float errorExit, int settleTimeMs);
        void turnPointMotion(float x, float y, bool forwards, int sides, AngularDirection direction,
                             float maxSpeed, float minSpeed, int timeout, float velocityExit, float errorExit,
                             int settleTimeMs);
        pros::Mutex mutex;

        // Needs protected access (drivetrain, motionRunning, distTraveled, ...)
        // the same way every other motion here does - see definition below.
        friend void followRamseteLQR(Chassis& chassis, const asset& path, RamseteLQRParams params);
};

// The real implementation of Chassis::followRamseteLQR(). Free-function form,
// same relationship as arc::getPose()/arc::setPose() in odom.hpp: call it
// either as chassis.followRamseteLQR(path, params) or as
// arc::followRamseteLQR(chassis, path, params) - both reach this function.
void followRamseteLQR(Chassis& chassis, const asset& path, RamseteLQRParams params = {});

} // namespace arc
