#include "gen/chassis/chassis.hpp"
#include "gen/chassis/odom.hpp"
#include "gen/features.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <memory>
#include <algorithm>

arc::damp::MotionResult arc::Chassis::followDamp(const damp::Trajectory& path,
                                               const damp::Model& model,damp::MotionParams params) {
#if !ARC_DAMP_ENABLED
    (void)path; (void)model; (void)params;
    return {damp::MotionStatus::disabled};
#else
    using damp::MotionStatus;
    if (!model.valid() || !path.valid() || !params.follower.valid() ||
        params.follower.maxVoltage>12 || params.timeoutMs==0 ||
        params.maxSensorAgeMs==0 || params.maxSensorAgeMs>100 ||
        !drivetrain.leftMotors || !drivetrain.rightMotors) return {MotionStatus::invalidInput};
    // Allocate bounded optimizer storage outside the small PROS task stack.
    auto follower=std::make_unique<damp::Follower>(model,params.follower);
    follower->reset();
    requestMotionStart();
    damp::MotionResult result{MotionStatus::timedOut};
    const auto start=pros::millis();
    auto previous=start;
    auto wake=start;
    distTraveled=0;
    auto last=getOdomSnapshot().pose;
    while (pros::millis()-start<params.timeoutMs) {
        if (!motionRunning) { result.status=MotionStatus::cancelled; break; }
        const auto now=pros::millis();
        const auto snapshot=getOdomSnapshot();
        if (!snapshot.valid || now-snapshot.timestampMs>params.maxSensorAgeMs ||
            !(snapshot.dtSeconds>0 && snapshot.dtSeconds<=.1f)) {
            result.status=MotionStatus::sensorFault; break;
        }
        const auto battery=pros::battery::get_voltage();
        if (battery==PROS_ERR || battery<params.follower.maxVoltage*1000) {
            result.status=MotionStatus::lowBattery; break;
        }
        const auto state=damp::compassState(snapshot.pose.x,snapshot.pose.y,snapshot.pose.theta,
            snapshot.forwardVelocity,snapshot.leftVelocity,snapshot.clockwiseAngularVelocity);
        const double dt=now==previous ? params.follower.stepSeconds : (now-previous)*.001;
        previous=now;
        const auto output=follower->update(state,path,dt);
        result.referenceTime=output.referenceTime;
        if (output.status==damp::FollowerStatus::invalid) { result.status=MotionStatus::invalidInput; break; }
        if (output.status==damp::FollowerStatus::succeeded) { result.status=MotionStatus::succeeded; break; }
        if (!motionRunning) { result.status=MotionStatus::cancelled; break; }
        const auto left=drivetrain.leftMotors->move_voltage(static_cast<int>(std::lround(output.voltage.left*1000)));
        const auto right=drivetrain.rightMotors->move_voltage(static_cast<int>(std::lround(output.voltage.right*1000)));
        if (left==PROS_ERR || right==PROS_ERR) { result.status=MotionStatus::motorFault; break; }
        distTraveled+=std::hypot(snapshot.pose.x-last.x,snapshot.pose.y-last.y);
        last=snapshot.pose;
        pros::Task::delay_until(&wake,static_cast<std::uint32_t>(std::lround(params.follower.stepSeconds*1000)));
    }
    drivetrain.leftMotors->move_voltage(0);
    drivetrain.rightMotors->move_voltage(0);
    headingTarget=getPose().theta;
    endMotion();
    return result;
#endif
}

arc::damp::MotionResult arc::Chassis::logDampCalibration(
    const std::vector<damp::CalibrationStep>& steps,std::FILE* output,double voltageLimit) {
#if !ARC_DAMP_ENABLED
    (void)steps; (void)output; (void)voltageLimit;
    return {damp::MotionStatus::disabled};
#else
    using damp::MotionStatus;
    if (!output || steps.empty() || steps.size()>100 || !std::isfinite(voltageLimit) ||
        voltageLimit<0 || voltageLimit>12 || !drivetrain.leftMotors || !drivetrain.rightMotors)
        return {MotionStatus::invalidInput};
    for (const auto& step:steps)
        if (!damp::finite(step.voltage) || step.durationMs<20 || step.durationMs>10000 ||
            std::max(std::abs(step.voltage.left),std::abs(step.voltage.right))>voltageLimit)
            return {MotionStatus::invalidInput};
    requestMotionStart();
    damp::MotionResult result{MotionStatus::succeeded};
    const auto start=pros::millis();
    std::fprintf(output,"time_s,forward_mps,lateral_mps,yaw_radps,left_volts,right_volts,measured_left_volts,measured_right_volts\n");
    for (const auto& segment:steps) {
        const auto segmentStart=pros::millis();
        while (pros::millis()-segmentStart<segment.durationMs) {
            if (!motionRunning) { result.status=MotionStatus::cancelled; break; }
            const auto snapshot=getOdomSnapshot();
            const auto now=pros::millis();
            if (!snapshot.valid || now-snapshot.timestampMs>50 ||
                !(snapshot.dtSeconds>0 && snapshot.dtSeconds<=.1f)) {
                result.status=MotionStatus::sensorFault; break;
            }
            const auto battery=pros::battery::get_voltage();
            if (battery==PROS_ERR || battery<voltageLimit*1000) { result.status=MotionStatus::lowBattery; break; }
            const auto left=drivetrain.leftMotors->move_voltage(std::lround(segment.voltage.left*1000));
            const auto right=drivetrain.rightMotors->move_voltage(std::lround(segment.voltage.right*1000));
            if (left==PROS_ERR || right==PROS_ERR) { result.status=MotionStatus::motorFault; break; }
            // Record commanded voltage held for the following interval. Actual
            // motor telemetry is included to detect current/thermal limiting.
            const auto state=damp::compassState(0,0,0,snapshot.forwardVelocity,
                                               snapshot.leftVelocity,snapshot.clockwiseAngularVelocity);
            const auto measuredLeft=drivetrain.leftMotors->get_voltage_all();
            const auto measuredRight=drivetrain.rightMotors->get_voltage_all();
            auto mean=[](const std::vector<std::int32_t>& values) {
                if(values.empty()) return NAN;
                double sum=0;
                for(auto value:values) { if(value==PROS_ERR) return NAN; sum+=value; }
                return static_cast<float>(sum/values.size()/1000);
            };
            const auto ml=mean(measuredLeft),mr=mean(measuredRight);
            if(!std::isfinite(ml)||!std::isfinite(mr)) { result.status=MotionStatus::motorFault; break; }
            if (std::fprintf(output,"%.6f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f\n",(now-start)*.001,
                state.forward,state.lateral,state.yaw,segment.voltage.left,segment.voltage.right,ml,mr)<0) {
                result.status=MotionStatus::invalidInput; break;
            }
            pros::delay(10);
        }
        if (result.status!=MotionStatus::succeeded) break;
    }
    drivetrain.leftMotors->move_voltage(0);
    drivetrain.rightMotors->move_voltage(0);
    if(std::fflush(output)!=0) result.status=MotionStatus::invalidInput;
    endMotion();
    return result;
#endif
}
