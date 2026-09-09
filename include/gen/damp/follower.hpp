#pragma once
#include "gen/damp/trajectory.hpp"
#include <array>
#include <cstddef>
namespace arc::damp {
using Vector6 = std::array<double,6>;
using Matrix6 = std::array<Vector6,6>;
using Gain = std::array<Vector6,2>;
struct FollowerLinearization { Matrix6 A{}; std::array<std::array<double,2>,6> B{}; };
// Field-state Euler prediction, paired with its exact analytic Jacobian.
State followerStep(const Model&,const State&,Voltage,double dt);
FollowerLinearization linearizeFollower(const Model&,const State&,double dt);
struct BoxSolution { std::array<double,2> delta{}; std::array<bool,2> free{}; bool valid=false; };
// Symmetric positive definite H = [[a,b],[b,c]]. Invalid inputs fail closed.
BoxSolution solveBoxQP(double a,double b,double c,std::array<double,2> gradient,
                       std::array<double,2> lower,std::array<double,2> upper);
struct FollowerConfig {
    double maxVoltage=0; // Required physical voltage limit, supplied by caller.
    unsigned horizon=24, maxIterations=1;
    double stepSeconds=.01;
    Vector6 stateWeights{{500,500,8,100,100,10}};
    double inputWeight=1, terminalScale=1;
    double positionTolerance=.02,headingTolerance=.04;
    double forwardTolerance=.03,lateralTolerance=.02,yawTolerance=.05,settleSeconds=.15;
    double launchSeconds=.1,crawlSpeed=.03,allowanceVoltage=1;
    double relaxationSeconds=1,minimumRelaxationSeconds=.05;
    double sideslipScale=.5,slackMultiplier=.03,cornerMultiplier=1,exitMultiplier=4;
    double exitSpeedMultiplier=6,exitWindow=.5;
    bool valid() const;
};
enum class FollowerStatus { tracking,succeeded,invalid };
struct FollowerResult {
    Voltage voltage{}; FollowerStatus status=FollowerStatus::invalid;
    double referenceTime=0,pace=1,cost=0;
    unsigned iterations=0; bool saturated=false;
};
// Bounded real-time iLQR, with fixed storage (no per-update allocation).
// Deliberate variation from paper section 7: field pose instead of rotating
// pose-error coordinates. Position cost follows the reference body axes. Prediction is
// Euler, consistently used for derivatives and line search; plant may use RK4.
// Only stopping endpoints are accepted. reset() is required for every motion.
class Follower {
public:
    static constexpr unsigned maxHorizon=48;
    Follower(Model model,FollowerConfig config):model_(model),config_(config){}
    void reset();
    FollowerResult update(const State&,const Trajectory&,double actualDt);
private:
    double rollout(const State&,bool trial,double scale=0);
    bool backward();
    Model model_; FollowerConfig config_;
    bool started_=false,ready_=false; FollowerStatus status_=FollowerStatus::invalid;
    double time_=0,elapsed_=0,settled_=0,pace_=1;
    Voltage unconstrained_{}; Gain unconstrainedGain_{};
    std::array<Sample,maxHorizon+1> reference_{};
    std::array<Vector6,maxHorizon+1> weights_{};
    std::array<State,maxHorizon+1> states_{},trialStates_{};
    std::array<Voltage,maxHorizon> controls_{},trialControls_{},feedforward_{};
    std::array<Gain,maxHorizon> gains_{};
};
}
