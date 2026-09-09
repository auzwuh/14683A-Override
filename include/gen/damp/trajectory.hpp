#pragma once
#include "gen/damp/model.hpp"
#include <cstddef>
#include <vector>

namespace arc::damp {
struct Sample { double time=0; State state{}; Voltage voltage{}; };
struct Trajectory {
    std::vector<Sample> samples;
    bool valid() const;
    double duration() const;
    Sample sample(double time) const;
};
struct Waypoint { double x=0,y=0; };
struct GeneratorConfig {
    // Physical limits must be supplied. No built-in robot calibration.
    double maxSpeed=0, maxYawRate=0, maxVoltage=0;
    double maxSideslip=.35, correctionMargin=.2;
    double launchAcceleration=0, brakeAcceleration=0;
    double spacing=0, taperDistance=0;
    double samplePeriod=.01, speedFloor=.005, sideslipGain=1, dwellLimit=3;
    double residualDistance=.01;
    std::size_t maxNodes=12000, maxSamples=30000;
    bool reverse=false;
};
enum class GenerationStatus { succeeded, invalidInput, invalidGeometry, tooLarge, infeasible };
struct GenerationResult {
    Trajectory trajectory;
    GenerationStatus status=GenerationStatus::invalidInput;
    double peakResidual=0, sustainedResidual=0, peakVoltage=0;
    std::size_t nodeCount=0;
    bool ok() const { return status==GenerationStatus::succeeded; }
};
// Stopping endpoints, endpoint tangents along first/last chord.
GenerationResult generate(const std::vector<Waypoint>&,const Model&,GeneratorConfig);
// Paper section 4.7 magnitude construction, sign opposite curvature.
double balanceAngle(const Floor&,double curvature,double speed,double tangentialAcceleration,double limit);
} // namespace arc::damp
