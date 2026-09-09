#pragma once
#include "gen/damp/trajectory.hpp"
#include <array>

namespace arc::damp {
struct PathPoint { double x=0,y=0,heading=0,curvature=0; };
// C4 quintic interpolation. Parameter derivatives are exposed for validation.
class Spline {
public:
    bool build(const std::vector<Waypoint>& points);
    Waypoint evaluate(std::size_t segment,double t,unsigned derivative=0) const;
    PathPoint at(double distance) const;
    double length() const { return length_; }
    double width(std::size_t segment) const { return segments_.at(segment).width; }
    std::size_t size() const { return segments_.size(); }
private:
    struct Segment { std::array<double,6> x{},y{}; double width=0; };
    struct Cell { std::size_t segment; double t,start,length,a,b,c; };
    std::vector<Segment> segments_;
    std::vector<Cell> cells_;
    double length_=0;
};
} // namespace arc::damp
