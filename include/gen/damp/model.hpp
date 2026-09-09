#pragma once

namespace arc::damp {
// SI, +x/+y field axes; heading CCW from +x; body forward/left velocities.
struct State { double x=0,y=0,heading=0,forward=0,lateral=0,yaw=0; };
struct Voltage { double left=0,right=0; };
struct Channel { double alpha=0,lambda=0,sigma=0,epsilon=.01; };
struct Floor { double mu=0,knee=0,viscous=0; };
struct Model { Channel drive{},turn{}; Floor floor{}; bool valid() const; };
bool finite(const State&);
bool finite(const Voltage&);
double wrapAngle(double);
double floorForce(const Floor&,double lateral);
State derivative(const Model&,const State&,Voltage);
// Invalid model/input/interval returns a nonfinite state, never a plausible zero.
State step(const Model&,const State&,Voltage,double dt);
} // namespace arc::damp
