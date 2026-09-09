#include "gen/damp/model.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace arc::damp;
void near(double got, double want, double tol=1e-8) { assert(std::isfinite(got)); assert(std::abs(got-want)<tol); }
int main() {
    // Synthetic coefficients, not robot tuning.
    Model m{{2,3,.4,.01},{4,5,.2,.01},{1,.1,.2}};
    assert(m.valid()); assert(!Model{}.valid());
    auto d=derivative(m,{0,0,0,1,.2,.5},{2,4});
    near(d.x,1); near(d.y,.2); near(d.heading,.5);
    near(d.forward,2.7); near(d.lateral,-1.54); near(d.yaw,1.3);
    auto mirrored=derivative(m,{0,0,0,1,-.2,-.5},{4,2});
    near(mirrored.forward,d.forward); near(mirrored.lateral,-d.lateral); near(mirrored.yaw,-d.yaw);
    auto reverse=derivative(m,{0,0,0,-1,0,0},{-3,-3});
    near(reverse.forward,-2.6); // friction opposes velocity, including braking
    auto braking=derivative(m,{0,0,0,1,0,0},{-3,-3}); near(braking.forward,-9.4);
    near(floorForce(m.floor,.02),.204); near(floorForce(m.floor,-.2),-1.04);
    auto rotated=derivative(m,{0,0,1.5707963267948966,1,.2,0},{0,0});
    near(rotated.x,-.2); near(rotated.y,1);
    Model linear{{2,3,0,.01},{4,5,0,.01},{1,.1,0}};
    State s{};
    for(int i=0;i<100;i++) s=step(linear,s,{3,3},.001);
    near(s.forward,2*(1-std::exp(-.3)),1e-9);
    near(s.x,2*(.1-(1-std::exp(-.3))/3),1e-9);
    auto invalid=m; invalid.floor.knee=0; assert(!invalid.valid());
    assert(!finite(step(invalid,{}, {},.01)));
    assert(!finite(step(m,{}, {},-.01)));
    assert(!finite(step(m,{}, {std::numeric_limits<double>::quiet_NaN(),0},.01)));
    near(wrapAngle(7),.7168146928204138);
    std::cout << "DAMP model tests passed\n";
}
