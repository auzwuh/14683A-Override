#include "gen/damp/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace arc::damp {
namespace {
bool channelValid(const Channel& c) {
    return std::isfinite(c.alpha)&&c.alpha>0 && std::isfinite(c.lambda)&&c.lambda>0 &&
           std::isfinite(c.sigma)&&c.sigma>=0 && std::isfinite(c.epsilon)&&c.epsilon>0;
}
State invalidState() {
    const double n=std::numeric_limits<double>::quiet_NaN();
    return {n,n,n,n,n,n};
}
State add(State a,State b,double scale) {
    return {a.x+b.x*scale,a.y+b.y*scale,a.heading+b.heading*scale,
            a.forward+b.forward*scale,a.lateral+b.lateral*scale,a.yaw+b.yaw*scale};
}
}
bool Model::valid() const {
    return channelValid(drive)&&channelValid(turn)&&std::isfinite(floor.mu)&&floor.mu>0 &&
           std::isfinite(floor.knee)&&floor.knee>0 && std::isfinite(floor.viscous)&&floor.viscous>=0;
}
bool finite(const State& s) {
    return std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.heading)&&
           std::isfinite(s.forward)&&std::isfinite(s.lateral)&&std::isfinite(s.yaw);
}
bool finite(const Voltage& v) { return std::isfinite(v.left)&&std::isfinite(v.right); }
double wrapAngle(double v) { return std::remainder(v,6.2831853071795864769); }
double floorForce(const Floor& f,double v) {
    return f.mu*std::clamp(v/f.knee,-1.0,1.0)+f.viscous*v;
}
State derivative(const Model& m,const State& s,Voltage u) {
    if(!m.valid()||!finite(s)||!finite(u)) return invalidState();
    const double vd=(u.left+u.right)*.5,vt=(u.right-u.left)*.5;
    const double c=std::cos(s.heading),sn=std::sin(s.heading);
    return {c*s.forward-sn*s.lateral,sn*s.forward+c*s.lateral,s.yaw,
            m.drive.alpha*vd-m.drive.lambda*s.forward-
                m.drive.sigma*std::clamp(s.forward/m.drive.epsilon,-1.0,1.0)+s.yaw*s.lateral,
            -s.yaw*s.forward-floorForce(m.floor,s.lateral),
            m.turn.alpha*vt-m.turn.lambda*s.yaw-
                m.turn.sigma*std::clamp(s.yaw/m.turn.epsilon,-1.0,1.0)};
}
State step(const Model& m,const State& input,Voltage u,double dt) {
    if(!m.valid()||!finite(input)||!finite(u)||!std::isfinite(dt)||dt<0||dt>1) return invalidState();
    // Bound RK4's step relative to the fastest friction/drive pole.
    const double rate=std::max({500.0,4*(m.drive.lambda+m.drive.sigma/m.drive.epsilon),
        4*(m.turn.lambda+m.turn.sigma/m.turn.epsilon),4*(m.floor.mu/m.floor.knee+m.floor.viscous)});
    const double count=std::max(1.0,std::ceil(dt*rate));
    if(!std::isfinite(count)||count>10000) return invalidState();
    const int n=static_cast<int>(count);
    const double h=dt/n;
    State s=input;
    for(int i=0;i<n;i++) {
        const State a=derivative(m,s,u),b=derivative(m,add(s,a,h*.5),u);
        const State c=derivative(m,add(s,b,h*.5),u),d=derivative(m,add(s,c,h),u);
        s=add(add(add(add(s,a,h/6),b,h/3),c,h/3),d,h/6);
        if(!finite(s)) return invalidState();
    }
    return s;
}
} // namespace arc::damp
