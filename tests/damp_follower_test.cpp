#include "gen/damp/follower.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace arc::damp;
int main() {
    // Synthetic coefficients for host tests only; never robot calibration.
    Model model{{2,3,.1,.05},{4,5,.1,.05},{.2,.1,1}};
    for(int n=0;n<60;++n) {
        double a=1+.1*n,b=.3*std::sin(n),c=2+.03*n;
        std::array<double,2> g{{3*std::sin(n*.4),4*std::cos(n*.3)}};
        auto q=solveBoxQP(a,b,c,g,{{-1,-.7}},{{.6,1.2}});
        assert(q.valid);
        auto cost=[&](double x,double y){return .5*(a*x*x+2*b*x*y+c*y*y)+g[0]*x+g[1]*y;};
        for(int i=0;i<=80;++i) for(int j=0;j<=80;++j)
            assert(cost(q.delta[0],q.delta[1])<=cost(-1+1.6*i/80.,-.7+1.9*j/80.)+1e-10);
        for(int i=0;i<2;++i) if(q.free[i]) assert(q.delta[i]>(i==0?-1:-.7)&&q.delta[i]<(i==0?.6:1.2));
    }
    assert(!solveBoxQP(0,0,0,{{0,0}},{{-1,-1}},{{1,1}}).valid);
    State s{.1,-.2,.3,.8,.25,.4}; Voltage u{1,2}; double dt=.01;
    auto v=[](State z){return std::array<double,6>{{z.x,z.y,z.heading,z.forward,z.lateral,z.yaw}};};
    auto st=[](std::array<double,6> z){return State{z[0],z[1],z[2],z[3],z[4],z[5]};};
    auto lin=linearizeFollower(model,s,dt);
    for(int j=0;j<8;++j) {
        auto plus=v(s),minus=plus; Voltage up=u,um=u; double h=1e-6;
        if(j<6){plus[j]+=h;minus[j]-=h;} else if(j==6){up.left+=h;um.left-=h;} else {up.right+=h;um.right-=h;}
        auto p=v(followerStep(model,st(plus),up,dt)),m=v(followerStep(model,st(minus),um,dt));
        for(int i=0;i<6;++i) assert(std::abs((p[i]-m[i])/(2*h)-(j<6?lin.A[i][j]:lin.B[i][j-6]))<1e-7);
    }
    FollowerConfig cfg; cfg.maxVoltage=6;
    Trajectory hold; hold.samples={{0,State{},Voltage{}},{.1,State{},Voltage{}}};
    Follower f(model,cfg); f.reset();
    for(int i=0;i<100;++i) assert(f.update(State{0,0,0,0,.2,0},hold,.01).status==FollowerStatus::tracking);
    f.reset(); FollowerResult r{};
    for(int i=0;i<100;++i) r=f.update(State{},hold,.01);
    assert(r.status==FollowerStatus::succeeded && r.voltage.left==0 && r.voltage.right==0);
    f.reset(); auto bad=State{};bad.x=std::numeric_limits<double>::quiet_NaN();
    assert(f.update(bad,hold,.01).status==FollowerStatus::invalid);
    f.reset(); assert(f.update(State{},hold,0).status==FollowerStatus::invalid);
    auto stiff=model;stiff.floor.knee=.00001;
    Follower unstable(stiff,cfg);unstable.reset();
    assert(unstable.update(State{},hold,.01).status==FollowerStatus::invalid);
    // Recover longitudinal disturbance against independently integrated RK4 plant.
    f.reset(); State plant{-.15,0,0,0,0,0};
    for(int i=0;i<1500;++i) {
        r=f.update(plant,hold,.01); assert(r.status!=FollowerStatus::invalid);
        assert(std::abs(r.voltage.left)<=cfg.maxVoltage && std::abs(r.voltage.right)<=cfg.maxVoltage);
        plant=step(model,plant,r.voltage,.01);
        if(r.status==FollowerStatus::succeeded) break;
    }
    assert(r.status==FollowerStatus::succeeded); assert(std::abs(plant.x)<cfg.positionTolerance);
    cfg.maxVoltage=.1; Follower limited(model,cfg); limited.reset();
    r=limited.update(State{-10,0,0,0,0,0},hold,.01);
    assert(r.status==FollowerStatus::tracking && r.saturated && r.pace<1);
    std::cout<<"damp follower tests passed\n";
}
