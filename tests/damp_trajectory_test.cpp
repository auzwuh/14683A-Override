#include "gen/damp/trajectory.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace arc::damp;
void near(double a,double b,double t=1e-7){assert(std::isfinite(a));assert(std::abs(a-b)<t);}
int main(){
    Model m{{2,3,.1,.01},{4,5,.1,.01},{1,.1,.2}};
    GeneratorConfig cfg;cfg.maxSpeed=.8;cfg.maxYawRate=4;cfg.maxVoltage=6;
    cfg.launchAcceleration=.7;cfg.brakeAcceleration=.7;cfg.spacing=.01;cfg.taperDistance=.3;
    cfg.residualDistance=.03;
    assert(!generate({{0,0},{1,0}},Model{},cfg).ok());
    assert(!generate({{0,0},{0,0}},m,cfg).ok());
    assert(!Trajectory{}.valid());
    Trajectory bad;bad.samples={{1,{},{}},{0,{},{}}};assert(!bad.valid());
    for(double curvature:{-.1,.1,-5.,5.}) {
        double beta=balanceAngle(m.floor,curvature,1,0,1.4);
        assert(beta*curvature<0);
        near(std::abs(curvature)*std::cos(beta),std::abs(floorForce(m.floor,std::sin(beta))),1e-7);
        near(beta,-balanceAngle(m.floor,-curvature,1,0,1.4));
    }
    auto straight=generate({{0,0},{2,0}},m,cfg);
    std::cout<<"straight status="<<static_cast<int>(straight.status)<<" residual="<<straight.sustainedResidual<<std::endl;
    assert(straight.ok());assert(straight.trajectory.valid());
    const auto& tr=straight.trajectory;
    near(tr.samples.front().state.forward,0);near(tr.samples.back().state.forward,0);
    near(tr.samples.back().state.x,2);near(tr.samples.back().voltage.left,0);
    assert(tr.duration()>2/.8 && tr.duration()<10);
    for(const auto& s:tr.samples){near(s.state.y,0);near(s.state.lateral,0);near(s.voltage.left,s.voltage.right);}
    cfg.reverse=true;auto rev=generate({{0,0},{2,0}},m,cfg);assert(rev.ok());
    for(std::size_t i=0;i<tr.samples.size();i++){
        near(tr.samples[i].state.x,rev.trajectory.samples[i].state.x);
        near(tr.samples[i].state.forward,-rev.trajectory.samples[i].state.forward);
        near(tr.samples[i].voltage.left,-rev.trajectory.samples[i].voltage.right);
    }
    cfg.reverse=false;
    auto curve=generate({{0,0},{1,.2},{2,1},{3,1.2}},m,cfg);
    std::cout<<"curve status="<<static_cast<int>(curve.status)<<" residual="<<curve.sustainedResidual<<std::endl;
    assert(curve.ok());
    auto mirror=generate({{0,0},{1,-.2},{2,-1},{3,-1.2}},m,cfg);assert(mirror.ok());
    near(curve.trajectory.duration(),mirror.trajectory.duration());
    double slip=0;
    for(std::size_t i=0;i<curve.trajectory.samples.size();i++){
        auto a=curve.trajectory.samples[i],b=mirror.trajectory.samples[i];
        near(a.state.x,b.state.x);near(a.state.y,-b.state.y);near(a.state.lateral,-b.state.lateral);
        near(a.voltage.left,b.voltage.right);slip=std::max(slip,std::abs(a.state.lateral));
        assert(std::abs(a.voltage.left)<=4.8000001&&std::abs(a.voltage.right)<=4.8000001);
    }
    assert(slip>.001);
    cfg.maxNodes=3;assert(generate({{0,0},{2,0}},m,cfg).status==GenerationStatus::tooLarge);
    std::cout<<"DAMP trajectory tests passed\n";
}
