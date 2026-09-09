#include "gen/damp/motion.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace arc::damp;
int main() {
    auto compass=compassState(10,20,0,-10,5,1);
    assert(std::abs(compass.heading-1.5707963267948966)<1e-10);
    assert(std::abs(compass.forward+.254)<1e-10 && compass.yaw==-1);
    Model model{{2,3,.1,.01},{4,5,.1,.01},{1,.1,.2}};
    GeneratorConfig cfg;cfg.maxSpeed=.8;cfg.maxYawRate=4;cfg.maxVoltage=6;
    cfg.launchAcceleration=.7;cfg.brakeAcceleration=.7;cfg.spacing=.01;cfg.taperDistance=.3;
    cfg.residualDistance=.03;
    for (bool reverse:{false,true}) {
        cfg.reverse=reverse;
        auto generated=generate({{0,0},{1,.2},{2,1},{3,1.2}},model,cfg);
        assert(generated.ok());
        FollowerConfig fc;fc.maxVoltage=6;
        Follower follower(model,fc);follower.reset();
        State plant=generated.trajectory.samples.front().state;
        FollowerResult result;
        double peak=0;
        for(int i=0;i<2500;++i) {
            if(i==180) plant.lateral+=.08; // sideways disturbance during travel
            result=follower.update(plant,generated.trajectory,.01);
            assert(result.status!=FollowerStatus::invalid);
            assert(std::abs(result.voltage.left)<=6 && std::abs(result.voltage.right)<=6);
            plant=step(model,plant,result.voltage,.01); // independent RK4 plant
            auto ref=generated.trajectory.sample(result.referenceTime).state;
            peak=std::max(peak,std::hypot(plant.x-ref.x,plant.y-ref.y));
            if(result.status==FollowerStatus::succeeded) break;
        }
        std::cout<<"reverse="<<reverse<<" peak error="<<peak<<" status="<<int(result.status)<<std::endl;
        assert(result.status==FollowerStatus::succeeded);
        assert(peak<.15);
    }
}
