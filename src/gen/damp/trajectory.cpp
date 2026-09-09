#include "gen/damp/trajectory.hpp"
#include "gen/damp/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace arc::damp {
namespace {
double lerp(double a,double b,double t){return a+(b-a)*t;}
bool positive(double v){return std::isfinite(v)&&v>0;}
double square(double v){return v*v;}
double friction(const Channel& c,double v){return c.sigma*std::clamp(v/c.epsilon,-1.0,1.0);}
double thrust(const Model& m,double v,double volts){return m.drive.alpha*volts-m.drive.lambda*v-friction(m.drive,v);}
double budget(const Model& m,double curvature,double v,double beta,double rail,bool braking,double spent=-1){
    const double yaw=curvature*v;
    const double turn=spent>=0?spent:std::abs((m.turn.lambda*yaw+friction(m.turn,yaw))/m.turn.alpha);
    const double available=std::max(rail-turn,0.0);
    const double a=thrust(m,v*std::cos(beta),braking?-available:available)*std::cos(beta)-
                   floorForce(m.floor,v*std::sin(beta))*std::sin(beta);
    return std::max(braking?-a:a,0.0);
}
struct Node {
    double distance=0;PathPoint point{};
    double ceiling=0,u=0,betaCeiling=0,yawCeiling=0,target=0,beta=0,yaw=0;
};
double slipSpeed(const Floor& f,double k,double beta){
    if(k<1e-10)return std::numeric_limits<double>::infinity();
    const double sn=std::sin(beta),cs=std::cos(beta);
    // Check the measured friction knee before using the saturated quadratic.
    const double linear=(f.mu/f.knee+f.viscous)*sn/(k*cs);
    if(linear*sn<=f.knee)return linear;
    return (f.viscous*sn+std::sqrt(square(f.viscous*sn)+4*k*cs*f.mu))/(2*k*cs);
}
}
bool Trajectory::valid() const {
    if(samples.size()<2||samples.front().time!=0)return false;
    double previous=-1;
    for(const auto& s:samples){
        if(!std::isfinite(s.time)||s.time<=previous||!finite(s.state)||!finite(s.voltage))return false;
        previous=s.time;
    }
    return true;
}
double Trajectory::duration() const {return samples.empty()?0:samples.back().time;}
Sample Trajectory::sample(double time) const {
    if(samples.empty()||!std::isfinite(time)){
        const double n=std::numeric_limits<double>::quiet_NaN();return {n,{n,n,n,n,n,n},{n,n}};
    }
    if(time<=samples.front().time)return samples.front();
    if(time>=samples.back().time)return samples.back();
    const auto it=std::upper_bound(samples.begin(),samples.end(),time,[](double t,const Sample& s){return t<s.time;});
    const auto& b=*it;const auto& a=*(it-1);const double dt=b.time-a.time,t=(time-a.time)/dt;
    State s{lerp(a.state.x,b.state.x,t),lerp(a.state.y,b.state.y,t),
        a.state.heading+t*wrapAngle(b.state.heading-a.state.heading),
        lerp(a.state.forward,b.state.forward,t),lerp(a.state.lateral,b.state.lateral,t),lerp(a.state.yaw,b.state.yaw,t)};
    // Hermite position interpolation respects the trajectory's field velocities.
    const double h00=2*t*t*t-3*t*t+1,h10=t*t*t-2*t*t+t,h01=-2*t*t*t+3*t*t,h11=t*t*t-t*t;
    auto vx=[](const State& z){return std::cos(z.heading)*z.forward-std::sin(z.heading)*z.lateral;};
    auto vy=[](const State& z){return std::sin(z.heading)*z.forward+std::cos(z.heading)*z.lateral;};
    s.x=h00*a.state.x+h10*dt*vx(a.state)+h01*b.state.x+h11*dt*vx(b.state);
    s.y=h00*a.state.y+h10*dt*vy(a.state)+h01*b.state.y+h11*dt*vy(b.state);
    return {time,s,{lerp(a.voltage.left,b.voltage.left,t),lerp(a.voltage.right,b.voltage.right,t)}};
}
double balanceAngle(const Floor& f,double curvature,double speed,double acceleration,double limit){
    if(!std::isfinite(curvature)||!std::isfinite(speed)||speed<0||!std::isfinite(acceleration)||
       !positive(f.mu)||!positive(f.knee)||!std::isfinite(f.viscous)||f.viscous<0||!positive(limit)||limit>=1.5707963267948966)
        return std::numeric_limits<double>::quiet_NaN();
    if(std::abs(curvature)<1e-10||speed==0)return 0;
    const double a=std::abs(curvature)*speed*speed;
    double beta=std::atan2(a,(f.mu/f.knee+f.viscous)*speed+std::abs(acceleration));
    if(speed*std::sin(beta)>f.knee){
        const double b=std::abs(acceleration)+f.viscous*speed,h=std::hypot(a,b);
        if(h<f.mu)return std::numeric_limits<double>::quiet_NaN();
        beta=std::acos(std::clamp(f.mu/h,-1.0,1.0))-std::atan2(b,a);
    }
    return -std::copysign(std::clamp(beta,0.0,limit),curvature);
}
GenerationResult generate(const std::vector<Waypoint>& points,const Model& model,GeneratorConfig cfg){
    GenerationResult out;
    if(!model.valid())return out;
    for(double v:{cfg.maxSpeed,cfg.maxYawRate,cfg.maxVoltage,cfg.maxSideslip,cfg.launchAcceleration,
        cfg.brakeAcceleration,cfg.spacing,cfg.taperDistance,cfg.samplePeriod,cfg.speedFloor,
        cfg.sideslipGain,cfg.residualDistance})if(!positive(v))return out;
    if(!std::isfinite(cfg.correctionMargin)||cfg.correctionMargin<=0||cfg.correctionMargin>=1||
       !std::isfinite(cfg.dwellLimit)||cfg.dwellLimit<0||cfg.maxSideslip>=1.3||cfg.samplePeriod>.05||
       cfg.samplePeriod<.001||cfg.maxNodes<3||cfg.maxNodes>100000||cfg.maxSamples<3||cfg.maxSamples>100000)return out;
    const double margin=1-cfg.correctionMargin;
    cfg.maxSpeed*=margin;cfg.maxYawRate*=margin;cfg.maxVoltage*=margin;cfg.maxSideslip*=margin;
    Spline spline;
    if(!spline.build(points)){out.status=GenerationStatus::invalidGeometry;return out;}
    std::vector<Node> nodes;
    double distance=0,heading=0;
    while(true){
        if(nodes.size()>=cfg.maxNodes){out.status=GenerationStatus::tooLarge;return out;}
        Node node;node.distance=distance;node.point=spline.at(distance);
        if(!std::isfinite(node.point.curvature)){out.status=GenerationStatus::invalidGeometry;return out;}
        if(!nodes.empty())node.point.heading=heading+wrapAngle(node.point.heading-heading);
        heading=node.point.heading;
        const double k=std::abs(node.point.curvature);
        node.ceiling=std::min(square(cfg.maxSpeed),k>1e-10?square(cfg.maxYawRate/k):square(cfg.maxSpeed));
        node.betaCeiling=cfg.maxSideslip*std::clamp(std::min(distance,spline.length()-distance)/cfg.taperDistance,0.0,1.0);
        node.yawCeiling=cfg.maxYawRate;
        nodes.push_back(node);
        if(distance>=spline.length())break;
        distance=std::min(spline.length(),distance+std::clamp(cfg.spacing/(1+30*k),.4*cfg.spacing,cfg.spacing));
    }
    if(nodes.size()<3){out.status=GenerationStatus::invalidGeometry;return out;}
    const std::size_t n=nodes.size();out.nodeCount=n;
    nodes.front().ceiling=nodes.back().ceiling=0;nodes.back().yawCeiling=0;
    for(std::size_t ii=n-1;ii>0;ii--){
        auto& a=nodes[ii-1];const auto& b=nodes[ii];const double ds=b.distance-a.distance;
        a.ceiling=std::min(a.ceiling,b.ceiling+2*cfg.brakeAcceleration*ds);
        a.yawCeiling=std::min(a.yawCeiling,b.yawCeiling+model.turn.alpha*cfg.maxVoltage*ds/std::max(std::sqrt(a.ceiling),cfg.speedFloor));
    }
    for(std::size_t i=1;i<n;i++)nodes[i].ceiling=std::min(nodes[i].ceiling,nodes[i-1].ceiling+2*cfg.launchAcceleration*(nodes[i].distance-nodes[i-1].distance));
    // Pass 1: grip/thrust, yaw, and sideslip limits (paper sections 3.7, 4.9).
    for(auto& node:nodes){
        const double k=std::abs(node.point.curvature);node.u=node.ceiling;
        if(k<1e-10)continue;
        for(int j=0;j<2;j++)node.u=std::min(node.u,std::hypot(thrust(model,std::sqrt(node.u),cfg.maxVoltage),model.floor.mu)/k);
        node.u=std::min(node.u,square(slipSpeed(model.floor,k,cfg.maxSideslip)));
        if(cfg.dwellLimit>0){
            const double dwell=model.turn.lambda/(k*std::max(std::sqrt(node.u),cfg.speedFloor));
            node.u=std::min(node.u,square(slipSpeed(model.floor,k,cfg.maxSideslip/(1+dwell/cfg.dwellLimit))));
        }
    }
    // Pass 2: backward braking sweep, two local evaluations, no convergence loop.
    for(std::size_t ii=n-1;ii>0;ii--){
        auto& a=nodes[ii-1];const auto& b=nodes[ii];const double ds=b.distance-a.distance;
        const double cap=a.u;double candidate=cap;
        for(int j=0;j<2;j++){
            double beta=balanceAngle(model.floor,a.point.curvature,std::sqrt(candidate),(b.u-candidate)/(2*ds),cfg.maxSideslip);
            if(!std::isfinite(beta)){candidate=0;break;}
            candidate=std::min(cap,b.u+2*budget(model,a.point.curvature,std::sqrt(candidate),beta,cfg.maxVoltage,true)*ds);
        }
        a.u=candidate;
    }
    for(std::size_t i=0;i<n;i++){
        const std::size_t lo=i?i-1:i,hi=std::min(i+1,n-1);
        const double acceleration=.5*(nodes[hi].u-nodes[lo].u)/(nodes[hi].distance-nodes[lo].distance);
        double target=balanceAngle(model.floor,nodes[i].point.curvature,std::sqrt(nodes[i].u),acceleration,cfg.maxSideslip);
        if(!std::isfinite(target))target=i?nodes[i-1].target:0;
        nodes[i].target=std::clamp(target,-nodes[i].betaCeiling,nodes[i].betaCeiling);
    }
    // Pass 3: launch speed and reachable sideslip are integrated together.
    double beta=0,yaw=0,spent=0;
    for(std::size_t i=0;i+1<n;i++){
        auto& a=nodes[i];auto& b=nodes[i+1];const double ds=b.distance-a.distance;
        b.u=std::min(b.u,a.u+2*budget(model,a.point.curvature,std::sqrt(a.u),beta,cfg.maxVoltage,false,spent)*ds);
        const double v=std::max(std::sqrt(b.u),cfg.speedFloor);
        const auto& ahead=nodes[std::min(i+2,n-1)];
        const double slope=(ahead.target-a.target)/(ahead.distance-a.distance);
        const double wanted=v*(b.point.curvature-slope)-cfg.sideslipGain*model.turn.lambda*(b.target-beta);
        const double slew=std::max(0.0,model.turn.alpha*cfg.maxVoltage-model.turn.lambda*std::abs(yaw)-friction(model.turn,std::abs(yaw)))*ds/v;
        const double before=yaw;
        yaw=std::clamp(std::clamp(wanted,yaw-slew,yaw+slew),-b.yawCeiling,b.yawCeiling);
        beta=std::clamp(beta+ds*(b.point.curvature-yaw/v),-b.betaCeiling,b.betaCeiling);
        if(b.target>0)beta=std::max(beta,0.0);else if(b.target<0)beta=std::min(beta,0.0);
        b.beta=beta;
        spent=std::abs((v*(yaw-before)/ds+model.turn.lambda*yaw+friction(model.turn,yaw))/model.turn.alpha);
    }
    // Recover yaw from the finished beta schedule, then both voltages.
    for(std::size_t i=0;i<n;i++){
        const auto& lo=nodes[i?i-1:i];const auto& hi=nodes[std::min(i+1,n-1)];
        nodes[i].yaw=std::clamp(std::sqrt(nodes[i].u)*(nodes[i].point.curvature-(hi.beta-lo.beta)/(hi.distance-lo.distance)),
                               -nodes[i].yawCeiling,nodes[i].yawCeiling);
    }
    Trajectory raw;raw.samples.reserve(n);double time=0;
    for(std::size_t i=0;i<n;i++){
        const auto& a=nodes[i];const auto& lo=nodes[i?i-1:i];const auto& hi=nodes[std::min(i+1,n-1)];
        const double v=std::sqrt(a.u),ds=hi.distance-lo.distance,sn=std::sin(a.beta),cs=std::cos(a.beta);
        const double acceleration=.5*(hi.u-lo.u)/ds;
        const double vd=((acceleration+floorForce(model.floor,v*sn)*sn)/cs+model.drive.lambda*v*cs+friction(model.drive,v*cs))/model.drive.alpha;
        const double vt=(v*(hi.yaw-lo.yaw)/ds+model.turn.lambda*a.yaw+friction(model.turn,a.yaw))/model.turn.alpha;
        Voltage volts{vd-vt,vd+vt};
        const double scale=std::max({1.0,std::abs(volts.left)/cfg.maxVoltage,std::abs(volts.right)/cfg.maxVoltage});
        volts.left/=scale;volts.right/=scale;
        if(i){
            const double sum=v+std::sqrt(nodes[i-1].u);
            if(sum<=1e-10){out.status=GenerationStatus::infeasible;return out;}
            time+=2*(a.distance-nodes[i-1].distance)/sum;
        }
        State state{a.point.x,a.point.y,a.point.heading-a.beta,v*cs,v*sn,a.yaw};
        if(cfg.reverse){state.heading+=3.14159265358979323846;state.forward=-state.forward;state.lateral=-state.lateral;volts={-volts.right,-volts.left};}
        if(i==n-1)volts={};
        raw.samples.push_back({time,state,volts});
    }
    if(!raw.valid()){out.status=GenerationStatus::infeasible;return out;}
    const double count=std::ceil(time/cfg.samplePeriod);
    if(!std::isfinite(count)||count+1>cfg.maxSamples){out.status=GenerationStatus::tooLarge;return out;}
    for(std::size_t i=0;i<static_cast<std::size_t>(count);i++)out.trajectory.samples.push_back(raw.sample(i*cfg.samplePeriod));
    out.trajectory.samples.push_back(raw.samples.back());
    // Resampled body-acceleration consistency detects normal AND tangential
    // defects, including any changes made by the shared voltage rail.
    const auto& samples=out.trajectory.samples;
    std::vector<double> residuals;residuals.reserve(samples.size());
    double sumSquares=0;std::size_t first=0;
    for(std::size_t i=0;i<samples.size();i++){
        const auto& s=samples[i];const auto& lo=samples[i?i-1:i];const auto& hi=samples[std::min(i+1,samples.size()-1)];
        const double dt=hi.time-lo.time;
        const State d=derivative(model,s.state,s.voltage);
        const double residual=std::hypot((hi.state.forward-lo.state.forward)/dt-d.forward,(hi.state.lateral-lo.state.lateral)/dt-d.lateral);
        if(!std::isfinite(residual)){out.status=GenerationStatus::infeasible;out.trajectory.samples.clear();return out;}
        residuals.push_back(residual*residual);sumSquares+=residual*residual;
        while(first<i&&samples[i].time-samples[first].time>1/model.turn.lambda)sumSquares-=residuals[first++];
        out.peakResidual=std::max(out.peakResidual,residual);
        out.sustainedResidual=std::max(out.sustainedResidual,std::sqrt(std::max(0.0,sumSquares)/(i-first+1)));
        out.peakVoltage=std::max({out.peakVoltage,std::abs(s.voltage.left),std::abs(s.voltage.right)});
    }
    out.status=out.sustainedResidual<=2*cfg.residualDistance*square(model.turn.lambda)?GenerationStatus::succeeded:GenerationStatus::infeasible;
    if(!out.ok())out.trajectory.samples.clear();
    return out;
}
} // namespace arc::damp
