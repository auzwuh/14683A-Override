#include "gen/damp/follower.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace arc::damp {
namespace {
Vector6 vec(State s){return {{s.x,s.y,s.heading,s.forward,s.lateral,s.yaw}};}
State state(Vector6 s){return {s[0],s[1],s[2],s[3],s[4],s[5]};}
Vector6 difference(State a,State b){auto x=vec(a),y=vec(b);for(int i=0;i<6;++i)x[i]-=y[i];x[2]=wrapAngle(x[2]);return x;}
double clip(double x,double lo,double hi){return std::max(lo,std::min(x,hi));}
bool positive(double x){return std::isfinite(x)&&x>0;}
Matrix6 costH(const Sample& ref,const Vector6& w) {
    Matrix6 h{};for(int i=0;i<6;++i)h[i][i]=w[i];
    double c=std::cos(ref.state.heading),s=std::sin(ref.state.heading);
    h[0][0]=w[0]*c*c+w[1]*s*s;h[1][1]=w[0]*s*s+w[1]*c*c;
    h[0][1]=h[1][0]=(w[0]-w[1])*c*s;return h;
}
Vector6 multiply(const Matrix6& h,const Vector6& x){Vector6 y{};for(int i=0;i<6;++i)for(int j=0;j<6;++j)y[i]+=h[i][j]*x[j];return y;}
}
State followerStep(const Model& m,const State& s,Voltage u,double dt){
    auto x=vec(s),d=vec(derivative(m,s,u));for(int i=0;i<6;++i)x[i]+=dt*d[i];return state(x);
}
FollowerLinearization linearizeFollower(const Model& m,const State& s,double dt){
    FollowerLinearization l;auto& a=l.A;double c=std::cos(s.heading),sn=std::sin(s.heading);
    a[0][2]=-sn*s.forward-c*s.lateral;a[0][3]=c;a[0][4]=-sn;
    a[1][2]=c*s.forward-sn*s.lateral;a[1][3]=sn;a[1][4]=c;a[2][5]=1;
    a[3][3]=-m.drive.lambda-(std::abs(s.forward)<m.drive.epsilon?m.drive.sigma/m.drive.epsilon:0);
    a[3][4]=s.yaw;a[3][5]=s.lateral;a[4][3]=-s.yaw;a[4][5]=-s.forward;
    a[4][4]=-m.floor.viscous-(std::abs(s.lateral)<m.floor.knee?m.floor.mu/m.floor.knee:0);
    a[5][5]=-m.turn.lambda-(std::abs(s.yaw)<m.turn.epsilon?m.turn.sigma/m.turn.epsilon:0);
    for(int i=0;i<6;++i){for(int j=0;j<6;++j)a[i][j]*=dt;a[i][i]+=1;}
    l.B[3]={{dt*m.drive.alpha*.5,dt*m.drive.alpha*.5}};
    l.B[5]={{-dt*m.turn.alpha*.5,dt*m.turn.alpha*.5}};return l;
}
BoxSolution solveBoxQP(double a,double b,double c,std::array<double,2> g,std::array<double,2> lo,std::array<double,2> hi){
    BoxSolution out;double det=a*c-b*b;
    if(!positive(a)||!positive(c)||!positive(det)||!std::isfinite(b))return out;
    for(int i=0;i<2;++i)if(!std::isfinite(g[i])||!std::isfinite(lo[i])||!std::isfinite(hi[i])||lo[i]>hi[i])return out;
    double best=std::numeric_limits<double>::infinity();
    auto consider=[&](double x,double y){
        if(x<lo[0]||x>hi[0]||y<lo[1]||y>hi[1])return;
        double cost=.5*(a*x*x+2*b*x*y+c*y*y)+g[0]*x+g[1]*y;
        if(std::isfinite(cost)&&cost<best){best=cost;out.delta={{x,y}};out.valid=true;}
    };
    consider((-c*g[0]+b*g[1])/det,(b*g[0]-a*g[1])/det);
    for(double x:{lo[0],hi[0]})consider(x,clip((-g[1]-b*x)/c,lo[1],hi[1]));
    for(double y:{lo[1],hi[1]})consider(clip((-g[0]-b*y)/a,lo[0],hi[0]),y);
    for(double x:{lo[0],hi[0]})for(double y:{lo[1],hi[1]})consider(x,y);
    for(int i=0;i<2;++i)out.free[i]=out.delta[i]>lo[i]+1e-10&&out.delta[i]<hi[i]-1e-10;
    return out;
}
bool FollowerConfig::valid()const{
    if(horizon<2||horizon>Follower::maxHorizon||maxIterations<1||maxIterations>4)return false;
    for(double x:stateWeights)if(!positive(x))return false;
    for(double x:{maxVoltage,stepSeconds,inputWeight,terminalScale,positionTolerance,headingTolerance,
        forwardTolerance,lateralTolerance,yawTolerance,settleSeconds,crawlSpeed,relaxationSeconds,
        minimumRelaxationSeconds,sideslipScale,slackMultiplier,cornerMultiplier,exitMultiplier,
        exitSpeedMultiplier,exitWindow})if(!positive(x))return false;
    return stepSeconds>=.001&&stepSeconds<=.05&&std::isfinite(launchSeconds)&&launchSeconds>=0&&std::isfinite(allowanceVoltage)&&allowanceVoltage>=0;
}
void Follower::reset(){started_=false;ready_=true;status_=FollowerStatus::tracking;time_=elapsed_=settled_=0;pace_=1;controls_={};}
double Follower::rollout(const State& initial,bool trial,double scale){
    auto& xs=trial?trialStates_:states_;auto& us=trial?trialControls_:controls_;xs[0]=initial;double cost=0;
    for(unsigned k=0;k<=config_.horizon;++k){
        auto d=difference(xs[k],reference_[k].state);auto h=costH(reference_[k],weights_[k]);auto hd=multiply(h,d);
        double factor=k==config_.horizon?config_.terminalScale:config_.stepSeconds;
        for(int i=0;i<6;++i)cost+=.5*factor*d[i]*hd[i];
        if(k==config_.horizon)break;
        if(trial){auto dx=difference(xs[k],states_[k]);double u[2]={controls_[k].left+scale*feedforward_[k].left,controls_[k].right+scale*feedforward_[k].right};
            for(int i=0;i<2;++i)for(int j=0;j<6;++j)u[i]+=gains_[k][i][j]*dx[j];
            us[k]={clip(u[0],-config_.maxVoltage,config_.maxVoltage),clip(u[1],-config_.maxVoltage,config_.maxVoltage)};
        }
        double dl=us[k].left-reference_[k].voltage.left,dr=us[k].right-reference_[k].voltage.right;
        cost+=.5*config_.stepSeconds*config_.inputWeight*(dl*dl+dr*dr);
        xs[k+1]=followerStep(model_,xs[k],us[k],config_.stepSeconds);
        if(!finite(xs[k+1])||!finite(us[k]))return std::numeric_limits<double>::infinity();
    }return cost;
}
bool Follower::backward(){
    unsigned n=config_.horizon;Matrix6 vxx=costH(reference_[n],weights_[n]);
    for(auto& row:vxx)for(double& x:row)x*=config_.terminalScale;
    Vector6 vx=multiply(vxx,difference(states_[n],reference_[n].state));
    for(unsigned kk=n;kk>0;--kk){unsigned k=kk-1;auto l=linearizeFollower(model_,states_[k],config_.stepSeconds);
        auto qxx=costH(reference_[k],weights_[k]);for(auto& row:qxx)for(double& x:row)x*=config_.stepSeconds;
        auto qx=multiply(qxx,difference(states_[k],reference_[k].state));
        double qu[2]={config_.stepSeconds*config_.inputWeight*(controls_[k].left-reference_[k].voltage.left),config_.stepSeconds*config_.inputWeight*(controls_[k].right-reference_[k].voltage.right)};
        double h[2][2]={{config_.stepSeconds*config_.inputWeight,0},{0,config_.stepSeconds*config_.inputWeight}};Gain qux{};
        Matrix6 va{};double vb[6][2]{};
        for(int i=0;i<6;++i)for(int j=0;j<6;++j)for(int z=0;z<6;++z)va[i][j]+=vxx[i][z]*l.A[z][j];
        for(int i=0;i<6;++i)for(int j=0;j<2;++j)for(int z=0;z<6;++z)vb[i][j]+=vxx[i][z]*l.B[z][j];
        for(int i=0;i<6;++i){for(int z=0;z<6;++z)qx[i]+=l.A[z][i]*vx[z];
            for(int j=0;j<6;++j)for(int z=0;z<6;++z)qxx[i][j]+=l.A[z][i]*va[z][j];}
        for(int i=0;i<2;++i){for(int z=0;z<6;++z)qu[i]+=l.B[z][i]*vx[z];
            for(int j=0;j<2;++j)for(int z=0;z<6;++z)h[i][j]+=l.B[z][i]*vb[z][j];
            for(int j=0;j<6;++j)for(int z=0;z<6;++z)qux[i][j]+=l.B[z][i]*va[z][j];}
        double a=h[0][0],b=.5*(h[0][1]+h[1][0]),c=h[1][1],det=a*c-b*b;
        h[0][1]=h[1][0]=b;
        auto box=solveBoxQP(a,b,c,{{qu[0],qu[1]}},{{-config_.maxVoltage-controls_[k].left,-config_.maxVoltage-controls_[k].right}},{{config_.maxVoltage-controls_[k].left,config_.maxVoltage-controls_[k].right}});
        if(!box.valid)return false;
        auto& gain=gains_[k];gain={};
        Gain unc{};for(int j=0;j<6;++j){unc[0][j]=(-c*qux[0][j]+b*qux[1][j])/det;unc[1][j]=(b*qux[0][j]-a*qux[1][j])/det;
            if(box.free[0]&&box.free[1]){gain[0][j]=unc[0][j];gain[1][j]=unc[1][j];}
            else {if(box.free[0])gain[0][j]=-qux[0][j]/a;if(box.free[1])gain[1][j]=-qux[1][j]/c;}}
        if(k==0){unconstrainedGain_=unc;unconstrained_={controls_[k].left+(-c*qu[0]+b*qu[1])/det,controls_[k].right+(b*qu[0]-a*qu[1])/det};}
        feedforward_[k]={box.delta[0],box.delta[1]};
        // Full constrained value recursion: none of the Quu terms may be dropped.
        vx=qx;vxx=qxx;
        for(int i=0;i<6;++i){for(int u=0;u<2;++u){vx[i]+=gain[u][i]*qu[u]+qux[u][i]*box.delta[u];for(int v=0;v<2;++v)vx[i]+=gain[u][i]*h[u][v]*box.delta[v];}
            for(int j=0;j<6;++j)for(int u=0;u<2;++u){vxx[i][j]+=gain[u][i]*qux[u][j]+qux[u][i]*gain[u][j];for(int v=0;v<2;++v)vxx[i][j]+=gain[u][i]*h[u][v]*gain[v][j];}}
        for(int i=0;i<6;++i){if(!std::isfinite(vx[i]))return false;for(int j=i;j<6;++j){double z=.5*(vxx[i][j]+vxx[j][i]);if(!std::isfinite(z))return false;vxx[i][j]=vxx[j][i]=z;}}
    }return finite(unconstrained_);
}
FollowerResult Follower::update(const State& measured,const Trajectory& trajectory,double dt){
    FollowerResult out;out.referenceTime=time_;out.pace=pace_;
    auto invalid=[&](){status_=FollowerStatus::invalid;ready_=false;out.status=status_;return out;};
    if(!ready_||!model_.valid()||!config_.valid()||!finite(measured)||!positive(dt)||dt>.1)return invalid();
    const double pole=std::max({model_.drive.lambda+model_.drive.sigma/model_.drive.epsilon,
        model_.turn.lambda+model_.turn.sigma/model_.turn.epsilon,model_.floor.viscous+model_.floor.mu/model_.floor.knee});
    if(config_.stepSeconds*pole>=1) return invalid(); // Resolve friction relaxation in Euler prediction.
    if(!started_){if(!trajectory.valid())return invalid();const auto& end=trajectory.samples.back().state;
        if(std::abs(end.forward)>1e-8||std::abs(end.lateral)>1e-8||std::abs(end.yaw)>1e-8)return invalid();}
    if(trajectory.samples.empty()||!std::isfinite(trajectory.duration())||trajectory.duration()<time_)return invalid();
    if(status_==FollowerStatus::succeeded){out.status=status_;return out;}
    unsigned n=config_.horizon;double endTime=trajectory.duration();
    for(unsigned k=0;k<=n;++k){double t=std::min(endTime,time_+k*config_.stepSeconds*pace_);
        reference_[k]=trajectory.sample(t);if(!finite(reference_[k].state)||!finite(reference_[k].voltage))return invalid();
        auto& ref=reference_[k];weights_[k]=config_.stateWeights;
        double lean=clip(std::abs(std::atan2(ref.state.lateral,std::abs(ref.state.forward)))/config_.sideslipScale,0,1);
        double into=clip((config_.exitWindow-(endTime-t))/config_.exitWindow,0,1),phi=into*into;
        double mult=std::max(config_.slackMultiplier+(config_.cornerMultiplier-config_.slackMultiplier)*lean,config_.slackMultiplier+(config_.exitMultiplier-config_.slackMultiplier)*phi);
        weights_[k][2]*=mult;weights_[k][4]*=mult;weights_[k][3]*=1+(config_.exitSpeedMultiplier-1)*phi;
        ref.state.forward*=pace_;ref.state.lateral*=pace_;ref.state.yaw*=pace_;
    }
    for(unsigned k=0;k<n;++k){Voltage u=started_&&k+1<n?controls_[k+1]:reference_[k].voltage;
        controls_[k]={clip(u.left,-config_.maxVoltage,config_.maxVoltage),clip(u.right,-config_.maxVoltage,config_.maxVoltage)};}
    started_=true;double cost=rollout(measured,false);if(!std::isfinite(cost))return invalid();
    unconstrained_=controls_[0];unconstrainedGain_={};
    for(unsigned i=0;i<config_.maxIterations;++i){if(!backward())return invalid();++out.iterations;bool accepted=false;
        for(double scale:{1.,.5,.25,.1}){double next=rollout(measured,true,scale);if(std::isfinite(next)&&next<cost){cost=next;controls_=trialControls_;states_=trialStates_;accepted=true;break;}}
        if(!accepted)break;
    }
    out.voltage=controls_[0];out.cost=cost;
    const auto now=trajectory.sample(time_).state;
    double speed=std::hypot(measured.forward,measured.lateral),refSpeed=std::hypot(now.forward,now.lateral);
    double course=now.heading+(refSpeed>config_.crawlSpeed?std::atan2(now.lateral,now.forward):0);
    double actual=measured.heading+(speed>config_.crawlSpeed?std::atan2(measured.lateral,measured.forward):0);
    double courseGate=elapsed_<config_.launchSeconds?1:std::abs(std::cos(course-actual));
    double c=std::cos(measured.heading),s=std::sin(measured.heading);
    // Transform the field-state feedback column into positive reference-ahead error.
    double g=std::max(0.,-.5*((unconstrainedGain_[0][0]+unconstrainedGain_[1][0])*c+(unconstrainedGain_[0][1]+unconstrainedGain_[1][1])*s))*model_.drive.alpha/model_.drive.lambda;
    double ex=c*(now.x-measured.x)+s*(now.y-measured.y);
    double excess=g*ex-config_.allowanceVoltage*model_.drive.alpha/model_.drive.lambda;
    double closing=g*refSpeed*courseGate,hold=0;
    if(excess>0&&closing>1e-12)hold=excess/closing*dt/std::max(config_.minimumRelaxationSeconds,std::min(config_.relaxationSeconds,endTime-time_));
    double demand=std::max(std::abs(unconstrained_.left),std::abs(unconstrained_.right));
    double saturationGate=demand>config_.maxVoltage?config_.maxVoltage/demand:1;
    pace_=clip(std::max(dt*courseGate-hold,0.)*saturationGate/dt,0,1);
    time_=std::min(endTime,time_+dt*pace_);elapsed_+=dt;
    out.referenceTime=time_;out.pace=pace_;out.saturated=saturationGate<1;
    const auto end=trajectory.samples.back().state;
    bool settled=time_>=endTime&&std::hypot(end.x-measured.x,end.y-measured.y)<=config_.positionTolerance&&std::abs(wrapAngle(end.heading-measured.heading))<=config_.headingTolerance&&std::abs(measured.forward)<=config_.forwardTolerance&&std::abs(measured.lateral)<=config_.lateralTolerance&&std::abs(measured.yaw)<=config_.yawTolerance;
    settled_=settled?settled_+dt:0;
    if(settled_>=config_.settleSeconds){status_=FollowerStatus::succeeded;out.voltage={};}
    out.status=status_;return out;
}
}
