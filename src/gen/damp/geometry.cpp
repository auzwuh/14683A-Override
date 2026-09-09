#include "gen/damp/geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace arc::damp {
namespace {
using Row=std::vector<double>;
constexpr double basis[6][6]={
    {1,0,0,-10,15,-6},{0,1,0,-6,8,-3},{0,0,.5,-1.5,1.5,-.5},
    {0,0,0,10,-15,6},{0,0,0,-4,7,-3},{0,0,0,.5,-1,.5}};
bool solve(std::vector<Row>& a,Row& x,Row& y) {
    const std::size_t n=a.size();
    for(std::size_t i=0;i<n;i++) {
        double scale=0;
        for(double v:a[i]) scale=std::max(scale,std::abs(v));
        if(!std::isfinite(scale)||scale==0) return false;
        for(double& v:a[i]) v/=scale;
        x[i]/=scale;y[i]/=scale;
    }
    for(std::size_t i=0;i<n;i++) {
        std::size_t pivot=i;
        for(std::size_t j=i+1;j<n;j++) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
        if(std::abs(a[pivot][i])<1e-12) return false;
        std::swap(a[pivot],a[i]);std::swap(x[pivot],x[i]);std::swap(y[pivot],y[i]);
        for(std::size_t j=i+1;j<n;j++) {
            const double factor=a[j][i]/a[i][i];
            for(std::size_t k=i;k<n;k++) a[j][k]-=factor*a[i][k];
            x[j]-=factor*x[i];y[j]-=factor*y[i];
        }
    }
    for(std::size_t k=n;k>0;k--) {
        const std::size_t i=k-1;
        for(std::size_t j=i+1;j<n;j++){x[i]-=a[i][j]*x[j];y[i]-=a[i][j]*y[j];}
        x[i]/=a[i][i];y[i]/=a[i][i];
        if(!std::isfinite(x[i])||!std::isfinite(y[i]))return false;
    }
    return true;
}
double polynomial(const std::array<double,6>& a,double t,unsigned derivative) {
    double out=0;
    for(int i=5;i>=static_cast<int>(derivative);i--) {
        double coefficient=a[i];
        for(unsigned j=0;j<derivative;j++)coefficient*=i-j;
        out=out*t+coefficient;
    }
    return out;
}
}
bool Spline::build(const std::vector<Waypoint>& points) {
    segments_.clear();cells_.clear();length_=0;
    if(points.size()<2||points.size()>64)return false;
    const std::size_t n=points.size(),dim=2*n;
    Row h(n-1),x(dim),y(dim);
    std::vector<Row> matrix(dim,Row(dim));
    for(const auto& p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y))return false;
    for(std::size_t i=0;i+1<n;i++){
        const double chord=std::hypot(points[i+1].x-points[i].x,points[i+1].y-points[i].y);
        if(chord<1e-5)return false;
        h[i]=std::sqrt(chord);
    }
    // Endpoint first derivatives point along the adjacent chord; curvature zero.
    matrix[0][0]=matrix[1][1]=matrix[dim-2][dim-2]=matrix[dim-1][dim-1]=1;
    x[0]=(points[1].x-points[0].x)/h[0];y[0]=(points[1].y-points[0].y)/h[0];
    x[dim-2]=(points[n-1].x-points[n-2].x)/h[n-2];
    y[dim-2]=(points[n-1].y-points[n-2].y)/h[n-2];
    for(std::size_t i=1;i+1<n;i++){
        const double l=h[i-1],r=h[i],l2=l*l,r2=r*r,l3=l2*l,r3=r2*r,l4=l3*l,r4=r3*r;
        auto& a=matrix[2*i];auto& b=matrix[2*i+1];
        a[2*i-2]=-24/l2;a[2*i-1]=-3/l;a[2*i]=36/r2-36/l2;
        a[2*i+1]=9/l+9/r;a[2*i+2]=24/r2;a[2*i+3]=-3/r;
        b[2*i-2]=-168/l3;b[2*i-1]=-24/l2;b[2*i]=-192/l3-192/r3;
        b[2*i+1]=36/l2-36/r2;b[2*i+2]=-168/r3;b[2*i+3]=24/r2;
        const double dxl=points[i].x-points[i-1].x,dxr=points[i+1].x-points[i].x;
        const double dyl=points[i].y-points[i-1].y,dyr=points[i+1].y-points[i].y;
        x[2*i]=60*dxr/r3-60*dxl/l3;y[2*i]=60*dyr/r3-60*dyl/l3;
        x[2*i+1]=-360*dxl/l4-360*dxr/r4;y[2*i+1]=-360*dyl/l4-360*dyr/r4;
    }
    if(!solve(matrix,x,y))return false;
    for(std::size_t i=0;i+1<n;i++){
        Segment seg;seg.width=h[i];
        const double bx[]={points[i].x,h[i]*x[2*i],h[i]*h[i]*x[2*i+1],points[i+1].x,h[i]*x[2*i+2],h[i]*h[i]*x[2*i+3]};
        const double by[]={points[i].y,h[i]*y[2*i],h[i]*h[i]*y[2*i+1],points[i+1].y,h[i]*y[2*i+2],h[i]*h[i]*y[2*i+3]};
        for(int j=0;j<6;j++)for(int k=0;k<6;k++){seg.x[k]+=bx[j]*basis[j][k];seg.y[k]+=by[j]*basis[j][k];}
        segments_.push_back(seg);
    }
    constexpr double nodes[]={0,-.5384693101056831,.5384693101056831,-.9061798459386640,.9061798459386640};
    constexpr double weights[]={.5688888888888889,.4786286704993665,.4786286704993665,.2369268850561891,.2369268850561891};
    constexpr int cells=32;
    for(std::size_t i=0;i<segments_.size();i++)for(int j=0;j<cells;j++){
        const double t=static_cast<double>(j)/cells,half=.5/cells;
        double length=0;
        for(int k=0;k<5;k++){
            auto d=evaluate(i,t+half+half*nodes[k],1);
            const double speed=std::hypot(d.x,d.y)*h[i];
            if(!std::isfinite(speed)||speed<1e-8){segments_.clear();cells_.clear();length_=0;return false;}
            length+=weights[k]*speed*half;
        }
        const auto left=evaluate(i,t,1),right=evaluate(i,t+1.0/cells,1);
        const double c=std::hypot(left.x,left.y)*h[i]/cells,e=std::hypot(right.x,right.y)*h[i]/cells;
        cells_.push_back({i,t,length_,length,c+e-2*length,3*length-2*c-e,c});
        length_+=length;
    }
    return std::isfinite(length_)&&length_>0;
}
Waypoint Spline::evaluate(std::size_t segment,double t,unsigned derivative) const {
    if(segment>=segments_.size()||!std::isfinite(t)||derivative>5){
        const double n=std::numeric_limits<double>::quiet_NaN();return {n,n};
    }
    const auto& s=segments_[segment];
    const double scale=std::pow(s.width,static_cast<int>(derivative));
    return {polynomial(s.x,t,derivative)/scale,polynomial(s.y,t,derivative)/scale};
}
PathPoint Spline::at(double distance) const {
    if(cells_.empty()||!std::isfinite(distance)){
        const double n=std::numeric_limits<double>::quiet_NaN();return {n,n,n,n};
    }
    distance=std::clamp(distance,0.0,length_);
    auto it=std::upper_bound(cells_.begin(),cells_.end(),distance,[](double d,const Cell& c){return d<c.start;});
    if(it!=cells_.begin())--it;
    const auto& c=*it;double f=std::clamp((distance-c.start)/c.length,0.0,1.0);
    for(int i=0;i<2;i++){
        const double slope=(3*c.a*f+2*c.b)*f+c.c;
        if(slope>1e-12)f=std::clamp(f-(((c.a*f+c.b)*f+c.c)*f-(distance-c.start))/slope,0.0,1.0);
    }
    const double t=c.t+f/32;
    const auto p=evaluate(c.segment,t),d=evaluate(c.segment,t,1),dd=evaluate(c.segment,t,2);
    const double speed=std::hypot(d.x,d.y);
    return {p.x,p.y,std::atan2(d.y,d.x),(d.x*dd.y-d.y*dd.x)/(speed*speed*speed)};
}
} // namespace arc::damp
