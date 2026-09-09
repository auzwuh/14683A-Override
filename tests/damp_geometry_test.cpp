#include "gen/damp/geometry.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace arc::damp;
void near(double a,double b,double tol=1e-7){assert(std::isfinite(a));assert(std::abs(a-b)<tol);}
int main(){
    Spline s; assert(!s.build({{0,0},{0,0}}));
    assert(s.build({{0,0},{2,0}}));near(s.length(),2);
    auto p=s.at(1);near(p.x,1);near(p.y,0);near(p.curvature,0);
    const std::vector<Waypoint> pts{{0,0},{.6,.1},{1,1},{2,1.2}};
    assert(s.build(pts));
    for(std::size_t i=0;i<s.size();i++) {
        auto a=s.evaluate(i,0),b=s.evaluate(i,1);near(a.x,pts[i].x);near(a.y,pts[i].y);
        near(b.x,pts[i+1].x);near(b.y,pts[i+1].y);
        if(i+1<s.size()) for(unsigned k=1;k<=4;k++) {
            auto l=s.evaluate(i,1,k),r=s.evaluate(i+1,0,k);
            near(l.x,r.x,1e-6);near(l.y,r.y,1e-6);
        }
    }
    near(s.at(0).curvature,0);near(s.at(s.length()).curvature,0);
    Spline mirror;assert(mirror.build({{0,0},{.6,-.1},{1,-1},{2,-1.2}}));near(s.length(),mirror.length());
    for(int i=0;i<=100;i++){
        auto a=s.at(s.length()*i/100),b=mirror.at(mirror.length()*i/100);
        near(a.x,b.x);near(a.y,-b.y);near(a.curvature,-b.curvature);
    }
    std::cout<<"DAMP geometry tests passed\n";
}
