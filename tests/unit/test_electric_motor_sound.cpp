#include "rg/electric_motor_sound.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
namespace {
double rms(const std::vector<float>& v){double sum=0;for(auto x:v)sum+=x*x;return std::sqrt(sum/v.size());}
int crossings(const std::vector<float>& v){int n=0;for(size_t i=1;i<v.size();++i)if(v[i-1]<0&&v[i]>=0)++n;return n;}
void settle(rg::ElectricMotorSound& v){for(int i=0;i<12;++i)v.render(8192,1);}
}
TEST_CASE("Electric motor sound follows shaft speed and load, with silent standstill", "[electric_audio]") {
 rg::ElectricMotorSound stopped;stopped.set_state(0,1200,0,true);settle(stopped);CHECK(rms(stopped.render(8192,1))==0);
 rg::ElectricMotorSound low,high,loaded;
 low.set_state(1500,0,0,true);high.set_state(6000,0,0,true);loaded.set_state(1500,1200,200000,true);
 settle(low);settle(high);settle(loaded);
 const auto a=low.render(8192,1),b=high.render(8192,1),c=loaded.render(8192,1);
 CHECK(rms(a)>0.01);CHECK(crossings(b)>3*crossings(a));CHECK(rms(c)>2*rms(a));
 high.set_state(6000,0,0,false);settle(high);CHECK(rms(high.render(8192,1))<1e-6);
 CHECK(rms(loaded.render(8192,0))==0);
}
TEST_CASE("Electric motor synthesis is bounded and independent of render block size", "[electric_audio]") {
 rg::ElectricMotorSound a,b;
 a.set_state(-10000,-600,-250000,true);b.set_state(-10000,-600,-250000,true);
 const auto whole=a.render(8192,1);auto split=b.render(4096,1);const auto tail=b.render(4096,1);split.insert(split.end(),tail.begin(),tail.end());
 CHECK(whole==split);
 for(float s:whole)REQUIRE((std::isfinite(s)&&std::abs(s)<=1));
 a.set_state(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),0,true);
 for(float s:a.render(1024,1))REQUIRE(std::isfinite(s));
 CHECK(a.render(-1,1).empty());CHECK(a.render(1000000,1).size()==16384);
}
