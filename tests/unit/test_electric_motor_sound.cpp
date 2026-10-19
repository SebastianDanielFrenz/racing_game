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

// ---- owner 2026-10-18: the whine layer and the base tone were too loud (EV at 40 km/h is wind/tyre noise) -----------------
#include "rg/tyre_sound.h"
#include <cstdio>
namespace {
// The pre-change mix: every layer at 0 dB.
rg::ElectricMotorSoundConfig legacy_levels() { rg::ElectricMotorSoundConfig c; c.base_db=0; c.gear_whine_db=0; c.inverter_db=0; return c; }
rg::ElectricMotorSoundConfig only(double base, double whine, double inverter) {
    rg::ElectricMotorSoundConfig c; c.base_db=base; c.gear_whine_db=whine; c.inverter_db=inverter; return c;
}
double layer_rms(const rg::ElectricMotorSoundConfig& c, double rpm, double torque_nm) {
    rg::ElectricMotorSound v; v.configure(c); v.set_state(rpm,torque_nm,0,true);
    settle(v);
    double sum=0; int n=0;
    for(int i=0;i<12;++i){auto b=v.render(8192,1);for(float x:b){sum+=double(x)*x;++n;}}
    return std::sqrt(sum/n);
}
double tyre_rms(double speed_mps) {
    rg::TyreSoundBank bank(4,1234,nullptr);
    rg::TyreSoundInput in; in.speed=speed_mps; in.tread_speed=speed_mps; in.load_n=4000;
    for(int w=0;w<4;++w) bank.update(w,in);
    double sum=0; std::size_t n=0;
    bank.render(48000,1.0f); // settle the smoothing
    for(float x:bank.render(96000,1.0f)){sum+=double(x)*x;++n;}
    return std::sqrt(sum/n);
}
double db(double a, double b) { return 20.0*std::log10(a/b); }
// car_electric: ratio 9, wheel radius 0.317 m -> motor rpm at a road speed.
double motor_rpm_at(double kmh) { return kmh/3.6/0.317*9.0*30.0/3.14159265358979; }
}

TEST_CASE("Electric motor layers: defaults cut the whine by 30 dB and the base tone by 8 dB", "[electric_audio]") {
 const rg::ElectricMotorSoundConfig d;
 CHECK(d.base_db<=-6.0); CHECK(d.base_db>=-10.0);
 CHECK(d.gear_whine_db<=-30.0); CHECK(d.inverter_db<=-30.0);
 for(double kmh : {40.0,100.0}) {
  const double rpm=motor_rpm_at(kmh), torque=kmh<60?60.0:120.0;
  const double base_old=layer_rms(only(0,-200,-200),rpm,torque), base_new=layer_rms(only(d.base_db,-200,-200),rpm,torque);
  const double whine_old=layer_rms(only(-200,0,-200),rpm,torque), whine_new=layer_rms(only(-200,d.gear_whine_db,-200),rpm,torque);
  const double inv_old=layer_rms(only(-200,-200,0),rpm,torque), inv_new=layer_rms(only(-200,-200,d.inverter_db),rpm,torque);
  INFO("km/h " << kmh);
  CHECK(db(base_old,base_new)>=6.0); CHECK(db(base_old,base_new)<=10.0);
  CHECK(db(whine_old,whine_new)>=29.9);
  CHECK(db(inv_old,inv_new)>=29.9);
  // With the defaults the high-frequency layers are far below the motor tone itself.
  CHECK(whine_new+inv_new < 0.1*base_new);
 }
}

TEST_CASE("Electric motor layers: all at 0 dB reproduces the old mix and a layer at the floor is silent", "[electric_audio]") {
 rg::ElectricMotorSound legacy,silent_whine;
 legacy.configure(legacy_levels());
 silent_whine.configure(only(0,-90,-90));
 legacy.set_state(4000,300,0,true);silent_whine.set_state(4000,300,0,true);
 settle(legacy);settle(silent_whine);
 const auto a=legacy.render(4096,1),b=silent_whine.render(4096,1);
 CHECK(a!=b);                       // whine and inverter are really gone from b
 CHECK(rms(a)>rms(b));
 rg::ElectricMotorSound all_off; all_off.configure(only(-90,-90,-90)); all_off.set_state(4000,300,0,true); settle(all_off);
 CHECK(rms(all_off.render(4096,1))==0);
 rg::ElectricMotorSound weird; weird.configure(only(std::numeric_limits<double>::quiet_NaN(),1e9,-1e9)); weird.set_state(4000,300,0,true);
 for(float s:weird.render(1024,1))REQUIRE((std::isfinite(s)&&std::abs(s)<=1));
}

TEST_CASE("Electric motor level report: RMS per layer at 40 and 100 km/h, before and after, next to the tyre bank", "[.][diag][electric_audio]") {
 const rg::ElectricMotorSoundConfig d;
 for(double kmh : {40.0,100.0}) {
  const double rpm=motor_rpm_at(kmh), torque=kmh<60?60.0:120.0;
  std::printf("%5.0f km/h motor %5.0f rpm %4.0f Nm | base %.5f -> %.5f | gear whine %.5f -> %.5f | inverter %.5f -> %.5f | total %.5f -> %.5f | tyres(4 wheels) %.5f\n",
   kmh,rpm,torque,
   layer_rms(only(0,-200,-200),rpm,torque),layer_rms(only(d.base_db,-200,-200),rpm,torque),
   layer_rms(only(-200,0,-200),rpm,torque),layer_rms(only(-200,d.gear_whine_db,-200),rpm,torque),
   layer_rms(only(-200,-200,0),rpm,torque),layer_rms(only(-200,-200,d.inverter_db),rpm,torque),
   layer_rms(legacy_levels(),rpm,torque),layer_rms(d,rpm,torque),tyre_rms(kmh/3.6));
 }
}
