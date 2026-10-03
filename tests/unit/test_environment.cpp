#include "rg/environment.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
TEST_CASE("Aerodynamic environment preserves defaults and pressure density consistency", "[environment]") {
 rg::EnvironmentConfig c;auto disabled=rg::sample_environment(c,1000,20);CHECK(disabled.air_density==1.225);
 c.enabled=true;auto sea=rg::sample_environment(c,0,0),hill=rg::sample_environment(c,1000,0);
 CHECK(sea.air_density==Catch::Approx(1.225).margin(.00001));CHECK(hill.air_density<sea.air_density);CHECK(hill.pressure_pa<sea.pressure_pa);
 CHECK(hill.air_density*287.05287*hill.temperature_k==Catch::Approx(hill.pressure_pa));
 c.lapse_k_per_m=0;CHECK(std::isfinite(rg::sample_environment(c,11000,0).air_density));
 c.altitude_density=false;CHECK(rg::sample_environment(c,1000,0).air_density==rg::sample_environment(c,0,0).air_density);
}
TEST_CASE("Wind gusts remain reproducible and bounded in simulation time", "[environment]") {
 rg::EnvironmentConfig c;c.enabled=true;c.wind_world_m_s={4,-2,0};c.gust_amplitude_m_s={2,3,1};
 for(int tick=0;tick<1000;++tick){auto a=rg::sample_environment(c,100,tick/240.0),b=rg::sample_environment(c,100,tick/240.0);
 CHECK(a.wind_world_m_s.x==b.wind_world_m_s.x);CHECK(std::abs(a.wind_world_m_s.x-4)<=2);CHECK(std::abs(a.wind_world_m_s.y+2)<=3);CHECK(std::abs(a.wind_world_m_s.z)<=1);}
}
