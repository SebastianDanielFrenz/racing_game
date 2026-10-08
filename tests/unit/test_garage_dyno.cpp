#include "rg/garage_dyno.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <chrono>
#include <cmath>
#include <thread>
namespace {
rg::DynoRequest request(const std::string& key) {
    const std::string root=RG_SOURCE_DIR;
    std::string error;
    auto catalog=rg::load_vehicle_catalog(root+"/data/vehicles/catalog.json",root,&error);
    REQUIRE(catalog.has_value());
    auto table=rg::load_setup_options(root+"/data/vehicles/setup_options.json",&error);
    REQUIRE(table.has_value());
    rg::DynoRequest r;
    r.entry=*catalog->find("car_hyper"); r.table=*table;
    r.context.work_root=root+"/out/dyno_unit_work";
    r.context.engine_map_cache_dir=root+"/out/godot_engine_cache";
    r.setup.vehicle_id=r.entry.id; r.key=key;
    return r;
}
rg::DynoResult finished(rg::GarageDyno& dyno) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(120);
    while(dyno.result().busy && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto result=dyno.result();
    INFO(result.error);
    REQUIRE_FALSE(result.busy);
    REQUIRE(result.error.empty());
    REQUIRE(result.points.size()==33);
    return result;
}
}
TEST_CASE("garage dyno: actual WOT output, power units, newest setup wins", "[garage_dyno]") {
    rg::GarageDyno dyno;
    dyno.request(request("stock"));
    const auto stock=finished(dyno);
    double peak=0;
    for(const auto& p:stock.points) {
        CHECK(std::isfinite(p.torque_nm));
        CHECK(p.power_kw==Catch::Approx(p.torque_nm*p.rpm*3.14159265358979323846/30000.0));
        peak=std::max(peak,p.power_kw);
    }
    CHECK(peak>900);
    CHECK(peak<1100);
    dyno.request(request("stock"));
    CHECK_FALSE(dyno.result().busy);
    CHECK(dyno.result().revision==stock.revision);
    auto stale=request("stale"); stale.setup.values["turbo_install"]=std::string("hyper_single");
    dyno.request(stale);
    CHECK(dyno.result().points.size()==stock.points.size());
    CHECK(dyno.result().key=="stock");
    auto newest=request("NA"); newest.setup.values["turbo_install"]=std::string("hyper_na");
    dyno.request(newest);
    const auto na=finished(dyno);
    CHECK(na.key=="NA");
    CHECK(na.revision>stock.revision);
    double na_peak=0;
    for(const auto& p:na.points) na_peak=std::max(na_peak,p.power_kw);
    CHECK(na_peak<peak*0.75);
    dyno.request(request("cancelled"));
    dyno.cancel();
    CHECK_FALSE(dyno.result().busy);
    CHECK(dyno.result().key=="NA");
    CHECK(dyno.result().revision==na.revision);
    auto invalid=request("invalid"); invalid.setup.values["turbo_install"]=std::string("not_a_part");
    dyno.request(invalid);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(dyno.result().busy && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    CHECK_FALSE(dyno.result().error.empty());
    CHECK(dyno.result().key=="NA");
    CHECK(dyno.result().revision==na.revision);
}
