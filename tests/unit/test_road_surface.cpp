#define _CRT_SECURE_NO_WARNINGS
#include "rg/road_surface.h"
#include "ps/world/world.h"
#include "rg/world_terrain.h"
#include "rg/route_check.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstdlib>
#include <cstdio>
namespace {
g2m::RoadGeomTile geometry(bool bridge=false) {
    g2m::RoadGeomTile tile;
    g2m::RoadProfile p;
    p.attributes.width_mm=10000; p.attributes.shoulder_mm=1000;
    p.attributes.blend_mm=5000; p.attributes.crown_per_mille=0;
    p.attributes.land_class=g2m::LandClass::PavedRoad; p.attributes.bridge=bridge;
    p.reference.length_m=512;
    g2m::RoadReferenceSegment s; s.curve={0,128.5,0,512,0,0};
    p.reference.segments.push_back(s);
    p.vertical.segments.push_back({0,512,100,125.6,.05,.05});
    tile.entries.push_back({1,{0,1},std::move(p),{},{},{}});
    return tile;
}
std::unique_ptr<g2m::HeightTile> terrain(int x) {
    auto t=std::make_unique<g2m::HeightTile>();t->key={{32},0,x,0};t->h.fill(90*256);return t;
}
}
TEST_CASE("Accepted road profiles remove depressions while preserving grade and tile seams", "[road_surface]") {
    const rg::RoadSurfacePatch patch(geometry());
    auto left=terrain(0),right=terrain(1);
    left->h[128*256+90]=g2m::kHeightNoData;
    REQUIRE(patch.apply(*left)>0); REQUIRE(patch.apply(*right)>0);
    CHECK(left->h[128*256+90]==g2m::kHeightNoData);
    CHECK(left->h[0]==90*256);
    for(int x=1;x<256;++x) {
        if(x==90 || x==91) continue;
        CHECK(std::abs((left->h[128*256+x]-left->h[128*256+x-1])/256.0-.05)<.004);
    }
    CHECK((right->h[128*256]-left->h[128*256+255])/256.0==Catch::Approx(.05).margin(.004));
    CHECK(left->h[128*256+20]/256.0==Catch::Approx(101.025).margin(.004));
}
TEST_CASE("Raised carriageway verge leaves the driving surface unchanged", "[road_surface]") {
    const rg::RoadSurfacePatch flat(geometry(),0),raised(geometry(),.06);
    auto a=terrain(0),b=terrain(0);flat.apply(*a);raised.apply(*b);
    for(int y=124;y<=133;++y)for(int x=1;x<255;++x)CHECK(a->h[y*256+x]==b->h[y*256+x]);
    CHECK((a->h[134*256+100]-b->h[134*256+100])/256.0==Catch::Approx(.06).margin(.004));
    CHECK(a->h[150*256+100]==b->h[150*256+100]);
    const rg::RoadSurfacePatch bridge(geometry(true),.06);auto t=terrain(0);CHECK(bridge.apply(*t)==0);
}
TEST_CASE("Terrain cannot protrude through a bridge collision slab", "[road_surface]") {
    auto t=terrain(0);t->h.fill(130*256);t->h[128*256+90]=g2m::kHeightNoData;
    const rg::RoadSurfacePatch bridge(geometry(true));REQUIRE(bridge.apply(*t)>0);
    CHECK(t->h[128*256+100]/256.0==Catch::Approx(104.475).margin(.004));
    CHECK(t->h[128*256+90]==g2m::kHeightNoData);CHECK(t->h[0]==130*256);
}
TEST_CASE("Grade-separated road profiles never fill the ground underneath", "[road_surface]") {
    const rg::RoadSurfacePatch bridge(geometry(true));auto t=terrain(0);
    CHECK(bridge.separated==1); CHECK(bridge.apply(*t)==0);
}
TEST_CASE("B8 real-data road-profile roughness comparison", "[.][realdata][road_surface]") {
    if(!std::getenv("RG_G2M_HOME")) { SKIP("Set RG_G2M_HOME for the local B8 store"); }
    std::string error;
    auto config=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config.json",&error);
    REQUIRE(config.has_value());
    auto route=rg::load_route(std::string(RG_SOURCE_DIR)+"/data/routes/home_r1_drive.json",&error);
    REQUIRE(route.has_value());
    auto paved=rg::WorldTerrain::open(*config,&error);REQUIRE(paved);
    config->physics.road_surfaces.enabled=false;
    config->terrain_smoothing.enabled=false;
    auto raw=rg::WorldTerrain::open(*config,&error);REQUIRE(raw);
    double raw_rough=0,paved_rough=0;std::size_t count=0,changed=0;
    // Compare the complete B8 route, including its tagged and inferred spans.
    double previous_raw=0,previous_paved=0,previous_grade_raw=0,previous_grade_paved=0,length=0;
    auto* csv=std::fopen((std::string(RG_SOURCE_DIR)+"/out/b8_surface_comparison.csv").c_str(),"w");
    REQUIRE(csv);std::fputs("easting,northing,raw_height,road_height\n",csv);
    for(std::size_t i=0;i<route->waypoints.size();++i) {
        const auto& point=route->waypoints[i];
        const double e=point.x+config->session_origin_utm.e0,n=point.y+config->session_origin_utm.n0;
        auto lookup_raw=[&](const g2m::TileKey& key){return raw->height_tile(key);};
        auto lookup_paved=[&](const g2m::TileKey& key){return paved->height_tile(key);};
        auto a=rg::sample_l0_height(lookup_raw,{32},0,0,e,n);
        auto b=rg::sample_l0_height(lookup_paved,{32},0,0,e,n);
        REQUIRE(a.has_value()); REQUIRE(b.has_value());
        // Ground remains underneath a bridge. Follow only decks aligned with
        // this road, so an overhead crossing does not become the route height.
        const auto& next=route->waypoints[std::min(i+1,route->waypoints.size()-1)];
        const auto& prev=route->waypoints[i==0?0:i-1];
        const double rx=next.x-prev.x,ry=next.y-prev.y,rlen=std::hypot(rx,ry);
        for(const auto& d:paved->road_decks()) {
            const auto& m=d->mesh;const auto rows=m.positions.size()/3/10;
            for(std::size_t row=0;row+1<rows;++row) {
                const double dx=m.positions[(row+1)*15+6]-m.positions[row*15+6],dy=m.positions[(row+1)*15+7]-m.positions[row*15+7];
                if(rlen<.001||std::abs(dx*rx+dy*ry)<.8*rlen*std::hypot(dx,dy)) continue;
                for(std::size_t t=row*24;t<row*24+24;t+=3) {
                    const auto ia=static_cast<std::size_t>(m.indices[t])*3,ib=static_cast<std::size_t>(m.indices[t+1])*3,ic=static_cast<std::size_t>(m.indices[t+2])*3;
                    const double ax=m.origin[0]+m.positions[ia],ay=m.origin[1]+m.positions[ia+1];
                    const double bx=m.origin[0]+m.positions[ib],by=m.origin[1]+m.positions[ib+1];
                    const double cx=m.origin[0]+m.positions[ic],cy=m.origin[1]+m.positions[ic+1];
                    const double det=(by-cy)*(ax-cx)+(cx-bx)*(ay-cy);if(std::abs(det)<1e-9) continue;
                    const double u=((by-cy)*(e-cx)+(cx-bx)*(n-cy))/det,v=((cy-ay)*(e-cx)+(ax-cx)*(n-cy))/det;
                    if(u>=0&&v>=0&&u+v<=1) *b=std::max(*b,u*m.positions[ia+2]+v*m.positions[ib+2]+(1-u-v)*m.positions[ic+2]);
                }
            }
        }
        std::fprintf(csv,"%.3f,%.3f,%.4f,%.4f\n",e,n,*a,*b);
        if(std::abs(*a-*b)>.01) ++changed;
        if(i>0) {
            const auto& q=route->waypoints[i-1];const double ds=std::hypot(point.x-q.x,point.y-q.y);length+=ds;
            if(ds>.01) {
                const double ga=(*a-previous_raw)/ds,gb=(*b-previous_paved)/ds;
                if(count>0) {raw_rough+=std::abs(ga-previous_grade_raw);paved_rough+=std::abs(gb-previous_grade_paved);}
                previous_grade_raw=ga;previous_grade_paved=gb;++count;
            }
        }
        previous_raw=*a;previous_paved=*b;if(length>10000) break;
    }
    std::fclose(csv);
    std::printf("B8_SURFACE length_m=%.1f changed=%zu samples=%zu raw_roughness=%.6f road_roughness=%.6f\n",length,changed,count,raw_rough,paved_rough);
    CHECK(changed>0);CHECK(paved_rough<raw_rough*.8);
}

TEST_CASE("Inspect reported B8 structures", "[.][realdata][structures]") {
    if(!std::getenv("RG_G2M_HOME")) SKIP("Local source store required");
    std::string error;auto config=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config.json",&error);REQUIRE(config);
    auto terrain=rg::WorldTerrain::open(*config,&error);REQUIRE(terrain);
    const double points[][2]={{50.121806,8.524495},{50.123512,8.515598},{50.126719,8.502332},{50.130483,8.491895},{50.135940,8.486279},{50.141900,8.479882},{50.149487,8.466567}};
    g2m::geo::Utm utm;
    for(const auto& point:points) {
        const auto p=utm.forward({32},point[0],point[1]);
        std::printf("STRUCTURE_POINT lat=%.6f lon=%.6f E=%.3f N=%.3f\n",point[0],point[1],p.easting,p.northing);
        auto geometry=terrain->road_geometry_at(p.easting,p.northing);REQUIRE(geometry);
        const auto decks=terrain->road_decks();
        const std::vector<std::vector<std::int64_t>> expected{{8099864,8099866},{5558250},{1096866567,1096866569},{14799333},{23384258,5217272,319863002},{23091108,5217272,319863002},{32275368}};
        const auto index=static_cast<std::size_t>(&point-points);
        for(const auto id:expected[index]) {
            INFO("Required structure way "<<id);
            CHECK(std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==id;}));
        }
        double nearest=1e9;std::shared_ptr<const rg::RoadDeck> nearest_deck;
        for(const auto& d:decks) for(std::size_t v=0;v<d->mesh.positions.size();v+=3) {
            const double distance=std::hypot(d->mesh.origin[0]+d->mesh.positions[v]-p.easting,d->mesh.origin[1]+d->mesh.positions[v+1]-p.northing);
            if(distance<nearest) {nearest=distance;nearest_deck=d;}
        }
        INFO("Reported crossing "<<point[0]<<","<<point[1]<<" nearest deck="<<nearest);
        REQUIRE(nearest<40);REQUIRE(nearest_deck);
        ps::WorldConfig wc;wc.job_workers=1;ps::World world(wc);
        ps::MeshShape shape;const auto& mesh=nearest_deck->mesh;
        for(std::size_t v=0;v<mesh.positions.size();v+=3) shape.vertices.push_back({mesh.positions[v],mesh.positions[v+1],mesh.positions[v+2]});
        shape.indices.assign(mesh.indices.begin(),mesh.indices.end());
        ps::BodyDesc body;body.motion=ps::BodyMotionType::Static;body.shape=std::move(shape);world.create_body(body);
        // A point inside the first deck quad verifies actual engine collision
        // winding, not only that a render mesh was generated.
        const ps::Vec3 test_point{(mesh.positions[6]+mesh.positions[21])/2.0,(mesh.positions[7]+mesh.positions[22])/2.0,(mesh.positions[8]+mesh.positions[23])/2.0};
        const auto hit=world.backend().ray_cast(test_point+ps::Vec3{0,0,2},{0,0,-1},4);
        CHECK(hit.hit);CHECK(hit.point.z==Catch::Approx(test_point.z).margin(.05)); const auto contact=world.backend().shape_cast(ps::SphereShape{.1},ps::Pose{test_point+ps::Vec3{0,0,2}},{0,0,-1},4); CHECK(contact.hit);CHECK(contact.normal.z>.95);
        for(const auto& way:geometry->source.graph.ways) {
            double distance=1e9;
            for(std::size_t i=1;i<way.node_ids.size();++i) {
                const auto node=[&](auto id){return std::lower_bound(geometry->source.graph.nodes.begin(),geometry->source.graph.nodes.end(),id,[](const auto& n,auto value){return n.osm_id<value;})->position;};
                const auto a=node(way.node_ids[i-1]),b=node(way.node_ids[i]);
                const double dx=(b.x-a.x)/1000.0,dy=(b.y-a.y)/1000.0;
                const double x=p.easting-a.x/1000.0,y=p.northing-a.y/1000.0;
                const double t=std::clamp((x*dx+y*dy)/std::max(1e-9,dx*dx+dy*dy),0.0,1.0);
                distance=std::min(distance,std::hypot(x-t*dx,y-t*dy));
            }
            if(distance>45) continue;
            std::printf("STRUCTURE_WAY id=%lld distance=%.1f refs=%zu",static_cast<long long>(way.osm_id),distance,way.node_ids.size());
            for(const auto& tag:way.tags) if(tag.first=="highway"||tag.first=="bridge"||tag.first=="tunnel"||tag.first=="layer"||tag.first=="ref"||tag.first=="name") std::printf(" %s=%s",tag.first.c_str(),tag.second.c_str());
            std::puts("");
            for(const auto& entry:geometry->entries) if(entry.way_id==way.osm_id) std::printf("STRUCTURE_PROFILE refs=%u:%u accepted=%d reason=%s\n",entry.stretch.start_ref,entry.stretch.end_ref,entry.profile.has_value(),entry.decline?entry.decline->message.c_str():"ok");
        }
    }
}


TEST_CASE("Owner B8 crossing has no terrain protruding through its roof", "[.][realdata][owner_impact]") {
 if(!std::getenv("RG_G2M_HOME"))SKIP("Local source store required");std::string error;
 auto config=rg::load_world_config(std::string(RG_SOURCE_DIR)+"/data/world/world_config.json",&error);REQUIRE(config);
 auto terrain=rg::WorldTerrain::open(*config,&error);REQUIRE(terrain);
 auto geometry=terrain->road_geometry_at(463270.877,5553889.740);REQUIRE(geometry);
 const auto decks=terrain->road_decks();auto found=std::find_if(decks.begin(),decks.end(),[](const auto& d){return d->way_id==5217272&&d->inferred;});REQUIRE(found!=decks.end());
 const auto& deck=**found;const auto& mesh=deck.mesh;const auto rows=mesh.positions.size()/30;
 const auto sample=[&](double e,double n){return rg::sample_l0_height([&](const auto& k){return terrain->height_tile_shared(k).tile.get();},{32},0,0,e,n);};
 for(std::size_t i=6;i+6<rows;++i) {
  const auto a=i*15+6,b=(i+1)*15+6;const double dx=mesh.positions[b]-mesh.positions[a],dy=mesh.positions[b+1]-mesh.positions[a+1],length=std::hypot(dx,dy);
  const double station=deck.start_station+(deck.end_station-deck.start_station)*i/(rows-1);
  for(double lateral:{-1.,0.,1.}) {
   const double e=mesh.origin[0]+mesh.positions[a]-dy/length*lateral,n=mesh.origin[1]+mesh.positions[a+1]+dx/length*lateral;
   const auto ground=sample(e,n),top=rg::road_deck_height(deck,station,lateral);REQUIRE(ground);REQUIRE(top);
   INFO("roof station="<<station<<" lateral="<<lateral<<" ground="<<*ground<<" top="<<*top);
   CHECK(*ground<=*top-.45);
  }
 }
 for(const auto& entry:geometry->entries)if(entry.way_id==deck.way_id&&entry.profile) {
  auto start=entry.profile->at(deck.start_station),end=entry.profile->at(deck.end_station);REQUIRE(start);REQUIRE(end);
  auto roof_start=deck.vertical.front().at(deck.start_station),roof_end=deck.vertical.back().at(deck.end_station);REQUIRE(roof_start);REQUIRE(roof_end);
  CHECK(roof_start->grade==Catch::Approx(start->grade).margin(1e-8));CHECK(roof_end->grade==Catch::Approx(end->grade).margin(1e-8));
 }
}
