#include "rg/route_surface_mesh.h"

#include "g2m/core/geo/utm.h"
#include "g2m/layer/osm_roads.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

g2m::osm::OsmBuilder::TagList tags(std::initializer_list<std::pair<std::string, std::string>> values) {
    return {values.begin(), values.end()};
}

void add_utm_node(g2m::osm::OsmBuilder& builder, std::int64_t id, g2m::geo::UtmZone zone,
                  double e, double n) {
    const auto ll = g2m::geo::Utm{}.reverse(zone, e, n);
    builder.add_node(id, static_cast<std::int32_t>(std::llround(ll.lat * 1e7)),
                     static_cast<std::int32_t>(std::llround(ll.lon * 1e7)));
}

bool inside_triangle(double px, double py, const g2m::world::RoadRibbonVertex& a,
                     const g2m::world::RoadRibbonVertex& b, const g2m::world::RoadRibbonVertex& c) {
    const auto cross=[](double ax,double ay,double bx,double by,double x,double y) {
        return (bx-ax)*(y-ay)-(by-ay)*(x-ax);
    };
    const double e0=cross(a.x,a.y,b.x,b.y,px,py), e1=cross(b.x,b.y,c.x,c.y,px,py),
                 e2=cross(c.x,c.y,a.x,a.y,px,py);
    return (e0>=-1e-5&&e1>=-1e-5&&e2>=-1e-5)||(e0<=1e-5&&e1<=1e-5&&e2<=1e-5);
}

bool has_contact_class(g2m::world::RoadRibbonTile const& mesh, const g2m::world::RoadRibbonGround& ground,
                       std::uint32_t required_flags, g2m::geo::UtmZone zone,
                       g2m::LandClass land, g2m::SurfaceKind surface) {
    for (std::size_t i=0;i+2<mesh.indices.size();i+=3) {
        const auto& a=mesh.vertices[static_cast<std::size_t>(mesh.indices[i])];
        const auto& b=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+1])];
        const auto& c=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+2])];
        const auto flags=static_cast<std::uint32_t>(std::lround(a.flags));
        if ((flags & required_flags) != required_flags) continue;
        const double e=mesh.key.min_easting()+(a.x+b.x+c.x)/3.0;
        const double n=mesh.key.min_northing()+(a.y+b.y+c.y)/3.0;
        const double ce=std::round(e),cn=std::round(n);
        if (!inside_triangle(ce-mesh.key.min_easting(),cn-mesh.key.min_northing(),a,b,c)) continue;
        const auto tile_key=g2m::tile_key_at(zone,std::uint8_t{0},ce,cn);
        if (!tile_key) continue;
        g2m::TerrainClassTile classes;
        classes.reset(*tile_key);
        rg::stamp_route_surface_classes(classes,mesh,ground);
        const int x=static_cast<int>(ce-tile_key->min_easting()),y=static_cast<int>(cn-tile_key->min_northing());
        if(x<0||y<0||x>=g2m::kTerrainClassSamples||y>=g2m::kTerrainClassSamples) continue;
        const auto node=g2m::TerrainClassTile::index(x,y);
        if(classes.land_class[node]==static_cast<std::uint8_t>(land)&&
           classes.surface[node]==static_cast<std::uint8_t>(surface)&&
           classes.role[node]==static_cast<std::uint8_t>(g2m::TerrainRole::Sidewalk)) return true;
    }
    return false;
}

} // namespace

TEST_CASE("standalone foot and cycle paths stay out of car routing and share rendered grip geometry",
          "[route_surface][terrain_class]") {
    const g2m::geo::UtmZone zone{32,g2m::geo::Hemisphere::North};
    const auto left_key=g2m::tile_key_at(zone,std::uint8_t{2},500000.0,5550000.0);
    REQUIRE(left_key.has_value());
    auto right_key=*left_key;
    ++right_key.x;
    const double e0=left_key->min_easting(), n0=left_key->min_northing();
    g2m::osm::OsmBuilder builder;
    const std::vector<std::pair<double,double>> path_nodes{{e0+1018,n0+400},{e0+1021,n0+402},
        {e0+1024,n0+407},{e0+1029,n0+411},{e0+1034,n0+412}};
    for(std::size_t i=0;i<path_nodes.size();++i)
        add_utm_node(builder,static_cast<std::int64_t>(i+1),zone,path_nodes[i].first,path_nodes[i].second);
    for(std::size_t i=0;i<path_nodes.size();++i)
        add_utm_node(builder,static_cast<std::int64_t>(i+11),zone,path_nodes[i].first,path_nodes[i].second+5.0);
    for(std::size_t i=0;i<path_nodes.size();++i)
        add_utm_node(builder,static_cast<std::int64_t>(i+21),zone,path_nodes[i].first,path_nodes[i].second+10.0);
    builder.add_way(100,{1,2,3,4,5},tags({{"highway","path"},{"surface","gravel"},{"width","1.6"}}));
    builder.add_way(101,{11,12,13,14,15},tags({{"highway","footway"},{"footway","sidewalk"},{"surface","paving_stones"}}));
    builder.add_way(102,{21,22,23,24,25},tags({{"highway","cycleway"}}));
    const auto data=builder.build();
    REQUIRE(data);
    CHECK(g2m::extract_road_segments(*data,zone,g2m::RoadStyle::default_style()).empty()); // no car-road promotion
    const g2m::OsmTile osm{128,*data};
    const auto ground=[](double e,double) -> std::optional<double> { return 140.0+0.01*(e-500000.0); };
    g2m::world::RoadRibbonTile left,right;
    rg::append_standalone_path_mesh(left,osm,*left_key,ground,0.0);
    rg::append_standalone_path_mesh(right,osm,right_key,ground,0.0);
    REQUIRE(left.indices.size()>0);
    REQUIRE(right.indices.size()>0);
    REQUIRE(left.vertices.size()>0);
    REQUIRE(right.vertices.size()>0);
    std::size_t dirt_vertices=0,stone_vertices=0,cycleway_vertices=0;
    for(const auto* mesh:{&left,&right}) for(const auto& vertex:mesh->vertices) {
        const auto flags=static_cast<std::uint32_t>(std::lround(vertex.flags));
        if((flags&32768u)!=0&&(flags&8192u)!=0&&std::lround(vertex.lanes)==static_cast<int>(g2m::SurfaceKind::Gravel)) ++dirt_vertices;
        if((flags&32768u)!=0&&(flags&2048u)!=0&&std::lround(vertex.lanes)==static_cast<int>(g2m::SurfaceKind::PavingStones)) ++stone_vertices;
        if((flags&32768u)!=0&&std::lround(vertex.lanes)==static_cast<int>(g2m::SurfaceKind::Asphalt)) ++cycleway_vertices;
        CHECK(vertex.x>=-2.1f);
        CHECK(vertex.x<=1026.1f);
    }
    CHECK(dirt_vertices>0);
    CHECK(stone_vertices>0);
    CHECK(cycleway_vertices>0);
    CHECK(has_contact_class(left,ground,32768u|8192u,zone,g2m::LandClass::UnpavedRoad,g2m::SurfaceKind::Gravel));
    CHECK(has_contact_class(right,ground,32768u|8192u,zone,g2m::LandClass::UnpavedRoad,g2m::SurfaceKind::Gravel));
    CHECK(has_contact_class(left,ground,32768u|2048u,zone,g2m::LandClass::PavedRoad,g2m::SurfaceKind::PavingStones));
    CHECK(has_contact_class(right,ground,32768u|2048u,zone,g2m::LandClass::PavedRoad,g2m::SurfaceKind::PavingStones));
    CHECK(has_contact_class(left,ground,32768u,zone,g2m::LandClass::PavedRoad,g2m::SurfaceKind::Asphalt));
    CHECK(has_contact_class(right,ground,32768u,zone,g2m::LandClass::PavedRoad,g2m::SurfaceKind::Asphalt));
}

TEST_CASE("two-node paths follow terrain relief and keep metre texture coordinates",
          "[route_surface][path_drape]") {
    const g2m::geo::UtmZone zone{32,g2m::geo::Hemisphere::North};
    const auto key = g2m::tile_key_at(zone,std::uint8_t{2},500000.0,5550000.0);
    REQUIRE(key);
    const double e0=key->min_easting(), n0=key->min_northing();
    g2m::osm::OsmBuilder builder;
    add_utm_node(builder,1,zone,e0+100,n0+200);
    add_utm_node(builder,2,zone,e0+180,n0+200);
    builder.add_way(100,{1,2},tags({{"highway","path"},{"surface","dirt"},{"width","1.5"}}));
    const auto data=builder.build(); REQUIRE(data);
    const auto ground=[&](double e,double) -> std::optional<double> {
        const double t=(e-e0-100.0)/80.0;
        return 100.0-24.0*t*(1.0-t); // six-metre depression between the mapped endpoints
    };
    g2m::world::RoadRibbonTile mesh;
    rg::append_standalone_path_mesh(mesh,g2m::OsmTile{128,*data},*key,ground,0.0);
    REQUIRE_FALSE(mesh.indices.empty());
    float station_max=0;
    for(const auto& v:mesh.vertices) {
        station_max=std::max(station_max,v.v);
        CHECK(std::abs(v.u)<=.751f);
    }
    CHECK(station_max>79.9f);
    CHECK(station_max<80.1f);
    for(std::size_t i=0;i+2<mesh.indices.size();i+=3) {
        const auto& a=mesh.vertices[static_cast<std::size_t>(mesh.indices[i])];
        const auto& b=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+1])];
        const auto& c=mesh.vertices[static_cast<std::size_t>(mesh.indices[i+2])];
        const double e=e0+(static_cast<double>(a.x)+b.x+c.x)/3;
        const double n=n0+(static_cast<double>(a.y)+b.y+c.y)/3;
        const double z=(static_cast<double>(a.z)+b.z+c.z)/3;
        CHECK(std::abs(z-(*ground(e,n)+.01))<.02);
        for(const auto edge: {std::pair{&a,&b},std::pair{&b,&c},std::pair{&c,&a}}) {
            const double distance=std::hypot(edge.first->x-edge.second->x,edge.first->y-edge.second->y);
            CHECK(distance<2.0);
            CHECK(std::abs(std::hypot(edge.first->u-edge.second->u,edge.first->v-edge.second->v)-distance)<.001);
        }
    }
}
