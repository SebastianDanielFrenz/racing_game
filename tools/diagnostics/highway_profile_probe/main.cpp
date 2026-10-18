#include "g2m/world/terrain_service.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_underpass.h"
#include "g2m/layer/road_rules.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <limits>
#include <iomanip>
#include <filesystem>
using namespace g2m;
int main(int argc,char** argv){
 if(argc!=4){std::cerr<<"usage: highway_profile_probe SOURCE_STORE SOURCE_SCOPE OUTPUT_DIR\n";return 1;}
 const std::filesystem::path output=argv[3];std::filesystem::create_directories(output);
 world::TerrainServiceConfig cfg;
 cfg.stores.source_dir=argv[1];
 cfg.stores.source_scope=argv[2];
 cfg.stores.derived_dir=output/"cache";
 cfg.e0_m=464000;cfg.n0_m=5559000;cfg.derive_workers=4;
 auto service=world::TerrainService::open(cfg);if(!service){std::cerr<<service.error().message;return 1;}
 auto key=parse_tile_path("utm32n/2/452/5419");auto ticket=(*service)->road_geom(*key,world::Priority::Gate);
 while(!ticket.wait_for(std::chrono::milliseconds(100))){}
 auto geom=ticket.value();if(!geom){std::cerr<<ticket.message();return 2;}
 std::ofstream entries(output/"highway_served_entries.csv");entries<<"way,start,end,profile,provenance,error\n";
 for(size_t i=0;i<geom->entries.size();++i){auto& e=geom->entries[i];entries<<e.way_id<<','<<e.stretch.start_ref<<','<<e.stretch.end_ref<<','<<bool(e.profile)<<','<<(i<geom->origins.size()?int(geom->origins[i].provenance):-1)<<",\""<<(e.decline?e.decline->message:"")<<"\"\n";}
 std::ofstream out(output/"highway_served_samples.csv");out<<std::setprecision(12);out<<"e,n,distance,way,station,height,grade,provenance\n";
 for(int j=0;j<=100;++j){double e=463650-4*j,n=5549905-2.27*j,best=1e9,h=0,g=0,station=0;long long way=0;int origin=-1;
 for(size_t i=0;i<geom->entries.size();++i){auto& ent=geom->entries[i];if(!ent.profile)continue;auto& p=*ent.profile;
 for(double s=0;s<=p.reference.length_m;s+=1){auto xy=p.reference.at(s);if(!xy)continue;double d=std::hypot(xy->x-e,xy->y-n);if(d<best){auto z=p.at(s);if(!z)continue;best=d;h=z->height_m;g=z->grade;way=ent.way_id;station=s;origin=i<geom->origins.size()?int(geom->origins[i].provenance):-1;}}}
 out<<e<<','<<n<<','<<best<<','<<way<<','<<station<<','<<h<<','<<g<<','<<origin<<'\n';}
 for(auto& edge:geom->source.graph.edges)if(edge.from_node==3306247908LL||edge.to_node==3306247908LL){std::cout<<"JOIN way="<<edge.way_id<<" from="<<edge.from_node<<" to="<<edge.to_node<<'\n';for(auto& w:geom->source.graph.ways)if(w.osm_id==edge.way_id){for(auto& tag:w.tags)std::cout<<tag.first<<'='<<tag.second<<' ';std::cout<<'\n';}}
 for(auto& w:geom->source.graph.ways)if(w.osm_id==323855192LL){double length=0,px=0,py=0;bool first=true;for(auto id:w.node_ids)for(auto& node:geom->source.graph.nodes)if(node.osm_id==id){double x=node.position.x/1000.0,y=node.position.y/1000.0;if(!first)length+=std::hypot(x-px,y-py);std::cout<<std::setprecision(12)<<"GAP_NODE id="<<id<<" e="<<x<<" n="<<y<<'\n';px=x;py=y;first=false;}std::cout<<"GAP_LENGTH m="<<length<<'\n';}
 for(double e=463000;e<=463800;e+=200)for(double n=5549400;n<=5550100;n+=200){auto k=tile_key_at(key->zone,0,e,n);if(k){auto t=(*service)->height(*k,world::Priority::Gate);while(!t.wait_for(std::chrono::milliseconds(100))){} }}
 std::vector<std::shared_ptr<const HeightTile>> held;std::vector<const HeightTile*> heights;
 for(double e=463000;e<=463800;e+=200)for(double n=5549400;n<=5550100;n+=200){auto k=tile_key_at(key->zone,0,e,n);if(k){auto h=(*service)->peek_height(*k);if(h&&std::find(heights.begin(),heights.end(),h.get())==heights.end()){heights.push_back(h.get());held.push_back(h);}}}
 auto dem=RoadDemSampler::make(heights);auto builder=RoadReferenceBuilder::make(geom->source.graph);
 if(dem&&builder){RoadUnderpassFinder finder(geom->source.graph,*dem,key->zone,builtin_road_rules_de().underpass);for(const auto& entry:geom->entries)if(entry.way_id==313879319||entry.way_id==323855192){ReferenceFitLimits lim;lim.allow_curvature_exceptions=true;auto reference=builder->fit_stretch(entry.way_id,entry.stretch,lim);if(reference){RoadUnderpassStats st;auto spans=finder.spans(entry.way_id,entry.stretch,*reference,&st);std::cout<<"MASK_STAT way="<<entry.way_id<<" start="<<entry.stretch.start_ref<<" end="<<entry.stretch.end_ref<<" crossings="<<st.crossings<<" served="<<st.served<<" shallow="<<st.shallow<<" no_rim="<<st.no_rim<<" too_long="<<st.too_long<<" reaches_end="<<st.reaches_end<<" dem_missing="<<st.dem_missing<<" covered="<<st.covered<<'\n';}}}
 std::ofstream ground(output/"highway_ground_cross_sections.csv");ground<<std::setprecision(12)<<"e,n,lateral,dem_height\n";
 for(int j=0;j<=80;++j){double e=463700-j*10,n=5549933.375-j*5.675;
  for(int lateral=-40;lateral<=40;lateral+=5){const double x=e-lateral*.4936,y=n+lateral*.8697;auto h=(*service)->surface_height_bilinear(x-cfg.e0_m,y-cfg.n0_m);if(h)ground<<x<<','<<y<<','<<lateral<<','<<*h<<'\n';}}
 for(const auto& w:geom->source.graph.ways){bool near=false;for(auto id:w.node_ids)for(const auto& node:geom->source.graph.nodes)if(node.osm_id==id&&std::hypot(node.position.x/1000.0-463374.298,node.position.y/1000.0-5549746.392)<150)near=true;if(near){std::cout<<"NEAR_WAY "<<w.osm_id;for(auto& tag:w.tags)std::cout<<' '<<tag.first<<'='<<tag.second;std::cout<<'\n';}}
 std::cout<<"entries="<<geom->entries.size()<<" underpasses="<<geom->underpasses.size()<<'\n';
}
