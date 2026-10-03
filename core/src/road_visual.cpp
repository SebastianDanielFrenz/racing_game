#include "rg/road_visual.h"
#include "rg/road_structures.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <map>
namespace rg {
namespace {
std::string tag(const g2m::RoadGraphWay& way,std::string_view key) {for(const auto& [k,v]:way.tags)if(k==key)return v;return {};}
int integer(const std::string& s) {int n=0;auto r=std::from_chars(s.data(),s.data()+s.size(),n);return r.ec==std::errc{}&&r.ptr==s.data()+s.size()&&n>=0&&n<=30?n:0;}
}
RoadVisualMesh build_road_visual(const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadVisualGround& ground,double lift) {
 RoadVisualMesh mesh;const auto decks=build_road_decks(geometry);
 std::map<std::int64_t,int> degree;for(const auto& edge:geometry.source.graph.edges){++degree[edge.from_node];++degree[edge.to_node];}
 const double ox=key.min_easting(),oy=key.min_northing();
 for(const auto& entry:geometry.entries) {
  if(!entry.profile)continue;const auto& p=*entry.profile;if(p.attributes.land_class!=g2m::LandClass::PavedRoad)continue;
  auto it=std::lower_bound(geometry.source.graph.ways.begin(),geometry.source.graph.ways.end(),entry.way_id,[](const auto& w,auto id){return w.osm_id<id;});
  if(it==geometry.source.graph.ways.end()||it->osm_id!=entry.way_id)continue;const auto& w=*it;
  const auto highway=tag(w,"highway");if(highway=="footway"||highway=="path"||highway=="cycleway"||highway=="steps"||highway=="pedestrian")continue;
  const auto direction=tag(w,"oneway");const bool one=direction=="yes"||direction=="1"||direction=="true"||direction=="-1"||(direction.empty()&&(highway=="motorway"||tag(w,"junction")=="roundabout"));
  const bool fast=highway=="motorway"||highway=="trunk"||highway=="motorway_link"||highway=="trunk_link";
  const bool edge_lines=fast||highway=="primary"||highway=="secondary"||highway=="tertiary";
  const double width=p.attributes.width_mm/1000.0,half=width*.5;
  int lanes=p.attributes.lanes;const int f=integer(tag(w,"lanes:forward")),b=integer(tag(w,"lanes:backward"));
  if(lanes==0&&f+b>0)lanes=f+b;
  if(lanes==0)lanes=one?std::clamp(static_cast<int>(std::round(width/3.25)),1,fast?4:2):(width>=5.5?2:1);
  lanes=std::clamp(lanes,1,30);int forward=one?(direction=="-1"?0:lanes):(f>0?f:(b>0?lanes-b:(lanes+1)/2));forward=std::clamp(forward,0,lanes);
  const bool marking=tag(w,"lane_markings")!="no"&&width/lanes>=2.4&&highway!="living_street";
  const bool solid=tag(w,"overtaking")=="no";
  const int flags=(marking?1:0)|(edge_lines?2:0)|(solid?4:0)|(fast?8:0);
  const double length=p.reference.length_m;if(length<=.01)continue;
  const bool start_junction=p.stretch.start_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.start_ref]]>2;
  const bool end_junction=p.stretch.end_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.end_ref]]>2;
  const int count=std::max(1,static_cast<int>(std::ceil(length))); // metre spacing matches the terrain detail.
  const int across=std::max(2,static_cast<int>(std::ceil(width)));const double shoulder=.30;
  for(int i=0;i<count;++i) {
   const double stations[2]={length*i/count,length*(i+1)/count};auto mid=p.reference.at((stations[0]+stations[1])*.5);if(!mid)continue;
   if(std::floor(mid->x/1024)!=key.x||std::floor(mid->y/1024)!=key.y)continue;
   const auto first=mesh.vertices.size();bool valid=true;
   for(double station:stations) {
    auto xy=p.reference.at(station);auto z=p.at(station);if(!xy||!z){valid=false;break;}
    const double nx=-std::sin(xy->heading),ny=std::cos(xy->heading);
    const bool separated=p.attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&station>=d->start_station&&station<=d->end_station;});
    const double extent=half+(separated?0:shoulder);
    double fade=1;if(start_junction)fade=std::min(fade,std::clamp((station-3)/8.0,0.0,1.0));if(end_junction)fade=std::min(fade,std::clamp((length-station-3)/8.0,0.0,1.0));
    for(int j=0;j<=across;++j) {
     const double lateral=-extent+2*extent*j/across,e=xy->x+nx*lateral,n=xy->y+ny*lateral;
     const auto height=separated?std::optional<double>{z->height_m-p.attributes.crown_per_mille/1000.0*(std::sqrt(lateral*lateral+.25)-.5)}:ground(e,n);
     if(!height){valid=false;break;}
     mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift),static_cast<float>(lateral),static_cast<float>(station),static_cast<float>(width),static_cast<float>(fade),static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags)});
    }
    if(!valid)break;
   }
   if(!valid){mesh.vertices.resize(first);continue;}
   for(int j=0;j<across;++j){const auto a=static_cast<std::int32_t>(first+j),b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
  }
  // Direction arrows only where OSM explicitly records turn:lanes.
  const auto arrows=[&](std::string turns,int direction,int lane_start,int lane_count) {
   if(turns.empty()||lane_count<=0||length<20)return;
   std::vector<std::string> tokens;std::size_t begin=0;
   while(true){auto end=turns.find('|',begin);tokens.push_back(turns.substr(begin,end-begin));if(end==std::string::npos)break;begin=end+1;}
   if(static_cast<int>(tokens.size())!=lane_count)return;
   const double station=direction>0?length-15:15;
   auto center=p.reference.at(station);if(!center||std::floor(center->x/1024)!=key.x||std::floor(center->y/1024)!=key.y)return;
   const double lane_width=width/lanes;if(lane_width<2.4)return;
   for(int lane=0;lane<lane_count;++lane){
    const double offset=direction>0?-half+(lane_start+lane_count-lane-.5)*lane_width:-half+(lane_start+lane+.5)*lane_width;
    const auto vertex=[&](double side,double along)->std::optional<RoadVisualVertex>{
     const double st=station+direction*along;auto xy=p.reference.at(st);auto z=p.at(st);if(!xy||!z)return {};
     const double lateral=offset+direction*side,e=xy->x-std::sin(xy->heading)*lateral,n=xy->y+std::cos(xy->heading)*lateral;
     const bool separated=p.attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&st>=d->start_station&&st<=d->end_station;});
     auto height=separated?std::optional<double>{z->height_m-p.attributes.crown_per_mille/1000.0*(std::sqrt(lateral*lateral+.25)-.5)}:ground(e,n);if(!height)return {};
     return RoadVisualVertex{static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+.003),0,0,static_cast<float>(width),1,0,0,16};
    };
    const auto triangle=[&](double ax,double ay,double bx,double by,double cx,double cy){
     auto a=vertex(ax,ay),b=vertex(bx,by),c=vertex(cx,cy);if(!a||!b||!c)return;const auto first=static_cast<std::int32_t>(mesh.vertices.size());
     mesh.vertices.insert(mesh.vertices.end(),{*a,*b,*c});mesh.indices.insert(mesh.indices.end(),{first,first+1,first+2});
    };
    const auto stroke=[&](double ax,double ay,double bx,double by){const double dx=bx-ax,dy=by-ay,len=std::hypot(dx,dy);if(len<.01)return;const double nx=-dy/len*.11,ny=dx/len*.11;triangle(ax+nx,ay+ny,ax-nx,ay-ny,bx+nx,by+ny);triangle(ax-nx,ay-ny,bx-nx,by-ny,bx+nx,by+ny);};
    const auto& token=tokens[lane];
    const bool through=token.find("through")!=std::string::npos;
    const bool left=token.find("left")!=std::string::npos,right=token.find("right")!=std::string::npos;
    if(!through&&!left&&!right)continue;
    stroke(0,-2,0,0);
    if(through){stroke(0,0,0,1.1);triangle(-.45,.85,.45,.85,0,1.7);}
    if(left){stroke(0,0,.7,.5);triangle(.65,.15,.65,.85,1.15,.5);}
    if(right){stroke(0,0,-.7,.5);triangle(-.65,.15,-.65,.85,-1.15,.5);}
   }
  };
  if(one)arrows(tag(w,"turn:lanes"),direction=="-1"?-1:1,0,lanes);
  else {arrows(tag(w,"turn:lanes:forward"),1,0,forward);arrows(tag(w,"turn:lanes:backward"),-1,forward,lanes-forward);}
 }
 return mesh;
}
}
