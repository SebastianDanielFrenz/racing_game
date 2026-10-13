"""Build reusable cockpit controls. Stdlib only; pinned assetgen is read-only.

python tools/cockpitgen/build.py [--out PATH]
Each part tree has its mounting origin at zero. The library intentionally
overlaps: instantiate one part tree at a car socket, never the entire library.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "external/physics_sim/tools/assetgen"))
from assetgen.scene import Model
from assetgen import mesh as M
from assetgen.vec import translate

# Small geometry font, no downloaded fonts/textures/runtime text dependency.
# Coordinates are horizontal/vertical fractions of one character cell.
SEGMENTS = {
    "a": ((0, 1), (1, 1)), "b": ((1, 1), (1, .5)),
    "c": ((1, .5), (1, 0)), "d": ((0, 0), (1, 0)),
    "e": ((0, .5), (0, 0)), "f": ((0, 1), (0, .5)),
    "g": ((0, .5), (1, .5)), "h": ((0, 1), (.5, .5)),
    "i": ((1, 1), (.5, .5)), "j": ((.5, .5), (0, 0)),
    "k": ((.5, .5), (1, 0)), "l": ((.5, 1), (.5, .5)),
    "m": ((.5, .5), (.5, 0)), "n": ((0, 0), (1, 1))
}
GLYPHS = dict(A="abcefg", B="bcdgilm", C="adef", D="bcdilm",
              E="adefg", F="aefg", G="acdef", H="bcefg", I="adlm",
              J="bcde", K="efik", L="def", M="bcefhil", N="bcefhk",
              O="abcdef", P="abefg", R="abefgk", S="acdfg", T="alm",
              U="bcdef", V="efjn", W="bcefjkm", Y="him", Z="adij",
              **{"0":"abcdef", "1":"bc", "2":"abdeg", "3":"abcdg",
                 "4":"bcfg", "5":"acdfg", "6":"acdefg", "7":"abc",
                 "8":"abcdefg", "9":"abcdfg", "+":"glm", "-":"g", "/":"n"})


class Builder:
    def __init__(self, spec):
        self.spec, self.g = spec, spec["geometry"]
        self.m = Model("cockpit_parts", "Modular cockpit controls",
                       "Standalone control trees; instantiate only the selected part at its socket.",
                       "Every part_<id> root is its own mounting origin. +Z is the mounting normal, +X forward.")
        self.parts = []
        for name in ("housing", "cap", "rubber", "metal", "red"):
            self.m.mat(name, color=spec["palette"][name],
                       metallic=.85 if name == "metal" else 0,
                       roughness=.28 if name == "metal" else .48)

    def root(self, pid, title):
        name = self.m.node("part_" + pid, part_id=pid, mounting_normal=[0,0,1], mounting_up=[1,0,0])
        self.parts.append({"id":pid, "title":title, "root":name})
        return name

    def lamp(self, pid, segment, color):
        name = "lamp_" + pid + "_" + segment
        self.m.mat(name, color=tuple(c*.20 for c in color), roughness=.30,
                   light="ctl." + pid + "." + segment, emissive=color, light_strength=2.5)
        return name

    def legend(self, pid):
        name = "legend_" + pid
        self.m.mat(name, color=self.spec["palette"]["legend"], roughness=.55,
                   light="legend." + pid, emissive=self.spec["palette"]["legend"], light_strength=.8)
        return name

    def slab(self, node, pos, size, material, radius=None):
        # Chamfered top/bottom plus rounded footprint: far cheaper than a dense rounded box.
        radius = radius if radius is not None else self.g["button_corner_m"]
        x,y,z = pos
        sx,sy,sz = size
        bevel = min(.0007, sz*.2)
        sections = []
        for zz,inset in ((z-sz/2,bevel), (z-sz/2+bevel,0),
                         (z+sz/2-bevel,0), (z+sz/2,bevel)):
            outline = M.rrect(sx-2*inset,sy-2*inset,max(.00015,radius-inset),self.g["corner_segments"])
            sections.append([(x+a,y+b,zz) for a,b in outline])
        M.loft(self.m.mb, sections, material, node)

    def stroke(self, node, a, b, mat, width=None):
        # Flat, rectangular extrusion: six faces, 12 triangles, no circular tubes.
        width = width or self.g["legend_stroke_m"]
        dx,dy = b[0]-a[0],b[1]-a[1]
        length = math.hypot(dx,dy)
        nx,ny = -dy/length*width/2,dx/length*width/2
        poly = [(a[0]+nx,a[1]+ny),(a[0]-nx,a[1]-ny),
                (b[0]-nx,b[1]-ny),(b[0]+nx,b[1]+ny)]
        M.extrude(self.m.mb,poly,self.g["legend_depth_m"],mat,node,xf=translate((0,0,a[2])))

    def text(self,node,text,centre,height,max_width,mat):
        width = height*.58
        pitch = width*(1+self.g["legend_gap_factor"])
        total = pitch*len(text)-pitch+width
        if total>max_width:
            scale = max_width/total
            height,width,pitch,total = height*scale,width*scale,pitch*scale,max_width
        # Label's horizontal points toward the driver's right (-Y), top points +X.
        for i,ch in enumerate(text):
            for seg in GLYPHS.get(ch, ""):
                a,b = SEGMENTS[seg]
                def point(p):
                    return (centre[0]+(p[1]-.5)*height,
                            centre[1]+total/2-i*pitch-p[0]*width,centre[2])
                self.stroke(node,point(a),point(b),mat)

    def hit(self,pid,parent,point,half,kind="poke",axis=(0,0,-1),**extra):
        self.m.node("hit_"+pid,parent,pivot=point,
                    interaction={"frame":"ISO 8855, local to hit node", "kind":kind,
                                 "shape":"box", "half_extents_m":list(half),
                                 "poke_point":[0,0,0], "grab_point":[0,0,0],
                                 "press_axis":list(axis), **extra})

    def button(self,pid,label,led_count=1,color="green",parent=None,pos=(0,0,0),title=None):
        root = parent or self.root(pid,title or label)
        px,py,pz = pos
        g = self.g
        self.slab(root,(px,py,pz+.002),g["button_size_m"],"housing")
        z = pz+g["button_cap_height_m"]
        ctl = self.m.node("ctl_"+pid,root,(px,py,z),Model.prismatic((0,0,-1),0,g["button_travel_m"]),
                          behaviour={"spring_return":True,"press_at_m":g["button_travel_m"]*self.spec["interaction"]["press_fraction"],
                                     "release_at_m":g["button_travel_m"]*self.spec["interaction"]["release_fraction"]})
        self.slab(ctl,(px,py,z),g["button_cap_size_m"],"cap")
        surface = z+g["button_cap_size_m"][2]/2+g["legend_depth_m"]/2
        if pid=="hazard":
            mat=self.lamp(pid,"triangle",self.spec["palette"]["red"])
            for size in (.0045,.003):
                points=[(px+size,py,surface),(px-size*.65,py+size*.8,surface),(px-size*.65,py-size*.8,surface)]
                for a,b in zip(points,points[1:]+points[:1]): self.stroke(ctl,a,b,mat,.0006)
        else:
            self.text(ctl,label,(px+.0006,py,surface),g["legend_height_m"],.014,self.legend(pid))
            if pid=="tc":
                # Small plan-view car and paired skid lines above the TC legend.
                mat=self.legend(pid)
                car=[(px+.0035,py-.002,surface),(px+.0075,py-.002,surface),
                     (px+.0075,py+.002,surface),(px+.0035,py+.002,surface)]
                for a,b in zip(car,car[1:]+car[:1]): self.stroke(ctl,a,b,mat)
                for y in (-.0015,.0015):
                    self.stroke(ctl,(px+.0035,py+y,surface),(px+.0027,py+y+.0005,surface),mat)
                    self.stroke(ctl,(px+.0027,py+y+.0005,surface),(px+.002,py+y,surface),mat)
        for i in range(led_count):
            mat=self.lamp(pid,"led"+str(i),self.spec["palette"][color])
            self.slab(ctl,(px-.007,py+(i-(led_count-1)/2)*.0024,surface),(.0012,.0015,.0005),mat,.0002)
        self.hit(pid,ctl,(px,py,surface),(.011,.008,.002))
        return ctl

    def start(self):
        pid="start"
        root=self.root(pid,"Engine start / stop")
        g=self.g
        r=g["start_radius_m"]
        M.cylinder(self.m.mb,(0,0,0),(0,0,.005),r+.002,"metal",root,count=g["curve_segments"])
        ctl=self.m.node("ctl_start",root,(0,0,.006),Model.prismatic((0,0,-1),0,g["button_travel_m"]),behaviour={"spring_return":True})
        M.cylinder(self.m.mb,(0,0,.005),(0,0,.010),r,"red",ctl,count=g["curve_segments"])
        ring=self.lamp(pid,"ring",self.spec["palette"]["green"])
        M.lathe(self.m.mb,[(r-.001,.0101),(r-.001,.0105),(r-.002,.0105),(r-.002,.0101)],ring,ctl,count=g["curve_segments"],closed_profile=True)
        self.text(ctl,"START",(.0023,0,.0102),.003,.020,self.legend(pid))
        self.text(ctl,"STOP",(-.0023,0,.0102),.003,.017,self.legend(pid))
        self.hit(pid,ctl,(0,0,.010),(r,r,.003))

    def epb(self):
        pid="epb"; root=self.root(pid,"Electric parking brake rocker")
        self.slab(root,(0,0,.002),self.g["button_size_m"],"housing")
        ctl=self.m.node("ctl_epb",root,(0,0,.006),Model.revolute((0,1,0),-math.degrees(self.g["epb_travel_rad"]),math.degrees(self.g["epb_travel_rad"])),
                        behaviour={"spring_return":True,"apply_at_rad":self.g["epb_click_rad"],"release_at_rad":-self.g["epb_click_rad"]})
        self.slab(ctl,(0,0,.011),self.g["epb_size_m"],"cap")
        self.slab(ctl,(-.009,0,.020),(.003,.018,.004),"metal",.001)
        self.text(ctl,"P",(.001,0,.0192),.005,.006,self.legend(pid))
        # Explicit parking-brake brackets around P.
        for y in (-.0045,.0045): self.stroke(ctl,(-.0025,y,.0192),(.0025,y,.0192),self.legend(pid))
        self.slab(ctl,(.008,0,.0192),(.0013,.004,.0005),self.lamp(pid,"state",self.spec["palette"]["red"]),.0003)
        self.hit(pid,ctl,(-.008,0,.020),(.008,.009,.007),"grab",apply="epb=1",release="epb=0")

    def nitrous(self):
        pid="n2o"; root=self.root(pid,"Guarded nitrous arm switch")
        self.slab(root,(0,0,.003),(.042,.028,.008),"housing")
        for x in (-.017,.017):
            for y in (-.010,.010): M.cylinder(self.m.mb,(x,y,.007),(x,y,.008),.0015,"metal",root,count=8)
        ctl=self.m.node("ctl_n2o",root,(0,0,.008),Model.revolute((0,1,0),-math.degrees(self.g["toggle_travel_rad"]),math.degrees(self.g["toggle_travel_rad"])),
                        behaviour={"detents_rad":[-self.g["toggle_travel_rad"],self.g["toggle_travel_rad"]]})
        M.cylinder(self.m.mb,(0,0,.008),(0,0,.024),.0025,"metal",ctl,count=12)
        self.slab(ctl,(0,0,.025),(.009,.008,.005),"red",.001)
        pivot=self.g["guard_pivot_m"]
        cover=self.m.node("ctl_n2o_cover",root,pivot,Model.revolute((0,1,0),0,math.degrees(self.g["guard_open_rad"])),
                          behaviour={"enables":"hit_n2o","enabled_above_rad":self.g["guard_enable_rad"],
                                     "spring_below_rad":self.g["guard_spring_threshold_rad"],"closing_disarms":True})
        self.slab(cover,(0,0,.031),self.g["guard_size_m"],"red",.003)
        # Protective side walls with an open underside and separate hinge.
        for y in (-.010,.010): self.slab(cover,(0,y,.023),(.030,.002,.016),"red",.0008)
        M.cylinder(self.m.mb,(pivot[0],-.014,pivot[2]),(pivot[0],.014,pivot[2]),.003,"metal",root,count=12)
        self.text(cover,"N2O",(0,0,.0337),.005,.018,self.legend(pid))
        self.slab(root,(-.017,0,.008),(.0015,.003,.0006),self.lamp(pid,"armed",self.spec["palette"]["red"]),.0003)
        self.hit(pid,ctl,(0,0,.025),(.005,.005,.010),"grab",requires={"node":cover,"above_rad":self.g["guard_enable_rad"]})
        self.hit(pid+"_cover",cover,(-.014,0,.031),(.008,.010,.006),"grab")

    def shifter(self,kind):
        pid="shifter_"+kind; root=self.root(pid,{"seq":"Sequential lever","h":"H-pattern lever","sel":"Automatic selector"}[kind])
        g=self.g
        self.slab(root,(0,0,.003),g["lever_base_size_m"],"housing",.006)
        pivot=(0,0,g["lever_pivot_height_m"])
        parent=root
        if kind!="seq":
            parent=self.m.node("ctl_"+pid+"_gate",root,pivot,Model.revolute((1,0,0),0 if kind=="sel" else -math.degrees(g["lever_cross_travel_rad"]),math.degrees(g["lever_cross_travel_rad"])),
                               behaviour={"spring_return":kind=="sel","detents_rad":[0,g["lever_cross_travel_rad"]] if kind=="sel" else [-g["lever_cross_travel_rad"],0,g["lever_cross_travel_rad"]]})
        ctl=self.m.node("ctl_"+pid,parent,pivot,Model.revolute((0,1,0),-math.degrees(g["lever_travel_rad"]),math.degrees(g["lever_travel_rad"])),
                        behaviour={"spring_return":kind!="h","click_at_rad":g["lever_click_rad"]})
        # Accordion boot belongs to the moving assembly; pivot sits at its lower seal.
        prof=[]
        for i in range(9):
            prof.append((g["lever_boot_radius_m"]*(1-i*.065)+(.001 if i%2 else 0),.009+i*g["lever_boot_height_m"]/8))
        M.lathe(self.m.mb,prof,"rubber",ctl,count=g["curve_segments"])
        h=g["lever_height_m"]
        M.cylinder(self.m.mb,(0,0,.037),(0,0,h-.025),g["lever_shaft_radius_m"],"metal",ctl,count=g["curve_segments"])
        if kind=="h":
            # Nonzero end rings avoid zero-area pole triangles in a UV sphere.
            sections=[]
            for i in range(9):
                angle=.06+(math.pi-.12)*i/8
                sections.append([(.021*math.sin(angle)*math.cos(math.tau*k/16),
                                  .020*math.sin(angle)*math.sin(math.tau*k/16),
                                  h-.009-.020*math.cos(angle)) for k in range(16)])
            M.loft(self.m.mb,sections,"cap",ctl)
            M.cylinder(self.m.mb,(0,0,h+.007),(0,0,h+.011),.015,"metal",ctl,count=16)
            M.cylinder(self.m.mb,(0,0,h+.010),(0,0,h+.011),.014,"cap",ctl,count=16)
        elif kind=="sel":
            # A curved monostable grip rather than the sequential lever's straight knob.
            sections=[]
            for z,x,sx,sy in ((h-.034,0,.022,.022),(h-.024,-.003,.025,.024),
                              (h-.005,-.010,.035,.026),(h+.008,-.007,.032,.024)):
                sections.append([(x+a,b,z) for a,b in M.rrect(sx,sy,.005,2)])
            M.loft(self.m.mb,sections,"cap",ctl)
            self.slab(ctl,(-.007,0,h+.010),(.031,.024,.003),"metal",.005)
            self.slab(ctl,(-.007,0,h+.012),(.028,.021,.002),"cap",.004)
        else:
            self.slab(ctl,(0,0,h-.012),g["lever_grip_size_m"],"cap",.008)
            self.slab(ctl,(0,0,h+.008),(.031,.025,.003),"metal",.006)
            self.slab(ctl,(0,0,h+.010),(.028,.022,.002),"cap",.005)
        surface=h+.0112
        if kind=="h":
            # Six-speed H + reverse: separate columns, no ambiguous seven-speed engraving.
            for y in (-.007,0,.007): self.stroke(ctl,(-.006,y,surface),(.006,y,surface),self.legend(pid))
            self.stroke(ctl,(0,-.007,surface),(0,.007,surface),self.legend(pid))
            for text,x,y in (("1",.009,.007),("2",-.009,.007),("3",.009,0),("4",-.009,0),("5",.009,-.007),("6",-.009,-.007)):
                self.text(ctl,text,(x,y,surface),.003,.003,self.legend(pid))
            self.text(ctl,"R",(.0035,.010,surface),.002,.002,self.legend(pid))
        else:
            self.text(root,"+",(-.025,.015,.0092),.004,.005,self.legend(pid))
            self.text(root,"-",(.025,.015,.0092),.004,.005,self.legend(pid))
            self.knob_button(pid+"_pbtn","P",ctl,(-.007,0,h+.014 if kind=="sel" else h+.012))
            if kind=="seq":
                self.knob_button(pid+"_ambtn","A/M",ctl,(.007,0,h+.012))
            else:
                # Visible top legend strip on the fixed base, individually controllable lamps.
                for i,char in enumerate("PRNDM"):
                    mat=self.lamp(pid,char,self.spec["palette"]["normal"])
                    self.text(root,char,(.018,.018-i*.008,.010),.004,.005,mat)
                unlock=self.m.node("ctl_shifter_sel_unlock",ctl,(0,-.015,h-.011),Model.prismatic((0,1,0),0,g["selector_unlock_travel_m"]),behaviour={"spring_return":True})
                self.slab(unlock,(0,-.015,h-.011),(.018,.004,.015),"metal",.001)
                self.hit("shifter_sel_unlock",unlock,(0,-.017,h-.011),(.009,.003,.008),axis=(0,1,0))
        self.hit(pid,ctl,(0,0,h-.012),(.019,.016,.030),"grab")

    def knob_button(self,pid,label,parent,pos):
        x,y,z=pos
        self.slab(parent,(x,y,z),(.012,.010,.002),"housing",.002)
        ctl=self.m.node("ctl_"+pid,parent,pos,Model.prismatic((0,0,-1),0,self.g["knob_button_travel_m"]),behaviour={"spring_return":True})
        self.slab(ctl,(x,y,z+.0015),(.010,.008,.002),"cap",.0015)
        mat=self.lamp(pid,"state",self.spec["palette"]["red"] if label=="P" else self.spec["palette"]["normal"])
        self.text(ctl,label,(x,y,z+.0027),.003,.007,mat)
        self.hit(pid,ctl,(x,y,z+.0027),(.006,.005,.002))

    def paddle(self,side):
        pid="paddle_"+side; root=self.root(pid,"Left downshift paddle" if side=="L" else "Right upshift paddle")
        sign=1 if side=="L" else -1
        self.slab(root,(0,0,0),(.016,.016,.012),"housing")
        ctl=self.m.node("ctl_"+pid,root,(0,0,0),Model.revolute((0,1,0),0,math.degrees(self.g["paddle_travel_rad"])),
                        behaviour={"spring_return":True,"click_at_rad":self.g["paddle_click_rad"]})
        self.slab(ctl,(0,sign*.012,.025),self.g["paddle_size_m"],"metal",.005)
        # Top-facing engraving, additional grip ridges on the finger face.
        self.text(ctl,"-" if side=="L" else "+",(0,sign*.012,.0612),.005,.008,self.legend(pid))
        for z in (.017,.025,.033,.041): self.slab(ctl,(-.007,sign*.012,z),(.001,.018,.0013),"rubber",.0003)
        self.hit(pid,ctl,(-.008,sign*.012,.026),(.006,.012,.026),"poke",axis=(1,0,0))

    def stalk(self,side):
        pid="stalk_"+side; root=self.root(pid,"Indicator / flash stalk" if side=="L" else "Wiper stalk")
        sign=1 if side=="L" else -1; g=self.g
        self.slab(root,(0,0,0),(.023,.022,.024),"housing")
        ctl=self.m.node("ctl_"+pid,root,(0,0,0),Model.revolute((1,0,0),-math.degrees(g["stalk_travel_rad"]),math.degrees(g["stalk_travel_rad"])),
                        behaviour={"spring_return":False,"momentary_rad":g["stalk_half_step_rad"],
                                   "detents_rad":[-g["stalk_travel_rad"],0,g["stalk_travel_rad"]] if side=="L" else [-g["stalk_travel_rad"]*i/3 for i in range(4)]})
        parent=ctl
        if side=="L":
            parent=self.m.node("ctl_stalk_L_flash",ctl,(0,0,0),Model.revolute((0,0,1),0,math.degrees(g["stalk_flash_rad"])),behaviour={"spring_return":True})
        end=sign*g["stalk_length_m"]
        M.cylinder(self.m.mb,(0,0,0),(0,end,0),.005,"cap",parent,count=12)
        self.slab(parent,(0,end*.90,0),(.019,.042,.020),"cap",.004)
        self.text(parent,"<>" if side=="L" else "WIPER",(0,end*.90,.0102),.003,.026,self.legend(pid))
        if side=="L":
            for yy in (end*.90-.010,end*.90+.010):
                self.stroke(parent,(-.003,yy,.0102),(0,yy+.003,.0102),self.legend(pid))
                self.stroke(parent,(0,yy+.003,.0102),(.003,yy,.0102),self.legend(pid))
        self.hit(pid,parent,(0,end*.90,0),(.012,.025,.014),"grab")

    def dial(self):
        pid="lights"; root=self.root(pid,"Headlight rotary dial"); r=self.g["dial_radius_m"]
        M.cylinder(self.m.mb,(0,0,0),(0,0,.004),r+.004,"housing",root,count=24)
        ctl=self.m.node("ctl_lights",root,(0,0,.004),Model.revolute((0,0,1),0,math.degrees(self.g["dial_detent_rad"]*3)),
                        behaviour={"spring_return":False,"detents_rad":[i*self.g["dial_detent_rad"] for i in range(4)]})
        M.cylinder(self.m.mb,(0,0,.004),(0,0,.014),r,"cap",ctl,count=24)
        self.slab(ctl,(0,0,.015),(.025,.005,.004),"metal",.001)
        for i in range(20):
            a=i*math.tau/20
            M.box(self.m.mb,(r*math.cos(a),r*math.sin(a),.009),(.002,.002,.007),"rubber",ctl)
        for i in range(4):
            a=i*self.g["dial_detent_rad"]
            self.slab(root,((r+.003)*math.cos(a),(r+.003)*math.sin(a),.0045),(.001,.001,.0006),self.legend(pid),.0002)
        self.hit(pid,ctl,(0,0,.009),(r,r,.011),"grab")

    def finish(self):
        for item in self.spec["buttons"]:
            self.button(item["id"],item["label"],item["led_count"],item["lamp"])
        self.start(); self.epb(); self.nitrous()
        for kind in ("seq","h","sel"): self.shifter(kind)
        for side in ("L","R"): self.paddle(side); self.stalk(side)
        self.dial()
        root=self.root("horn","Steering wheel horn pad")
        ctl=self.m.node("ctl_horn",root,(0,0,.006),Model.prismatic((0,0,-1),0,self.g["button_travel_m"]),behaviour={"spring_return":True})
        self.slab(ctl,(0,0,.006),self.g["horn_size_m"],"cap",.010)
        self.text(ctl,"HORN",(0,0,.0122),.006,.034,self.legend("horn"))
        self.hit("horn",ctl,(0,0,.012),(.035,.026,.008))
        root=self.root("blank_cap","Unfitted control blank")
        self.slab(root,(0,0,.002),self.g["button_size_m"],"housing")
        self.slab(root,(0,0,.007),self.g["button_cap_size_m"],"cap")
        self.m.extra_rig["cockpit_parts"]={"format":"rg.cockpit_parts/1","parts":self.parts,"interaction_defaults":self.spec["interaction"]}
        self.m.notes.append("All parts overlap at their own mounting origin; extract part_<id> and its descendants, not the entire scene.")
        self.m.notes.append("Interaction vectors stay explicitly ISO in extras; joint axes in GLB are converted to glTF by assetgen.")
        self.m.notes.append("No actions are executed by this asset. Physics/game wiring belongs to C0/C1/P2/P3.")
        return self.m


def build():
    spec=json.loads((REPO/"data/cockpits/parts_spec.json").read_text(encoding="utf-8"))
    return Builder(spec).finish()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,default=REPO/"data/models/cockpit_parts")
    args=parser.parse_args()
    model=build()
    stats=model.write(str(args.out))
    spec=json.loads((REPO/"data/cockpits/parts_spec.json").read_text(encoding="utf-8"))
    assert stats["triangles"]<=spec["triangle_budget"],stats
    print(json.dumps({**stats,"sha256":hashlib.sha256((args.out/"cockpit_parts.glb").read_bytes()).hexdigest()},indent=2))


if __name__=="__main__": main()
