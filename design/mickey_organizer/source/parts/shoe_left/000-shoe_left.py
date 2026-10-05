# Footprint and shape parameters are shared by both colour sections of this shoe.
w=param("shoe_left_width",98.0)
d=param("shoe_left_length",130.0)
h=param("shoe_left_toe_height",72.0)
sole=param("shoe_left_sole_height",8.0)
cw=param("shoe_left_cuff_width",58.3)
cd=param("shoe_left_cuff_depth",61.6)
ch=param("shoe_left_cuff_height",14.0)
cy=param("shoe_left_cuff_y",20.0)
cz=param("shoe_left_cuff_base_z",70.0)
nw=param("shoe_left_neck_width",51.0)
nd=param("shoe_left_neck_depth",54.0)
front_fullness=param("shoe_left_front_fullness",1.4)
heel_extension=param("shoe_left_heel_extension",5.0)
sole_round=param("shoe_left_sole_round_radius",4.0)
bump=param("shoe_left_toe_bump",8.0)
fullness=param("shoe_left_fullness",1.6)
heel_ratio=param("shoe_left_heel_width_ratio",0.94)
assert d>w and cd>=cw and 0<nw<min(w,cw) and nw<nd<min(d,cd) and h>sole and cz<h
assert 0<front_fullness<=fullness and heel_extension>=0
# Two unequal round ends joined along their common tangent: narrower heel.
# Quintic easing retains zero endpoint first/second derivatives.
def ease(t):
    return t*t*t*(10+t*(-15+6*t))
def smooth_wire(width,length,yc,zfront,zback,ratio,front_delta=0):
    from math import pi, asin, sin, cos
    rf=width/2
    rb=rf*ratio
    yf=yc-length/2+rf
    yb=yc+length/2-rb
    sn=(rf-rb)/(yb-yf)
    cs=(1-sn*sn)**.5
    alpha=asin(sn)
    xf=rf*cs
    xb=rb*cs
    af=yf+rf*sn
    ab=yb+rb*sn
    lf=rf*(pi+2*alpha)
    lb=rb*(pi-2*alpha)
    ls=(yb-yf)*cs
    per=lf+lb+2*ls
    pts=[]
    for j in range(64):
        q=j*per/64
        if q<lf:
            angle=alpha-q/rf
            pts.append((rf*cos(angle),yf+rf*sin(angle),zfront))
        elif q<lf+ls:
            u=(q-lf)/ls
            v=u*u*(3-2*u)
            pts.append((-xf+(xf-xb)*u,af+(ab-af)*u,zfront+(zback-zfront)*v))
        elif q<lf+ls+lb:
            angle=pi-alpha-(q-lf-ls)/rb
            pts.append((rb*cos(angle),yb+rb*sin(angle),zback))
        else:
            u=(q-lf-ls-lb)/ls
            v=u*u*(3-2*u)
            pts.append((xb+(xf-xb)*u,ab+(af-ab)*u,zback+(zfront-zback)*v))
    # Local front-side narrowing with smooth zero-slope falloff toward the heel.
    narrowed=[]
    for x,y,z in pts:
        u=max(0.0,min(1.0,(yb-y)/(yb-yf)))
        weight=ease(u)
        narrowed.append((x*(1-front_delta*weight/width),y,z))
    return Wire([Spline(narrowed,periodic=True).edge()])
wires=[]
for i in range(17):
    t=i/16
    s=ease(t**fullness)
    z=sole+(h-sole)*t
    width=w+(nw-w)*s
    # The extra heel length fades smoothly to zero at the unchanged ankle.
    extra=heel_extension*(1-s)
    length=d+(nd-d)*s+extra
    front_delta=(w-nw)*(ease(t**front_fullness)-s)
    # The front half rises gently while remaining one continuous shoe surface.
    rise=bump*16*t*t*(1-t)*(1-t)
    ratio=heel_ratio+(1-heel_ratio)*s
    wires.append(smooth_wire(width,length,cy*s+extra/2,z+rise,z,ratio,front_delta))
upper=Solid.make_loft(wires)
if upper.volume<0:
    upper=Solid(upper.wrapped.Reversed())
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_C2
skin_faces=upper.faces().filter_by(GeomType.BSPLINE)
assert len(skin_faces)==1
skin=BRepAdaptor_Surface(skin_faces[0].wrapped)
assert int(skin.UContinuity())>=int(GeomAbs_C2) and int(skin.VContinuity())>=int(GeomAbs_C2), "shoe skin must retain C2 continuity"
# Build the flat sole separately so its lower fillet has a stable vertical wall.
assert 0<sole_round<sole
base=extrude(Face(smooth_wire(w,d+heel_extension,heel_extension/2,0,0,heel_ratio)),amount=sole,dir=(0,0,1))
bottom_edges=[e for e in base.edges() if abs(e.bounding_box().min.Z)<0.001 and abs(e.bounding_box().max.Z)<0.001]
assert len(bottom_edges)==1
base=base.fillet(sole_round,bottom_edges)
assert base.is_valid
toe=base+upper
assert toe.is_valid and len(toe.solids())==1
cuff=loft([Pos(0,cy,cz)*Ellipse(cw*.47,cd*.47),Pos(0,cy,cz+ch*.2)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch*.75)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch)*Ellipse(cw*.47,cd*.47)])
shoe=toe+cuff
assert abs(shoe.bounding_box().min.Z)<0.001, f"lost sole: upper {upper.volume}, toe {toe.volume}, minZ {shoe.bounding_box().min.Z}"

assert len(shoe.solids())==1 and shoe.is_valid

# The former groove starts at sole; retain that exact Z=8 mm boundary.
# Shared footprint dimensions drive both sections; sole height and R4 are the lower section's dimensions.
shoe_top=upper+cuff
shoe_sole=base
# Socket follows the matching leg's lower footprint and insertion depth.
leg_w=param("leg_left_width",43.0)
leg_d=param("leg_left_depth",43.0)
insertion=param("leg_left_insertion_depth",5.0)
clearance=param("shoe_left_leg_socket_clearance",0.2)
assert clearance>=0 and 0<insertion<ch
socket_profile=offset(Ellipse(leg_w*.48,leg_d*.48),amount=clearance)
socket_bottom=cz+ch-insertion
socket=Pos(0,cy,socket_bottom)*extrude(socket_profile,amount=insertion+1.0)
before_socket=shoe_top.volume
shoe_top=shoe_top-socket
assert before_socket-shoe_top.volume>1
floors=[f for f in shoe_top.faces() if f.geom_type==GeomType.PLANE and abs(f.bounding_box().min.Z-socket_bottom)<1e-6 and abs(f.bounding_box().max.Z-socket_bottom)<1e-6]
assert len(floors)==1, "socket must have one flat assembly floor"
assert abs(floors[0].bounding_box().size.X-(leg_w*.96+2*clearance))<1e-5
assert abs(floors[0].bounding_box().size.Y-(leg_d*.96+2*clearance))<1e-5
assert shoe_top.is_valid and shoe_sole.is_valid
assert len(shoe_top.solids())==1 and len(shoe_sole.solids())==1
assert abs(shoe_top.bounding_box().min.Z-sole)<0.001
assert abs(shoe_sole.bounding_box().max.Z-sole)<0.001
assert abs(shoe_sole.bounding_box().min.Z)<0.001
assert abs(shoe_top.bounding_box().max.Z-(cz+ch))<0.001
assert shoe_top.distance_to(shoe_sole)<1e-6
joined=shoe_top+shoe_sole
assert joined.is_valid and len(joined.solids())==1
shoe_top.color=Color(1.0,.68,.008)
shoe_sole.color=Color(.018,.018,.022)
publish("shoe_left",shoe_top,"左侧黄色鞋面",material="resin")
publish("sole_left",shoe_sole,"左侧黑色鞋底",material="resin")
print("Shoe boundary at Z=",sole,"mm; yellow upper, black lower; groove removed.")
