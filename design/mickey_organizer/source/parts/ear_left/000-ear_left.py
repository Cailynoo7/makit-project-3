w=param("ear_left_width",80.0)  # Circular outside diameter.
h=param("ear_left_height",82.0)  # Existing mounting top height in the ear frame.
t=param("ear_left_thickness",12.0)
rim=param("ear_left_rim",2.0)
recess=param("ear_left_recess_depth",4.5)
spread=param("ear_left_recess_spread",0.55)
assert 0<rim<min(w/4,t/2) and 0<recess<t and 0<spread<1
radius=w/2
inner_radius=radius-rim
front=-t/2
back=t/2
# Degree-seven profile extends the depression into the outer shoulder, including the upper-outside region.
# First and second radial derivatives vanish at the centre and outer face rim.
# Both faces follow the same displacement, keeping 12 mm axial material thickness.
front_points=[(i*inner_radius/7,front+recess*q,0) for i,q in enumerate([1,1,1,1,spread,0,0,0])]
back_points=[(x,y+t,z) for x,y,z in reversed(front_points)]
assert all(abs(b[1]-a[1]-t)<1e-9 for a,b in zip(front_points,reversed(back_points)))
profile=Wire([
    Bezier(*front_points).edge(),
    ThreePointArc((inner_radius,front,0),(inner_radius+rim*2**-.5,front+rim*(1-2**-.5),0),(radius,front+rim,0)).edge(),
    Edge.make_line((radius,front+rim,0),(radius,back-rim,0)),
    ThreePointArc((radius,back-rim,0),(inner_radius+rim*2**-.5,back-rim*(1-2**-.5),0),(inner_radius,back,0)).edge(),
    Bezier(*back_points).edge(),
    Edge.make_line((0,back+recess,0),(0,front+recess,0))
])
ear=revolve(Face(profile),axis=Axis.Y)
assert ear.is_valid and len(ear.solids())==1
bb=ear.bounding_box()
assert abs(bb.size.X-w)<0.001 and abs(bb.size.Z-w)<0.001

# Directional shoulder depression: strongest above and outward, smoothly fading around the ear.
# Reuse the exact revolved NURBS main faces; only their Y poles move.
# Three unchanged radial rows at both ends preserve the centre and rim derivatives.
from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.Geom import Geom_RectangularTrimmedSurface
from OCP.GeomConvert import GeomConvert
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
from OCP.TopoDS import TopoDS
from OCP.BRepLib import BRepLib
from math import sqrt
upper_recess=param("ear_left_upper_outer_recess",1.0)
assert 0<=upper_recess<=rim
outward=-1
radial_weights=[0,0,0,0,0,0,0.4,1,1,0,0,0]
sewing=BRepBuilderAPI_Sewing(1e-6)
changed=0
main_poles=[]
for face in ear.faces():
    bounds=face.bounding_box()
    # Identify the two degree-seven main faces using their parameter-derived span.
    if abs(bounds.size.X-2*inner_radius)<0.01 and abs(bounds.size.Z-2*inner_radius)<0.01 and abs(bounds.size.Y-recess)<0.01:
        u0,u1,v0,v1=BRepTools.UVBounds_s(face.wrapped)
        original=BRep_Tool.Surface_s(face.wrapped)
        surf=GeomConvert.SurfaceToBSplineSurface_s(Geom_RectangularTrimmedSurface(original,u0,u1,v0,v1))
        assert surf.VDegree()==7
        surf.IncreaseDegree(surf.UDegree(),11)
        base=surf.Copy()
        changes=[]
        for j in range(1,surf.NbVPoles()+1):
            rbase=sqrt(surf.Pole(1,j).X()**2+surf.Pole(1,j).Z()**2)
            index=round(rbase/inner_radius*11)
            radial=radial_weights[index]
            for i in range(1,surf.NbUPoles()+1):
                p=surf.Pole(i,j)
                # Affine harmonic in the exact rational circle gives a smooth periodic direction weight.
                angular=0.5*(1+(outward*p.X()+p.Z())/(sqrt(2)*rbase)) if rbase>1e-8 else 0
                dy=radial*angular
                changes.append((i,j,p.Y(),dy))
                p.SetY(p.Y()+dy)
                surf.SetPole(i,j,p)
        peak=max(surf.Value(u0+(u1-u0)*a/72,v0+(v1-v0)*b/80).Y()-base.Value(u0+(u1-u0)*a/72,v0+(v1-v0)*b/80).Y() for a in range(73) for b in range(81))
        assert peak>0
        for i,j,y,dy in changes:
            p=surf.Pole(i,j)
            p.SetY(y+dy*upper_recess/peak)
            surf.SetPole(i,j,p)
        main_poles.append({(round(surf.Pole(i,j).X(),6),round(surf.Pole(i,j).Z(),6)):surf.Pole(i,j).Y() for i in range(1,surf.NbUPoles()+1) for j in range(1,surf.NbVPoles()+1)})
        newface=BRepBuilderAPI_MakeFace(surf,u0,u1,v0,v1,1e-7).Face()
        newface.Orientation(face.wrapped.Orientation())
        sewing.Add(newface)
        changed+=1
    else:
        sewing.Add(face.wrapped)
assert changed==2
assert set(main_poles[0])==set(main_poles[1])
assert all(abs(abs(main_poles[0][key]-main_poles[1][key])-t)<1e-5 for key in main_poles[0])
sewing.Perform()
shell=TopoDS.Shell_s(sewing.SewedShape())
solid=BRepBuilderAPI_MakeSolid(shell).Solid()
assert BRepLib.OrientClosedSolid_s(solid)
ear=Solid(solid)
assert ear.is_valid and len(ear.solids())==1
print("Upper-outside rearward depression amplitude:",upper_recess,"mm; front and back use identical displacement.")

ear=Pos(0,0,h-radius)*ear
# Ear root follows the host shell exterior, so it does not protrude into storage.
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sx=param("ear_left_shell_center_x",70.0)
sy=param("ear_left_shell_center_y",-5.0)
sz=param("ear_left_shell_base_z",-118.0)
host_profile=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
body_lower_front_extension=param("body_lower_front_extension",5.0)
def belly_weight(h):
    # Smooth rise from the outer base, broad belly, then ease into the upper body.
    def smooth(t):
        t=max(0,min(1,t))
        return t*t*t*(10+t*(-15+6*t))
    return smooth(h/.095)*(1-smooth((h-.23)/.47))
def belly_outer(width,depth,height):
    sections=[]
    for h,rx,ry in host_profile:
        extension=body_lower_front_extension*belly_weight(h)
        sections.append(Pos(0,-extension/2,h*height)*Ellipse(width/2*rx,depth/2*ry+extension/2))
    result=loft(sections)
    assert result.is_valid and len(result.solids())==1
    return result

host=Pos(sx,sy,sz)*belly_outer(bw,bd,bh)
ear=ear-host
assert ear.is_valid and len(ear.solids())==1

tongue_w=param("ear_left_tongue_width",4.0)
tongue_h=param("ear_left_tongue_height",4.0)
tongue_l=param("ear_left_tongue_length",3.0)
tongue_y=param("ear_left_tongue_y",2.5)
tongue_z=param("ear_left_tongue_z",32.0)
assert min(tongue_w,tongue_h,tongue_l)>0

from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin,gp_Pnt,gp_Dir
def ear_joint_frame(surface,side,y,z,width):
    centre=surface.bounding_box().center().X
    probe=IntCurvesFace_ShapeIntersector()
    probe.Load(surface.wrapped,1e-7)
    probe.Perform(gp_Lin(gp_Pnt(centre-width*2,y,z),gp_Dir(1,0,0)),0,width*4)
    assert probe.NbPnt()>=2
    i=min(range(1,probe.NbPnt()+1),key=lambda j:side*-probe.Pnt(j).X())
    root=Vector(probe.Pnt(i).X(),probe.Pnt(i).Y(),probe.Pnt(i).Z())
    normal=Face(probe.Face(i)).normal_at(root)
    inward=normal if normal.X*side<0 else -normal
    inward=inward.normalized()
    across=Vector(0,1,0)
    across=(across-inward*inward.dot(across)).normalized()
    return root,inward,across

root,axis,across=ear_joint_frame(host,-1,tongue_y,tongue_z,bw)
embed=max(tongue_w,tongue_h)
tongue=Plane(origin=root-axis*embed,x_dir=across,z_dir=axis).location*Box(tongue_w,tongue_h,embed+tongue_l,align=(Align.CENTER,Align.CENTER,Align.MIN))
old_volume=ear.volume
ear=ear+tongue
assert ear.is_valid and len(ear.solids())==1, "ear tongue must fuse to its own ear"
assert ear.volume-old_volume>tongue_w*tongue_h*tongue_l*.5, "ear tongue extension is too small"

ear.color=Color(.018,.018,.022)
publish("ear_left",ear,"左侧大耳朵",material="resin")
