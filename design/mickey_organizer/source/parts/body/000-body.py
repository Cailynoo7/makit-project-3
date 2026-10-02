body_width = param("body_width", 180.0)
body_depth = param("body_depth", 160.0)
body_height = param("body_height", 190.0)
body_wall = param("body_wall", 4.5)
body_floor = param("body_floor", 4.0)
body_opening_height = param("body_opening_height", 122.0)
body_lip_radius = param("body_lip_radius",1.2)
body_opening_top_anchor = param("body_opening_top_anchor",181.28838915004377)
# Section proportions reconstructed from reference_photo; unseen back is symmetric.
profile = [(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
outer_sections = [Pos(0,0,h*body_height)*Ellipse(body_width/2*rx, body_depth/2*ry) for h,rx,ry in profile]
body_lower_front_extension=param("body_lower_front_extension",5.0)
def belly_weight(h):
    # Smooth rise from the outer base, broad belly, then ease into the upper body.
    def smooth(t):
        t=max(0,min(1,t))
        return t*t*t*(10+t*(-15+6*t))
    return smooth(h/.095)*(1-smooth((h-.23)/.47))
def belly_outer(width,depth,height):
    sections=[]
    for h,rx,ry in profile:
        extension=body_lower_front_extension*belly_weight(h)
        sections.append(Pos(0,-extension/2,h*height)*Ellipse(width/2*rx,depth/2*ry+extension/2))
    result=loft(sections)
    assert result.is_valid and len(result.solids())==1
    return result
outer=belly_outer(body_width,body_depth,body_height)
# The belly section at 23% body height moves its front by exactly the requested 5 mm.
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin,gp_Pnt,gp_Dir
def front_at(shape,z,depth):
    probe=IntCurvesFace_ShapeIntersector()
    probe.Load(shape.wrapped,1e-7)
    probe.Perform(gp_Lin(gp_Pnt(0,-depth*2,z),gp_Dir(0,1,0)),0,depth*4)
    return min(probe.Pnt(i).Y() for i in range(1,probe.NbPnt()+1))
reference_outer=loft(outer_sections)
measured_extension=front_at(reference_outer,body_height*.23,body_depth)-front_at(outer,body_height*.23,body_depth)
assert abs(measured_extension-body_lower_front_extension)<.001, f"belly extension {measured_extension}"
assert abs(outer.bounding_box().max.Z-body_height)<.01


body_cavity_floor_raise = param("body_cavity_floor_raise",20.0)
body_cavity_blend = param("body_cavity_blend",12.0)
# Constant normal offset provides the requested 4.5 mm wall.
inner = offset(outer,amount=-body_wall)
floor_z = body_floor + body_cavity_floor_raise
inner = inner & Pos(0,0,floor_z)*Box(body_width*3,body_depth*3,body_height*2,align=(Align.CENTER,Align.CENTER,Align.MIN))
bottom_edges=[e for e in inner.edges() if abs(e.bounding_box().min.Z-floor_z)<.01 and abs(e.bounding_box().max.Z-floor_z)<.01]
inner = fillet(bottom_edges,radius=body_cavity_blend)


# opening_redline_v25: single rational conic fitted to the new red side contour.
# Control coordinates are reference geometry, rather than independent product dimensions.
body_opening_profile_bulge = param("body_opening_profile_bulge",1.0)
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin,gp_Pnt,gp_Dir
from math import sqrt,comb
ray=IntCurvesFace_ShapeIntersector()
ray.Load(outer.wrapped,1e-7)
def section_radii(z):
    ray.Perform(gp_Lin(gp_Pnt(0,-body_depth*2,z),gp_Dir(0,1,0)),0,body_depth*4)
    assert ray.NbPnt()>=2
    ry=-min(ray.Pnt(i).Y() for i in range(1,ray.NbPnt()+1))
    ray.Perform(gp_Lin(gp_Pnt(-body_width*2,0,z),gp_Dir(1,0,0)),0,body_width*4)
    assert ray.NbPnt()>=2
    rx=-min(ray.Pnt(i).X() for i in range(1,ray.NbPnt()+1))
    return rx,ry
# 314 red-line centre samples: circle fit RMS 1.027 px; fixed reference-derived geometry.
reference_controls=[(0,0),(1.3907054274772708,.34411867531732754),(1,1)]
reference_weights=[1,.8018296515668949,1]
def cut_mouth(zbottom):
    ztop=zbottom+body_opening_height
    ya=-section_radii(zbottom)[1]
    yb=-section_radii(ztop)[1]
    controls=[Vector(0,ya+(yb-ya)*(qz+body_opening_profile_bulge*(qy-qz)),zbottom+body_opening_height*qz) for qy,qz in reference_controls]
    curve=Edge.make_bezier(*controls,weights=reference_weights)
    # Tangent continuations lie outside the shell, avoiding horizontal caps at either lip endpoint.
    start=controls[0]-(controls[1]-controls[0]).normalized()*body_depth
    end=controls[-1]+(controls[-1]-controls[-2]).normalized()*body_depth
    front_top=Vector(0,-body_depth*3,end.Z)
    front_bottom=Vector(0,-body_depth*3,start.Z)
    outline=Wire([Edge.make_line(start,controls[0]),curve,Edge.make_line(controls[-1],end),Edge.make_line(end,front_top),Edge.make_line(front_top,front_bottom),Edge.make_line(front_bottom,start)])
    cutter=extrude(Face(outline),amount=body_width*3,dir=(1,0,0),both=True)
    candidate=outer-inner-cutter
    lip_edges=[e for e in candidate.edges() if e.center().Y<-20 and e.bounding_box().min.Z>zbottom-1 and e.bounding_box().max.Z<ztop+1 and e.length>30]
    assert len(lip_edges)>=2, f"mouth edges: {len(lip_edges)}"
    candidate=fillet(lip_edges,radius=body_lip_radius)
    assert candidate.is_valid and len(candidate.solids())==1
    lip_boundary=[e for e in candidate.edges() if e.bounding_box().min.Z>zbottom-3 and e.bounding_box().max.Z<ztop+4 and e.bounding_box().max.Y<-10 and e.length>30]
    assert lip_boundary
    peak=max(e.position_at(i/1024).Z for e in lip_boundary for i in range(1025))
    widths=[]
    for i in range(1,256):
        p=curve.position_at(i/256)
        rx,ry=section_radii(p.Z)
        widths.append(2*rx*sqrt(max(0,1-(p.Y/ry)**2)) if -ry<p.Y<0 else 0)
    return candidate,peak,max(widths)
# Preserve the previous rounded upper lip's measured height.
effective_bottom=body_opening_top_anchor-body_opening_height-body_lip_radius
for _ in range(4):
    body,peak,projected_width=cut_mouth(effective_bottom)
    error=body_opening_top_anchor-peak
    if abs(error)<.01:break
    effective_bottom+=error
assert abs(peak-body_opening_top_anchor)<.01, f"opening top moved: actual {peak}, target {body_opening_top_anchor}"
assert projected_width>121.44, f"opening was not enlarged: {projected_width}"
assert inner.is_valid
body_bottom_blend_height=param("body_bottom_blend_height",12.0)
body_bottom_bulge=param("body_bottom_bulge",3.0)
from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.Geom import Geom_BSplineCurve,Geom_BSplineSurface
from OCP.TColgp import TColgp_Array1OfPnt,TColgp_Array2OfPnt
from OCP.TColStd import TColStd_Array1OfReal,TColStd_Array1OfInteger,TColStd_Array2OfReal
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid
from OCP.TopoDS import TopoDS
from OCP.BRepLib import BRepLib
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Pnt,gp_Vec,gp_Lin,gp_Dir
def smooth_base(shape,reference,width,depth,height):
    z=body_bottom_blend_height
    assert 0<z<24 and 0<=body_bottom_bulge<z
    upper=shape & Pos(0,0,z)*Box(width*3,depth*3,height*2,align=(Align.CENTER,Align.CENTER,Align.MIN))
    disks=[f for f in upper.faces() if abs(f.bounding_box().min.Z-z)<.001 and abs(f.bounding_box().max.Z-z)<.001]
    assert len(disks)==1
    retained=[f for f in upper.faces() if not f.is_same(disks[0])]
    ray=IntCurvesFace_ShapeIntersector();ray.Load(reference.wrapped,1e-7)
    ray.Perform(gp_Lin(gp_Pnt(0,-depth*2,z),gp_Dir(0,1,0)),0,depth*4)
    cy=(min(ray.Pnt(i).Y() for i in range(1,ray.NbPnt()+1))+max(ray.Pnt(i).Y() for i in range(1,ray.NbPnt()+1)))/2
    caps=[]
    for face in reference.faces():
        if face.geom_type!=GeomType.BSPLINE:continue
        surf=BRep_Tool.Surface_s(face.wrapped)
        u0,u1,v0,v1=BRepTools.UVBounds_s(face.wrapped)
        lo,hi=v0,v1
        increasing=surf.Value(u0,v1).Z()>surf.Value(u0,v0).Z()
        for _ in range(60):
            mid=(lo+hi)/2
            if (surf.Value(u0,mid).Z()<z)==increasing:lo=mid
            else:hi=mid
        join=(lo+hi)/2
        assert abs(surf.Value(u0,join).Z()-z)<1e-6
        poles=TColgp_Array2OfPnt(1,surf.NbUPoles(),1,6)
        weights=TColStd_Array2OfReal(1,surf.NbUPoles(),1,6)
        alpha=None
        for i in range(1,surf.NbUPoles()+1):
            pts=TColgp_Array1OfPnt(1,surf.NbVPoles())
            ws=TColStd_Array1OfReal(1,surf.NbVPoles())
            w=surf.Weight(i,1)
            for j in range(1,surf.NbVPoles()+1):
                assert abs(surf.Weight(i,j)-w)<1e-8, "height-varying rational weight"
                pts.SetValue(j,surf.Pole(i,j));ws.SetValue(j,surf.Weight(i,j))
            curve=Geom_BSplineCurve(pts,ws,surf.VKnots(),surf.VMultiplicities(),surf.VDegree(),surf.IsVPeriodic())
            p=gp_Pnt();d1=gp_Vec();d2=gp_Vec()
            curve.D2(join,p,d1,d2)
            if alpha is None:alpha=z*1.2/d1.Z()
            # The last three rows match position, first and second derivatives exactly.
            p4=gp_Pnt(p.X()-alpha*d1.X()/5,p.Y()-alpha*d1.Y()/5,p.Z()-alpha*d1.Z()/5)
            p3=gp_Pnt(2*p4.X()-p.X()+alpha*alpha*d2.X()/20,2*p4.Y()-p.Y()+alpha*alpha*d2.Y()/20,2*p4.Z()-p.Z()+alpha*alpha*d2.Z()/20)
            # Lower the first radial rows only; the final three rows preserve the C2 join exactly.
            rows=[gp_Pnt(0,0,-body_bottom_bulge),gp_Pnt(.3*p.X(),.3*(p.Y()-cy),-body_bottom_bulge),gp_Pnt(.6*p.X(),.6*(p.Y()-cy),z*.08-body_bottom_bulge*.6),p3,p4,p]
            for j,point in enumerate(rows,1):
                poles.SetValue(i,j,point);weights.SetValue(i,j,w)
        vk=TColStd_Array1OfReal(1,2);vk.SetValue(1,0);vk.SetValue(2,1)
        vm=TColStd_Array1OfInteger(1,2);vm.SetValue(1,6);vm.SetValue(2,6)
        cap=Geom_BSplineSurface(poles,weights,surf.UKnots(),vk,surf.UMultiplicities(),vm,surf.UDegree(),5,surf.IsUPeriodic(),False)
        # Numerically verify C2 after the linear radial reparameterisation.
        for k in range(17):
            u=u0+(u1-u0)*k/16
            a=gp_Pnt();au=gp_Vec();av=gp_Vec();auu=gp_Vec();avv=gp_Vec();auv=gp_Vec()
            b=gp_Pnt();bu=gp_Vec();bv=gp_Vec();buu=gp_Vec();bvv=gp_Vec();buv=gp_Vec()
            surf.D2(u,join,a,au,av,auu,avv,auv)
            cap.D2(u,1,b,bu,bv,buu,bvv,buv)
            assert a.Distance(b)<1e-6
            assert (bv-av.Multiplied(alpha)).Magnitude()<1e-5
            assert (bvv-avv.Multiplied(alpha*alpha)).Magnitude()<1e-4
        caps.append(Face(BRepBuilderAPI_MakeFace(cap,u0,u1,0,1,1e-7).Face()))
    assert len(caps)==2
    sewing=BRepBuilderAPI_Sewing(1e-5)
    for f in retained+caps:sewing.Add(f.wrapped)
    sewing.Perform()
    shell=TopoDS.Shell_s(sewing.SewedShape())
    solid=BRepBuilderAPI_MakeSolid(shell).Solid()
    assert BRepLib.OrientClosedSolid_s(solid)
    result=Solid(solid).fix()
    assert result.is_valid and len(result.solids())==1
    assert abs(result.bounding_box().min.Z+body_bottom_bulge)<.01, f"lowest base height {result.bounding_box().min.Z}"
    assert not any(f.geom_type==GeomType.PLANE and f.bounding_box().max.Z<z-.01 for f in result.faces()), "planar exterior base remains"
    return result

body=smooth_base(body,outer,body_width,body_depth,body_height)
body.color=Color(.34,.012,.045)
publish("body",body,"酒红收纳盒身",material="resin")
