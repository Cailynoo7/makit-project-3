
w=param("leg_right_width",43.0)
d=param("leg_right_depth",43.0)
h=param("leg_right_height",42.0)
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sx=param("leg_right_shell_center_x",-52.0)
sy=param("leg_right_shell_center_y",0.0)
sz=param("leg_right_shell_base_z",16.0)
insertion=param("leg_right_insertion_depth",5.0)
assert insertion>0
leg=loft([Ellipse(w*.48,d*.48),Pos(0,0,h*.12)*Ellipse(w/2,d/2),Pos(0,0,h)*Ellipse(w*.52,d*.52)])
p=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
body_lower_front_extension=param("body_lower_front_extension",5.0)
def belly_weight(h):
    # Smooth rise from the outer base, broad belly, then ease into the upper body.
    def smooth(t):
        t=max(0,min(1,t))
        return t*t*t*(10+t*(-15+6*t))
    return smooth(h/.095)*(1-smooth((h-.23)/.47))
def belly_outer(width,depth,height):
    sections=[]
    for h,rx,ry in p:
        extension=body_lower_front_extension*belly_weight(h)
        sections.append(Pos(0,-extension/2,h*height)*Ellipse(width/2*rx,depth/2*ry+extension/2))
    result=loft(sections)
    assert result.is_valid and len(result.solids())==1
    return result

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

base=belly_outer(bw,bd,bh)
host=Pos(sx,sy,sz)*smooth_base(base,base,bw,bd,bh)
# Raise the curved root boundary without changing the exposed leg loft.
root_up=param("leg_right_root_extension",3.0)
assert root_up>0
probe=IntCurvesFace_ShapeIntersector()
probe.Load(host.wrapped,1e-7)
probe.Perform(gp_Lin(gp_Pnt(0,0,-bh),gp_Dir(0,0,1)),0,bh*3)
assert probe.NbPnt()>=2
old_root_z=min(probe.Pnt(i).Z() for i in range(1,probe.NbPnt()+1))
leg=leg-Pos(0,0,root_up)*host
assert leg.is_valid and len(leg.solids())==1
assert leg.is_inside((0,0,old_root_z+root_up-0.001))
assert not leg.is_inside((0,0,old_root_z+root_up+0.001))
# Extend the original lower footprint into the shoe without changing the exposed loft.
peg=Pos(0,0,-insertion)*extrude(Ellipse(w*.48,d*.48),amount=insertion)
leg=leg+peg
assert leg.is_valid and len(leg.solids())==1
assert abs(leg.bounding_box().min.Z+insertion)<1e-6
leg.color=Color(.27,.014,.040)
publish("leg_right",leg,"右侧短腿",material="resin")
