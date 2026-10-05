badge_width=param("badge_width",26.0)
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sy=param("badge_shell_center_y",91.0)
sz=param("badge_shell_base_z",-35.0)
profile=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
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

badge=Sphere(badge_width/2)
host=Pos(0,sy,sz)*belly_outer(bw,bd,bh)
# Boolean trim the hidden rear portion; the spherical exterior remains a separate object.
badge=badge-host
assert badge.is_valid and len(badge.solids())==1
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin,gp_Pnt,gp_Dir
pin_d=param("badge_pin_diameter",5.0)
pin_l=param("badge_pin_length",3.0)
pin_x=param("badge_pin_x",0.0)
pin_z=param("badge_pin_z",0.0)
assert pin_d>0 and pin_l>0

def pin_frame(surface,x,z,depth):
    probe=IntCurvesFace_ShapeIntersector()
    probe.Load(surface.wrapped,1e-7)
    probe.Perform(gp_Lin(gp_Pnt(x,-depth*2,z),gp_Dir(0,1,0)),0,depth*4)
    assert probe.NbPnt()>=2
    i=min(range(1,probe.NbPnt()+1),key=lambda i:probe.Pnt(i).Y())
    point=Vector(probe.Pnt(i).X(),probe.Pnt(i).Y(),probe.Pnt(i).Z())
    normal=Face(probe.Face(i)).normal_at(point)
    inward=normal if normal.Y>0 else -normal
    return point,inward.normalized()

root,axis=pin_frame(host,pin_x,pin_z,bd)
# Embed the cylinder in the sphere, leaving the specified length beyond the centre contact.
embed=badge_width/2
pin=Plane(origin=root-axis*embed,z_dir=axis).location*Cylinder(pin_d/2,pin_l+embed,align=(Align.CENTER,Align.CENTER,Align.MIN))
badge=badge+pin
assert badge.is_valid and len(badge.solids())==1
assert abs((root+axis*pin_l-root).length-pin_l)<1e-7
badge.color=Color(.82,.84,.86)
publish("badge",badge,"银色球饰",material="stainless_steel")
