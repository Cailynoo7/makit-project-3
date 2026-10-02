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
badge.color=Color(.82,.84,.86)
publish("badge",badge,"银色球饰",material="stainless_steel")
