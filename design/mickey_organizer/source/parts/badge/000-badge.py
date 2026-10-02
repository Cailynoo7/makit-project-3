
w=param("badge_width",26.0)
h=param("badge_height",26.0)
d=param("badge_depth",10.0)
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sy=param("badge_shell_center_y",80.0)
sz=param("badge_shell_base_z",-35.0)
badge=scale(Sphere(1),by=(w/2,d/2,h/2))
p=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
host=loft([Pos(0,sy,sz+z*bh)*Ellipse(bw/2*rx,bd/2*ry) for z,rx,ry in p])
badge=badge-host
badge.color=Color(.82,.84,.86)
publish("badge",badge,"银色圆饰",material="stainless_steel")
