
w=param("leg_left_width",43.0)
d=param("leg_left_depth",43.0)
h=param("leg_left_height",42.0)
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sx=param("leg_left_shell_center_x",52.0)
sy=param("leg_left_shell_center_y",0.0)
sz=param("leg_left_shell_base_z",16.0)
leg=loft([Ellipse(w*.48,d*.48),Pos(0,0,h*.12)*Ellipse(w/2,d/2),Pos(0,0,h)*Ellipse(w*.52,d*.52)])
p=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
host=loft([Pos(sx,sy,sz+z*bh)*Ellipse(bw/2*rx,bd/2*ry) for z,rx,ry in p])
# Nominal contour contact: leg meets the curved shell without entering the cavity.
leg=leg-host
leg.color=Color(.27,.014,.040)
publish("leg_left",leg,"左侧短腿",material="resin")
