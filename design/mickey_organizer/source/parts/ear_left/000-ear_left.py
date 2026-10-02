
w=param("ear_left_width",80.0)
h=param("ear_left_height",82.0)
t=param("ear_left_thickness",12.0)
rim=param("ear_left_rim",2.0)
# Fan contour traced by proportion from reference_photo.
pts=[(-.50,.58),(-.22,.89),(.04,1),(.30,.91),(.48,.65),(.50,.31),(.36,.08),(.02,0),(-.30,.05),(-.45,.28)]
face=Face(Wire([Spline(*[(x*w*-1,0,z*h) for x,z in pts],periodic=True)]))
bb=face.bounding_box()
face=scale(face,by=(w/bb.size.X,1,h/bb.size.Z))
bb=face.bounding_box()
face=Pos(-(bb.min.X+bb.max.X)/2,0,-bb.min.Z)*face
small=Pos(0,0,h/2)*scale(Pos(0,0,-h/2)*face,by=(1-2*rim/w,1,1-2*rim/h))
ear=loft([Pos(0,-t/2,0)*small,Pos(0,-t/2+rim,0)*face,Pos(0,t/2-rim,0)*face,Pos(0,t/2,0)*small])
bb=ear.bounding_box()
ear=scale(ear,by=(w/bb.size.X,t/bb.size.Y,h/bb.size.Z))
bb=ear.bounding_box()
ear=Pos(-(bb.min.X+bb.max.X)/2,-(bb.min.Y+bb.max.Y)/2,-bb.min.Z)*ear

# Ear root follows the host shell exterior, so it does not protrude into storage.
bw=param("body_width",180.0)
bd=param("body_depth",160.0)
bh=param("body_height",190.0)
sx=param("ear_left_shell_center_x",70.0)
sy=param("ear_left_shell_center_y",-5.0)
sz=param("ear_left_shell_base_z",-118.0)
host_profile=[(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
host=loft([Pos(sx,sy,sz+z*bh)*Ellipse(bw/2*rx,bd/2*ry) for z,rx,ry in host_profile])
ear=ear-host
ear.color=Color(.018,.018,.022)
publish("ear_left",ear,"左侧大耳朵",material="resin")
