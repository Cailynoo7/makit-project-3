w=param("shoe_right_width",98.0)
d=param("shoe_right_length",130.0)
h=param("shoe_right_toe_height",72.0)
sole=param("shoe_right_sole_height",8.0)
groove=param("shoe_right_sole_groove_depth",0.6)
cw=param("shoe_right_cuff_width",53.0)
cd=param("shoe_right_cuff_depth",56.0)
ch=param("shoe_right_cuff_height",14.0)
cy=param("shoe_right_cuff_y",20.0)
cz=param("shoe_right_cuff_base_z",70.0)
gh=param("shoe_right_sole_groove_height",0.8)
# Reference photo: compact inflated toe, curved heel, flat thick sole.
# One continuous loft closes into the ankle, avoiding a forward-sloping slab.
p=[(0,.89,.90,0),(sole/h,.98,.98,0),(.30,1,1,0),(.55,.91,.92,.08),(.76,.70,.72,.25),(.91,.45,.44,.55),(1,.23,.25,1)]
toe=loft([Pos(0,cy*y,z*h)*Ellipse(w/2*rx,d/2*ry) for z,rx,ry,y in p])
bb=toe.bounding_box()
toe=scale(toe,by=(w/bb.size.X,d/bb.size.Y,h/bb.size.Z))
bb=toe.bounding_box()
toe=Pos(-(bb.min.X+bb.max.X)/2,-(bb.min.Y+bb.max.Y)/2,-bb.min.Z)*toe
ring=Pos(0,0,sole)*extrude(Ellipse(w,d)-Ellipse(w*.49-groove,d*.49-groove),amount=gh)
toe=toe-ring
cuff=loft([Pos(0,cy,cz)*Ellipse(cw*.47,cd*.47),Pos(0,cy,cz+ch*.2)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch*.75)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch)*Ellipse(cw*.47,cd*.47)])
shoe=toe+cuff
shoe.color=Color(1.0,.68,.008)
assert len(shoe.solids())==1
publish("shoe_right",shoe,"右侧黄色鞋子",material="resin")
