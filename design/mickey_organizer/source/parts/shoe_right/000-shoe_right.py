
w=param("shoe_right_width",98.0)
d=param("shoe_right_length",145.0)
h=param("shoe_right_toe_height",72.0)
sole=param("shoe_right_sole_height",8.0)
groove=param("shoe_right_sole_groove_depth",0.6)
cw=param("shoe_right_cuff_width",53.0)
cd=param("shoe_right_cuff_depth",56.0)
ch=param("shoe_right_cuff_height",14.0)
cy=param("shoe_right_cuff_y",34.99528471939117)
cz=param("shoe_right_cuff_base_z",70.0)
# Full rounded toe, thick flat sole and raised rear ankle collar from reference_photo.
p=[(0,.88,.89,0),(.07,.97,.97,0),(.18,1,1,0),(.45,.98,.94,2),(.70,.85,.78,10),(.90,.59,.52,22),(1,.27,.26,30)]
toe=loft([Pos(0,y,z*h)*Ellipse(w/2*rx,d/2*ry) for z,rx,ry,y in p])
bb=toe.bounding_box()
toe=scale(toe,by=(w/bb.size.X,d/bb.size.Y,h/bb.size.Z))
bb=toe.bounding_box()
toe=Pos(-(bb.min.X+bb.max.X)/2,-(bb.min.Y+bb.max.Y)/2,-bb.min.Z)*toe
ring=Pos(0,0,sole)*extrude(Ellipse(w,d)-Ellipse(w/2-groove,d/2-groove),amount=.8)
toe=toe-ring
cuff=loft([Pos(0,cy,cz)*Ellipse(cw*.47,cd*.47),Pos(0,cy,cz+ch*.2)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch*.75)*Ellipse(cw/2,cd/2),Pos(0,cy,cz+ch)*Ellipse(cw*.47,cd*.47)])
shoe=toe+cuff
shoe.color=Color(1.0,.68,.008)
assert len(shoe.solids())==1
publish("shoe_right",shoe,"右侧黄色鞋子",material="resin")
