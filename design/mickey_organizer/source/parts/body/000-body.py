
body_width = param("body_width", 180.0)
body_depth = param("body_depth", 160.0)
body_height = param("body_height", 190.0)
body_wall = param("body_wall", 4.5)
body_floor = param("body_floor", 4.0)
body_opening_width = param("body_opening_width", 132.0)
body_opening_height = param("body_opening_height", 122.0)
body_opening_bottom = param("body_opening_bottom", 57.0)
body_seam_height = param("body_seam_height", 50.0)
body_seam_depth = param("body_seam_depth", 0.5)
body_lip_radius = param("body_lip_radius",1.2)
# Section proportions reconstructed from reference_photo; unseen back is symmetric.
profile = [(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
outer_sections = [Pos(0,0,h*body_height)*Ellipse(body_width/2*rx, body_depth/2*ry) for h,rx,ry in profile]
outer = loft(outer_sections)

body_cavity_floor_raise = param("body_cavity_floor_raise",20.0)
body_cavity_blend = param("body_cavity_blend",12.0)
# Constant normal offset provides the requested 4.5 mm wall.
inner = offset(outer,amount=-body_wall)
floor_z = body_floor + body_cavity_floor_raise
inner = inner & Pos(0,0,floor_z)*Box(body_width*3,body_depth*3,body_height*2,align=(Align.CENTER,Align.CENTER,Align.MIN))
bottom_edges=[e for e in inner.edges() if abs(e.bounding_box().min.Z-floor_z)<.01 and abs(e.bounding_box().max.Z-floor_z)<.01]
inner = fillet(bottom_edges,radius=body_cavity_blend)
w = body_opening_width/2
h = body_opening_height
b = body_opening_bottom
# Rounded arch with a broad soft lower lip, matching the photograph.

edges = [
Bezier((0,0,b),(-.78*w,0,b),(-w,0,b+.06*h),(-w,0,b+.40*h)),
Bezier((-w,0,b+.40*h),(-w,0,b+.84*h),(-.58*w,0,b+h),(0,0,b+h)),
Bezier((0,0,b+h),(.58*w,0,b+h),(w,0,b+.84*h),(w,0,b+.40*h)),
Bezier((w,0,b+.40*h),(w,0,b+.06*h),(.78*w,0,b),(0,0,b))]
mouth = Face(Wire(edges))
cutter = extrude(Pos(0,-body_depth,0)*mouth, amount=body_depth*1.1,dir=(0,1,0))
body = outer - inner - cutter
# Fine horizontal molded seam below the mouth, as seen on the original.
seam_outer = Pos(0,0,body_seam_height)*Ellipse(body_width,body_depth)
seam_inner = Pos(0,0,body_seam_height)*Ellipse(body_width/2-body_seam_depth,body_depth/2-body_seam_depth)
# Keep the seam shallow at the broad lower-body section.
seam = extrude(seam_outer-seam_inner, amount=.65)
body = body-seam
lip_edges=[e for e in body.edges() if e.center().Y < -20 and e.bounding_box().min.Z > b-1 and e.bounding_box().max.Z < b+h+1 and e.length > 30]
body=fillet(lip_edges,radius=body_lip_radius)
assert inner.is_valid
body.color = Color(.34,.012,.045)
assert len(body.solids()) == 1
publish("body",body,"酒红收纳盒身",material="resin")
