
body_width = param("body_width", 180.0)
body_depth = param("body_depth", 160.0)
body_height = param("body_height", 190.0)
body_wall = param("body_wall", 3.0)
body_floor = param("body_floor", 4.0)
body_opening_width = param("body_opening_width", 132.0)
body_opening_height = param("body_opening_height", 122.0)
body_opening_bottom = param("body_opening_bottom", 57.0)
body_seam_height = param("body_seam_height", 50.0)
body_seam_depth = param("body_seam_depth", 0.5)
# Section proportions reconstructed from reference_photo; unseen back is symmetric.
profile = [(0,.36,.42),(.025,.65,.64),(.095,.89,.87),(.23,1,1),(.48,.94,.96),(.70,.83,.86),(.84,.75,.75),(.92,.60,.58),(.975,.35,.32),(1,.04,.04)]
outer_sections = [Pos(0,0,h*body_height)*Ellipse(body_width/2*rx, body_depth/2*ry) for h,rx,ry in profile]
outer = loft(outer_sections)
inner_sections = [Pos(0,0,body_floor)*Ellipse(body_width*.36/2-body_wall, body_depth*.42/2-body_wall)]
inner_sections += [Pos(0,0,h*body_height)*Ellipse(body_width/2*rx-body_wall,body_depth/2*ry-body_wall) for h,rx,ry in profile[1:-1]]
inner_sections += [Pos(0,0,body_height-body_wall)*Ellipse(5,5)]
inner = loft(inner_sections)
w = body_opening_width/2
h = body_opening_height
b = body_opening_bottom
# Rounded arch with a broad soft lower lip, matching the photograph.
pts = [(0,0,b),(-.65*w,0,b+.04*h),(-.96*w,0,b+.20*h),(-w,0,b+.48*h),(-.90*w,0,b+.76*h),(-.60*w,0,b+.96*h),(0,0,b+h),(.60*w,0,b+.96*h),(.90*w,0,b+.76*h),(w,0,b+.48*h),(.96*w,0,b+.20*h),(.65*w,0,b+.04*h)]

edges = [
Bezier((0,0,b),(-.90*w,0,b),(-w,0,b+.06*h),(-w,0,b+.40*h)),
Bezier((-w,0,b+.40*h),(-w,0,b+.84*h),(-.70*w,0,b+h),(0,0,b+h)),
Bezier((0,0,b+h),(.70*w,0,b+h),(w,0,b+.84*h),(w,0,b+.40*h)),
Bezier((w,0,b+.40*h),(w,0,b+.06*h),(.90*w,0,b),(0,0,b))]
mouth = Face(Wire(edges))
cutter = extrude(Pos(0,-body_depth,0)*mouth, amount=body_depth*1.1,dir=(0,1,0))
body = outer - inner - cutter
# Fine horizontal molded seam below the mouth, as seen on the original.
seam_outer = Pos(0,0,body_seam_height)*Ellipse(body_width,body_depth)
seam_inner = Pos(0,0,body_seam_height)*Ellipse(body_width/2-body_seam_depth,body_depth/2-body_seam_depth)
# Keep the seam shallow at the broad lower-body section.
seam = extrude(seam_outer-seam_inner, amount=.65)
body = body-seam
body.color = Color(.34,.012,.045)
assert len(body.solids()) == 1
publish("body",body,"酒红收纳盒身",material="resin")
