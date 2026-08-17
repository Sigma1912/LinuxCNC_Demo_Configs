#!/usr/bin/env python3

from vtk_vismach import *
import hal
import sys

#for setting in sys.argv[1:]: exec(setting)
options = sys.argv[1:]


comp_name = 'dh-sim-gui'

if not hal.component_exists(comp_name):
    # create a hal component for this model
    c = hal.component(comp_name)
    c.newpin("a1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a6", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha6", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d6", hal.HAL_S32, hal.HAL_IN)
    c.newpin("joint0", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint1", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint2", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint3", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint4", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint5", hal.HAL_REAL, hal.HAL_IN)
    c.ready()

# joint/link color
col_base = '#D3D3D3' # 'LightGray'
col1 =     '#DCDCDC' # 'Gainsboro'
col2 =     '#DEB887' # 'BurlyWood'
col3 =     '#FF8C00' # 'DarkOrange'
col4 =     '#FFFF00' # 'Yellow'
col5 =     '#008000' # 'Green'
col6 =     '#008080' # 'Teal'

#guivars
myvars = []
myvars.append(["a1", -1000, 1000,  85, col1])
myvars.append(["alpha1", -90, 90, -90, col1])
myvars.append(["d1", -1000, 1000, 350, col1])

myvars.append(["a2", -1000, 1000, 380, col2])
myvars.append(["alpha2", -90, 90,   0, col2])
myvars.append(["d2", -1000, 1000,   0, col2])

myvars.append(["a3", -1000, 1000, 100, col3])
myvars.append(["alpha3", -90, 90, -90, col3])
myvars.append(["d3", -1000, 1000,   0, col3])

myvars.append(["a4", -1000, 1000,   0, col4])
myvars.append(["alpha4", -90, 90,  90, col4])
myvars.append(["d4", -1000, 1000, 425, col4])

myvars.append(["a5", -1000, 1000,   0, col5])
myvars.append(["alpha5", -90, 90, -90, col5])
myvars.append(["d5", -1000, 1000,   0, col5])

myvars.append(["a6", -1000, 1000,   0, col6])
myvars.append(["alpha6", -90, 90,   0, col6])
myvars.append(["d6", -1000, 1000,  85, col6])

  # joint/link color
baseplane = Collection([
                  Axes(150),
                  Box(100,100,2),
                  Color([Grid(800,10)],col_base,0.3),
                  Capture('work')
                  ])
alpha = 1         # joint/link color alpha
d_joint = 40/2      # diameter joint cylinder
l_joint = 30        # length of joint cylinder
d_link = 30/2       # diameter link cylinder



hand = Collection([ Axes(120), Capture('tool') ])

# Here we draw the A-offset before the D-offset so we don't have the corner sphere at the end when a6 is zero
joint6 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col6, alpha) ])
arm = Collection([hand, Axes(80), joint6 ])
arm = Rotate([hand],c, "alpha6", 1,0,0)
arm = Translate([arm],c,0,0,'d6')
d6 = Color([ CylinderZ(c, 'd6', d_link)], col6, alpha)
a6 = Color([ CylinderX(c, '-{a6}', d_link)], col6, alpha)
link5 = Collection([ a6,
                     d6,
                     Color([Sphere(d_link)], col6, alpha)
                  ])
arm = Translate([arm, link5],c, 'a6',0,0)


# From hero on we draw the D-offset before the A-offset because it makes the manipulator look better :)
joint5 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col6, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col5, alpha)
                    ])
arm = Collection([arm, Axes(80), joint5 ])
arm = Rotate([arm],c, "joint5", 0,0,1)
arm = Rotate([arm],c, "alpha5", 1,0,0)
arm = Translate([arm],c,'a5',0,0)
d5 = Color([ CylinderZ(c, '-{d5}', d_link)], col5, alpha)
a5 = Color([ CylinderX(c, 'a5', d_link)], col5, alpha)
link4 = Collection([ a5,
                     d5,
                     Color([Sphere(d_link)], col5, alpha)
                  ])
arm = Translate([arm, link4],c, 0,0,'d5')


joint4 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col5, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col4, alpha)
                    ])
arm = Collection([arm, Axes(80), joint4 ])
arm = Rotate([arm],c, "joint4", 0,0,1)
arm = Rotate([arm],c, "alpha4", 1,0,0)
arm = Translate([arm],c,'a4',0,0)
d4 = Color([ CylinderZ(c, '-{d4}', d_link)], col4, alpha)
a4 = Color([ CylinderX(c, 'a4', d_link)], col4, alpha)
link3 = Collection([ a4,
                     d4,
                     Color([Sphere(d_link)], col4, alpha)
                  ])
arm = Translate([arm, link3],c, 0,0,'d4')


joint3 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col4, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col3, alpha)
                    ])
arm = Collection([arm, Axes(80), joint3 ])
arm = Rotate([arm],c, "joint3", 0,0,1)
arm = Rotate([arm],c, "alpha3", 1,0,0)
arm = Translate([arm],c,'a3',0,0)
d3 = Color([ CylinderZ(c, '-{d3}', d_link)], col3, alpha)
a3 = Color([ CylinderX(c, 'a3', d_link)], col3, alpha)
link2 = Collection([ a3,
                     d3,
                     Color([Sphere(d_link)], col3, alpha)
                  ])
arm = Translate([arm, link2],c, 0,0,'d3')


joint2 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col3, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col2, alpha)
                    ])
arm = Collection([arm, Axes(80), joint2 ])
arm = Rotate([arm],c, "joint2", 0,0,1)
arm = Rotate([arm],c, "alpha2", 1,0,0)
arm = Translate([arm],c,'a2',0,0)
d2 = Color([ CylinderZ(c, '-{d2}', d_link)], col2, alpha)
a2 = Color([ CylinderX(c, 'a2', d_link)], col2, alpha)
link1 = Collection([ a2,
                     d2,
                     Color([Sphere(d_link)], col2, alpha)
                  ])
arm = Translate([arm, link1],c, 0,0,'d2')


joint1 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col2, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col1, alpha)
                    ])
arm = Collection([arm, Axes(80), joint1 ])
arm = Rotate([arm],c, "joint1", 0,0,1)
arm = Rotate([arm],c, "alpha1", 1,0,0)
arm = Translate([arm],c,'a1',0,0)
d1 = Color([ CylinderZ(c, '-{d1}', d_link)], col1, alpha)
a1 = Color([ CylinderX(c, 'a1', d_link)], col1, alpha)
link0 = Collection([ a1,
                     d1,
                     Color([Sphere(d_link)], col1, alpha)
                  ])
arm = Translate([arm, link0],c, 0,0,'d1')


joint0 = Collection([ Color([ CylinderZ(l_joint/2, d_joint)], col1, alpha)
                    ])
arm = Collection([arm, Axes(80), joint0 ])
arm = Rotate([arm],c, "joint0", 0,0,1)

model = Collection([
      baseplane,
      arm,
        ])
#hud
myhud = Hud(color='mint',opacity=0.4) # This will always be displayed
myhud.add_txt('Visualizer for "standard" DH-Parameters')
myhud.add_txt('--------------------------------------')
myhud.extra_text_enable = True

main(options,
     comp_name,
     model,
     huds = myhud,
     guivars = myvars,
     window_width = 1400,
     window_height = 1000,
     window_title = 'DH-Simulator')
