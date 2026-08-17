#!/usr/bin/env python3

from vtk_vismach import *
import hal
import sys

#for setting in sys.argv[1:]: exec(setting)
options = sys.argv[1:]


comp_name = 'mod-dh-sim-gui'

if not hal.component_exists(comp_name):
    # create a hal component for this model
    c = hal.component(comp_name)
    c.newpin("a0", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("a5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha0", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("alpha5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d0", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d1", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d2", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d3", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d4", hal.HAL_S32, hal.HAL_IN)
    c.newpin("d5", hal.HAL_S32, hal.HAL_IN)
    c.newpin("joint0", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint1", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint2", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint3", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint4", hal.HAL_REAL, hal.HAL_IN)
    c.newpin("joint5", hal.HAL_REAL, hal.HAL_IN)
    c.ready()

# joint/link color
col_base = '#D3D3D3' # 'LightGray'
col0 =     '#DCDCDC' # 'Gainsboro'
col1 =     '#DEB887' # 'BurlyWood'
col2 =     '#FF8C00' # 'DarkOrange'
col3 =     '#FFFF00' # 'Yellow'
col4 =     '#008000' # 'Green'
col5 =     '#008080' # 'Teal'

#guivars
myvars = []
myvars.append(["a0", -1000, 1000,   0, col0])
myvars.append(["alpha0", -90, 90,   0, col0])
myvars.append(["d0", -1000, 1000, 350, col0])

myvars.append(["a1", -1000, 1000,  85, col1])
myvars.append(["alpha1", -90, 90, -90, col1])
myvars.append(["d1", -1000, 1000,   0, col1])

myvars.append(["a2", -1000, 1000, 380, col2])
myvars.append(["alpha2", -90, 90,   0, col2])
myvars.append(["d2", -1000, 1000,   0, col2])

myvars.append(["a3", -1000, 1000, 100, col3])
myvars.append(["alpha3", -90, 90, -90, col3])
myvars.append(["d3", -1000, 1000, 425, col3])

myvars.append(["a4", -1000, 1000,   0, col4])
myvars.append(["alpha4", -90, 90,  90, col4])
myvars.append(["d4", -1000, 1000,   0, col4])

myvars.append(["a5", -1000, 1000,   0, col5])
myvars.append(["alpha5", -90, 90, -90, col5])
myvars.append(["d5", -1000, 1000,  85, col5])

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


link5 = Collection([ Translate([hand],c, 0,0,'d5'),
                     Color([Sphere(d_link)], col5, alpha)
                  ])
d5 = Color([ CylinderZ(c, 'd5', d_link)], col5, alpha)
a5 = Color([ CylinderX(c, '-{a5}', d_link)], col5, alpha)
arm = Rotate([link5,d5],c, "joint5", 0,0,1)
arm = Rotate([arm,a5],c, "alpha5", 1,0,0)
arm = Translate([arm],c, 'a5',0,0)



joint4 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col5, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col4, alpha)
                    ])
arm = Collection([arm, Axes(80), joint4 ])
link4 = Collection([ Translate([arm],c, 0,0,'d4'),
                     Color([Sphere(d_link)], col4, alpha)
                  ])
d4 = Color([ CylinderZ(c, 'd4', d_link)], col4, alpha)
a4 = Color([ CylinderX(c, '-{a4}', d_link)], col4, alpha)
arm = Rotate([link4,d4],c, "joint4", 0,0,1)
arm = Rotate([arm,a4],c, "alpha4", 1,0,0)
arm = Translate([arm],c, 'a4',0,0)



joint3 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col4, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col3, alpha)
                    ])
arm = Collection([arm, Axes(80), joint3 ])
link3 = Collection([ Translate([arm],c, 0,0,'d3'),
                     Color([Sphere(d_link)], col3, alpha)
                  ])
d3 = Color([ CylinderZ(c, 'd3', d_link)], col3, alpha)
a3 = Color([ CylinderX(c, '-{a3}', d_link)], col3, alpha)
arm = Rotate([link3,d3],c, "joint3", 0,0,1)
arm = Rotate([arm,a3],c, "alpha3", 1,0,0)
arm = Translate([arm],c, 'a3',0,0)



joint2 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col3, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col2, alpha)
                    ])
arm = Collection([arm, Axes(80), joint2 ])
link2 = Collection([ Translate([arm],c, 0,0,'d2'),
                     Color([Sphere(d_link)], col2, alpha)
                  ])
d2 = Color([ CylinderZ(c, 'd2', d_link)], col2, alpha)
a2 = Color([ CylinderX(c, '-{a2}', d_link)], col2, alpha)
arm = Rotate([link2,d2],c, "joint2", 0,0,1)
arm = Rotate([arm,a2],c, "alpha2", 1,0,0)
arm = Translate([arm],c, 'a2',0,0)



joint1 = Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col2, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col1, alpha)
                    ])
arm = Collection([arm, Axes(80), joint1 ])
link1 = Collection([ Translate([arm],c, 0,0,'d1'),
                     Color([Sphere(d_link)], col1, alpha)
                  ])
d1 = Color([ CylinderZ(c, 'd1', d_link)], col1, alpha)
a1 = Color([ CylinderX(c, '-{a1}', d_link)], col1, alpha)
arm = Rotate([link1,d1],c, "joint1", 0,0,1)
arm = Rotate([arm,a1],c, "alpha1", 1,0,0)
arm = Translate([arm],c, 'a1',0,0)



joint0 =  Collection([ Color([ CylinderZ( *(0,0,-l_joint/2), l_joint/2, d_joint)], col1, alpha),
                      Color([ CylinderZ(l_joint/2, d_joint)], col0, alpha)
                    ])
arm = Collection([arm, Axes(80), joint0 ])
link0 = Collection([ Translate([arm],c, 0,0,'d0'),
                     Color([Sphere(d_link)], col0, alpha)
                  ])
d0 = Color([ CylinderZ(c, 'd0', d_link)], col0, alpha)
a0 = Color([ CylinderX(c, '-{a0}', d_link)], col0, alpha)
arm = Rotate([link0,d0],c, "joint0", 0,0,1)
arm = Rotate([arm,a0],c, "alpha0", 1,0,0)
arm = Translate([arm],c, 'a0',0,0)


model = Collection([
      baseplane,
      arm,
        ])
#hud
myhud = Hud(color='mint',opacity=0.4) # This will always be displayed
myhud.add_txt('Visualizer for "modified" DH-Parameters')
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
