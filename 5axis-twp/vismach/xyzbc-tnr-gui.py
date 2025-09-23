#!/usr/bin/env python3

# This is a machine simulation for the 'xyzbca-trsrn' simulation config in linuxcnc
# Author: David mueller
# email: mueller_david@hotmail.com

from twp_vismach import *
import hal
import math
import sys
import os

# getting the name of the directory
# where the this file is present.
current = os.path.dirname(os.path.realpath(__file__))
# Getting the parent directory name
# where the current directory is present.
parent = os.path.dirname(current)
# adding the parent directory to
# the sys.path.
sys.path.append(parent)
# now we can import the module in the parent
# directory.
from twp_vismach import *

# Machine zero as measured from center surface of the rotary c table
machine_zero_x = -1000
machine_zero_y =  1000
machine_zero_z =  1000

'''
try: # Expect files in working directory
    # NOTE to run this file as standalone python script absolute paths might have to be used to find the stl files
    # create the work piece from file
    work_piece = AsciiSTL(filename="./work_piece_1.stl")

except Exception as detail:
    print(detail)
    raise SystemExit("xyzbc-tnr-gui requires stl files in working directory")
'''

for setting in sys.argv[1:]: exec(setting)

c = hal.component("xyzbc-tnr-gui")
# axis_x
c.newpin("axis_x", hal.HAL_FLOAT, hal.HAL_IN)
# axis_y
c.newpin("axis_y", hal.HAL_FLOAT, hal.HAL_IN)
# head vertical slide
c.newpin("axis_z", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_a
c.newpin("rotary_a", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_b
c.newpin("rotary_b", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_c
c.newpin("rotary_c", hal.HAL_FLOAT, hal.HAL_IN)
# nutation-angle
c.newpin("nutation_angle", hal.HAL_FLOAT, hal.HAL_IN)
# nutation-angle
c.newpin("table_angle", hal.HAL_FLOAT, hal.HAL_IN)
# tool offsets
c.newpin("tool_length", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_diameter", hal.HAL_FLOAT, hal.HAL_IN)
# geometric offsets in the spindle-rotary-assembly
c.newpin("offset_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("offset_z", hal.HAL_FLOAT, hal.HAL_IN)
# rot-point offsets: distances from rotation point (table BC) in absolute (machine) coordinates
c.newpin("x_rot_point", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("y_rot_point", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("z_rot_point", hal.HAL_FLOAT, hal.HAL_IN)
# selected kinematics
c.newpin("kinstype_select", hal.HAL_FLOAT, hal.HAL_IN)
# work piece show/hide
c.newpin("hide_work_piece_1", hal.HAL_BIT, hal.HAL_IN)
# spindle body show/hide
c.newpin("hide_spindle_body", hal.HAL_BIT, hal.HAL_IN)
# work piece show/hide
c.newpin("hide_somethingelse", hal.HAL_BIT, hal.HAL_IN)
# scale coordinate system indicators
c.newpin("scale_coords", hal.HAL_FLOAT, hal.HAL_IN)
# twp pins
c.newpin("twp_status", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_defined", hal.HAL_BIT, hal.HAL_IN)
c.newpin("twp_active", hal.HAL_BIT, hal.HAL_IN)
# the origin of the twp plane display needs to be independent of the work offsets
# because those offsets change as the kinematic switches between world and tool
c.newpin("twp_ox_world", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_oy_world", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_oz_world", hal.HAL_FLOAT, hal.HAL_IN)
# origin of the twp
c.newpin("twp_ox", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_oy", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_oz", hal.HAL_FLOAT, hal.HAL_IN)
# normal vector defining the temporary work plane as set by user with G68.2
c.newpin("twp_zx", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_zy", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_zz", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_xx", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_xy", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("twp_xz", hal.HAL_FLOAT, hal.HAL_IN)

c.ready()


# give endpoint Z values and radii
# resulting cylinder is on the Z axis
class HalToolCylinder(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 6 # default if hal pin not set
        if (c.tool_diameter > 0): r = c.tool_diameter/2
        return -c.tool_length, r, 0, r

# used to create the spindle housing as set by the variable 'pivot_z'
class HalSpindleHousingZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 140
        length = c.pivot_z + 200
        # start the spindle housing at height 100 above the spindle nose
        return length, r, 100, r

# used to create the spindle rotary as set by the variable 'pivot_x'
class HalSpindleHousingX(CylinderX):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 120
        length =  c.pivot_x + 150
        return length, r-30, 0, r

# used to create an indicator for the variable 'pivot_z'
class HalPivotZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = c.pivot_z
        # start the spindle housing at height 100 above the spindle nose
        return length, 1, 0, r

# used to create an indicator for the variable 'pivot_x'
class HalPivotX(CylinderX):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.pivot_x
        return length, r, 0, 1

# used to create an indicator for the variable 'offset_x'
class HalOffsetX(CylinderX):
    def __init__(self, comp, *args):
        CylinderX.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.offset_x
        return length, r, 0, 1

# used to create visual indicators for the variable z-offset
class HalOffsetZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = c.offset_z
        return length, 1, 0, r

# used to create a thin tool-offset indicator
class HalToolOffset(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        return -c.tool_length, 1, 0, r

class CoordSystem(Collection):
    # creates a visual object for a coordinate system
    def __init__(self, comp, r=4, l=500):
        # set arrow length
        al = l/10
        self.parts = [Color([1,0,0,1],[CylinderX(0,r,l,r)]),
                      Color([1,0,0,1],[CylinderX(l,2*r,l+al,1)]),
                      Color([0,1,0,1],[CylinderY(0,r,l,r)]),
                      Color([0,1,0,1],[CylinderY(l,2*r,l+al,1)]),
                      Color([0,0,1,1],[CylinderZ(0,r,l,r)]),
                      Color([0,0,1,1],[CylinderZ(l,2*r,l+al,1)]),
                     ]

class Point(Collection):
    # creates a visual object for a point
    def __init__(self, comp, r=1, l=20, color=[1,1,1,1]):
        self.parts = [Color(color,[CylinderX(-l,r,l,r)],1),
                      Color(color,[CylinderY(-l,r,l,r)],1),
                      Color(color,[CylinderZ(-l,r,l,r)],1)
                     ]

# used to rotate parts around the nutation axis
class HalRotateNutation(HalVectorRotate):
    def __init__(self, parts, comp, var, th, x, y, z):
        HalVectorRotate.__init__(self, parts, comp, var, th, x, y, z)
        self.parts = parts
        self.comp = c
        self.var = var
        self.th = th

    def get_values(self):
        th = self.th
        x = 0
        y = -th*sin(radians(c.nutation_angle))
        z =  th*cos(radians(c.nutation_angle))

        return th, x, y, z

class HalLineComplex(HalLine):
    def __init__(self, comp, x1var, y1var, z1var, x2var, y2var, z2var, stretch, r=5):
        self.comp = comp
        self.x1var = x1var
        self.y1var = y1var
        self.z1var = z1var
        self.x2var = x2var
        self.y2var = y2var
        self.z2var = z2var
        self.stretch = stretch
        self.r = r
        self.q = gluNewQuadric()

    def draw(self):
        x1 = 0 if self.x1var == 0 else self.comp[self.x1var]
        y1 = 0 if self.y1var == 0 else self.comp[self.y1var]
        z1 = 0 if self.z1var == 0 else self.comp[self.z1var]
        x2 = 0 if self.x2var == 0 else self.comp[self.x2var] - cos(radians(-c.table_angle))*c.offset_x
        y2 = 0 if self.y2var == 0 else self.comp[self.y2var] - sin(radians(-c.table_angle))*c.offset_x
        z2 = 0 if self.z2var == 0 else self.comp[self.z2var] - c.offset_z
        r = self.r
        v = [x2,y2,z2]
        length, angle, axis = self.polar(v)
        glPushMatrix()
        glTranslate(x1, y1, z1)
        glRotate(angle,*axis)
        gluCylinder(self.q, r, r, length, 32, 1)


class HalVectorTranslateComplex(HalVectorTranslate):
    def __init__(self, parts, comp, xvar, yvar, zvar, scale=1):
        self.parts = parts
        self.comp = comp
        self.xvar = xvar
        self.yvar = yvar
        self.zvar = zvar
        self.sc   = scale

    def apply(self):
        # check for zero vector components
        xvar = cos(radians(-c.table_angle))*c.offset_x
        yvar = sin(radians(-c.table_angle))*c.offset_x
        zvar = c.offset_z
        sc = self.sc
        glPushMatrix()
        glTranslatef(sc*xvar, sc*yvar, sc*zvar)


#indicators
# create vismach coordinates to show the origin during construction of the model
vismach_coords = CoordSystem(c,2,5000)

# create absolute coordinates
abs_coords = CoordSystem(c,8,300)
abs_coords = HalScale([abs_coords],c,1,1,1,"scale_coords")
# create work coordinates that represent work_offset offset
work_coords = CoordSystem(c,3,200)
work_coords = HalScale([work_coords],c,1,1,1,"scale_coords")
# work coordinates for identity mode
work_coords_ident = HalShow([work_coords],c,0,"kinstype_select")
work_coords_ident = HalVectorTranslateComplex([work_coords_ident],c,"offset_x",0,"offset_z",-1)

# move the coords to the designated (x,y,z)-home position
abs_coords = Translate([abs_coords], machine_zero_x, machine_zero_y, machine_zero_z)
work_coords_ident = Translate([work_coords_ident], machine_zero_x, machine_zero_y, machine_zero_z)
# move the  work offsets by the work_offset
work_coords_ident = HalVectorTranslate([work_coords_ident],c,"twp_ox_world","twp_oy_world","twp_oz_world")

# create an indicator line from the reference point to the work_offset position
line_work_offset =  Color([1,1,1,1],[HalLineComplex(c,0,0,0,"twp_ox_world","twp_oy_world","twp_oz_world",1,1)])
line_work_offset = HalShow([line_work_offset],c,0,"kinstype_select")
line_work_offset = Translate([line_work_offset], machine_zero_x, machine_zero_y, machine_zero_z)
coords = Collection([
              abs_coords,
              work_coords_ident,
              line_work_offset
              ])

# create a visual for the position of the rotation point of the rotary assembly
# as set by the user with the "rot-axis-x","rot-axis-y" and "rot-axis-z" sliders in the axis gui
rot_point = Color([1,1,0.3,1],[Point(c)])
# move the rot_point indicator by the designated (x,y)-home offset
rot_point= Translate([rot_point], machine_zero_x,  machine_zero_z, machine_zero_z)
# now move it to the position set by the user
rot_point = HalVectorTranslate([rot_point],c,"x_rot_point","y_rot_point","z_rot_point")
#/indicators

#tool-side
tooltip = Capture()
tool_cylinder = Collection([
                HalToolCylinder(c),
                # this indicates the spindle nose when no tool-offset is active
                CylinderZ(-0.1, 0, 0.1, 10),
                ])
tool_cylinder = HalShow([tool_cylinder],c,True,"hide_spindle_body",0,1)
# create an indicator for the tool-offset when spindle-body is hidden
ind_tool_offset = Color([1,0,1,0],[HalToolOffset(c)])
tool = Collection([
       HalTranslate([tooltip], c, "tool_length", 0,0,-1),
       tool_cylinder,
       ind_tool_offset,
       # create a visual indicator for the position of the control point
       HalTranslate([Point(c)], c, "tool_length", 0,0,-1)
       ])
tool = Color([1,0,1,0], [tool] )

spindle_column = Collection([
                 # spindle
                 Color([0.7,0.7,0,1],[CylinderZ(0,100,100,200)]),
                 Color([0.7,0.7,0,1],[CylinderZ(100,200,400,200)]),
                 Color([0.7,0.7,0,1],[Box(-300, -300, 400, 300, 300, 2500)])
                 ])
spindle_column = HalShow([spindle_column],c,True,"hide_spindle_body",0,1)
spindle_column = Collection([
                 tool,
                 spindle_column
                 ])
# the spindle column moves with x and z
spindle_column = HalTranslate([spindle_column],c,"axis_x",1,0,0)
spindle_column = HalTranslate([spindle_column],c,"axis_z",0,0,1)
# add a block for the y-axis to the spindle assembly
slide_y = Color([0.6,0.8,0.3,0], [Box(-500, 300, 450, 2500, 3000, 1500)])
slide_y = HalShow([slide_y],c,True,"hide_spindle_body",0,1)
spindle_xyz = Collection([
             spindle_column,
             slide_y
             ])
#spindle_xyz moves with y
spindle_xyz = HalTranslate([spindle_xyz],c,"axis_y",0,1,0)
# move the spindle_xyz to it's designated home position
spindle_xyz = Translate([spindle_xyz], machine_zero_x, machine_zero_y, machine_zero_z)
#/tool-side


# work-side
work = Capture()
# create a simple cube
work_piece = BoxCenteredXY(600, 600, 600)
# create a more complex work piece from stl
#work_piece = Translate([work_piece],0,0,0)
work_piece = Color([0.5,0.5,0.5,0.9], [work_piece])
# make the work piece hideable
work_piece = HalShow([work_piece],c,1,"hide_work_piece_1",0,1)
# create rotary_table_c
rotary_table_c = Collection([
                 Color([0.1,0.7,0.9,0],[CylinderZ(0,920,-100,920)]),
                 Color([0.1,0.7,0.9,0],[CylinderZ(-100,920,-200,870)])
                 ])
rotary_c_attachment = Collection([
                 Color([1,0.5,0,0],[CylinderZ(-200,870,-400,870)]),
                 Color([1,0.5,0,0],[CylinderZ(-400,870,-700,500)])
                 ])
# create 'slots' in x and y on the rotary table
slot_x_w = Color([1,1,1,1],[BoxCenteredXY(1840, 10, 0.9)])
slot_y_w = Color([1,1,1,1],[BoxCenteredXY(10, 1840, 0.9)])
# create indicator of at positive end of x slot
slot_pocket = Color([1,1,1,1],[CylinderZ(0,20,1,20)])
slot_pocket = Translate([slot_pocket],900,0,0)
# work coordinates for tcp mode
work_coords_tcp = HalShow([work_coords],c,1,"kinstype_select")
work_coords_tcp = Translate([work_coords_tcp]  , machine_zero_x, machine_zero_y, machine_zero_z)
work_coords_tcp = HalVectorTranslate([work_coords_tcp]  ,c,"twp_ox_world","twp_oy_world","twp_oz_world")

# create the work_plane using the origin (as measured from current work offset (world),
# normal vector vz(zx,zy,zz) and  x-orientation vector vx(xx,xy,xz)
# these are used to create a transformation matrix
# with y-orientation vector being the cross_product(vz, vx)
work_plane =  HalGridFromNormalAndDirection(c,
        "twp_ox", "twp_oy", "twp_oz",
        "twp_xx", "twp_xy", "twp_xz",
        "twp_zx", "twp_zy", "twp_zz"
        )
# create a coordinate system for the twp-plane
work_plane_coords =  HalCoordsFromNormalAndDirection(c,
        "twp_ox", "twp_oy", "twp_oz",
        "twp_xx", "twp_xy", "twp_xz",
        "twp_zx", "twp_zy", "twp_zz",
        10,3
        )
# for twp-defined = true, we show the plane in gray
work_plane_defined = Color([0.7,0.7,0.7,1],[work_plane])
work_plane_defined = Collection([work_plane_defined, work_plane_coords])
work_plane_defined = HalShow([work_plane_defined],c,0,"twp_active")
# for twp-active = true, we show the plane in pink
work_plane_active = Color([1,0,1,1],[work_plane])
work_plane_active = Collection([work_plane_active, work_plane_coords])
work_plane_active = HalShow([work_plane_active],c,1,"twp_active")

work_plane = Collection([work_plane_defined, work_plane_active])
# make the work_plane hidable
work_plane = HalShow([work_plane],c,1,"twp_defined")
# move plane by home offset
work_plane = Translate([work_plane], machine_zero_x, machine_zero_y, machine_zero_z)
# move plane to current work offset
work_plane = HalVectorTranslate([work_plane],c,"twp_ox_world","twp_oy_world","twp_oz_world")

rotary_table_c = Collection([
                 work_piece,
                 rotary_table_c,
                 slot_x_w,
                 slot_y_w,
                 slot_pocket,
                 work_coords_tcp,
                 work_plane,
                 work
                 ])
# create HAL-link for rotary table
rotary_table_c = HalRotate([rotary_table_c],c,"rotary_c",-1,0,0,1)
# rotate the table against the table angle
rotary_table_c  = HalRotate([rotary_table_c ],c,"table_angle",1,0,0,1)

# create an indicator for x-offset
ind_offset_x =  Color([1,0,0,1],[HalOffsetX(c)])
# create an indicator for z-offset
ind_offset_z =  Color([0,0,1,1],[HalOffsetZ(c)])
ind_offset_z = HalVectorTranslate([ind_offset_z],c,"offset_x",0,"offset_z",-1)
# create an indicator for the rotation point of the table rotary assembly
ind_rot_point = Collection([
                  Color([1,1,1,1],[CylinderX(-200,1,200,1)]),
                  Color([1,1,1,1],[CylinderY(-200,1,200,1)]),
                  # arrow indicating the up position
                  Color([1,1,1,1],[CylinderY(200,5,210,1)]),
                  # sphere for the actual pivot point
                  Color([1,1,0.3,1],[Sphere(0,0,0,5)])
                  ])
# create nutation_joint_b
rotary_b_disc = Collection([
                 Color([1,0.5,0,0],[CylinderZ(-1000,700,-1050,800)]),
                 Color([1,0.5,0,0],[CylinderZ(-1050,800,-1300,800)])
                 ])
rotary_b_arm = Color([1,0.5,0,0],[Box(-500, -350, -1300, 500, 0, -500)])
rotary_b_arm = Rotate([rotary_b_arm],25,1,0,0)
rotary_b_arm = Translate([rotary_b_arm],0,300,0)
rotary_b_arm = HalTranslate([rotary_b_arm],c,"offset_z",0,-1,0)
rotary_b_arm = HalTranslate([rotary_b_arm],c,"nutation_angle",0,-10,1)
rotary_b = Collection([
           rotary_b_disc,
           rotary_b_arm
           ])

# create an indicator for the rotation axis of rotary b
rot_axis_b = Color([1,1,0.3,1],[CylinderZ(-2000,2,500,2)])
rotary_b = Collection([
           rotary_b,
           rot_axis_b])
# rotate the nutation joint to the nutation angle
rotary_b = HalRotate([rotary_b],c,"nutation_angle",1,1,0,0)
# move the joint using the offset values as set by the user in the gui panel
rotary_table_c_assembly = Collection([
                          rotary_table_c,
                          rotary_c_attachment])
rotary_table_c_assembly = HalVectorTranslate([rotary_table_c_assembly],c,"offset_x",0,"offset_z",-1)
rotary_table_bc = Collection([
                  rotary_b,
                  rotary_table_c_assembly,
                  ind_rot_point,
                  ind_offset_x,
                  ind_offset_z
                  ])
# create HAL-link for rotary b
rotary_table_bc = HalRotateNutation([rotary_table_bc],c,"rotary_b",1,1,0,0)
rotary_b_attachment = Color([0.2,0.2,0.2,1],[CylinderZ(-1300,800,-2200,1000)])

# rotate the nutation joint attachment to the nutation angle
rotary_b_attachment = HalRotate([rotary_b_attachment],c,"nutation_angle",1,1,0,0)


# move the rotary_table_c so the nutation joint is in the lower back
rotary_table_assembly = Collection([
                        rotary_table_bc,
                        rotary_b_attachment,
                        ])
# rotate the nutation joint attachment to the nutation angle
rotary_b_attachment = HalRotate([rotary_b_attachment],c,"nutation_angle",1,1,0,0)
# rotate the nutation joint attachment to the table angle
rotary_table_assembly = HalRotate([rotary_table_assembly],c,"table_angle",-1,0,0,1)
# nutation table back
nut_table_back = Color([0.2,0.2,0.2,1],[Box(-1000, 1500, -1500, 2000, 2500, 1000)])
# rotate the nutation table back to the table angle
nut_table_back = HalRotate([nut_table_back],c,"table_angle",-1,0,0,1)

table = Collection([
        rotary_table_assembly,
        rot_point,
        coords,
        nut_table_back
        ])

#/work-side

#base
base = Collection([
       # base
       Box(-3500, -1000, -2000, 3500, 3000, -1500),
       # column back
       Box(-2000, 1500, -1500, 2000, 4000, 3500)
       ])
base = Color([0.2,0.2,0.2,1], [base] )
#/base

model = Collection([table, spindle_xyz, base,])

#hud
myhud = Hud()
myhud.show("XYZBC-tnr")
myhud.show("----------------")
myhud.show("TWP-Status:")
myhud.add_txt("Undefined",0)
myhud.add_txt("Defined",1)
myhud.add_txt("Table oriented",2)
myhud.add_txt("",[1,2,3])
myhud.add_txt("TWP-Orientation Vector X:",[1,2,3])
myhud.add_pin("Xx: {:8.3f}","xyzbc-tnr-gui.twp_xx",[1,2,3])
myhud.add_pin("Xy: {:8.3f}","xyzbc-tnr-gui.twp_xy",[1,2,3])
myhud.add_pin("Xz: {:8.3f}","xyzbc-tnr-gui.twp_xz",[1,2,3])
myhud.add_txt("",[1,2,3])
myhud.add_txt("TWP-Orientation Vector Z:",[1,2,3])
myhud.add_pin("Zx: {:8.3f}","xyzbc-tnr-gui.twp_zx",[1,2,3])
myhud.add_pin("Zy: {:8.3f}","xyzbc-tnr-gui.twp_zy",[1,2,3])
myhud.add_pin("Zz: {:8.3f}","xyzbc-tnr-gui.twp_zz",[1,2,3])
myhud.add_txt("",[1,2,3])
myhud.show_tags_in_pin("xyzbc-tnr-gui.twp_status")
#/hud

main(model, tooltip, work, size=4000, hud=myhud, lat=-60, lon=25)
