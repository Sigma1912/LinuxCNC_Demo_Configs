#!/usr/bin/env python3

from twp_vismach import *
import hal
import math
import sys

# Machine zero as measured from center surface of the rotary c table
machine_zero_x = -1000
machine_zero_y =  1000
machine_zero_z =  1000


try: # Expect files in working directory
    # NOTE to run this file as standalone python script absolute paths might have to be used to find the stl files
    # create the work piece from file
    work_piece = AsciiSTL(filename="./work_piece_1.stl")

except Exception as detail:
    print(detail)
    raise SystemExit("xyzbc-sntr-gui requires stl files in working directory")

for setting in sys.argv[1:]: exec(setting)

c = hal.component("xyzbc-sntr-gui")
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
# tool offsets
c.newpin("tool_length", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_diameter", hal.HAL_FLOAT, hal.HAL_IN)
# geometric offsets in the spindle-rotary-assembly
c.newpin("pivot_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("pivot_z", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("offset_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("offset_z", hal.HAL_FLOAT, hal.HAL_IN)
# rot-point offsets distances from pivot point ( spindle AB)
# to rotation axis ( table C)
c.newpin("rot_axis_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("rot_axis_y", hal.HAL_FLOAT, hal.HAL_IN)
# active work offset values (ie g54,g55...)
c.newpin("work_offset_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("work_offset_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("work_offset_z", hal.HAL_FLOAT, hal.HAL_IN)
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
# twp plane rotation, This is only used to show euler rotations of the plane in vismach
# Currently not used anymore
c.newpin("rot_order", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("rot_th1", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("rot_th2", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("rot_th3", hal.HAL_FLOAT, hal.HAL_IN)
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

# used to create an indicator for the rotational axis a
# this will be rotated 45° later
class HalRotAxisA(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        # calculate length from the pivot-point to the tool-axis
        # note: sin(45°) = 1/sqrt(2)
        length = 0
        return 0, r, length, r

# used to create the spindle housing as set by the variable 'pivot_z'
class HalSpindleHousingZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 140
        length = c.pivot_z + 200
        # start the spindle housing at hight 100 above the spindle nose
        return length, r, 100, r

# used to create the spindle rotary as set by the variable 'pivot_y'
class HalSpindleHousingY(CylinderY):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 140
        length = c.pivot_y
        return length, r, 0, r

# used to create an indicator for the variable 'pivot_y'
class HalPivotY(CylinderY):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.pivot_y
        return length, r, 0, 1

# used to create an indicator for the variable 'pivot_z'
class HalPivotZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = c.pivot_z
        # start the spindle housing at hight 100 above the spindle nose
        return length, 1, 0, r


# used to create an indicator for the variable 'offset_x'
class HalOffsetX(CylinderX):
    def __init__(self, comp, *args):
        CylinderX.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = c.offset_x
        return length, 1, 0, r

# used to create an indicator for the variable 'offset_z'
class HalOffsetZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.offset_z
        return length, r, 0, 1

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
        y = th*sin(radians(c.nutation_angle))
        z = th*cos(radians(c.nutation_angle))

        return th, x, y, z

#indicators
# create vismach coordinates to show the origin during construction of the model
vismach_coords = CoordSystem(c,2,5000)

# create absolute coordinates
abs_coords = CoordSystem(c,8,300)
abs_coords = HalScale([abs_coords],c,1,1,1,"scale_coords")
# create work coordinates that represent work_offset offset
work_coords = CoordSystem(c,2,100)
work_coords = HalScale([work_coords],c,1,1,1,"scale_coords")
# rotate to match the current tool orientation
work_coords = HalRotateNutation([work_coords],c,"rotary_b",1,0,0,0)

# move the  work offset by the work_offset
work_coords = HalVectorTranslate([work_coords],c,"twp_ox_world","twp_oy_world","twp_oz_world")
# create a visual indicator for the position of the control point
ctrl_pt = Point(c)
# move the control point indicator to the control-position
#ctrl_pt = HalVectorTranslate([ctrl_pt],c,"abs_pos_x","abs_pos_y","abs_pos_z")
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_length",0,0,-1)
# create an indicator line from the reference point to the work_offset position
line_work_offset =  Color([1,1,1,1],[HalLine(c,0,0,0,"twp_ox_world","twp_oy_world","twp_oz_world",1,1)])
coords = Collection([
              abs_coords,
              work_coords,
              ctrl_pt,
              line_work_offset,
              ])
# move the above indicators to by the designated (x,y,z)-home offset
coords = Translate([coords], machine_zero_x, machine_zero_y,  machine_zero_z)
# indicators to be displayed when in identity mode
ident_coords = HalShow([coords],c,0,"kinstype_select")
# indicators to be displayed when in tcp mode
tcp_coords = HalShow([coords],c,1,"kinstype_select")

# move the tool_ctrl_pt to the designated (x,y,z)-home position
tool_coords = Translate([coords], -machine_zero_x, -machine_zero_y, -machine_zero_z)
# add a visual indicator for the position of the tool-rotation point
tool_coords = Collection([
              tool_coords,
              Point(c,1,30,[1,0.8,0,1])
              ])

# move the tool_ctrl_pt to the designated (x,y,z)-home position
tool_coords = Translate([tool_coords], machine_zero_x, machine_zero_y, machine_zero_z)
# indicators to be displayed when in tool mode
# note: we hide the tool-rotation point when identity or tcp mode (ie show it in tool mode)
tool_coords = HalShow([tool_coords],c,0,"kinstype_select",0,1)
tool_coords = HalShow([tool_coords],c,1,"kinstype_select",0,1)

# create a visual for the position of the rotational axis of the rotary C table
# as set by the user with the "rot-axis-x" and "rot-axis-y" sliders in the axis gui
rot_axis = Color([1,1,0.3,1],[CylinderZ(0,2,2000,2)])
# move the rot_axis indicator by the designated (x,y)-home offset
rot_axis= Translate([rot_axis], machine_zero_x, machine_zero_y, 0)
# now move it to the position set by the user
rot_axis = HalVectorTranslate([rot_axis],c,"rot_axis_x","rot_axis_y",0)

# create an indicator for x-offset
ind_offset_x =  Color([1,0,0,1],[HalOffsetX(c)])
# move the offset_x indicator so it extends from the pivot point
ind_offset_x = HalVectorTranslate([ind_offset_x],c,0,"pivot_y","pivot_z",1)
# create an indicator for z-offset
ind_offset_z =  Color([0,1,1,1],[HalOffsetZ(c)])
ind_offset_z = HalTranslate([ind_offset_z],c,"offset_x", -1,0,0)
# create an indicator for the variable pivot_z
ind_pivot_z = Color([0,0,0.75,1],[HalPivotZ(CylinderZ)])
# create the spindle housing that contains the spindle
# create an indicator for the variable pivot_y
ind_pivot_y = Color([0,0.75,0,1],[HalPivotY(CylinderY)])
# move the pivot_y indicator so it extends from the pivot point
ind_pivot_y = HalVectorTranslate([ind_pivot_y],c,0,"pivot_y","pivot_z",1)
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
       ind_tool_offset
       ])
tool = Color([1,0,1,0], [tool] )
spindle_housing = Collection([
                  # spindle
                  Color([0.7,0.7,0,1],[CylinderZ(0,50,50,100)]),
                  Color([0.7,0.7,0,1],[CylinderZ(50,100,100,100)]),
                  # spindle housing vertical length as set by 'pivot_z'
                  Color([0.7,0.7,0,1],[HalSpindleHousingZ(CylinderZ)])
                  ])
spindle_housing = HalShow([spindle_housing],c,True,"hide_spindle_body",0,1)
spindle_housing = Collection([
                  tool,
                  spindle_housing,
                  ind_pivot_z,
                  ind_offset_x,
                  ind_offset_z,
                  ])

spindle_housing_horizontal = Color([0.7,0.7,0,1],[HalSpindleHousingY(c)])
# move farther up by the pivot_z value
spindle_housing_horizontal = HalVectorTranslate([spindle_housing_horizontal],c,0,0,"pivot_z",1)
# make it hidable
spindle_housing_horizontal = HalShow([spindle_housing_horizontal],c,True,"hide_spindle_body",0,1)

spindle_housing = Collection([
                  spindle_housing,
                  spindle_housing_horizontal,
                  ind_pivot_y
                  ])
# move the spindle and it's vertical housing by the values set for the pivot lengths
# so the vismach origin is in the pivot point
spindle_housing = HalVectorTranslate([spindle_housing],c,0,"pivot_y","pivot_z",-1)

# create a visual for the position of the spindle A,B pivot-point
ind_pivot_point = Collection([
                  Color([1,1,1,1],[CylinderX(-200,1,200,1)]),
                  Color([1,1,0.3,1],[CylinderY(0,2,3000,2)]),
                  Color([1,1,1,1],[CylinderZ(-200,1,200,1)]),
                  # arrow indicating the up position
                  Color([1,1,1,1],[CylinderZ(200,5,210,1)]),
                  # sphere for the actual pivot point
                  Color([1,1,0.3,1],[Sphere(0,0,0,5)])
                  ])
# rotate the indicator for the pivot point to the nutation angle, 90° should have the nutation axis in the horizontal plane
ind_pivot_point = Rotate([ind_pivot_point],-90,-1,0,0)
ind_pivot_point = HalRotate([ind_pivot_point],c,"nutation_angle",-1,1,0,0)
# create HAL-link for b-axis rotational joint"
ind_pivot_point = HalRotateNutation([ind_pivot_point],c,"rotary_b",1,0,-1,0)

# create nutation joint  for the spindle housing
spindle_housing_nut = Collection([
                      Color([0.7,0.7,0,1],[CylinderY(0,145,-70,145)]),
                      Color([0.7,0.7,0,1],[CylinderY(-70,145,-150,60)])
                      ])
# rotate the nutation joint to the nutation angle, 90° should have the nutation axis in the horizontal plane
spindle_housing_nut = Rotate([spindle_housing_nut],-90,-1,0,0)
spindle_housing_nut = HalRotate([spindle_housing_nut],c,"nutation_angle",-1,1,0,0)
spindle_housing_nut = HalShow([spindle_housing_nut],c,True,"hide_spindle_body",0,1)

# move the joint using the offset values as set by the user in the gui panel
spindle_housing = HalVectorTranslate([spindle_housing],c,"offset_x",0,"offset_z",-1)
spindle_housing = Collection([
                  spindle_housing,
                  spindle_housing_nut
                  ])
# create HAL-link for b-axis rotational joint"
spindle_housing = HalRotateNutation([spindle_housing],c,"rotary_b",1,0,-1,0)

# create middle part b rotary and a nutating axis
spindle_rotary = Collection([
                 # cylinder for b axis
                 CylinderY(0,120,100,120),
                 CylinderY(100,120,300,160),
                 ])
# create nutation joint for the b-rotary part
spindle_rotary_nut = Collection([
                     CylinderY(0,145,70,145),
                     CylinderY(70,145,150,60)
                     ])
# rotate the nutation joint to the nutation angle, 90° should have the nutation axis in the horizontal plane
spindle_rotary_nut = Rotate([spindle_rotary_nut],-90,-1,0,0)
spindle_rotary_nut = HalRotate([spindle_rotary_nut],c,"nutation_angle",-1,1,0,0)
spindle_rotary = Collection([spindle_rotary,
                             spindle_rotary_nut
                            ])
spindle_rotary = Color([1,0.5,0,0], [spindle_rotary] )
spindle_rotary = HalShow([spindle_rotary],c,True,"hide_spindle_body",0,1)


# rotate the nutation joint to the nutation angle, 90° should have the nutation axis in the horizontal plane
spindle_rotary_nut = Rotate([spindle_rotary_nut],-90,-1,0,0)
spindle_rotary_nut = HalRotate([spindle_rotary_nut],c,"nutation_angle",-1,1,0,0)
spindle_rotary = Collection([
                 spindle_rotary,

                 ind_pivot_point
                 ])
# join the two parts to a spindle assembly
spindle_assembly = Collection([
                   spindle_housing,
                   spindle_rotary,
                   ])
## create HAL-link for b-axis rotational joint"
#spindle_assembly = HalRotate([spindle_assembly],c,"rotary_b",1,0,1,0)
# add a block for the y-axis to the spindle_assembly
slide_yz = Color([0.6,0.8,0.3,0], [Box(-250, 200, -250, 250, 3000, 250)])
slide_yz = HalShow([slide_yz],c,True,"hide_spindle_body",0,1)
spindle_yz = Collection([
             spindle_assembly,
             slide_yz
             ])
# move the spindle and it's vertical housing by the values set for the pivot lengths
# so the vismach origin is in the center of the spindle nose
spindle_yz = HalVectorTranslate([spindle_yz],c,0,"pivot_y","pivot_z")
spindle_yz = HalVectorTranslate([spindle_yz],c,"offset_x",0,"offset_z")
# spindle head and y-slide move with y
spindle_yz = HalTranslate([spindle_yz],c,"axis_y",0,1,0)
# spindle head and y-slide move with z
spindle_yz = HalTranslate([spindle_yz],c,"axis_z",0,0,1)
# move the spindle_yz to it's designated home position
spindle_yz = Translate([spindle_yz], machine_zero_x, machine_zero_y, machine_zero_z)
#/tool-side

#work-side
work = Capture()
# create a simple cube
work_piece = BoxCenteredXY(600, 600, 600)
# create a more complex work piece from stl
work_piece = Translate([work_piece],0,0,0)
work_piece = Color([0.5,0.5,0.5,0.9], [work_piece])
work_piece = HalShow([work_piece],c,True,"hide_work_piece_1",0,1)

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

# create rotary_table_c
rotary_table_c = Color([0.1,0.7,0.9,0],[CylinderZ(0,920,-800,920)])
# create 'slots' in x and y on the rotary table
slot_x_w = Color([1,1,1,1],[BoxCenteredXY(1840, 10, 0.9)])
slot_y_w = Color([1,1,1,1],[BoxCenteredXY(10, 1840, 0.9)])
# create indicator of at positive end of x slot
slot_pocket = Color([1,1,1,1],[CylinderZ(0,20,1,20)])
slot_pocket = Translate([slot_pocket],900,0,0)
rotary_table_c = Collection([
                 work,
                 work_piece,
                 rotary_table_c,
                 slot_x_w,
                 slot_y_w,
                 slot_pocket,
                 tcp_coords,
                 work_plane
                 ])
# create HAL-link for c-axis rotational joint"
rotary_table_c = HalRotate([rotary_table_c],c,"rotary_c",1,0,0,-1)


table = Collection([
        rotary_table_c,
        rot_axis,
        ident_coords,
        tool_coords,
        # block that carries the rotary c
        Color([0.5,0.5,0.5,0], [Box(-1500, -1000, -1300, 1500, 1000, -800)])
        ])
table = HalTranslate([table],c,"axis_x",-1,0,0)
#/work-side

#base
base = Collection([
       # base
       Box(-3500, -1000, -1500, 3500, 3000, -1300),
       # column back
        Box(-2000, 1800, -1500, 2000, 4000, 3500)
       ])
base = Color([0.2,0.2,0.2,1], [base] )
#/base

model = Collection([table, spindle_yz, base,])

#hud
myhud = Hud()
myhud.show("XYZBC-sntr")
myhud.show("------------")
myhud.show("Kinematic Mode:")
myhud.add_txt("IDENTITY",[0,3])
myhud.add_txt("TCP",1)
myhud.add_txt("TOOL",2)
myhud.add_txt("")
myhud.add_txt("TWP-Orientation Vector X:")
myhud.add_pin("Xx: {:8.3f}","xyzbc-sntr-gui.twp_xx")
myhud.add_pin("Xy: {:8.3f}","xyzbc-sntr-gui.twp_xy")
myhud.add_pin("Xz: {:8.3f}","xyzbc-sntr-gui.twp_xz")
myhud.add_txt("")
myhud.add_txt("TWP-Orientation Vector Z:")
myhud.add_pin("Zx: {:8.3f}","xyzbc-sntr-gui.twp_zx")
myhud.add_pin("Zy: {:8.3f}","xyzbc-sntr-gui.twp_zy")
myhud.add_pin("Zz: {:8.3f}","xyzbc-sntr-gui.twp_zz")
myhud.add_txt("")
myhud.show_tags_in_pin("motion.switchkins-type")
#/hud

main(model, tooltip, work, size=4000, hud=myhud, lat=-60, lon=25)
