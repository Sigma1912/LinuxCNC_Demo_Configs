#!/usr/bin/env python3

from twp_vismach import *
import hal
import math
import sys

# Machine zero, equal to the absolute position of the center face of the work spindle
machine_zero_x = 0 #-1000
machine_zero_y = 0 #300
machine_zero_z = 0 #-900


try: # Expect files in working directory
    # NOTE to run this file as standalone python script absolute paths might have to be used to find the stl files
    # create the work piece from file
    work_piece = AsciiSTL(filename="./vismach/200y-stl-files/example_part.stl")
    tool_holder = AsciiSTL(filename="./vismach/200y-stl-files/HSK_turning_63T_square_25x2.stl")
    tool_turning = AsciiSTL(filename="./vismach/200y-stl-files/tool_svjn-r20x20x120.stl")
    #pass

except Exception as detail:
    print(detail)
    raise SystemExit("mazak-integrex-200y-gui requires stl files in working directory")

for setting in sys.argv[1:]: exec(setting)

c = hal.component("mazak-integrex-200y-gui")
# axis_x
c.newpin("axis_x", hal.HAL_FLOAT, hal.HAL_IN)
# axis_y
c.newpin("axis_y", hal.HAL_FLOAT, hal.HAL_IN)
# head vertical slide
c.newpin("axis_z", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_a
c.newpin("tool_spindle_angle", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_b
c.newpin("rotary_b", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_c
c.newpin("rotary_c", hal.HAL_FLOAT, hal.HAL_IN)
# tool offsets
c.newpin("tool_length", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_diameter", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_number", hal.HAL_U32, hal.HAL_IN)
c.newpin("tool_x_offset", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_y_offset", hal.HAL_FLOAT, hal.HAL_IN)
# geometric offsets in the spindle-rotary-assembly
c.newpin("pivot_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("angle_y_yt", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("offset_z", hal.HAL_FLOAT, hal.HAL_IN)
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
# workaround for 'HalRotate' operations used to add rotations to the 'tooltip_offset' for lathe modes
c.newpin("tooltip_rotate_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tooltip_rotate_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tooltip_rotate_z", hal.HAL_FLOAT, hal.HAL_IN)
c.ready()


class HalToolTriangle(TriangleXZ):
    def __init__(self, comp, *args):
        # get machine access so it can
        # change itself as it runs
        # specifically tool cylinder in this case.
        TriangleXZ.__init__(self, *args)
        self.comp = c
    def coords(self):
        leng = c.tool_length
        x1 = leng
        z1 = -self.comp["tool_x_offset"]
        x2 = 1#self.comp["tool-x-offset"]+10
        z2 = -leng/2
        x3 = 2#self.comp["tooldiameter"]
        z3 = 0
        return (x1, z1, x2, z2, x3, z3, 0, 1)

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

# used to create the spindle housing as set by the variable 'pivot_x'
class HalSpindleHousingZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 140
        length = c.pivot_x + 200
        # start the spindle housing at hight 100 above the spindle nose
        return length, r, 100, r

# used to create the spindle rotary as set by the variable 'pivot_y'
class HalSpindleHousingY(CylinderY):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 140
        length = 0
        return length, r, 0, r

# used to create an indicator for the variable 'pivot_x'
class HalPivotZ(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = c.pivot_x
        # start the spindle housing at hight 100 above the spindle nose
        return length, 1, 0, r


# used to create an indicator for the variable 'offset_z'
class HalOffsetZ(CylinderX):
    def __init__(self, comp, *args):
        CylinderX.__init__(self, *args)
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
    def __init__(self, comp, r=0.4, l=100, color=[1,1,1,1]):
        self.parts = [Color(color,[CylinderX(-l,r,l,r)],1),
                      Color(color,[CylinderY(-l,r,l,r)],1),
                      Color(color,[CylinderZ(-l,r,l,r)],1)
                     ]

#indicators
# create vismach coordinates to show the origin during construction of the model
vismach_coords = CoordSystem(c,2,5000)

# create absolute coordinates
abs_coords = CoordSystem(c,8,300)
abs_coords = HalScale([abs_coords],c,1,1,1,"scale_coords")
# move the above indicators to by the designated (x,y,z)-home offset
abs_coords = Translate([abs_coords], -machine_zero_x, -machine_zero_y, -machine_zero_z)
abs_coords = Rotate([abs_coords],90,0,0,1)
abs_coords = HalRotate([abs_coords],c,"angle_y_yt",1,0,0,1)

# create work coordinates that represent work_offset offset
work_coords = CoordSystem(c,2,100)
work_coords = HalScale([work_coords],c,1,1,1,"scale_coords")
work_coords = Translate([work_coords], -machine_zero_x, -machine_zero_y, -machine_zero_z)
# move to current work offset
work_coords = HalTranslate([work_coords],c,"twp_ox_world",1,0,0)
work_coords = HalTranslate([work_coords],c,"twp_oy_world",0,1,0)
work_coords = HalTranslate([work_coords],c,"twp_oz_world",0,0,1)
work_coords = Rotate([work_coords],90,0,0,1)
work_coords = HalRotate([work_coords],c,"angle_y_yt",1,0,0,1)


# create an indicator line from the reference point to the work_offset position
line_work_offset =  Color([1,1,1,1],[HalLine(c,0,0,0,"twp_ox_world","twp_oy_world","twp_oz_world",-1,1)])
line_work_offset = Translate([line_work_offset], -machine_zero_x, -machine_zero_y, -machine_zero_z)
# move to current work offset
line_work_offset = HalTranslate([line_work_offset],c,"twp_ox_world",1,0,0)
line_work_offset = HalTranslate([line_work_offset],c,"twp_oy_world",0,1,0)
line_work_offset = HalTranslate([line_work_offset],c,"twp_oz_world",0,0,1)
line_work_offset = Rotate([line_work_offset],90,0,0,1)
line_work_offset = HalRotate([line_work_offset],c,"angle_y_yt",1,0,0,1)

coords = Collection([
              abs_coords,
              work_coords,
              line_work_offset,
              ])
# indicators to be displayed when in identity and HALF TCP modes
ident_coords = coords
ident_coords = HalShow([ident_coords],c,4,"kinstype_select",0,1)

# indicators to be displayed when in FULL TCP mode
mill_tcp_coords = HalRotate([coords],c,"rotary_c",-1,0,0,1)
mill_tcp_coords = HalShow([mill_tcp_coords],c,0,"kinstype_select",0,1)
mill_tcp_coords = HalShow([mill_tcp_coords],c,1,"kinstype_select",0,1)
mill_tcp_coords = HalShow([mill_tcp_coords],c,2,"kinstype_select",0,1)
mill_tcp_coords = HalShow([mill_tcp_coords],c,3,"kinstype_select",0,1)


# indicators to be displayed when in twp mode
# create absolute coordinates that rotate with the b-axis
twp_abs_coords = CoordSystem(c,8,300)
twp_abs_coords = HalScale([twp_abs_coords],c,1,1,1,"scale_coords")
#twp_abs_coords = Rotate([twp_abs_coords],90,0,1,0)
#twp_abs_coords = HalRotate([twp_abs_coords],c,"rotary_b",1,0,1,0)
# move the above indicators to by the designated (x,y,z)-home offset
twp_abs_coords = Translate([twp_abs_coords], -machine_zero_x, -machine_zero_y, -machine_zero_z)
twp_abs_coords = Rotate([twp_abs_coords],90,0,0,1)
twp_abs_coords = HalRotate([twp_abs_coords],c,"angle_y_yt",1,0,0,1)

# create work coordinates that rotate with the b-axis
twp_work_coords = CoordSystem(c,2,100)
twp_work_coords = HalScale([twp_work_coords],c,1,1,1,"scale_coords")
twp_work_coords = Rotate([twp_work_coords],90,0,1,0)
twp_work_coords = HalRotate([twp_work_coords],c,"rotary_b",1,0,1,0)
twp_work_coords = Translate([twp_work_coords], -machine_zero_x, -machine_zero_y, -machine_zero_z)
# move to current work offset
twp_work_coords = HalTranslate([twp_work_coords],c,"twp_ox_world",1,0,0)
twp_work_coords = HalTranslate([twp_work_coords],c,"twp_oy_world",0,1,0)
twp_work_coords = HalTranslate([twp_work_coords],c,"twp_oz_world",0,0,1)
twp_work_coords = Rotate([twp_work_coords],90,0,0,1)
twp_work_coords = HalRotate([twp_work_coords],c,"angle_y_yt",1,0,0,1)


# add a visual indicator for coordinate system in TWP mode
twp_coords = Collection([
              twp_abs_coords,
              twp_work_coords,
              line_work_offset,
              ])
# indicators to be displayed when in tool mode
# note: we hide the tool-rotation point when identity or tcp mode (ie show it in tool mode)
twp_coords = HalShow([twp_coords],c,3,"kinstype_select")
twp_coords = HalShow([twp_coords],c,1,"twp_active")
#/indicators

#tool-side
tool_holder = Rotate([tool_holder],180,1,0,0)
tool_holder = Rotate([tool_holder],90,0,0,-1)
tool_turning = Rotate([tool_turning],90,0,-1,0)
tool_turning = Rotate([tool_turning],90,0,0,1)
tool_turning = Translate([tool_turning],30,25,-170)

tool_101 = Collection([
                tool_holder,
                tool_turning,
                ])
tool_101 = HalShow([tool_101],c,True,"hide_spindle_body",0,1)
tool_101 = HalShow([tool_101],c,101,"tool_number",1,0)


# create a visual indicator for the position of the control point (ie indicate the position in the DRO)
ctrl_pt = Point(c)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_x_offset",-1,0,0)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_y_offset",0,-1,0)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_length",0,0,-1)
ctrl_pt = Rotate([ctrl_pt],90,0,1,0)

# control point to be displayed when in identity mode
#ident_ctrl_pt = HalRotate([ctrl_pt],c,"tool_spindle_angle",1,0,0,1)
ident_ctrl_pt = Rotate([ctrl_pt],180,1,0,0)
ident_ctrl_pt = HalVectorTranslate([ident_ctrl_pt],c,0,0,"pivot_x",-1)
ident_ctrl_pt  = HalTranslate([ident_ctrl_pt ],c,"offset_z",-1,0,0)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,1,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,2,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,3,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,4,"kinstype_select",0,1)

# control_point to be displayed when in HALF TCP LATHE mode
lathe_tcp_ctrl_pt = HalShow([ctrl_pt],c,0,"kinstype_select",0,1)
lathe_tcp_ctrl_pt = HalShow([lathe_tcp_ctrl_pt],c,2,"kinstype_select",0,1)
lathe_tcp_ctrl_pt = HalShow([lathe_tcp_ctrl_pt],c,3,"kinstype_select",0,1)
lathe_tcp_ctrl_pt = HalShow([lathe_tcp_ctrl_pt],c,4,"kinstype_select",0,1)

# control_point to be displayed when in FULL/HALF TCP MILL and TWP modes
mill_tcp_ctrl_pt = Rotate([ctrl_pt],90,0,1,0)
mill_tcp_ctrl_pt = HalShow([mill_tcp_ctrl_pt],c,0,"kinstype_select",0,1)
mill_tcp_ctrl_pt = HalShow([mill_tcp_ctrl_pt],c,1,"kinstype_select",0,1)


tooltip = Capture()
tooltip_offset = HalTranslate([tooltip],c,"tool_x_offset",-1, 0,0)
tooltip_offset = HalTranslate([tooltip_offset],c,"tool_y_offset", 0,-1,0)
tooltip_offset = HalTranslate([tooltip_offset],c,"tool_length", 0,0,-1)
#the above works for mill modes tor lathe modes we have to rotate
#tooltip_offset = Rotate([tooltip_offset],90,0,1,0)
#tooltip_offset = Rotate([tooltip_offset],180,1,0,0) # for lathe modes
#using hal values to do the switch works, handled on the twp-handler comp
tooltip_offset = HalRotate([tooltip_offset],c,"tooltip_rotate_z",1,0,0,1)
tooltip_offset = HalRotate([tooltip_offset],c,"tooltip_rotate_y",1,0,1,0)
tooltip_offset = HalRotate([tooltip_offset],c,"tooltip_rotate_x",1,1,0,0)


# create a simple cylinder for millin tools
tool_cylinder = Collection([
                HalToolCylinder(c),
                # this indicates the spindle nose when no tool-offset is active
                CylinderZ(-0.1, 0, 0.1, 10),
                ])
tool_cylinder = HalShow([tool_cylinder],c,True,"hide_spindle_body",0,1)
# hide this for lathe tools
tool_cylinder = HalShow([tool_cylinder],c,101,"tool_number",0,1)

# create an indicator for the tool-offset when spindle-body is hidden
ind_tool_offset =  Color([1,0,1,0],[HalLine(c,0,0,0,"tool_x_offset","tool_y_offset","tool_length",-1,1)])
# for lathe modes
lathe_ind_tool_offset =  Rotate([ind_tool_offset],90,0,-1,0)
lathe_ind_tool_offset = HalShow([lathe_ind_tool_offset],c,2,"kinstype_select",0,1)
lathe_ind_tool_offset = HalShow([lathe_ind_tool_offset],c,3,"kinstype_select",0,1)
lathe_ind_tool_offset = HalShow([lathe_ind_tool_offset],c,4,"kinstype_select",0,1)
#for mill modes
mill_ind_tool_offset =  Rotate([ind_tool_offset],180,0,0,-1)
mill_ind_tool_offset = HalShow([mill_ind_tool_offset],c,0,"kinstype_select",0,1)
mill_ind_tool_offset = HalShow([mill_ind_tool_offset],c,1,"kinstype_select",0,1)


tool = Collection([
       #Rotate([tooltip_offset],180,1,0,0), #for some reason we cannot just add 'tooltip_offset'
       Rotate([lathe_tcp_ctrl_pt],180,1,0,0),
       Rotate([mill_tcp_ctrl_pt],180,1,0,0),
       tool_101,
       tool_cylinder,
       Rotate([lathe_ind_tool_offset],180,0,0,1),
       Rotate([mill_ind_tool_offset],0,0,0,1),
       tooltip_offset
       ])
tool = Color([1,0,1,0], [tool] )
# create HAL-link for a-axis rotational joint (tool-spindle rotation)"
tool = HalRotate([tool],c,"tool_spindle_angle",1,0,0,1)
# create an indicator for the variable pivot_x
ind_pivot_x = Color([1,0,0,1],[HalPivotZ(CylinderZ)])
# create the spindle housing that contains the spindle
spindle_housing = Collection([
                  # spindle
                  Color([0.7,0.7,0,1],[CylinderZ(0,50,50,100)]),
                  Color([0.7,0.7,0,1],[CylinderZ(50,100,100,100)]),
                  # spindle housing vertical length as set by 'pivot_x'
                  Color([0.7,0.7,0,1],[HalSpindleHousingZ(CylinderZ)])
                  ])
spindle_housing = HalShow([spindle_housing],c,True,"hide_spindle_body",0,1)
spindle_housing = Collection([
                  tool,
                  spindle_housing,
                  ind_pivot_x
                  ])
# move the spindle and it's vertical housing by the values set for the pivot lengths
# so the vismach origin is in the pivot point
spindle_housing = HalVectorTranslate([spindle_housing],c,0,0,"pivot_x",-1)
spindle_housing_horizontal = Color([0.7,0.7,0,1],[HalSpindleHousingY(c)])
spindle_housing_horizontal = HalShow([spindle_housing_horizontal],c,True,"hide_spindle_body",0,1)

spindle_housing = Collection([
                  spindle_housing,
                  spindle_housing_horizontal
                  ])
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

# move the joint using the offset values as set by the user in the gui panel
spindle_housing  = HalTranslate([spindle_housing ],c,"offset_z",-1,0,0)

# create middle part b rotary and a nutating axis
spindle_rotary = Collection([
                 # cylinder for b axis
                 CylinderY(0,120,100,120),
                 CylinderY(100,120,300,160),
                 ])
spindle_rotary = Color([1,0.5,0,0], [spindle_rotary] )
spindle_rotary = HalShow([spindle_rotary],c,True,"hide_spindle_body",0,1)

# create an indicator for x-offset
ind_offset_z=  Color([0,0,0.75,1],[HalOffsetZ(c)])

# join the two parts to a spindle assembly
spindle_assembly = Collection([
                   spindle_rotary,
                   ind_offset_z,
                   ind_pivot_point,
                   spindle_housing,
                   spindle_rotary,
                   ])
# create HAL-link for b-axis rotational joint"
spindle_assembly = HalRotate([spindle_assembly],c,"rotary_b",-1,0,1,0)
# rotate the tool-spindle so it is pointing in the negative z-direction
#spindle_assembly= Rotate([spindle_assembly],90,0,1,0)
# add a block for the y-axis to the spindle_assembly
slide_x = Color([0.6,0.8,0.3,0], [Box(-250, 200, -250, 250, 300, 500)])
slide_x = HalShow([slide_x],c,True,"hide_spindle_body",0,1)
spindle_x = Collection([
             spindle_assembly,
             slide_x,
             ident_ctrl_pt
             ])
# move the spindle and it's vertical housing by the values set for the pivot lengths
# so the vismach origin is in the center of the spindle nose
#spindle_x = HalTranslate([spindle_x],c,"pivot_x",0,0,1)
#spindle_x = HalTranslate([spindle_x],c,"offset_z",1,0,0)
# spindle head and x-slide move with x
spindle_x = HalTranslate([spindle_x],c,"axis_x",0,0,1)
# move the spindle_yz to it's designated home position
spindle_x = Translate([spindle_x], -machine_zero_z, machine_zero_y, -machine_zero_x)
# rotate the x-slide and tool-spindle by the slide-angle
spindle_x = Rotate([spindle_x],90,-1,0,0)
spindle_x = HalRotate([spindle_x],c,"angle_y_yt",1,1,0,0)
# spindle head and y-slide move with y
spindle_xy = HalTranslate([spindle_x],c,"axis_y",0,-1,0)
# spindle head and y-slide move with z
spindle_xyz = HalTranslate([spindle_xy],c,"axis_z",1,0,0)

spindle_xyz = Translate([spindle_xyz],-800,0,0)
#/tool-side

#work-side
work = Capture()
# create a simple cube
#work_piece = BoxCenteredXY(600, 600, 600)
# create a more complex work piece from stl
work_piece = Translate([work_piece],0,0,0)
work_piece = Color([0.5,0.5,0.5,0.9], [work_piece])
work_piece = HalShow([work_piece],c,True,"hide_work_piece_1",0,10)


# create the work_plane using the origin (as measured from current work offset (world),
# normal vector vz(zx,zy,zz) and  x-orientation vector vx(xx,xy,xz)
# these are used to create a transformation matrix
# with y-orientation vector being the cross_product(vz, vx)
work_plane =  HalGridFromNormalAndDirection(c,
        "twp_oz", "twp_oy", "twp_ox",
        "twp_xx", "twp_xy", "twp_xz",
        "twp_zx", "twp_zy", "twp_zz",
        s_ox=-1
        )
work_plane = Rotate([work_plane],180,0,0,1)

# move plane by home offset
work_plane = Translate([work_plane],-machine_zero_z, machine_zero_y, -machine_zero_x)
# move plane to current work offset
work_plane = HalTranslate([work_plane],c,"twp_ox_world",0,0,1)
work_plane = HalTranslate([work_plane],c,"twp_oy_world",0,-1,0)
work_plane = HalTranslate([work_plane],c,"twp_oz_world",1,0,0)
work_plane = Rotate([work_plane],90,-1,0,0)
work_plane = HalRotate([work_plane],c,"angle_y_yt",1,1,0,0)
work_plane = Translate([work_plane],-800,0,0)
# create a coordinate system for the twp-plane
work_plane_coords =  HalCoordsFromNormalAndDirection(c,
        "twp_oz", "twp_oy", "twp_ox",
        "twp_xx", "twp_xy", "twp_xz",
        "twp_zx", "twp_zy", "twp_zz",
        10,
        r=3, s_ox=-1
        )
work_plane_coords = Rotate([work_plane_coords],180,0,0,1)

# move plane by home offset
work_plane_coords = Translate([work_plane_coords],-machine_zero_z, machine_zero_y, -machine_zero_x)
# move plane to current work offset
#work_plane = HalVectorTranslate([work_plane],c,"twp_ox_world","twp_oy_world","twp_oz_world")
work_plane_coords = HalTranslate([work_plane_coords],c,"twp_ox_world",0,0,1)
work_plane_coords = HalTranslate([work_plane_coords],c,"twp_oy_world",0,-1,0)
work_plane_coords = HalTranslate([work_plane_coords],c,"twp_oz_world",1,0,0)
work_plane_coords = Rotate([work_plane_coords],90,-1,0,0)
work_plane_coords = HalRotate([work_plane_coords],c,"angle_y_yt",1,1,0,0)
work_plane_coords = Translate([work_plane_coords],-800,0,0)

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
                 ])
rotary_table_c = Rotate([rotary_table_c],90,0,0,1)
rotary_table_c = HalRotate([rotary_table_c],c,"angle_y_yt",1,0,0,1)
# create HAL-link for c-axis rotational joint"
rotary_table_c = HalRotate([rotary_table_c],c,"rotary_c",-1,0,0,1)


table = Collection([
        rotary_table_c,
        ident_coords,
        twp_coords,
        mill_tcp_coords,
        # block that carries the rotary c
        Color([0.5,0.5,0.5,0], [Box(-1500, -1000, -1300, 1500, 1000, -800)])
        ])
# rotate so it alignes with the z-axis
table = Rotate([table],90,0,1,0)
table = Translate([table],-800,0,0)
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

model = Collection([table, spindle_xyz, work_plane, base,])

main(model, tooltip, work, size=4000, lat=-60, lon=-35)
