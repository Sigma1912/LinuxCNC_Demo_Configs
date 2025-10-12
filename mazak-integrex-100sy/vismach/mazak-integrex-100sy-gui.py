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

    bed = AsciiSTL(filename="./vismach/100sy-stl-files/BED.STL")
    milling_spindle = AsciiSTL(filename="./vismach/100sy-stl-files/Milling_Spindle.STL")
    main_spindle_flange = AsciiSTL(filename="./vismach/100sy-stl-files/Main_Spindle.STL")
    sub_spindle = AsciiSTL(filename="./vismach/100sy-stl-files/Sub_Spindle.STL")
    counter_spindle = AsciiSTL(filename="./vismach/100sy-stl-files/W.STL")
    front_x1 = AsciiSTL(filename="./vismach/100sy-stl-files/Front_X1.STL")
    head_x1 = AsciiSTL(filename="./vismach/100sy-stl-files/Head_X1.STL")
    head_x2 = AsciiSTL(filename="./vismach/100sy-stl-files/Head_X2.STL")
    head_y1 = AsciiSTL(filename="./vismach/100sy-stl-files/Head_Y1.STL")
    head_y2 = AsciiSTL(filename="./vismach/100sy-stl-files/Head_Y2.STL")
    chuck_body = AsciiSTL(filename="./vismach/100sy-stl-files/chuck_body.STL")
    chuck_jaw = AsciiSTL(filename="./vismach/100sy-stl-files/chuck_jaw.STL")
    work_piece = AsciiSTL(filename="./vismach/100sy-stl-files/example_part.stl")
    tool_holder = AsciiSTL(filename="./vismach/100sy-stl-files/HSK_turning_63T_square_25x2.stl")
    tool_turning = AsciiSTL(filename="./vismach/100sy-stl-files/tool_svjn-r20x20x120.stl")
    #pass

except Exception as detail:
    print(detail)
    raise SystemExit("mazak-integrex-100sy-gui requires stl files in working directory")

for setting in sys.argv[1:]: exec(setting)

c = hal.component("mazak-integrex-100sy-gui")
# joint_0
c.newpin("joint_0", hal.HAL_FLOAT, hal.HAL_IN)
# joint_1
c.newpin("joint_1", hal.HAL_FLOAT, hal.HAL_IN)
# head vertical slide
c.newpin("joint_2", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_a
c.newpin("tool_spindle_angle", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_b
c.newpin("rotary_b", hal.HAL_FLOAT, hal.HAL_IN)
# rotary_c
c.newpin("rotary_c", hal.HAL_FLOAT, hal.HAL_IN)
# counter_spindle (axis W)
c.newpin("axis_w", hal.HAL_FLOAT, hal.HAL_IN)
# tool offsets
c.newpin("tool_length", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_diameter", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_number", hal.HAL_U32, hal.HAL_IN)
c.newpin("tool_x_offset", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_y_offset", hal.HAL_FLOAT, hal.HAL_IN)
# geometric offsets in the spindle-rotary-assembly
c.newpin("pivot_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("pivot_z", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("angle_y_yt", hal.HAL_FLOAT, hal.HAL_IN)
# active work offset values (ie g54,g55...)
c.newpin("work_offset_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("work_offset_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("work_pivot_x", hal.HAL_FLOAT, hal.HAL_IN)
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
class HalToolCylinder(CylinderX):
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
        length = 0
        return length, r, 0, r

# used to create an indicator for the variable 'pivot_z'
class HalPivotX(CylinderY):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.pivot_x
        # start the spindle housing at hight 100 above the spindle nose
        return length, r, 0, 1


# used to create an indicator for the variable 'pivot_x'
class HalPivotZ(CylinderX):
    def __init__(self, comp, *args):
        CylinderX.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 2
        length = -c.pivot_z
        return length, r, 0, 1

# used to create a thin tool-offset indicator
class HalToolOffset(CylinderX):
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
tool_101 = Rotate([tool_101],90,0,1,0)
tool_101 = Rotate([tool_101],-90,1,0,0)
tool_101 = HalShow([tool_101],c,True,"hide_spindle_body",0,1)
tool_101 = HalShow([tool_101],c,[10,101],"tool_number",1,0)


# create a visual indicator for the position of the control point (ie indicate the position in the DRO)
ctrl_pt = Point(c)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_x_offset",-1,0,0)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_y_offset",0,-1,0)
ctrl_pt = HalTranslate([ctrl_pt],c,"tool_length",0,0,-1)
ctrl_pt = Rotate([ctrl_pt],90,0,1,0)
ctrl_pt = Rotate([ctrl_pt],90,1,0,0)

# control point to be displayed when in identity mode
#ident_ctrl_pt = HalRotate([ctrl_pt],c,"tool_spindle_angle",1,0,0,1)
ident_ctrl_pt = HalVectorTranslate([ctrl_pt],c,"pivot_z",0,0,-1)
ident_ctrl_pt  = HalTranslate([ident_ctrl_pt ],c,"pivot_x",0,-1,0)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,1,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,2,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,3,"kinstype_select",0,1)
ident_ctrl_pt = HalShow([ident_ctrl_pt],c,4,"kinstype_select",0,1)

# control_point to be displayed when in FULL/HALF TCP MILL and TWP modes
tcp_ctrl_pt_mill= HalShow([ctrl_pt],c,0,"kinstype_select",0,1)
tcp_ctrl_pt_mill= HalShow([tcp_ctrl_pt_mill],c,2,"kinstype_select",0,1)
tcp_ctrl_pt_mill= HalShow([tcp_ctrl_pt_mill],c,4,"kinstype_select",0,1)

# control_point to be displayed when in HALF TCP LATHE mode
tcp_ctrl_pt_lathe = Rotate([ctrl_pt],-90,0,0,1)
tcp_ctrl_pt_lathe = HalShow([tcp_ctrl_pt_lathe],c,0,"kinstype_select",0,1)
tcp_ctrl_pt_lathe = HalShow([tcp_ctrl_pt_lathe],c,1,"kinstype_select",0,1)
tcp_ctrl_pt_lathe = HalShow([tcp_ctrl_pt_lathe],c,3,"kinstype_select",0,1)

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
                CylinderX(-0.1, 0, 0.1, 10),
                ])
tool_cylinder = HalShow([tool_cylinder],c,True,"hide_spindle_body",0,1)
# hide this for lathe tools
tool_cylinder = HalShow([tool_cylinder],c,[10,101],"tool_number",0,1)

# create an indicator for the tool-offset when spindle-body is hidden
ind_tool_offset =  Color([1,0,1,0],[HalLine(c,0,0,0,"tool_x_offset","tool_y_offset","tool_length",-1,1)])
# for mill modes
ind_tool_offset_mill = Rotate([ind_tool_offset],-90,0,1,0)
ind_tool_offset_mill = Rotate([ind_tool_offset_mill],90,1,0,0)
ind_tool_offset_mill = HalShow([ind_tool_offset_mill],c,2,"kinstype_select",0,1)
ind_tool_offset_mill = HalShow([ind_tool_offset_mill],c,3,"kinstype_select",0,1)
ind_tool_offset_mill = HalShow([ind_tool_offset_mill],c,4,"kinstype_select",0,1)
#for lathe modes
ind_tool_offset_lathe =  Rotate([ind_tool_offset],90,1,0,0)
ind_tool_offset_lathe = HalShow([ind_tool_offset_lathe],c,0,"kinstype_select",0,1)
ind_tool_offset_lathe = HalShow([ind_tool_offset_lathe],c,1,"kinstype_select",0,1)

# create a visual for the position of the spindle A,B pivot-point
ind_pivot_point = Collection([
                  Color([1,1,1,1],[CylinderX(-200,1,200,1)]),
                  #Rotational axis for b
                  Color([1,1,0.3,1],[CylinderZ(-300,2,300,2)]),
                  Color([1,1,1,1],[CylinderZ(-200,1,200,1)]),
                  # arrow indicating the up position
                  Color([1,1,1,1],[CylinderZ(200,5,210,1)]),
                  # sphere for the actual pivot point
                  Color([1,1,0.3,1],[Sphere(0,0,0,5)])
                  ])
# create an indicator for the variable pivot_z
ind_pivot_z=  Color([0,0,0.75,1],[HalPivotZ(c)])
# create an indicator for pivot_x
ind_pivot_x = Color([1,0,0,1],[HalPivotX(CylinderZ)])



tool = Collection([
       #Rotate([tooltip_offset],180,1,0,0), #for some reason we cannot just add 'tooltip_offset'
       tcp_ctrl_pt_lathe,
       tcp_ctrl_pt_mill,
       tool_101,
       tool_cylinder,
       Rotate([ind_tool_offset_mill],180,0,0,1),
       Rotate([ind_tool_offset_lathe],0,0,0,1),
       tooltip_offset
       ])
tool = Color([1,0,1,0], [tool] )
# create HAL-link for a-axis rotational joint (tool-spindle rotation)"
tool = HalRotate([tool],c,"tool_spindle_angle",1,1,0,0)
# create the spindle housing that contains the spindle
milling_spindle = Color([0.7,0.7,0,1],[milling_spindle])
milling_spindle = Rotate([milling_spindle],90,0,1,0)
milling_spindle = Rotate([milling_spindle],90,1,0,0)
milling_spindle = HalShow([milling_spindle],c,True,"hide_spindle_body",0,1)
spindle_housing = Collection([
                  HalTranslate([tool],c,"pivot_z",-1,0,0),
                  Translate([milling_spindle],-200,0,0),
                  ind_pivot_point,
                  ind_pivot_x,
                  HalTranslate([ind_pivot_z],c,"pivot_x",0,-1,0),
                  ])
spindle_housing = Rotate([spindle_housing],90,0,0,1)
# create HAL-link for b-axis rotational joint"
spindle_housing = HalRotate([spindle_housing],c,"rotary_b",1,0,0,1)
# join the two parts to a spindle assembly
slide_yt = Collection([
                    spindle_housing,
                    ident_ctrl_pt,
                    Translate([Color([1,0.5,0,0],[Rotate([head_x1],-60,1,0,0)])],-200,0,0),
                    Translate([Color([1,0.5,0,0],[Rotate([head_x2],-60,1,0,0)])],-200,0,0),
                   ])
# spindle head and x-slide move with x
slide_yt = HalTranslate([slide_yt],c,"joint_0",0,1,0)
slide_yt = Color([0.6,0.8,0.3,0], [slide_yt])
spindle_xy = Collection([
             slide_yt,
             Translate([Color([0.6,0.8,0.3,0],[Rotate([head_y1],-60,1,0,0)])],-200,0,0),
             Translate([Color([0.6,0.8,0.3,0],[Rotate([head_y2],-60,1,0,0)])],-200,0,0),
             ])

spindle_xy = HalRotate([spindle_xy],c,"angle_y_yt",1,1,0,0)
# spindle head and yt-slide move with y
spindle_xy = HalTranslate([spindle_xy],c,"joint_1",0,1,0)
# move the spindle_yz to it's designated home position
#spindle_xy = Translate([spindle_xy], -machine_zero_z, machine_zero_y, -machine_zero_x)
# spindle head and y-slide move with z
spindle_xy = HalTranslate([spindle_xy],c,"joint_2",1,0,0)



#spindle_xyz = Translate([spindle_xyz],-800,0,0)
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
# create a coordinate system for the twp-plane
work_plane_coords =  HalCoordsFromNormalAndDirection(c,
        "twp_oz", "twp_oy", "twp_ox",
        "twp_xx", "twp_xy", "twp_xz",
        "twp_zx", "twp_zy", "twp_zz",
        10,
        r=1, s_ox=-1
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




# create 'slots' in x and y on the rotary table
slot_x_w = Color([1,1,1,1],[BoxCenteredXY(250, 10, 0.9)])
slot_y_w = Color([1,1,1,1],[BoxCenteredXY(10, 250, 0.9)])
# create indicator of at positive end of x slot
slot_pocket = Color([1,1,1,1],[CylinderZ(0,10,1,10)])
slot_pocket = Translate([slot_pocket],120,0,0)
# three jaw main_chuck
# x translation of the first jaw controls how open the jaws are
main_chuck_jaw = Translate([Rotate([chuck_jaw],60,0,0,1)],-18,0,-1)
main_chuck = Collection([
                    Translate([Rotate([chuck_body],60,0,0,1)],0,0,50),
                    main_chuck_jaw,
                    Rotate([main_chuck_jaw],-120,0,0,1),
                    Rotate([main_chuck_jaw], 120,0,0,1)
                    ])
main_chuck = Scale([main_chuck],1.6,1.6,1.1)
main_chuck = Color([0.1,0.7,0.9,0],[main_chuck])
# /three jaw main_chuck
main_spindle = Collection([
                 main_chuck,
                 work,
                 Translate([work_piece],0,0,100),
                 slot_x_w,
                 slot_y_w,
                 slot_pocket,
                 ])
main_spindle = Rotate([main_spindle],90,0,0,1)
main_spindle = HalRotate([main_spindle],c,"angle_y_yt",1,0,0,1)
# create HAL-link for c-axis rotational joint"
main_spindle = HalRotate([main_spindle],c,"rotary_c",-1,0,0,1)

main_spindle= Collection([
        main_spindle,
        Rotate([Color([0.1,0.7,0.9,0],[main_spindle_flange])],60,0,0,1),
        ident_coords,
        twp_coords,
        mill_tcp_coords,
        ])
# rotate so it alignes with the z-axis
main_spindle = Rotate([main_spindle],90,0,1,0)

# counter spindle 
# three jaw sub_chuck
# x translation of the first jaw controls how open the jaws are
sub_chuck_jaw = Translate([Rotate([chuck_jaw],60,0,0,1)],-40,0,-1)
sub_chuck = Collection([
                    Translate([Rotate([chuck_body],60,0,0,1)],0,0,50),
                    sub_chuck_jaw,
                    Rotate([sub_chuck_jaw],-120,0,0,1),
                    Rotate([sub_chuck_jaw], 120,0,0,1)
                    ])
sub_chuck = Scale([sub_chuck],1.6,1.6,1.1)
sub_chuck = Color([0.2,0.8,1.0,0],[sub_chuck])
# /three jaw main_chuck

counter_spindle = Collection ([
                    Color([0.7,0.7,0,1], [counter_spindle]),
                    Color([0.2,0.8,1.0,0],[Rotate([sub_spindle],90,0,1,0)]),
                    Rotate([sub_chuck],-90,0,1,0)
                    ])
counter_spindle =  HalTranslate([counter_spindle],c,"axis_w",1,0,0)
# /counter spindle


#/work-side
bed = Color([0.4,0.4,0.4,1], [bed])
front_x1 = Color([0.4,0.4,0.4,1], [front_x1])

model = Collection([spindle_xy, work_plane, bed, front_x1, main_spindle, counter_spindle])

main(model, tooltip, work, size=4000, lat=-60, lon=-35)
