#!/usr/bin/env python3

from vtk_vismach import *
import hal
from math import sin, cos, radians, degrees
import sys

# get current working directory
cwd =  os.getcwd()
# define path for machine model files
path_stl = cwd + "/vismach-stl-files/"
# define path for tool model files
path_tool_stl = cwd + "/vismach-stl-files/tools/"

try: # Expect files in working directory
    # NOTE to run this file as standalone python script absolute paths might have to be used to find the stl files
    # create the work piece from file
    work_piece = ReadPolyData('work_piece_1.stl', path_stl).set_group('work')
    EGO_BC = ReadPolyData('160Pbase.stl', path_stl).set_group('machine')
    EGO_B = ReadPolyData('160PB.stl', path_stl).set_group('machine')
    EGO_C = ReadPolyData('160PC.stl', path_stl).set_group('machine')
    EGO_X = ReadPolyData('160PX.stl', path_stl).set_group('machine')
    EGO_Y = ReadPolyData('160PY.stl', path_stl).set_group('machine')
    EGO_Z = ReadPolyData('160PZ.stl', path_stl).set_group('machine')
except Exception as detail:
    print(detail)
    raise SystemExit('Vismach requires 3d files in working directory')

#for setting in sys.argv[1:]: exec(setting)
options = sys.argv[1:]

c = hal.component('vtk-dmu-160-p-gui')

c.newpin("tool_number", hal.HAL_U32, hal.HAL_IN)
# nutation-angle
c.newpin('nutation_angle', hal.HAL_FLOAT, hal.HAL_IN)
# virtual tool rotation
c.newpin('virtual_rotation', hal.HAL_FLOAT, hal.HAL_IN)
# geometric offsets in the spindle-rotary-assembly
c.newpin('pivot_y', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('pivot_z', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('offset_x', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('offset_z', hal.HAL_FLOAT, hal.HAL_IN)
# rot-point offsets distances from pivot point ( spindle AB)
# to rotation axis ( table C)
c.newpin('rot_axis_x', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('rot_axis_y', hal.HAL_FLOAT, hal.HAL_IN)
# active work offset values (ie g54,g55...)
c.newpin('work_offset_x', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('work_offset_y', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('work_offset_z', hal.HAL_FLOAT, hal.HAL_IN)
# selected kinematics
c.newpin('kinstype_select', hal.HAL_FLOAT, hal.HAL_IN)
# scale coordinate system indicators
c.newpin('scale_coords', hal.HAL_FLOAT, hal.HAL_IN)
# twp pins
c.newpin('twp_status', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_defined', hal.HAL_BIT, hal.HAL_IN)
c.newpin('twp_active', hal.HAL_BIT, hal.HAL_IN)
# the origin of the twp plane display needs to be independent of the work offsets
# because those offsets change as the kinematic switches between world and tool
c.newpin('twp_ox_world', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_oy_world', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_oz_world', hal.HAL_FLOAT, hal.HAL_IN)
# origin of the twp
c.newpin('twp_ox', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_oy', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_oz', hal.HAL_FLOAT, hal.HAL_IN)
# normal vector defining the temporary work plane as set by user with G68.2
c.newpin('twp_zx', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_zy', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_zz', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_xx', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_xy', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('twp_xz', hal.HAL_FLOAT, hal.HAL_IN)
# currently applied g52/g92 offset
c.newpin('g92_x', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('g92_y', hal.HAL_FLOAT, hal.HAL_IN)
c.newpin('g92_z', hal.HAL_FLOAT, hal.HAL_IN)
c.ready()


# custom subclass used to rotate parts around the nutation axis
class Nutate(Collection):
    def update(self):
        th,x,y,z = self.coords()
        # override x,y,z
        x = 0
        y = sin(radians(c.nutation_angle))
        z = cos(radians(c.nutation_angle))
        self.transformation = vtk.vtkTransform()
        self.transformation.PreMultiply()
        self.transformation.RotateWXYZ(th,x,y,z)

    def transform(self):
        self.SetUserTransform(self.transformation)


# Creates a 3d arrow pointing from (xs,ys,zs) to (xe,ye,ze)
# where 'ye' depends on two hal values (ye = 'twp_oy_world' - 'joint.1.pos-fb')
class CustomArrow(Arrow):
    def coords(self):
        xs, ys, zs, xe, ye, ze, radius = super().coords()
        ye -= hal.get_value('joint.1.pos-fb')
        return xs, ys, zs, xe, ye, ze, radius


# Machine zero as measured from center surface of the rotary c table
machine_zero_x = -1000
machine_zero_y =  1000
machine_zero_z =  1000

# Create an indicator for the machine reference coordinates
machine_coords = Axes(c,('scale_coords',200))
machine_coords = Translate([machine_coords], machine_zero_x, 0, machine_zero_z)

# start toolside
# Create the tooltip tracker (tool control point)
tooltip = Capture('tool')
# Create an indicator for the tool coordinate system in IDENTITY mode
tool_coords_idt = Scale([Axes(100)],hal,0,'motion.switchkins-type',1,0)
tool_coords_idt = Translate([tool_coords_idt],hal,0,0,('motion.tooloffset.z',-1))
tool_coords_idt = Translate([tool_coords_idt],c,0,('pivot_y',-1),('pivot_z',-1))
# Create an indicator for the tool coordinate system in TCP and TWP modes
tool_coords_tcp_twp = Scale([Axes(100)],hal,[1,2],'motion.switchkins-type',1,0)
tool_shape = Collection([
                CylinderZ(hal,'motion.tooloffset.z', ('halui.tool.diameter', 0.5)),
                # this indicates the spindle nose when no tool-offset is active
                CylinderZ(-0.1, 0),
                ])
tool_shape = Color([tool_shape],1,0,1,1)
#tool_stl = ReadPolyData(hal,"halui.tool.number", path_tool_stl)
#tool_stl = ReadPolyData(c,"tool_number", path_tool_stl)
#tool_stl = Rotate([tool_stl],180,0,1,0)
#tool_stl = Color([tool_stl],"magenta",1)
#tool_stl = Translate([tool_stl],hal,0,0,'motion.tooloffset.z')
tool = Collection([
                    tooltip,
                    tool_coords_tcp_twp,
                    tool_shape,
#                    tool_stl,
                    ])
tool = Rotate([tool],c,'virtual_rotation',0,0,1)
tool = Translate([tool],hal,0,0,('motion.tooloffset.z',-1))
tool = Translate([tool],c,0,('pivot_y',-1),('pivot_z',-1))
# create spindle head
EGO_B = Color([EGO_B],0.7,0.7,0,1)
# rotate the nutation joint to the nutation angle, 90° should have the nutation axis in the horizontal plane
EGO_B = Rotate([EGO_B],-90,-1,0,0)
EGO_B = Rotate([EGO_B],c,'nutation_angle',-1,0,0)
spindle_assembly = Collection([
                   tool,
                   EGO_B,
                   #ind_pivot_point,
                   #ind_pivot_z,
                   #ind_pivot_y
                   ])

# create HAL-link for b-axis rotational joint'
spindle_assembly = Nutate([spindle_assembly],hal,'joint.3.pos-fb',0,-1,0)
# Z carriage
EGO_Z = Color([EGO_Z],1,0.5,0,1)
EGO_Z = Translate([EGO_Z], 0, 0, -759.5)
spindle_z = Collection([
             spindle_assembly,
             EGO_Z,
             tool_coords_idt
             ])
# move by the values set for the pivot lengths so the vismach origin is in the center of the spindle nose
spindle_z = Translate([spindle_z],c,0,'pivot_y','pivot_z')
# spindle head and z-slide move with z
spindle_z = Translate([spindle_z],hal,0,0,'joint.2.pos-fb')
# move spindle_z to z-home position
spindle_z = Translate([spindle_z], 0, 0, machine_zero_z)
# x carriage
EGO_X = Color([EGO_X],0.6,0.8,0.3,1)
spindle_xz = Collection([
             spindle_z,
             EGO_X
             ])
# spindle head, z-slide and x-slide move with x
spindle_xz = Translate([spindle_xz],hal,'joint.0.pos-fb',0,0)
# move spindle_xz to x-home position
spindle_xz = Translate([spindle_xz], machine_zero_x, 0, 0)
# end toolside

# start workside
# Create the work tracker (top center of the rotary table)
work = Capture('work')
# Create an indicator for the work coordinates
work_axes = Axes(100)
# TWP Matrix
twp_matrix = ('twp_xx', 'twp_xy', 'twp_xz',
              'twp_zx', 'twp_zy', 'twp_zz',
              'twp_ox', 'twp_oy', 'twp_oz',)
# TWP-Defined
work_plane_defined = MatrixTransform([Grid(300, 10)],c,*twp_matrix)
wcs2twp_defined = Arrow(c,0,0,0,'twp_ox','twp_oy','twp_oz',3)
work_plane_coords_defined =  MatrixTransform([Axes(c,('scale_coords',300))],c,*twp_matrix)
# create an indicator for the currently active G52/G92 offset for definded twp
g92_twp_defined = MatrixTransform([Arrow(c,0,0,0,'g92_x','g92_y','g92_z',3)],c,*twp_matrix)
g92_twp_defined = Translate([g92_twp_defined], machine_zero_x,  machine_zero_y, machine_zero_z)
g92_twp_defined = Translate([g92_twp_defined],c,'twp_ox_world','twp_oy_world','twp_oz_world')
work_plane_defined = Collection([work_plane_defined,
                                 wcs2twp_defined,
                                 work_plane_coords_defined,
                                 g92_twp_defined
                                 ])
work_plane_defined = Color([work_plane_defined],c,None,('twp_defined',0.2))
# TWP-Active
work_plane_active =  MatrixTransform([Plane(300)],c,*twp_matrix)
# for twp-active = true, we show the plane in pink
work_plane_active = Color([work_plane_active],c,1,0,1,0.3)
wcs2twp_active = Arrow(c,0,0,0,'twp_ox','twp_oy','twp_oz',3)
work_plane_coords_active =  MatrixTransform([Axes(c,('scale_coords',300))],c,*twp_matrix)
# create an indicator for the currently active G52/G92 offset in active twp mode
g92_twp_active = MatrixTransform([Arrow(c,0,0,0,'g92_x','g92_y','g92_z',3)],c,*twp_matrix)
g92_twp_active = Translate([g92_twp_active], machine_zero_x,  machine_zero_y, machine_zero_z)
g92_twp_active = Translate([g92_twp_active],c,'twp_ox_world','twp_oy_world','twp_oz_world')
work_plane_active = Collection([work_plane_active,
                                wcs2twp_active,
                                work_plane_coords_active,
                                g92_twp_active
                                ])
work_plane_active = Scale([work_plane_active],c,True,'twp_active',1,0)
work_plane = Collection([work_plane_defined,
                         work_plane_active,
                         ])
# move to the current wcs origin
work_plane = Translate([work_plane], machine_zero_x,  machine_zero_y, machine_zero_z)
work_plane = Translate([work_plane],c,'twp_ox_world','twp_oy_world','twp_oz_world')
# create an indicator for the currently active G52/G92 offset in IDENTITY and TCP modes
g92 = Arrow(c,0,0,0,'g92_x','g92_y','g92_z',3)
g92 = Translate([g92], machine_zero_x,  machine_zero_y, machine_zero_z)
g92 = Translate([g92],c,'twp_ox_world','twp_oy_world','twp_oz_world')
g92_idt = Scale([g92],hal,0,'motion.switchkins-type',1,0)
g92_tcp = Scale([g92],hal,1,'motion.switchkins-type',1,0)
# Create geometry for the work piece
work_piece = Translate([Box(600,600,600)],0,0,300).set_group('work')
# Create rotary table
EGO_C = Color([EGO_C],0.1,0.7,0.9,1)
rotary_table_c = Collection([
                 work_piece,
                 EGO_C,
                 work_plane,
                 work_axes,
                 g92_tcp,
                 work
                 ])
# Rotary table and work roatae with axis C'
rotary_table_c = Rotate([rotary_table_c],hal,'joint.4.pos-fb',0,0,-1)
# y-carriage that carries the rotary table
EGO_Y = Color([EGO_Y],0.2,0.2,0.2,1)
table = Collection([
        rotary_table_c,
        EGO_Y,
        g92_idt
        ])
# Table moves with y axis
#table = HalTranslate_orig([table],c,'axis_y',0,-1,0)
table = Translate([table],hal,0,('joint.1.pos-fb',-1),0,'halui.flood.is-on').set_group('table')
# move table to y-home position
table = Translate([table], 0, -machine_zero_y, 0)
#/work-side

# Create machine base
base = Color([EGO_BC],0.3,0.3,0.3,1)
wcs = CustomArrow(c,0,0,0,'twp_ox_world','twp_oy_world','twp_oz_world',3)
wcs = Translate([wcs], machine_zero_x, 0, machine_zero_z)
model = Collection([
        machine_coords,
        spindle_xz,
        table,
        base,
        wcs
        ])
#hud
myhud = Hud(color='mint',opacity=0.4) # This will always be displayed
myhud.add_txt('DMU-160-P')
myhud.add_txt('---------')
myhud.add_txt('')
myhud.add_txt('Kinematic Mode: IDENTITY',0)
myhud.add_txt('Kinematic Mode: TCP',1)
myhud.add_txt('Kinematic Mode: TOOL',2)
myhud.add_txt('')
myhud.add_txt('TWP-Orientation:')
myhud.add_txt('   X             Z')
myhud.add_pin('{:6.3f}        {:6.3f}',c,['twp_xx','twp_zx'])
myhud.add_pin('{:6.3f}        {:6.3f}',c,['twp_xy','twp_zy'])
myhud.add_pin('{:6.3f}        {:6.3f}',c,['twp_xz','twp_zz'])
myhud.show_tag_eq_pin_offs(hal,'motion.switchkins-type')

#myhud2 = Hud(c,'kinstype_select',2,'mint', 1) # This is displayed when kintype is 2
myhud2 = Hud(color='yellow',opacity=0.4) # This will always be displayed
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_txt('')
myhud2.add_pin('{:6.3f}        {:6.3f}',c,['twp_xx','twp_zx'])
myhud2.add_pin('{:6.3f}        {:6.3f}',c,['twp_xy','twp_zy'])
myhud2.add_pin('{:6.3f}        {:6.3f}',c,['twp_xz','twp_zz'])
myhud2.extra_text_enable = True
#/hud

main(options,
     c,
     model,
     huds=[myhud,myhud2],
     window_width=1400,
     window_height=1000,
     window_title = 'Vtk_Vismach Tutorial')
