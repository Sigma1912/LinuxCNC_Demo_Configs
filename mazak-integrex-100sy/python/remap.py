# This is a customized version of the TWP python remap for mazak-integrex-200y.ini
#
# Copyright ()c) 2025 David Mueller <mueller_david@hotmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#
import sys
import traceback
import numpy as np
from math import sin,cos,tan,asin,acos,atan,atan2,sqrt,pi,degrees,radians,fabs
from interpreter import *
import emccanon
from util import lineno, call_pydevd
import hal

# logging
import logging
# this name will be printed first on each log message
log = logging.getLogger('remap.py; TWP')
# we have to setup a handler to be able to set the log level for this module
handler = logging.StreamHandler()
formatter = logging.Formatter('%(name)s %(levelname)s: %(message)s')
handler.setFormatter(formatter)
log.addHandler(handler)

# set up parsing of the inifile
import os
import configparser
# get the path for the ini file used to start this config
inifile = os.environ.get("INI_FILE_NAME")

# instantiate a parser in non-strict mode because we have multiple entries for
# some sections in the ini
config = configparser.ConfigParser(strict=False)
# ingest the ini file
config.read(inifile)

# debug setting
try:
    debug_setting = int(config['TWP']['debug'])
    if debug_setting > 4: debug_setting = 4
    if debug_setting < 0: debug_setting = 0
except Exception as error:
    debug_setting = 1
    log.warning("Unable to parse debug setting given in INI. Setting it to 1.")
debug_levels = (logging.CRITICAL, logging.ERROR, logging.WARNING, logging.INFO,  logging.DEBUG)
log.setLevel(debug_levels[debug_setting])

## ROTARY JOINT LETTERS
# primary rotary joint (independent of the secondary joint)
joint_letter_primary = (config['TWP']['PRIMARY']).capitalize()
# secondary rotary joint (dependent on the primary joint)
joint_letter_secondary = (config['TWP']['SECONDARY']).capitalize()

if not joint_letter_primary in ('A','B','C') or not joint_letter_secondary in ('A','B','C'):
    log.error("Unable to parse joint letters given in INI [TWP].")
elif joint_letter_primary == joint_letter_secondary:
    log.error("Letters for primary and secondary joints in INI [TWP] must not be the same.")
else:
    # get the MIN/MAX limits of the respective rotary joint letters
    category = 'AXIS_' +  joint_letter_primary
    primary_min_limit = radians(float(config[category]['MIN_LIMIT']))
    primary_max_limit = radians(float(config[category]['MAX_LIMIT']))
    log.info('Joint letter for work side rotation is %s with MIN/MAX limits: %s,%s',
                         joint_letter_primary, degrees(primary_min_limit), degrees(primary_max_limit))
    category = 'AXIS_' +  joint_letter_secondary
    secondary_min_limit = radians(float(config[category]['MIN_LIMIT']))
    secondary_max_limit = radians(float(config[category]['MAX_LIMIT']))
    log.info('Joint letter for tool side rotation is %s with MIN/MAX Limits: %s,%s',
                     joint_letter_secondary, degrees(secondary_min_limit), degrees(secondary_max_limit))

## CONNECTIONS TO THE HELPER COMPONENT
twp_comp = 'twp-helper-comp.'
twp_is_defined = twp_comp + 'twp-is-defined'
twp_is_active = twp_comp + 'twp-is-active'

# raise InterpreterException if execute() or read() fail
throw_exceptions = 1

## VALUE INITIALIZATION
# we start with the identity matrix (ie the twp is equal to the world coordinates)
twp_matrix = np.asmatrix(np.identity(4))
# container to store the current work offset during twp operations
current_work_offset_number = 1
saved_work_offset = [0,0,0]
## CONNECTIONS TO THE KINEMATIC COMPONENT
# get the name of the kinematic component (this seems to ingest also the next line)
kins_comp = 'mazak-integrex-100sy-kins'
kins_virtual_rotation = kins_comp + '.prerot-angle'
kins_pivot_z  = kins_comp + '.pivot-z'
kins_pivot_x  = kins_comp + '.pivot-x'

kins_primary_rotation = kins_comp + '.work-angle'
kins_secondary_rotation = kins_comp + '.b-orientation'

# defines the kinematic model for (world <-> tool) coordinates of the machine at hand
# returns 4x4 transformation matrix for given angles and 4x4 input matrix
# NOTE: these matrices must be the same as the ones used to derive the kinematic model
def kins_calc_transformation_matrix(theta_1, theta_2, virtual_rot, matrix_in, direction='fwd'): # expects radians
    theta_1=0 #TODO get full twp functionality with work rotation working
    global kins_nutation_angle
    T_in = matrix_in
    # Define 4x4 transformation for virtual rotation around z to orient tool-x and -y
    Stc = sin(virtual_rot)
    Ctc = cos(virtual_rot)
    Rtc=np.matrix([[ Ctc, -Stc, 0, 0],
                   [ Stc,  Ctc, 0, 0],
                   [ 0 ,  0 ,   1, 0],
                   [ 0,   0 ,   0, 1]])

    ## Define 4x4 transformation for the table joint
    # get the basic 3x3 rotation matrix (returns array)
    Rw = Rz(theta_1)
    # add fourth column on the right
    Rw = np.hstack((Rw, [[0],[0],[0]]))
    # expand to 4x4 array and make into a matrix
    row_4 = [0,0,0,1]
    Rw = np.vstack((Rw, row_4))
    Rw = np.asmatrix(Rw)

    ## Define 4x4 transformation matrix for the spindle joint
    # get the basic 3x3 rotation matrix (returns array)
    Rs = Ry(theta_2)
    # add fourth column on the right
    Rs = np.hstack((Rs, [[0],[0],[0]]))
    # expand to 4x4 array and make into a matrix
    row_4 = [0,0,0,1]
    Rs = np.vstack((Rs, row_4))
    Rs = np.asmatrix(Rs)

    Dx = hal.get_value(kins_pivot_x)

    # define the forward transformation for the geometric offsets
    To=np.matrix([[ 1, 0, 0,  -Dx],
                  [ 0, 1, 0,  0 ],
                  [ 0, 0, 1,  0 ],
                  [ 0, 0, 0,  1 ]])
    # define the inverse transformation for the geometric offsets
    Tio=np.matrix([[ 1, 0, 0,  Dx],
                   [ 0, 1, 0,  0 ],
                   [ 0, 0, 1,  0 ],
                   [ 0, 0, 0,  1 ]])


    # calculate the transformation matrix for the forward tool kinematic
    matrix_tool_fwd = Rw*T_in*Rs*To*Rtc
    # calculate the transformation matrix for the inverse tool kinematic
    matrix_tool_inv = To*np.transpose(Rtc)*Tio*np.transpose(Rs)*T_in*np.transpose(Rw)
    if direction == 'fwd':
        print("matrix tool fwd: \n", matrix_tool_fwd)
        print("inv would have been: \n", matrix_tool_inv)
        return matrix_tool_fwd
    elif direction == 'inv':
        print("matrix tool inv: \n", matrix_tool_inv)
        print("fwd would have been: \n", matrix_tool_fwd)
        return matrix_tool_inv
    else:
        return 0


# calculate the transformed work offset used after 53.n
def kins_calc_transformed_work_offset(current_offset, twp_offset, theta_1, theta_2, virtual_rot):
    # in this kinematic we need to keep in mind that we have rotations in the spindle and the table
    current_offset_incl_twp = tuple(np.add(current_offset, twp_offset))
    kins_rot_axis_abs = (0,0,0)
    work_offset_from_cntr_rot_table = tuple(np.subtract(current_offset_incl_twp, kins_rot_axis_abs))

    ## Define 4x4 transformation for the table joint
    # get the basic 3x3 rotation matrix (returns array)
    Rw = Rz(-theta_1)
    # add fourth column on the right
    Rw = np.hstack((Rw, [[0],[0],[0]]))
    # expand to 4x4 array and make into a matrix
    row_4 = [0,0,0,1]
    Rw = np.vstack((Rw, row_4))
    Rw = np.asmatrix(Rw)

    table_rot_offset = matrix_to_point(Rw*point_to_matrix(work_offset_from_cntr_rot_table))
    diff_table_rot_offset = tuple(np.subtract(table_rot_offset, work_offset_from_cntr_rot_table))
    offset_pre_trans =  tuple(np.add(current_offset_incl_twp, diff_table_rot_offset))
    P = matrix_to_point(kins_calc_transformation_matrix(theta_1,
                                                        theta_2,
                                                        virtual_rot,
                                                        point_to_matrix(offset_pre_trans),
                                                        'inv'))
    transformed_offset = (P[0], P[1], P[2])
    return transformed_offset


# pass required values to the kinematics component
def kins_set_values(theta_1, theta_2, virtual_rot): # expects radians
    #hal.set_p(kins_virtual_rotation, str(virtual_rot))
    hal.set_p(kins_primary_rotation, str(theta_1))
    hal.set_p(kins_secondary_rotation, str(theta_2))


# If the operator has requested a rotation by passing an R word in the 68.n command we need to
# create a rotation matrix that represents a rotation around the Z-axis of the TWP plane
def kins_calc_twp_origin_rot_matrix(r): # expects radians
    # we use z rotation to create the rotation matrix for the requested origin rotation
    twp_origin_rot_matrix = Rz(r)

    return twp_origin_rot_matrix


# This returns which transformation to use when checking calculated angles
# and when calculating the twp_matrix for G68.3
def kins_calc_transformation_get_direction():
    return 'fwd'



# forms a 4x4 transformation matrix from a given 1x3 point vector [x,y,z]
def point_to_matrix(point):
    # start with a 4x4 identity matrix and add the point vector to the 4th column
    matrix = np.identity(4)
    [matrix[0,3], matrix[1,3], matrix[2,3]] = point
    matrix = np.asmatrix(matrix)
    return matrix

# extracts the point vector form a given 4x4 transformation matrix
def matrix_to_point(matrix):
    point = (matrix[0,3],matrix[1,3],matrix[2,3])
    return point


# define the basic rotation matrices
def Rx(th):
   return np.array([[1, 0      ,  0      ],
                    [0, cos(th), -sin(th)],
                    [0, sin(th),  cos(th)]])

def Ry(th):
   return np.array([[ cos(th), 0, sin(th)],
                    [ 0      , 1, 0      ],
                    [-sin(th), 0, cos(th)]])

def Rz(th):
   return np.array([[cos(th), -sin(th), 0],
                    [sin(th),  cos(th), 0],
                    [0      ,  0      , 1]])



def calc_euler_rot_matrix(th1, th2, th3, order): # expects radians
    # returns the rotation matrices for given order and angles
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    debug_msg = (f'   Euler order {order} requested with angles: '
                 f'{degrees(th1):.4f}, {degrees(th2):.4f}, {degrees(th3):.4f}')
    log.debug(debug_msg)
    if order == '131':
        matrix = np.dot(np.dot(Rx(th1), Rz(th2)), Rx(th3))
    elif order=='121':
        matrix = np.dot(np.dot(Rx(th1), Ry(th2)), Rx(th3))
    elif order=='212':
        matrix = np.dot(np.dot(Ry(th1), Rx(th2)), Ry(th3))
    elif order=='232':
        matrix = np.dot(np.dot(Ry(th1), Rz(th2)), Ry(th3))
    elif order=='323':
        matrix = np.dot(np.dot(Rz(th1), Ry(th2)), Rz(th3))
    elif order=='313':
        matrix = np.dot(np.dot(Rz(th1), Rx(th2)), Rz(th3))
    elif order=='123':
        matrix = np.dot(np.dot(Rx(th1), Ry(th2)), Rz(th3))
    elif order=='132':
        matrix = np.dot(np.dot(Rx(th1), Rz(th2)), Ry(th3))
    elif order=='213':
        matrix = np.dot(np.dot(Ry(th1), Rx(th2)), Rz(th3))
    elif order=='231':
        matrix = np.dot(np.dot(Ry(th1), Rz(th2)), Rx(th3))
    elif order=='321':
        matrix = np.dot(np.dot(Rz(th1), Ry(th2)), Rx(th3))
    elif order=='312':
        matrix = np.dot(np.dot(Rz(th1), Rx(th2)), Ry(th3))
    #log.debug('   Returning euler rotation as matrix: \n %s', matrix)
    return matrix


def calc_twp_matrix_from_joint_position(self, matrix_in, virtual_rot, direction): # expects radians
    # transforms a 4x4 input matrix using the current transformation matrix
    # (forward or inverse) using the kinematic model of the machine
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global kins_virtual_rotation
    # read current spindle rotary angles (radians)
    theta_1, theta_2 = get_current_rotary_positions(self)
    # virtual-rot is the virtual rotary axis around the z-vector or work-z axis to align the x-vector
    log.debug(f"    requested virtual-rot value {degrees(virtual_rot):.4f}°")
    # run matrix_in through the kinematic transformation in the requested direction
    # using the current joint angles and virtual-rotation as requested
    try:
        twp_matrix = kins_calc_transformation_matrix(theta_1, theta_2, virtual_rot, matrix_in, direction)
    except Exception as error:
        log.error('calc_twp_matrix_from_joint_position, %s', error)
    return twp_matrix


def gui_update_twp():
    # The tilted-work-plane is created in identity mode and must NOT be updated after a switch
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global twp_matrix, saved_work_offset
    # twp origin as vector (in world coords) from current work-offset to the origin of the twp
    try:
        hal.set_p("twp-helper-comp.twp-ox-in",str(twp_matrix[0,3]))
        hal.set_p("twp-helper-comp.twp-oy-in",str(twp_matrix[1,3]))
        hal.set_p("twp-helper-comp.twp-oz-in",str(twp_matrix[2,3]))
        # twp x-vector
        hal.set_p("twp-helper-comp.twp-xx-in",str(twp_matrix[0,0]))
        hal.set_p("twp-helper-comp.twp-xy-in",str(twp_matrix[1,0]))
        hal.set_p("twp-helper-comp.twp-xz-in",str(twp_matrix[2,0]))
        # twp z-vector
        hal.set_p("twp-helper-comp.twp-zx-in",str(twp_matrix[0,2]))
        hal.set_p("twp-helper-comp.twp-zy-in",str(twp_matrix[1,2]))
        hal.set_p("twp-helper-comp.twp-zz-in",str(twp_matrix[2,2]))
    except Exception as error:
        log.error('gui_update_twp failed, %s', error)
    # publish the twp offset coordinates in world coordinates (ie identity)
    [work_offset_x, work_offset_y, work_offset_z] = saved_work_offset
    log.debug("   Setting work_offsets in the simulation: %s", (work_offset_x, work_offset_y, work_offset_z))
    # this is used to translate the rotated twp to the correct position
    # care must be taken that only the work_offsets in identity mode are sent as that is
    # what the model uses. The visuals for the offsets are created in the origin,
    # then rotated according to the rotary joint position and then translated.
    # The twp has to be rotated out of the machine xy plane using the g68.2 parameters and is then
    # translated by the offset values of the identity mode.
    try:
        hal.set_p("twp-helper-comp.twp-ox-world-in",str(work_offset_x))
        hal.set_p("twp-helper-comp.twp-oy-world-in",str(work_offset_y))
        hal.set_p("twp-helper-comp.twp-oz-world-in",str(work_offset_z))
    except Exception as error:
        log.error('gui_update_twp failed, %s', error)


# NOTE: Due to easier abort handling we currently restrict the use of twp to G54
# as LinuxCNC seems to revert to G54 as the default system
def get_current_work_offset(self):
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    # get which offset is active (g54=1 .. g59.3=9)
    active_offset = int(self.params[5220])
    current_work_offset_number = active_offset
    # set the relevant parameter numbers that hold the active offset values
    # (G54_x: #5221, G55_x:#[5221+20], G56_x:#[5221+40] ....)
    work_offset_x = (active_offset-1)*20 + 5221
    work_offset_y = work_offset_x + 1
    work_offset_z = work_offset_x + 2
    co_x = self.params[work_offset_x]
    co_y = self.params[work_offset_y]
    co_z = self.params[work_offset_z]
    current_work_offset = (co_x, co_y, co_z)
    return [current_work_offset_number, current_work_offset]


def get_current_rotary_positions(self):
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global joint_letter_primary, joint_letter_secondary
    if joint_letter_primary == 'A':
        theta_1 = radians(self.AA_current)
    elif joint_letter_primary == 'B':
        theta_1 = radians(self.BB_current)
    elif joint_letter_primary == 'C':
        theta_1 = radians(self.CC_current)
    log.debug(f'   Current position Primary joint: {degrees(theta_1):.4f}°')
    # read current spindle rotary angles and convert to radians
    if joint_letter_secondary == 'A':
        theta_2 = radians(self.AA_current)
    elif joint_letter_secondary == 'B':
        theta_2 = radians(self.BB_current)
    elif joint_letter_secondary == 'C':
        theta_2 = radians(self.CC_current)
    log.debug(f'   Current position Secondary joint: {degrees(theta_2):.4f}°')
    theta_1 = 0 # this config does not track the work rotation for the TWP
    return theta_1, theta_2


def reset_twp_params():
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global virtual_rot, twp_matrix, twp_flag, twp_build_params
    virtual_rot = 0
    # we must not change tool kins parameters when TOOL kins are active or we get sudden joint position changes
    # ie don't do this: kins_comp_set_virtual_rot(0)!
    twp_flag = []
    twp_build_params = {}
    log.info("   Resetting TWP-matrix")
    twp_matrix = np.asmatrix(np.identity(4))


# define a virtual tilted-work-plane (twp) that is perpendicular to the current tool-orientation
# Note: To avoid that this python code is run prematurely by the read ahead we need a quebuster at the beginning but
# because we need self.execute() to switch the WCS properly this remap needs to be called from
# an ngc that contains a quebuster before calling this code
def g68_core(self, **words):
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global twp_matrix, virtual_rot, twp_flag, saved_work_offset_number, saved_work_offset
    if self.task == 0: # ignore the preview interpreter
        yield INTERP_EXECUTE_FINISH
        return INTERP_OK

    if  hal.get_value(twp_is_defined):
         # reset the twp parameters
        reset_twp_params()
        msg =("G68.3 ERROR: TWP already defined.")
        log.debug(msg)
        emccanon.CANON_ERROR(msg)
        yield INTERP_EXECUTE_FINISH # w/o this the error message is not displayed
        yield INTERP_EXIT # w/o this the error does not abort a running gcode program
        return INTERP_ERROR

    # NOTE: Due to easier abort handling we currently restrict the use of twp to G54
    # as LinuxCNC seems to revert to G54 as the default system
    # get which offset is active (g54=1 .. g59.3=9)
    (n, offsets) = get_current_work_offset(self)
    if n != 1:
         # reset the twp parameters
        reset_twp_params()
        msg = "G68.3 ERROR: Must be in G54 to define TWP."
        log.debug(msg)
        emccanon.CANON_ERROR(msg)
        yield INTERP_EXECUTE_FINISH # w/o this the error message is not displayed
        yield INTERP_EXIT # w/o this the error does not abort a running gcode program
        return INTERP_ERROR

    c = self.blocks[self.remap_level]
    # parse the requested origin
    x = c.i_number if c.i_flag else None
    y = c.j_number if c.j_flag else None
    z = c.k_number if c.k_flag else None

    virtual_rot = 0 # This config does not implement virtual rotations (ie no optional 'R' word)

    # then we need to calculate the transformation matrix of the current orientation with the including the
    # calculated virtual-rotation.
    # for this we take the 4x4 identity matrix and pass it through the kinematic transformation using the
    # current rotary joint positions and the calculated virtual-rotation angle plus any additional angle
    # passed in the R word of the G68.3 command
    start_matrix = np.asmatrix(np.identity(4))
    # the required transformation direction may depend on the kinematic at hand
    try:
        direction = kins_calc_transformation_get_direction()
    except Exception as error:
        log.error('kins_calc_transformation_get_direction, %s', error)
    twp_matrix = calc_twp_matrix_from_joint_position(self, start_matrix, virtual_rot, direction)
    log.debug("G68.3: TWP matrix with oriented x-vector: \n%s", twp_matrix)
    # put the requested origin into the twp_matrix
    (twp_matrix[0,3], twp_matrix[1,3], twp_matrix[2,3]) = (x, y, z)

    log.info("G68.3: Built twp-transformation-matrix: \n%s", twp_matrix)
    # collect the currently active work offset values (ie g54, g55 or other)
    saved_work_offset = offsets
    saved_work_offset_number = n
    log.debug("G68.3: Saved work offsets: %s", (n, saved_work_offset))
    # set twp-state to 'defined' (1)
    self.execute("M68 E2 Q1")

    gui_update_twp()

    theta_1, theta_2 = get_current_rotary_positions(self) # radians
    # The twp kinematics in this config assumes the tool pointing in the Z-machine direction (ie B-90) hence we need
    # to offset the joint position so that the zero position is at B -90°
    theta_2 = theta_2 + radians(90)

    # set the virtual-rotation value in the kinematic component
    debug_msg = (f'   G53.n: Setting angle values in kins comp to theta1: {degrees(theta_1):.4f}°, '
                 f'theta2: {degrees(theta_2):.4f}°, virtual_rot: {degrees(virtual_rot):.4f}°')
    log.debug(debug_msg)
    try:
        kins_set_values(theta_1, theta_2, virtual_rot)
    except Exception as error:
        log.error('G53.n: kins_set_values failed, %s', error)

    # calculate the work offset in transformed-coordinatess
    log.debug("   G53.n: Saved work offset: %s", saved_work_offset)
    twp_offset = (twp_matrix[0,3],twp_matrix[1,3],twp_matrix[2,3])
    try:
        new_offset = kins_calc_transformed_work_offset(saved_work_offset, twp_offset, theta_1, theta_2, virtual_rot)
    except Exception as error:
        log.error('G53.n: Calculation of kins_calc_transformed_work_offset failed, %s', error)
    debug_msg = (f'   G53.n: Setting transformed work-offsets for twp-kins in G59, G59.1, '
                 f'G59.2 and G59.3 to: {new_offset[0]:.4f}, {new_offset[1]:.4f}, {new_offset[2]:.4f}')
    log.debug(debug_msg)
    # set the dedicated TWP work offset values (G53, G53.1, G53.2, G53.3)
    self.execute("G10 L2 P6 X%f Y%f Z%f" % (new_offset[0], new_offset[1], new_offset[2]), lineno())
    self.execute("G10 L2 P7 X%f Y%f Z%f" % (new_offset[0], new_offset[1], new_offset[2]), lineno())
    self.execute("G10 L2 P8 X%f Y%f Z%f" % (new_offset[0], new_offset[1], new_offset[2]), lineno())
    self.execute("G10 L2 P9 X%f Y%f Z%f" % (new_offset[0], new_offset[1], new_offset[2]), lineno())

    # switch to the dedicated TWP work offsets
    self.execute("G59", lineno())
    # activate TWP kinematics
    self.execute("M68 E3 Q3")
    # set twp-state to 'active' (2)
    self.execute("M68 E2 Q2")

    yield INTERP_EXECUTE_FINISH
    return INTERP_OK

# Cancel an active TWP definition and reset the parameters to zero
# Note: To avoid that this python code is run prematurely by the read ahead we need a quebuster at the beginning but
# because we need self.execute() to switch the WCS properly this remap needs to be called from
# an ngc that contains a quebuster before calling this code
def g69_core(self):
    log.debug('Entering: %s', sys._getframe(  ).f_code.co_name)
    global twp_flag, saved_work_offset_number, saved_work_offset
    if self.task == 0: # ignore the preview interpreter
        yield INTERP_EXECUTE_FINISH
        return INTERP_OK
    log.info('G69 called')
    # reset the twp parameters
    reset_twp_params()
    gui_update_twp()
    # set twp-state to 'undefined' (0)
    self.execute("M68 E2 Q0")
    yield INTERP_EXECUTE_FINISH
    return INTERP_OK
