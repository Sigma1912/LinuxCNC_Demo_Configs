# This is a python remap for LinuxCNC implementing 'Tilted Work Plane'
# G68.2, G68.3, G68.4 and related Gcodes G53.1, G53.3, G53.6, G69
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
import numpy as np
from math import sin,cos,tan,asin,acos,atan,atan2,sqrt,pi,degrees,radians,fabs
import hal


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

## SPINDLE ROTARY JOINT LETTERS
# table joint
joint_letter_primary = (config['TWP']['PRIMARY']).capitalize()
# spindle joint (ie the one closer to the tool)
joint_letter_secondary = (config['TWP']['SECONDARY']).capitalize()
# get the MIN/MAX limits of the respective rotary joint letters
category = 'AXIS_' +  joint_letter_primary
primary_min_limit = float(config[category]['MIN_LIMIT'])
primary_max_limit = float(config[category]['MAX_LIMIT'])
category = 'AXIS_' +  joint_letter_secondary
secondary_min_limit = float(config[category]['MIN_LIMIT'])
secondary_max_limit = float(config[category]['MAX_LIMIT'])

## CONNECTIONS TO THE KINEMATIC COMPONENT
# get the name of the kinematic component (this seems to ingest also the next line)
kins_comp = 'mazak-integrex-200y-kins'
kins_virtual_rotation = kins_comp + '.prerot-angle'
kins_offset_z  = kins_comp + '.z-offset'
kins_rot_axis_abs_x  = kins_comp + '.x-rot-axis'
kins_rot_axis_abs_y  = kins_comp + '.y-rot-axis'
kins_primary_rotation = kins_comp + '.work-angle'
kins_secondary_rotation = kins_comp + '.b-orientation'

# defines the kinematic model for (world <-> tool) coordinates of the machine at hand
# returns 4x4 transformation matrix for given angles and 4x4 input matrix
# NOTE: these matrices must be the same as the ones used to derive the kinematic model
def kins_calc_transformation_matrix(theta_1, theta_2, virtual_rot, matrix_in, direction='fwd'): # expects radians
    theta_1=0
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

    Dx = hal.get_value(kins_offset_z)

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


# calculates the spindle joint position for a given Z-vector
# Note: this uses functions derived from the custom kinematic
def kins_calc_spindle(log, z_vector_req, x_vector_req):
    global secondary_min_limit, secondary_max_limit
    global kins_nutation_angle
    epsilon = 0.000001
    theta_sec_list=[]
    (Kzx, Kzy, Kzz) = (z_vector_req[0], z_vector_req[1], z_vector_req[2])
    theta_sec = acos(Kzz)
    # since we are using acos() we really have two solutions theta_1 and -theta_1
    # we also need to check for limit violations
    for theta in [theta_sec, -theta_sec]:
        log.debug(f'    Checking possible secondary angle {degrees(theta):.4f}° for limit violations.')
        if degrees(theta) > secondary_min_limit and degrees(theta) < secondary_max_limit:
                theta_sec_list.append(theta)
    return theta_sec_list # returns radians


# calculates the primary joint position for a given tool-vector
# Note: this uses functions derived from the custom kinematic
def kins_calc_table(log, z_vector_req, x_vector_req, theta_sec_list):
    global primary_min_limit, primary_max_limit
    global kins_nutation_angle
    epsilon = 0.000001
    if theta_sec_list == None:
        return (None, None)
    theta_prim_list=[]
    (Kzx, Kzy, Kzz) = (z_vector_req[0], z_vector_req[1], z_vector_req[2])
    for theta_sec in theta_sec_list:
        # This kinematics has infinite solutions if the spindle is vertical so we set the table to 0°
        if theta_sec == 0:
            return [0]
        else:
            Ss = sin(theta_sec)
            theta_prim = acos(Kzx/Ss)
            # since we are using asin() we really have two solutions theta_1 and pi-theta_2
            for theta in [theta_prim, -theta_prim]:
                log.debug(f'    Checking possible primary angle {degrees(theta):.4f}° for limit violations.')
                if degrees(theta) > primary_min_limit and degrees(theta) < primary_max_limit:
                    theta_prim_list.append(theta)
    return theta_prim_list # returns radians


# define the order in which the joint angles need to be calculated
def kins_calc_possible_joint_angles(log, z_vector_req, x_vector_req):
    try:
        theta_2_calcd = kins_calc_spindle(log, z_vector_req, x_vector_req)
    except Exception as error:
        log.error('kins_calc_jnt_angles, kins_calc_spindle, %s', error)
    if theta_2_calcd == None:
        return (None, None)
    try:
        theta_1_calcd = kins_calc_table(log, z_vector_req, x_vector_req, theta_2_calcd)
    except Exception as error:
        log.error('kins_calc_jnt_angles, kins_calc_table, %s', error)
    return (theta_1_calcd, theta_2_calcd) # returns radians


# calculate the transformed work offset used after 53.n
def kins_calc_transformed_work_offset(current_offset, twp_offset, theta_1, theta_2, virtual_rot):
    # in this kinematic we need to keep in mind that we have rotations in the spindle and the table
    current_offset_incl_twp = tuple(np.add(current_offset, twp_offset))
    kins_rot_axis_abs = (hal.get_value(kins_rot_axis_abs_x),
                         hal.get_value(kins_rot_axis_abs_y),
                         0)
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
    #print(kins_virtual_rotation)
    hal.set_p(kins_virtual_rotation, str(virtual_rot))
    #print(kins_primary_rotation)
    hal.set_p(kins_primary_rotation, str(theta_1))
    #print(kins_secondary_rotation)
    hal.set_p(kins_secondary_rotation, str(theta_2))


# oriented x-vector should point in the machine x direction
def kins_calc_virtual_rot_get_values(x_vector_requested, z_vector_requested, twp_matrix):
    x_vector_requested = [twp_matrix[0,0],twp_matrix[1,0],twp_matrix[2,0]]
    z_vector_requested = [twp_matrix[0,2],twp_matrix[1,2],twp_matrix[2,2]]
    matrix_in = np.asmatrix(np.identity(4))
    direction = 'fwd'
    return (x_vector_requested, z_vector_requested, matrix_in, direction)


# This just returns zero for a kinematic with pure work side rotation
def kins_calc_virtual_rot_for_g683(theta_1, theta_2):
    # TODO
    return 0


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


# returns the pin name for the virtual rotation in the kinematics component
def kins_get_current_virtual_rot():
    current_virtual_rot = hal.get_value(kins_virtual_rotation)
    return current_virtual_rot # returns radians







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


# this is from 'mika-s.github.io'
# transforms a given angle to the interval of [-pi,pi]
def transform_to_pipi(input_angle):
    def truncated_remainder(dividend, divisor):
        divided_number = dividend / divisor
        divided_number = -int(-divided_number) if divided_number < 0 else int(divided_number)
        remainder = dividend - divisor * divided_number
        return remainder

    revolutions = int((input_angle + np.sign(input_angle) * pi) / (2 * pi))
    p1 = truncated_remainder(input_angle + np.sign(input_angle) * pi, 2 * pi)
    p2 = (np.sign(np.sign(input_angle)
                  + 2 * (np.sign(fabs((truncated_remainder(input_angle + pi, 2 * pi)) / (2 * pi))) - 1))) * pi
    output_angle = p1 - p2
    return output_angle


# define the basic rotation matrices, used for euler twp modes
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

