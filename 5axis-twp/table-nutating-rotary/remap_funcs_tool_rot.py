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
# spindle primary joint
joint_letter_primary = 'B'
# spindle secondary joint (ie the one closer to the tool)
joint_letter_secondary = 'C'
# get the MIN/MAX limits of the respective rotary joint letters
category = 'AXIS_' +  joint_letter_primary
primary_min_limit = float(config[category]['MIN_LIMIT'])
primary_max_limit = float(config[category]['MAX_LIMIT'])
category = 'AXIS_' +  joint_letter_secondary
secondary_min_limit = float(config[category]['MIN_LIMIT'])
secondary_max_limit = float(config[category]['MAX_LIMIT'])

## CONNECTIONS TO THE KINEMATIC COMPONENT
# get the name of the kinematic component (this seems to ingest also the next line)
kins_comp = (config['KINS']['KINEMATICS']).partition('\n')[0]
kins_nutation_angle = kins_comp + '.nut-angle'
kins_table_angle = kins_comp + '.table-angle'
kins_virtual_rotation = kins_comp + '.virtual-rot'
kins_offset_x  = kins_comp + '.x-offset'
kins_offset_z  = kins_comp + '.z-offset'
kins_rot_point_abs_x  = kins_comp + '.x-rot-point'
kins_rot_point_abs_y  = kins_comp + '.y-rot-point'
kins_rot_point_abs_z  = kins_comp + '.z-rot-point'


# defines the kinematic model for (world <-> tool) coordinates of the machine at hand
# returns 4x4 transformation matrix for given angles and 4x4 input matrix
# NOTE: these matrices must be the same as the ones used to derive the kinematic model
def kins_calc_transformation_matrix(theta_1, theta_2, virtual_rot, matrix_in, direction='fwd'):
    global kins_nutation_angle
    T_in = matrix_in
    # Note that on a real machine setup the x axis retains its orientation regardless of table-angle
    # however in the kinematic model it rotates with the table angle hence we compensate this by
    # subtracting the table_angle from the passed secondary angle as explained in the documentation
    theta_2 = theta_2 - radians(hal.get_value(kins_table_angle))
    # Define 4x4 transformation for virtual rotation around z to orient tool-x and -y
    Swc = sin(virtual_rot)
    Cwc = cos(virtual_rot)
    Rwc=np.matrix([[ Cwc, -Swc, 0, 0],
                   [ Swc,  Cwc, 0, 0],
                   [ 0 ,  0 ,   1, 0],
                   [ 0,   0 ,   0, 1]])

    ## Define 4x4 transformation for the primary joint
    # get the basic 3x3 rotation matrix (returns array)
    Rp = Ry(-theta_1)
    # add fourth column on the right
    Rp = np.hstack((Rp, [[0],[0],[0]]))
    # expand to 4x4 array and make into a matrix
    row_4 = [0,0,0,1]
    Rp = np.vstack((Rp, row_4))
    Rp = np.asmatrix(Rp)

    ## Define 4x4 transformation matrix for the secondary joint
    # get the basic 3x3 rotation matrix (returns array)
    Rs = Rz(theta_2)
    # add fourth column on the right
    Rs = np.hstack((Rs, [[0],[0],[0]]))
    # expand to 4x4 array and make into a matrix
    row_4 = [0,0,0,1]
    Rs = np.vstack((Rs, row_4))
    Rs = np.asmatrix(Rs)

    Dx = hal.get_value(kins_offset_x)
    Dz = hal.get_value(kins_offset_z)
    # define the forward transformation for the geometric offsets
    To=np.matrix([[ 1, 0, 0,  Dx],
                  [ 0, 1, 0,  0 ],
                  [ 0, 0, 1,  Dz],
                  [ 0, 0, 0,  1 ]])
    # define the inverse transformation for the geometric offsets
    Tio=np.matrix([[ 1, 0, 0,  -Dx],
                   [ 0, 1, 0,  0 ],
                   [ 0, 0, 1,  -Dz],
                   [ 0, 0, 0,  1 ]])

    # Additional definitions for nutating joint
    u = radians(hal.get_value(kins_table_angle))
    Su = sin(u)
    Cu = cos(u)
    v = radians(hal.get_value(kins_nutation_angle))
    Sv = sin(v)
    Cv = cos(v)
    Sp = sin(theta_1)
    Cp = cos(theta_1)
    r = Cp + Sv*Sv*(1-Cp)
    s = Cp + Cv*Cv*(1-Cp)
    t = -Sv*Cv*(1-Cp)
    # define rotation matrix for the secondary spindle joint
    Rp=np.matrix([[  Cp*Cu + Cv*Sp*Su,  Cu*Cv*Sp - Cp*Su,  Sv*Sp, 0],
                  [ -Cu*Cv*Sp + Su*r ,  Cv*Sp*Su + Cu*r ,      t, 0],
                  [ -Cu*Sv*Sp + Su*t ,  Su*Sv*Sp + Cu*t ,      s, 0],
                  [          0       ,      0           ,      0, 1]])

    # calculate the transformation matrix for the forward tool kinematic
    matrix_tool_fwd = Rs*To*Rp*Rwc*T_in
    # calculate the transformation matrix for the inverse tool kinematic
    matrix_tool_inv = np.transpose(Rwc)*np.transpose(Rp)*Tio*np.transpose(Rs)*T_in
    if direction == 'fwd':
        #log.debug("matrix tool fwd: \n", matrix_tool_fwd)
        #log.debug("inv would have been: \n", matrix_tool_inv)
        return matrix_tool_fwd
    elif direction == 'inv':
        #log.debug("matrix tool inv: \n", matrix_tool_inv)
        #log.debug("fwd would have been: \n", matrix_tool_fwd)
        return matrix_tool_inv
    else:
        return 0


# calculates the secondary joint position for a given tool-vector
# secondary being the joint closest to the tool
# Note: this uses functions derived from the custom kinematic
def kins_calc_secondary(z_vector_req, x_vector_req, theta_1_list=[]):
    global secondary_min_limit, secondary_max_limit
    global kins_nutation_angle
    epsilon = 0.000001
    theta_2_list=[]
    (Kzx, Kzy, Kzz) = (z_vector_req[0], z_vector_req[1], z_vector_req[2])
    (Kxx, Kxy, Kxz) = (x_vector_req[0], x_vector_req[1], x_vector_req[2])
    u = radians(hal.get_value(kins_table_angle))
    Su = sin(u)
    Cu = cos(u)
    v = radians(hal.get_value(kins_nutation_angle))
    Sv = sin(v)
    Cv = cos(v)
    for i in range(len(theta_1_list)):
        theta_1 = theta_1_list[i]
        Sp  = sin(theta_1)
        Cp  = cos(theta_1)
        if Cp == 1:
            theta_2 = atan2(Kxy, Kxx)
            theta_2_list.append(theta_2)
            theta_2_list.append(-theta_2)
        else:
            r = Cp + Sv*Sv*(1-Cp)
            s = Cp + Cv*Cv*(1-Cp)
            t = -Sv*Cv*(1-Cp)
            for i in range(len(theta_1_list)):
                theta_1 = theta_1_list[i]
                Sp  = sin(theta_1)
                Cp  = cos(theta_1)
                a = t
                b = Sv*Sp
                c = Kzy
                r = sqrt(a*a + b*b)
                theta_minus_phi= [acos(c/r), -acos(c/r)]
                phi = atan2(b,a)
                # we also need to check for limit violations
                for result in theta_minus_phi:
                    for theta in [result + phi, result - phi]:
                        if theta > secondary_min_limit and theta < secondary_max_limit:
                            theta_2_list.append(theta + u)

    return theta_2_list


# calculates the primary joint position for a given tool-vector
# Note: this uses functions derived from the custom kinematic
def kins_calc_primary(z_vector_req, x_vector_req, theta_2_list=[]):
    global primary_min_limit, primary_max_limit
    global kins_nutation_angle
    epsilon = 0.000001
    theta_1_list=[]
    (Kzx, Kzy, Kzz) = (z_vector_req[0], z_vector_req[1], z_vector_req[2])
    u = radians(hal.get_value(kins_table_angle))
    Su = sin(u)
    Cu = cos(u)
    v = radians(hal.get_value(kins_nutation_angle))
    Sv = sin(v)
    Cv = cos(v)
    # This kinematics nutation angle restricts the negative range of Kzz
    if Kzz < 2*Cv*Cv - 1:
        print('remap_funcs: Requested orientation not reachable with the current nutation angle!')
    else:
        u = radians(hal.get_value(kins_table_angle))
        Su = sin(u)
        Cu = cos(u)
        v = radians(hal.get_value(kins_nutation_angle))
        Sv = sin(v)
        Cv = cos(v)
        theta_1 = acos((Kzz - Cv*Cv) / (1-Cv*Cv))
        # since we are using acos() we really have two solutions theta_1 and -theta_1
        # we also need to check for limit violations
        for theta in [theta_1, -theta_1]:
            if degrees(theta) > primary_min_limit and degrees(theta) < primary_max_limit:
                theta_1_list.append(theta)

    return theta_1_list


# define the order in which the joint angles need to be calculated
def kins_calc_possible_joint_angles(z_vector_req, x_vector_req):
    try:
        theta_1_calcd = kins_calc_primary(z_vector_req, x_vector_req)
    except Exception as error:
        print('kins_calc_jnt_angles, kins_calc_primary, %s', error)
    try:
        theta_2_calcd = kins_calc_secondary(z_vector_req, x_vector_req, theta_1_calcd)
    except Exception as error:
        print('kins_calc_jnt_angles, kins_calc_secondary, %s', error)
    return (theta_1_calcd, theta_2_calcd)


# calculate the transformed work offset used after 53.n
def kins_calc_transformed_work_offset(current_offset, twp_offset, theta_1, theta_2, virtual_rot):
    kins_rot_point_abs = (hal.get_value(kins_rot_point_abs_x),
                          hal.get_value(kins_rot_point_abs_y),
                          hal.get_value(kins_rot_point_abs_z))
    work_offset_from_cntr_rot_table = tuple(np.subtract(current_offset, kins_rot_point_abs))
    tot_offset_pre = tuple(np.add(work_offset_from_cntr_rot_table, twp_offset))
    P = matrix_to_point(kins_calc_transformation_matrix(theta_1,
                                                        theta_2,
                                                        virtual_rot,
                                                        point_to_matrix(tot_offset_pre),
                                                        'inv'))
    transformed_offset = tuple(np.add((P[0], P[1], P[2]), kins_rot_point_abs))
    return transformed_offset


# pass required values to the kinematics component
def kins_set_values(theta_1, theta_2, virtual_rot):
    hal.set_p(kins_virtual_rotation, str(-virtual_rot))


# oriented x-vector should point in the machine x direction
def kins_calc_virtual_rot_get_values(x_vector_requested, z_vector_requested, twp_matrix):
    x_vector_requested = (1,0,0)
    z_vector_requested = None
    matrix_in = twp_matrix
    return (x_vector_requested, z_vector_requested, matrix_in)


# This just returns zero for a kinematic with pure work side rotation
def kins_calc_virtual_rot_for_g683(theta_1, theta_2):
    return 0


# If the operator has requested a rotation by passing an R word in the 68.n command we need to 
# create a rotation matrix that represents a rotation around the Z-axis of the TWP plane
def kins_calc_twp_origin_rot_matrix(r):
    # we use z rotation to create the rotation matrix for the requested origin rotation
    twp_origin_rot_matrix = Rz(radians(r))

    return twp_origin_rot_matrix


# This returns which transformation to use when checking calculated angles
# and when calculating the twp_matrix for G68.3
def kins_calc_transformation_get_direction():
    return 'fwd'


# returns the pin name for the virtual rotation in the kinematics component
def kins_get_current_virtual_rot():
    current_virtual_rot = hal.get_value(kins_virtual_rotation)
    return current_virtual_rot







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

