#   This is a python remap for LinuxCNC implementing 'Tilted Work Plane'
#   Copyright 2023 David Mueller <mueller_david@photmail.com>,
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
import sys
import traceback
import numpy as np
from math import sin, cos, pi
from interpreter import *
import emccanon
from util import lineno
import hal

import linuxcnc

from tkinter import *
from tkinter import messagebox


s = linuxcnc.stat()

throw_exceptions = 1  # raises InterpreterException if execute() or read() fail
# this transformation matrix describes the transformation of the world_coordinate system
# to the  current twp_coordinate system
# we start with the identity matrix (ie the twp is equal to the world coordinates)
twp_matrix = np.asmatrix(np.identity(4))
# container to store the current work offset during twp operations
saved_work_offset_number = False
saved_work_offset = [0, 0, 0]


# returns 4x4 transformation matrix for given angles and 4x4 input matrix
def kins_tool_transformation(angle_b, angle_c, matrix_in, direction='fwd'):
    b = angle_b
    tc = angle_c
    T_in = matrix_in
    # substitutions as used in the xyzbu_st_man kinematic
    Sb = sin(b)
    Cb = cos(b)
    Stc = sin(tc)
    Ctc = cos(tc)
    # define rotation matrices
    Rb = np.matrix(
        [[Cb, 0, Sb, 0], [0, 1, 0, 0], [-Sb, 0, Cb, 0], [0, 0, 0, 1]]
    )
    # virtual rotation around tool-z to orient tool-x and -y
    Rtc = np.matrix(
        [[Ctc, -Stc, 0, 0], [Stc, Ctc, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    )
    matrix_tool_fwd = Rb * Rtc * T_in
    matrix_tool_inv = np.transpose(Rtc) * np.transpose(Rb) * T_in
    if direction == 'fwd':
        return matrix_tool_fwd
    elif direction == 'inv':
        return matrix_tool_inv
    else:
        return 0


# forms a 4x4 transformation matrix from a given 1x3 point vector [x,y,z]
def point_to_matrix(point):
    # print("pointTomatrix: ", point)
    # start with a 4x4 identity matrix and add the point vector to the 4th column
    matrix = np.identity(4)
    [matrix[0, 3], matrix[1, 3], matrix[2, 3]] = point
    matrix = np.asmatrix(matrix)
    # print("pointTomatrix. \n", matrix)
    return matrix


# extracts the translation (point) vector form a given 4x4 transformation matrix
def matrix_to_point(matrix):
    point = (matrix[0, 3], matrix[1, 3], matrix[2, 3])
    return point


# define a virtual tilted-work-plane (twp) that is perpendicular to the current
# tool-orientation
def g689(self, **words):
    global b_j, twp_matrix, pre_rot, saved_work_offset_number, saved_work_offset
    print('\n')
    print('G68.2 start')
    c = self.blocks[self.remap_level]
    # parse the requested origin
    x = c.x_number if c.x_flag else 0
    y = c.y_number if c.y_flag else 0
    z = c.z_number if c.z_flag else 0
    # parse the requested inclination of the tilted work plane
    r = c.r_number if c.r_flag else 0
    twp_rotation = r * (pi / 180)
    # parse the requested rotation of tool-x around the origin
    i = c.i_number if c.i_flag else 0

    if self.task == 1:
        # collect the currently active work offset values (ie g54, g55 or other)
        s.poll()   # get current values
        # get which offset is active (g54=1 .. g59.3=9)
        n = s.g5x_index
        # It's an error it twp is already active'
        if hal.get_value('twp_helper_comp.twp-is-active-in') == 1:
            print('Error -> aborting G68.2')
            hal.set_p('twp_helper_comp.twp-error', str(1))
            yield INTERP_EXIT
        # It is an error if the active work offset system is G59 or 59.x
        if n > 5:
            print('Error -> aborting G68.2')
            hal.set_p('twp_helper_comp.twp-error', str(2))
            yield INTERP_EXIT

        # It's an error to enable TWP with G91 acitve
        elif '910' in str(s.gcodes):
            print('Error -> aborting G68.2')
            hal.set_p('twp_helper_comp.twp-error', str(3))
            yield INTERP_EXIT
        else:
            try:
                ws = Tk()
                # Tkinter way to find the screen resolution
                screen_width = ws.winfo_screenwidth()
                screen_height = ws.winfo_screenheight()
                size = tuple(int(_) for _ in ws.geometry().split('+')[0].split('x'))
                width = screen_width/2 - size[0]/2
                hight = screen_height/2 - size[1]/2
                ws.geometry("400x120+%d+%d" % (width, hight))
                ws.title('LinuxCNC')
                #ws.geometry('200x110')
                Label(ws, text=("\nTilt the Spindle to %s° and click 'Continue'\n" % i)).pack()
                button = Button(ws, text=('Continue'), padx=10, pady=5, command=ws.destroy)
                button.pack(pady=5)
                ws.bind_all("<Return>", lambda event: button.invoke())
                ws.mainloop()
            except Exception as e:
                print('show_operator_message: ', e)
            # For manual spindle B rotation using a halpin we use the argument passed with G68.2
            b_j = i * (pi / 180)
            print('TWP inclination argument given: ', i)
            # publish the twp angle through the nonrt_component
            hal.set_p('twp_helper_comp.twp-i-in', str(i))
            # goes to 'xyzbu_st_man_gui.rotary_b'
            # the twp rotation matrix is created using matrix rotation of an unrotated unit matrix
            # rotation order is around z-axis first (twp-rotation) and then around the Y axis (twp inclination)
            # this depends on the kinematic of the machine at hand.
            start_matrix = np.asmatrix(np.identity(4))
            twp_matrix = kins_tool_transformation(
                b_j, twp_rotation, start_matrix
            )
            # put the requested origin into the twp_matrix
            twp_matrix[0, 3] = x
            twp_matrix[1, 3] = y
            twp_matrix[2, 3] = z
            print('Built twp-matrix: \n', twp_matrix)
            # send the twp offset values to the rt-kinematic
            print(
                'setting twp-offset in rt-kinematic component to: \n',
                twp_matrix[0:3, 3],
            )
            # pre_rotation equal to the requested twp-rotation
            pre_rot = twp_rotation
            hal.set_p('twp_helper_comp.twp-r-in', str(twp_rotation * 180 / pi))
            # publish untransformed twp offset for the gui
            hal.set_p('twp_helper_comp.twp-x-in', str(x))
            hal.set_p('twp_helper_comp.twp-y-in', str(y))
            hal.set_p('twp_helper_comp.twp-z-in', str(z))
            saved_work_offset_number = n
            # get the active work offset values for x,y,z
            saved_work_offset = s.g5x_offset
            saved_xy_rotation = s.rotation_xy
            # expose the saved work offset to be used in the vismach visualization 
            hal.set_p('twp_helper_comp.saved-offset-index', str(n))
            hal.set_p('twp_helper_comp.saved-offset-x', str(saved_work_offset[0]))
            hal.set_p('twp_helper_comp.saved-offset-y', str(saved_work_offset[1]))
            hal.set_p('twp_helper_comp.saved-offset-z', str(saved_work_offset[2]))
            hal.set_p('twp_helper_comp.saved-xy-rot', str(saved_xy_rotation))
            # calculate the work offset with the requested twp-offset in tool-coords
            Q = matrix_to_point(
                kins_tool_transformation(
                    b_j,
                    twp_rotation,
                    point_to_matrix(
                        (
                            saved_work_offset[0]+x,
                            saved_work_offset[1]+y,
                            saved_work_offset[2]+z,
                        )
                    ),
                    'inv',
                )
            )
            print(
                'Setting transformed work-offsets for tool-kins in G59: ',
                Q,
            )

            self.execute(
                'G10 L2 P6 X%f Y%f Z%f R%f' % (Q[0], Q[1], Q[2], saved_xy_rotation), lineno()
            )
            # switch to G59 work offsets
            self.execute('G59', lineno())
            # NOTE: No 'self.execute(..)' command can be used after 'yield INTERP_EXECUTE_FINISH'
            yield INTERP_EXECUTE_FINISH
            # same as above but for twp-offsets
            current_twp_offset = [
                twp_matrix[0, 3],
                twp_matrix[1, 3],
                twp_matrix[2, 3],
            ]
            P = matrix_to_point(
                kins_tool_transformation(
                    b_j,
                    twp_rotation,
                    point_to_matrix(current_twp_offset),
                    'inv',
                )
            )
            print('Setting transformed twp-offset in tool-kins: ', P)
            hal.set_p("xyzbu_st_man_kins.pre-rot",str(twp_rotation))
            hal.set_p('xyzbu_st_man_kins.man-b-rot', str(i))

            yield INTERP_EXECUTE_FINISH
            # switch to tool kins
            print('G68.2: Activating TOOL kins')
            hal.set_p('motion.switchkins-type', str(2))
            yield INTERP_EXECUTE_FINISH
            hal.set_p('twp_helper_comp.twp-is-active-in', str(1))

    elif self.task == 0: # only used for the preview if g68.2 is used in gcode 
        self.execute('M101 p1 q' + str(x), lineno())
        self.execute('M101 p2 q' + str(y), lineno())
        self.execute('M101 p3 q' + str(z), lineno())
        self.execute('M101 p4 q' + str(r), lineno())
        self.execute('M101 p5 q' + str(i), lineno())
    yield INTERP_EXECUTE_FINISH
    return INTERP_OK


# cancel current twp, return to IDENTITY kins and restore the work- and twp-offsets
def g699(self):
    global b_j, twp_matrix, pre_rot, saved_work_offset_number, saved_work_offset
    print('\n')
    print('G69 start')
    # switch to identity kins
    print('G69: Activating IDENTITY kins')
    hal.set_p('motion.switchkins-type', str(0))
    hal.set_p('twp_helper_comp.twp-is-active-in', str(0))
    # If G69 is used w/o prior use of G68.2 we have no saved work offset to reset to
    if self.task == 1 and saved_work_offset_number is not False:
        try:
            ws = Tk()
            # Tkinter way to find the screen resolution
            screen_width = ws.winfo_screenwidth()
            screen_height = ws.winfo_screenheight()
            size = tuple(int(_) for _ in ws.geometry().split('+')[0].split('x'))
            x = screen_width/2 - size[0]/2
            y = screen_height/2 - size[1]/2
            ws.geometry("400x120+%d+%d" % (x, y))
            ws.title('LinuxCNC')
            #ws.geometry('200x110')
            Label(ws, text=("\nTilt the Spindle to 0.0° and click 'Continue'\n")).pack()
            b = Button(ws, text=('Continue'), padx=10, pady=5, command=ws.destroy)
            b.pack(pady=5)
            ws.bind_all("<Return>", lambda event: b.invoke())
            ws.mainloop()
        except Exception as e:
            print('show_operator_message: ', e)
        n = int(saved_work_offset_number)
        g = 53 + n
        print('G69 restoring Work offset to G' + str(g))
        self.execute('G' + str(g), lineno())

    elif self.task == 0: # only used for the preview if g69 is used in gcode 
        self.execute('M101 p0', lineno())
    # NOTE: after 'yield INTERP_EXECUTE_FINISH' we cannot use 'self.execute(..) anymore
    yield INTERP_EXECUTE_FINISH
    # we start with the unit matrix (ie the twp is equal to the world coordinates)
    twp_matrix = np.asmatrix(np.identity(4))
    print('Reseting TWP-matrix to: \n', twp_matrix)
    b_j = 0
    hal.set_p('twp_helper_comp.twp-i-in', str(0))
    hal.set_p('twp_helper_comp.twp-x-in', str(0))
    hal.set_p('twp_helper_comp.twp-y-in', str(0))
    hal.set_p('twp_helper_comp.twp-z-in', str(0))
    hal.set_p('twp_helper_comp.twp-r-in', str(0))
    yield INTERP_EXECUTE_FINISH
    # reset the trap for G69 without prior G68.2
    saved_work_offset_number = False

    return INTERP_OK



def m428raw(self):
    if self.task == 0:
        print('TASK = 0 -> ignoring M428')
        yield INTERP_OK
    else:
        # switch to identity kins
        print('M428: Activating IDENTITY kins')
        hal.set_p('motion.switchkins-type', str(0))
        yield INTERP_EXECUTE_FINISH
    return INTERP_OK


def m429raw(self):
    if self.task == 0:
        print('TASK = 0 -> ignoring M429')
        yield INTERP_OK
    else:
        # switch to identity kins
        print('M429: Activating TOOL kins')
        hal.set_p('motion.switchkins-type', str(2))
        yield INTERP_EXECUTE_FINISH
    return INTERP_OK









