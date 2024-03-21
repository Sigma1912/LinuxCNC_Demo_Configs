#    This is a component of AXIS, a front-end for emc
#    Copyright 2004, 2005, 2006 Jeff Epler <jepler@unpythonic.net>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
import math, gcode


class Translated:
    g92_offset_x = g92_offset_y = g92_offset_z = 0
    g92_offset_a = g92_offset_b = g92_offset_c = 0
    g92_offset_u = g92_offset_v = g92_offset_w = 0
    g5x_offset_x = g5x_offset_y = g5x_offset_z = 0
    g5x_offset_a = g5x_offset_b = g5x_offset_c = 0
    g5x_offset_u = g5x_offset_v = g5x_offset_w = 0
    rotation_xy = 0

    def rotate_and_translate(self, x,y,z,a,b,c,u,v,w):
        #print('interp.py line 30 ++++++++++++++++++++++++++ g5x-x: ', self.g5x_offset_x)
        x += self.g92_offset_x
        y += self.g92_offset_y
        z += self.g92_offset_z
        a += self.g92_offset_a
        b += self.g92_offset_b
        c += self.g92_offset_c
        u += self.g92_offset_u
        v += self.g92_offset_v
        w += self.g92_offset_w

        if self.rotation_xy:
            rotx = x * self.rotation_cos - y * self.rotation_sin
            y = x * self.rotation_sin + y * self.rotation_cos
            x = rotx

        x += self.g5x_offset_x
        y += self.g5x_offset_y
        z += self.g5x_offset_z
        a += self.g5x_offset_a
        b += self.g5x_offset_b
        c += self.g5x_offset_c
        u += self.g5x_offset_u
        v += self.g5x_offset_v
        w += self.g5x_offset_w

        return (x, y, z, a, b, c, u, v, w)

    def expanded_rotate_and_translate(self, x,y,z,a,b,c,u,v,w):
        return ((x, y, z, a, b, c, u, v, w), 
                self.g92_set, self.g5x_set, self.xy_rot_set)

    def straight_traverse(self, *args):
        self.straight_traverse_translated(*self.rotate_and_translate(*args))
    def straight_feed(self, *args):
        self.straight_feed_translated(*self.rotate_and_translate(*args))
    def set_g5x_offset(self, index, x, y, z, a, b, c, u=None, v=None, w=None):
        if self.lineno == 0:
            self.g5x_set = (False, False, False, False)
            print('def set_g5x_set , init call: ',self.lineno, self.g5x_set)
        else:
            print('def set_g5x_set , index,x,y,z ', index, x,y,z)
            self.g5x_set = (index, x,y,z)
        self.g5x_index = index
        self.g5x_offset_x = x
        self.g5x_offset_y = y
        self.g5x_offset_z = z
        self.g5x_offset_a = a
        self.g5x_offset_b = b
        self.g5x_offset_c = c
        self.g5x_offset_u = u
        self.g5x_offset_v = v
        self.g5x_offset_w = w

    def set_g92_offset(self, x, y, z, a, b, c, u=None, v=None, w=None):
        if self.lineno == 0:
            self.g92_set = (False, False, False)
            print('set_g92_set: init call: ', self.lineno, self.g92_set)
        elif self.lineno == 1:
            self.g92_set = (False, False, False)
            print('set_g92_set: 2nd init call (%?): ',self.lineno, self.g92_set)
        else:
            print('set_g92_set: lineno, x,y,z: ', self.lineno, x,y,z)
            self.g92_set = (x,y,z)
        self.g92_offset_x = x
        self.g92_offset_y = y
        self.g92_offset_z = z
        self.g92_offset_a = a
        self.g92_offset_b = b
        self.g92_offset_c = c
        self.g92_offset_u = u
        self.g92_offset_v = v
        self.g92_offset_w = w

    def set_xy_rotation(self, theta):
        if self.lineno == 0:
            self.xy_rot_set = False
            print('xy_rot_set: init',self.lineno, self.xy_rot_set)
        else:
            print('xy_rot_set: lineno, theta: ',self.lineno, self.lineno, theta)
            self.xy_rot_set = theta
        self.rotation_xy = theta
        t = math.radians(theta)
        self.rotation_sin = math.sin(t)
        self.rotation_cos = math.cos(t)

class ArcsToSegmentsMixin:
    plane = 1
    arcdivision = 64

    def set_plane(self, plane):
        self.plane = plane

    def arc_feed(self, x1, y1, cx, cy, rot, z1, a, b, c, u, v, w):
        # calulate arc segments in src/emc/rs274ngc/gcodemodule.cc
        segs1 = gcode.arc_to_segments(self, x1, y1, cx, cy, rot, z1, 
                                        a, b, c, u, v, w, self.arcdivision)
        segs2 = []
        for seg in segs1:
                # remove the work offset
                seg2_x = seg[0] - self.g5x_offset_x
                seg2_y = seg[1] - self.g5x_offset_y
                seg2_z = seg[2] - self.g5x_offset_z
                if self.rotation_xy:
                    # invert the xy rotation
                    rotx =     seg2_x * self.rotation_cos + seg2_y * self.rotation_sin
                    seg2_y = - seg2_x * self.rotation_sin + seg2_y * self.rotation_cos
                    seg2_x = rotx                
                # remove the g52/g92 offset
                seg2_x -= self.g92_offset_x 
                seg2_y -= self.g92_offset_y 
                seg2_z -= self.g92_offset_z 
                segs2.append((seg2_x, seg2_y, seg2_z))
        # append the segments to the list of 'arcfeed' in glcanon.py
        self.straight_arcsegments(segs1, segs2, 
                                  self.g92_set, self.g5x_set, self.xy_rot_set)
  

class PrintCanon:
    def set_g5x_offset(self, *args):
        print("set_g5x_offset", args)

    def set_g92_offset(self, *args):
        print("set_g92_offset", args)

    def next_line(self, state):
        print("next_line", state.sequence_number)
        self.state = state

    def set_plane(self, plane):
        print("set plane", plane)

    def set_feed_rate(self, arg):
        print("set feed rate", arg)

    def comment(self, arg):
        print("#", arg)

    def straight_traverse(self, *args):
        print("straight_traverse %.4g %.4g %.4g  %.4g %.4g %.4g" % args)

    def straight_feed(self, *args):
        print("straight_feed %.4g %.4g %.4g  %.4g %.4g %.4g" % args)

    def dwell(self, arg):
        if arg < .1:
            print("dwell %f ms" % (1000 * arg))
        else:
            print("dwell %f seconds" % arg)

    def arc_feed(self, *args):
        print("arc_feed %.4g %.4g  %.4g %.4g %.4g  %.4g  %.4g %.4g %.4g" % args)

global tool_in_spindle, empty_spindle_data
tool_in_spindle = -1
empty_spindle_data = -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0

class StatMixin:
    def __init__(self, s, r):
        self.s = s
        self.tools = list(s.tool_table)
        self.random = r

    def change_tool(self, idx):
        global tool_in_spindle
        if self.random:
            self.tools[0], self.tools[idx] = self.tools[idx], self.tools[0]
            tool_in_spindle = idx
        elif idx==0:
            self.tools[0] = empty_spindle_data
        else:
            self.tools[0] = self.tools[idx]

    def get_tool(self, idx):
        global tool_in_spindle
        if idx >= 0 and idx < len(self.tools):
            if (idx == tool_in_spindle):
                return tuple(self.tools[0])
            return tuple(self.tools[idx])
        return empty_spindle_data

    def get_external_angular_units(self):
        return self.s.angular_units or 1.0

    def get_external_length_units(self):
        return self.s.linear_units or 1.0

    def get_axis_mask(self):
        return self.s.axis_mask

    def get_block_delete(self):
        return self.s.block_delete


# vim:ts=8:sts=4:et:
