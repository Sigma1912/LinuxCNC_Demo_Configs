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

from rs274 import Translated, ArcsToSegmentsMixin, OpenGLTk
from OpenGL.GL import *
from OpenGL.GLU import *
import itertools
import math
import glnav
import hershey
import linuxcnc
import array
import gcode
import os
import re
import sys
from functools import reduce

import hal
import numpy as np
from math import sin, cos, pi


def minmax(*args):
    return min(*args), max(*args)


class GLCanon(Translated, ArcsToSegmentsMixin):

    lineno = -1
    def __init__(self, colors, geometry, is_foam=0):
        # traverse list of tuples - [(line number, (start position), (end position), (tlo x, tlo y, tlo z))]
        self.traverse = []; self.traverse_append = self.traverse.append
        # feed list of tuples - [(line number, (start position), (end position), feedrate, (tlo x, tlo y, tlo z))]
        self.feed = []; self.feed_append = self.feed.append
        # arcfeed list of tuples - [(line number, (start position), (end position), feedrate, (tlo x, tlo y, tlo z))]
        self.arcfeed = []; self.arcfeed_append = self.arcfeed.append
        # dwell list - [line number, color, pos x, pos y, pos z, plane]
        self.dwells = []; self.dwells_append = self.dwells.append

        # Segment lists to be used for custom limit checking these also contain the g52_/g92_ and g5x_offsets active in that segment
        # (line number, (start position), (end position),
        #  (tlo x, tlo y, tlo z), (g92_x, g92_y, g92_z), (index, g5x_x, g5x_y, g5x_z), xy_rot, (twp_x,twp_y,twp_z,twp_r,twp_i))
        self.traverse_exp = []; self.traverse_exp_append = self.traverse_exp.append

        self.feed_exp = []; self.feed_exp_append = self.feed_exp.append

        self.arcfeed_exp = []; self.arcfeed_exp_append = self.arcfeed_exp.append

        self.dwells_exp = []; self.dwells_exp_append = self.dwells_exp.append

        self.tool_list = []
        # preview list - combines the unrotated points of the lists: self.traverse, self.feed, self.arcfeed
        self.preview_zero_rxy = []
        self.preview_rotated = []
        self.choice = None
        self.feedrate = 1
        self.lo = (0,) * 9
        self.lo_abs = (0,) * 9
        self.first_move = True
        self.geometry = geometry
        # min and max extents - the largest bounding box around the currently displayed preview
        # bounding box is parallel to the machine axes
        self.min_extents = [9e99,9e99,9e99]
        self.max_extents = [-9e99,-9e99,-9e99]
        self.min_extents_notool = [9e99,9e99,9e99]
        self.max_extents_notool = [-9e99,-9e99,-9e99]

        self.colors = colors
        self.in_arc = 0
        self.xo = self.yo = self.zo = self.ao = self.bo = self.co = self.uo = self.vo = self.wo = 0
        self.dwell_time = 0
        self.suppress = 0
        self.g92_offset_x = 0.0
        self.g92_offset_y = 0.0
        self.g92_offset_z = 0.0
        self.g92_offset_a = 0.0
        self.g92_offset_b = 0.0
        self.g92_offset_c = 0.0
        self.g92_offset_u = 0.0
        self.g92_offset_v = 0.0
        self.g92_offset_w = 0.0
        self.g5x_index = 1
        self.g5x_offset_x = 0.0
        self.g5x_offset_y = 0.0
        self.g5x_offset_z = 0.0
        self.g5x_offset_a = 0.0
        self.g5x_offset_b = 0.0
        self.g5x_offset_c = 0.0
        self.g5x_offset_u = 0.0
        self.g5x_offset_v = 0.0
        self.g5x_offset_w = 0.0
        self.is_foam = is_foam
        self.foam_z = 0
        self.foam_w = 1.5
        self.notify = 0
        self.notify_message = ""
        self.highlight_line = None
        (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i) = (False, False, False, False, False)

    def get_foam_z(self):
        if self.canon: return self.canon.foam_z
        return 0

    def get_foam_w(self):
        if self.canon: return self.canon.foam_w
        return 1.5

    def comment(self, arg):
        if arg.startswith("AXIS,") or arg.startswith("PREVIEW,"):
            parts = arg.split(",")
            command = parts[1]
            if command == "stop": raise KeyboardInterrupt
            if command == "hide": self.suppress += 1
            if command == "show": self.suppress -= 1
            if command == "XY_Z_POS":
                if len(parts) > 2 :
                    try:
                        self.foam_z = float(parts[2])
                        if 210 in self.state.gcodes:
                            self.foam_z = self.foam_z / 25.4
                    except:
                        self.foam_z = 5.0/25.4
            if command == "UV_Z_POS":
                if len(parts) > 2 :
                    try:
                        self.foam_w = float(parts[2])
                        if 210 in self.state.gcodes:
                            self.foam_w = self.foam_w / 25.4
                    except:
                        self.foam_w = 30.0
            if command == "notify":
                self.notify = self.notify + 1
                self.notify_message = "(AXIS,notify):" + str(self.notify)
                if len(parts) > 2:
                    if len(parts[2]): self.notify_message = parts[2]

    def message(self, message): pass

    def check_abort(self): pass

    def next_line(self, st):
        self.state = st
        self.lineno = self.state.sequence_number

    def draw_lines(self, lines, for_selection, j=0, geometry=None):
        return linuxcnc.draw_lines(geometry or self.geometry, lines, for_selection)

    def colored_lines(self, color, lines, for_selection, j=0):
        if self.is_foam:
            if not for_selection:
                self.color_with_alpha(color + "_xy")
            glPushMatrix()
            glTranslatef(0, 0, self.foam_z)
            self.draw_lines(lines, for_selection, 2*j, 'XY')
            glPopMatrix()
            if not for_selection:
                self.color_with_alpha(color + "_uv")
            glPushMatrix()
            glTranslatef(0, 0, self.foam_w)
            self.draw_lines(lines, for_selection, 2*j+len(lines), 'UV')
            glPopMatrix()
        else:
            if not for_selection:
                self.color_with_alpha(color)
            self.draw_lines(lines, for_selection, j)

    def draw_dwells(self, dwells, alpha, for_selection, j0=0):
        return linuxcnc.draw_dwells(self.geometry, dwells, alpha, for_selection, self.is_lathe())

    def calc_extents(self):
        # in the event of a "blank" gcode file (M2 only for example) this sets each of the extents to [0,0,0]
        # to prevent passing the very large [9e99,9e99,9e99] values and populating the gcode properties with
        # unusably large values. Some screens use the extents information to set the view distance so 0 values are preferred.
        if not self.arcfeed and not self.feed and not self.traverse:
            self.min_extents = \
            self.max_extents = \
            self.min_extents_notool = \
            self.max_extents_notool = \
            self.min_extents_zero_rxy = \
            self.max_extents_zero_rxy = \
            self.min_extents_notool_zero_rxy = \
            self.max_extents_notool_zero_rxy = \
            self.min_extents_rotated = \
            self.max_extents_rotated = [0,0,0]
            return
        # we need to remove the g5x_ and g92_offset from all the point entries because 'gcode.calc_extents' expects only 5 arguments
        GLCanon.traverse = self.traverse 
        GLCanon.feed = self.feed 
        GLCanon.arcfeed = self.arcfeed

        GLCanon.traverse_exp = self.traverse_exp 
        GLCanon.feed_exp = self.feed_exp 
        GLCanon.arcfeed_exp = self.arcfeed_exp

        arcfeed = []
        for tuple in self.arcfeed:
            tuple = tuple[:5]
            arcfeed.append(tuple)
        self.arcfeed = arcfeed

        feed = []
        for tuple in self.feed:
            tuple = tuple[:5]
            feed.append(tuple)        
        self.feed = feed

        traverse = []
        for tuple in self.traverse:
            tuple = tuple[:5]
            traverse.append(tuple)
        self.traverse = traverse
        
        self.min_extents, self.max_extents, self.min_extents_notool, self.max_extents_notool = gcode.calc_extents(self.arcfeed, self.feed, self.traverse)
        #self.rotate_preview()
        self.min_extents_zero_rxy, self.max_extents_zero_rxy, self.min_extents_notool_rotated, self.max_extents_notool_rotated = (0,0,0,0)
     
    def tool_offset(self, xo, yo, zo, ao, bo, co, uo, vo, wo):
        # if the gcode does not start with a tool change we want to later (ie in GlCanonDraw) inject 
        # the offsets of the current tool in the spindle
        # default behavior was to preset the tool offset to (0,0,0)
        # if the gcode uses percent signs the tool and g92 offsets are set to (0.0,0,0,0,0) on the start
        if self.lineno == 0 :
            self.xo = False
            self.yo = False
            self.zo = False
            print('tool_offset , init call :',self.lineno, self.xo, self.yo, self.zo)
            return
        elif self.lineno == 1 and xo == 0 and yo ==0 and zo == 0 :
            self.xo = False
            self.yo = False
            self.zo = False
            print('tool_offset , 2nd init call (%?) :',self.lineno, self.xo, self.yo, self.zo)
            return
        print('def tool_offset,self.lineno, , xo, yo, zo: ',self.lineno,  xo, yo, zo)
        #self.first_move = True
        x, y, z, a, b, c, u, v, w = self.lo
        self.lo = (x - xo + self.xo, y - yo + self.yo, z - zo + self.zo,
                   a - ao + self.ao, b - bo + self.bo, c - bo + self.bo,
                   u - uo + self.uo, v - vo + self.vo, w - wo + self.wo)
        self.xo = xo
        self.yo = yo
        self.zo = zo
        self.so = ao
        self.bo = bo
        self.co = co
        self.uo = uo
        self.vo = vo
        self.wo = wo

    def set_spindle_rate(self, arg): pass
    def set_feed_rate(self, arg): self.feedrate = arg / 60.
    def select_plane(self, arg): pass

    def change_tool(self, arg):
        self.first_move = True
        try:
            self.tool_list.append(arg)
        except Exception as e:
            print(e)

    def straight_traverse(self, x,y,z, a,b,c, u,v,w):
        if self.suppress > 0: return
        l = self.rotate_and_translate(x,y,z,a,b,c,u,v,w)
        #print('st l:',self.lineno, l)
        l_exp = self.expanded_rotate_and_translate(x,y,z,a,b,c,u,v,w)
        #print('st self.lineno, l_exp:',self.lineno, l_exp)
        if not self.first_move:
            self.traverse_append((self.lineno,
                                  self.lo, l,
                                 (self.xo, self.yo, self.zo),
                                 (self.g92_offset_x, self.g92_offset_y, self.g92_offset_z), 
                                 (self.g5x_offset_x, self.g5x_offset_y, self.g5x_offset_z)))
            self.traverse_exp_append((self.lineno, self.lo_abs,
                                     l_exp[0], l_exp[1], l_exp[2], l_exp[3],
                                     (self.xo, self.yo, self.zo),
                                     (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))
        # first move in program gets start coordinates = end coordinates as we are not interested 
        # in the point from where the program is started
        else: 
            self.traverse_append((self.lineno, l, l, 
                                 (self.xo, self.yo, self.zo),
                                 (self.g92_offset_x, self.g92_offset_y, self.g92_offset_z),              
                                 (self.g5x_offset_x, self.g5x_offset_y, self.g5x_offset_z)))
            self.traverse_exp_append((self.lineno, l_exp[0], 
                                     l_exp[0], l_exp[1], l_exp[2], l_exp[3], 
                                     (self.xo, self.yo, self.zo),
                                     (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))
        self.first_move = False
        self.lo = l
        self.lo_abs = l_exp[0]
        #print('**GLCanon.traverse_exp: ', self.traverse_exp)

    def rigid_tap(self, x, y, z): #TODO This is currently untested
        if self.suppress > 0: return
        self.first_move = False
        l = self.rotate_and_translate(x,y,z,0,0,0,0,0,0)[:3]
        l_exp = self.expanded_rotate_and_translate(x,y,z,0,0,0,0,0,0)
        l += (self.lo[3], self.lo[4], self.lo[5],
               self.lo[6], self.lo[7], self.lo[8])
        self.feed_append((self.lineno, self.lo, l, self.feedrate, (self.xo, self.yo, self.zo)))
        #self.dwells_append((self.lineno, self.colors['dwell'], x + self.offset_x, y + self.offset_y, z + self.offset_z, 0))
        self.feed_append((self.lineno, l, self.lo, self.feedrate, (self.xo, self.yo, self.zo)))

        self.feed_exp_append((self.lineno, self.lo_abs, 
                             l_exp[0], l_exp[1], l_exp[2], l_exp[3], 
                             (self.xo, self.yo, self.zo),
                             (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))
        self.feed_exp_append((self.lineno, l_exp[0],
                              self.lo_abs, l_exp[1], l_exp[2], l_exp[3], 
                             (self.xo, self.yo, self.zo),
                             (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))

    def straight_feed(self, x,y,z, a,b,c, u,v,w):
        if self.suppress > 0: return
        self.first_move = False
        l = self.rotate_and_translate(x,y,z,a,b,c,u,v,w)
        l_exp = self.expanded_rotate_and_translate(x,y,z,a,b,c,u,v,w)
        #print('sf self.lineno, l_exp:',self.lineno, l_exp)
        self.feed_append((self.lineno, self.lo, l, self.feedrate,
                         (self.xo, self.yo, self.zo),
                         (self.g92_offset_x, self.g92_offset_y, self.g92_offset_z), 
                         (self.g5x_offset_x, self.g5x_offset_y, self.g5x_offset_z)))

        self.feed_exp_append((self.lineno, self.lo_abs, 
                             l_exp[0], l_exp[1], l_exp[2], l_exp[3], 
                             (self.xo, self.yo, self.zo),
                             (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))
        self.lo = l
        self.lo_abs = l_exp[0]
        #print('**GLCanon.straight_feed_exp: ', self.feed_exp)

    straight_probe = straight_feed

    def arc_feed(self, *args):
        if self.suppress > 0: return
        self.first_move = False
        self.in_arc = True
        try:
            ArcsToSegmentsMixin.arc_feed(self, *args)
        finally:
            self.in_arc = False

    def straight_arcsegments(self, segs1, segs2, g92_set, g5x_set, xy_rot_set):
        #print('straight_arcsegments: ', segs1, segs2, g92_set, g5x_set, xy_rot_set)
        self.first_move = False
        lo = self.lo
        lo_abs = self.lo_abs
        lineno = self.lineno
        feedrate = self.feedrate
        to = (self.xo, self.yo, self.zo)
        append = self.arcfeed_append
        append_exp = self.arcfeed_exp_append 
        for l in segs1:
            append((lineno, lo, l, feedrate, to, g92_set, g5x_set, xy_rot_set))
            lo = l
        for l in segs2:
            append_exp((lineno, lo_abs, l, g92_set, g5x_set, xy_rot_set, 
                        (self.xo, self.yo, self.zo),
                        (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)))
            lo_abs = l
        #print('self.arcfeed_exp: ', self.arcfeed_exp)
        self.lo = lo
        self.lo_abs = lo_abs

    def user_defined_function(self, i,p,q):
        if i == 1: # i = 1 => Mcode 'M101'
            if p == 0: # code to reset twp offset(x,y,z,r,i) to (0,0,0,0,0) 
                (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)=(0,0,0,0,0)
            elif p == 1: # twp-x
                self.twp_x = q
            elif p == 2: # twp-y
                self.twp_y = q
            elif p == 3: # twp-z
                self.twp_z = q
            elif p == 4: # twp-r
                self.twp_r = q
            elif p == 5: # twp-i
                self.twp_i = q    
        if self.suppress > 0: return
        color = self.colors['m1xx']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], int(self.state.plane/10-17)))

    def dwell(self, arg):
        if self.suppress > 0: return
        self.dwell_time += arg
        color = self.colors['dwell']
        self.dwells_append((self.lineno, color, self.lo[0], self.lo[1], self.lo[2], int(self.state.plane/10-17)))

    def highlight(self, lineno, geometry):
        glLineWidth(3)
        c = self.colors['selected']
        glColor3f(*c)
        glBegin(GL_LINES)
        coords = []
        for line in self.traverse:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        for line in self.arcfeed:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        for line in self.feed:
            if line[0] != lineno: continue
            linuxcnc.line9(geometry, line[1], line[2])
            coords.append(line[1][:3])
            coords.append(line[2][:3])
        glEnd()
        for line in self.dwells:
            if line[0] != lineno: continue
            self.draw_dwells([(line[0], c) + line[2:]], 2, 0)
            coords.append(line[2:5])
        glLineWidth(1)
        if coords:
            x = reduce(lambda x,y:x+y, [c[0] for c in coords]) / len(coords)
            y = reduce(lambda x,y:x+y, [c[1] for c in coords]) / len(coords)
            z = reduce(lambda x,y:x+y, [c[2] for c in coords]) / len(coords)
        else:
            x = (self.min_extents[0] + self.max_extents[0])/2
            y = (self.min_extents[1] + self.max_extents[1])/2
            z = (self.min_extents[2] + self.max_extents[2])/2
        return x, y, z

    def color_with_alpha(self, name):
        glColor4f(*(self.colors[name] + (self.colors.get(name+'_alpha', 1/3.),)))
    def color(self, name):
        glColor3f(*self.colors[name])

    def draw(self, for_selection=0, no_traverse=True):
        if not no_traverse:
            self.colored_lines('traverse', self.traverse, for_selection)
        else:
            self.colored_lines('straight_feed', self.feed, for_selection, len(self.traverse))

            self.colored_lines('arc_feed', self.arcfeed, for_selection, len(self.traverse) + len(self.feed))

            glLineWidth(2)
            self.draw_dwells(self.dwells, int(self.colors.get('dwell_alpha', 1/3.)), for_selection, len(self.traverse) + len(self.feed) + len(self.arcfeed))
            glLineWidth(1)

def with_context(f):
    def inner(self, *args, **kw):
        self.activate()
        try:
            return f(self, *args, **kw)
        finally:
            self.deactivate()
    return inner

def with_context_swap(f):
    def inner(self, *args, **kw):
        self.activate()
        try:
            return f(self, *args, **kw)
        finally:
            self.swapbuffers()
            self.deactivate()
    return inner


class GlCanonDraw:
    colors = {
        'traverse': (0.30, 0.50, 0.50),
        'traverse_alpha': 1/3.,
        'traverse_xy': (0.30, 0.50, 0.50),
        'traverse_alpha_xy': 1/3.,
        'traverse_uv': (0.30, 0.50, 0.50),
        'traverse_alpha_uv': 1/3.,
        'backplotprobing_alpha': 0.75,
        'backplotprobing': (0.63, 0.13, 0.94),
        'backplottraverse': (0.30, 0.50, 0.50),
        'label_ok': (1.00, 0.51, 0.53),
        'backplotjog_alpha': 0.0,
        'tool_diffuse': (0.60, 0.60, 0.60),
        'backplotfeed': (0.75, 0.25, 0.25),
        'back': (0.00, 0.00, 0.00),
        'lathetool_alpha': 0.10,
        'axis_x': (0.20, 1.00, 0.20),
        'cone': (1.00, 0.00, 1.00),
        'cone_xy': (0.00, 1.00, 0.00),
        'cone_uv': (0.00, 0.00, 1.00),
        'axis_z': (0.20, 0.20, 1.00),
        'label_limit': (1.00, 0.21, 0.23),
        'backplotjog': (0.20, 0.20, 0.00),
        'selected': (0.00, 1.00, 1.00),
        'lathetool': (0.80, 0.80, 0.80),
        'dwell': (1.00, 0.50, 0.50),
        'overlay_foreground': (1.00, 0.00, 1.00), # this doen't seem to work'
        'overlay_background': (0.00, 0.00, 0.00),
        'overlay_alpha': 1,
        'straight_feed': (1.00, 1.00, 1.00),
        'straight_feed_alpha': 1/3.,
        'straight_feed_xy': (0.20, 1.00, 0.20),
        'straight_feed_alpha_xy': 1/3.,
        'straight_feed_uv': (0.20, 0.20, 1.00),
        'straight_feed_alpha_uv': 1/3.,
        'small_origin': (0.00, 1.00, 1.00),
        'backplottoolchange_alpha': 0.25,
        'backplottraverse_alpha': 0.25,
        'tool': (0.80, 0.20, 0.80),
        'tool_ambient': (0.40, 0.40, 0.40),
        'tool_alpha': 0.00,
        'backplottoolchange': (1.00, 0.65, 0.00),
        'backplotarc': (0.75, 0.25, 0.50),
        'm1xx': (0.50, 0.50, 1.00),
        'backplotfeed_alpha': 0.75,
        'backplotarc_alpha': 0.75,
        'arc_feed': (1.00, 1.00, 1.00),
        'arc_feed_alpha': .5,
        'arc_feed_xy': (0.20, 1.00, 0.20),
        'arc_feed_alpha_xy': 1/3.,
        'arc_feed_uv': (0.20, 0.20, 1.00),
        'arc_feed_alpha_uv': 1/3.,
        'axis_y': (1.00, 0.20, 0.20),
        'grid': (0.15, 0.15, 0.15),
        'limits': (0.50, 0.0, 0.0),
        'limits_fill': (0.10, 0.10, 0.10),
        'twp_active': (0.35, 0.05, 0.35),
        'twp_inactive': (0.25, 0.25, 0.25),
    }
    def __init__(self, s=None, lp=None, g=None):
        self.stat = s
        self.lp = lp
        self.canon = g
        self._dlists = {}
        self.select_buffer_size = 100
        self.cached_tool = -1
        self.initialised = 0
        self.no_joint_display = True
        self.kinsmodule = "UNKNOWN"
        self.trajcoordinates = "unknown"
        self.dro_in = "% 9.4f"
        self.dro_mm = "% 9.3f"
        self.show_overlay = True
        self.show_live_plot = True
        self.enable_dro = True
        self.cone_basesize = .5
        self.show_small_origin = True
        self.show_icon_home_list = []

        self.g5x_offset = self.to_internal_units(s.g5x_offset)[:3]
        self.g5x_index = s.g5x_index
        self.g92_offset = self.to_internal_units(s.g92_offset)[:3]
        self.tool_offset = self.to_internal_units(s.tool_offset)[2]
        self.rotation_xy = s.rotation_xy
        self.machine_limit_min, self.machine_limit_max = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.g54_x = self.g54_y = self.g54_z = 0
        self.twp_active = False
        self.limit_violation = False
        self.twp_error_msg = ''
        self.twp_x = self.twp_y  =self.twp_z = self.twp_r = self.twp_i = 0
        self.pivot_z = 0
        self.saved_index = 0
        self.saved_x = self.saved_y = self.saved_z = 0
        self.config_with_twp = True
        self.jdx = 0
        self.jdz = 0
        self.last_kins_type = False

        try:
            if os.environ["INI_FILE_NAME"]:
                self.inifile = linuxcnc.ini(os.environ["INI_FILE_NAME"])
                if self.inifile.find("DISPLAY", "DRO_FORMAT_IN"):
                    temp = self.inifile.find("DISPLAY", "DRO_FORMAT_IN")
                    try:
                        test = temp % 1.234
                    except:
                        print ("Error: invalid [DISPLAY] DRO_FORMAT_IN in INI file")
                    else:
                        self.dro_in = temp
                if self.inifile.find("DISPLAY", "DRO_FORMAT_MM"):
                    temp = self.inifile.find("DISPLAY", "DRO_FORMAT_MM")
                    try:
                        test = temp % 1.234
                    except:
                        print ("Error: invalid [DISPLAY] DRO_FORMAT_MM in INI file")
                    else:
                        self.dro_mm = temp
                        self.dro_in = temp
                size = (self.inifile.find("DISPLAY", "CONE_BASESIZE") or None)
                if size is not None:
                    self.set_cone_basesize(float(size))
        except:
            # Probably started in an editor so no INI
            pass

        try:
            self.kins_type = hal.get_value('motion.switchkins-type')
            self.twp_active = hal.get_value('twp_helper_comp.twp-is-active-out')
            self.twp_i = hal.get_value('twp_helper_comp.twp-i-out')
            self.twp_r = hal.get_value('twp_helper_comp.twp-r-out')
            self.twp_x = hal.get_value('twp_helper_comp.twp-x-out') / 25.4
            self.twp_y = hal.get_value('twp_helper_comp.twp-y-out') / 25.4
            self.twp_z = hal.get_value('twp_helper_comp.twp-z-out') / 25.4
            self.saved_index = int((hal.get_value('twp_helper_comp.g5x-index')) - 53)
            self.saved_x = hal.get_value('twp_helper_comp.g5x-x') / 25.4
            self.saved_y = hal.get_value('twp_helper_comp.g5x-y') / 25.4
            self.saved_z = hal.get_value('twp_helper_comp.g5x-z') / 25.4
            self.pivot_z = hal.get_value('twp_helper_comp.pivot-z-out') / 25.4
        except Exception as e:
            # started from a config without twp
            self.config_with_twp = False
            print(' *** Config without TWP')
            self.twp_active = 0
            self.twp_x = self.twp_y = self.twp_z = self.twp_r = self.twp_i = 0
            self.saved_index = self.g5x_index
            self.saved_x = self.g5x_offset[0]
            self.saved_y = self.g5x_offset[1]
            self.saved_z = self.g5x_offset[2]
            self.pivot_z = 0

    def set_cone_basesize(self, size):
        if size >2 or size < .025: size =.5
        self.cone_basesize = size
        self._redraw()

    def init_glcanondraw(self,trajcoordinates="XYZABCUVW",kinsmodule="trivkins",msg=""):
        self.trajcoordinates = trajcoordinates.upper().replace(" ","")
        self.kinsmodule = kinsmodule
        self.no_joint_display = self.stat.kinematics_type == linuxcnc.KINEMATICS_IDENTITY
        if (msg != ""):
            print("init_glcanondraw %s coords=%s kinsmodule=%s no_joint_display=%d"%(
                   msg,self.trajcoordinates,self.kinsmodule,self.no_joint_display))

        g = self.get_geometry().upper()
        linuxcnc.gui_respect_offsets(self.trajcoordinates,int('!' in g))

        geometry_chars = "XYZABCUVW-!;"
        dupchars = []; badchars = []
        for ch in g:
            if g.count(ch) >1: dupchars.append(ch)
            if not ch in geometry_chars: badchars.append(ch)
        if dupchars:
            print("Warning: duplicate chars %s in geometry: %s"%(dupchars,g))
        if badchars:
            print("Warning: unknown chars %s in geometry: %s"%(badchars,g))

    def realize(self):
        self.hershey = hershey.Hershey()
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        self.basic_lighting()
        self.initialised = 1

    def set_canon(self, canon):
        self.canon = canon

    @with_context
    def basic_lighting(self):
        glLightfv(GL_LIGHT0, GL_POSITION, (1, -1, 1, 0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, self.colors['tool_ambient'] + (0,))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, self.colors['tool_diffuse'] + (0,))
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, (1,1,1,0))
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def select(self, x, y):
        if self.canon is None: return
        pmatrix = glGetDoublev(GL_PROJECTION_MATRIX)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        vport = glGetIntegerv(GL_VIEWPORT)
        gluPickMatrix(x, vport[3]-y, 5, 5, vport)
        glMultMatrixd(pmatrix)
        glMatrixMode(GL_MODELVIEW)

        while 1:
            glSelectBuffer(self.select_buffer_size)
            glRenderMode(GL_SELECT)
            glInitNames()

            if True and hasattr(self, 'traverse_transformed'):
                glTranslatef(*[-x for x in self.center_view_offset ])
                for lineno, start, end in self.traverse_transformed:
                    glPushName(lineno)
                    glColor3f(.9,.9,0)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
                    glPopName()

                for lineno, start, end in self.feed_transformed:
                    glPushName(lineno)
                    glColor3f(1,1,1)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
                    glPopName()

                for lineno, start, end in self.arcfeed_transformed:
                    glPushName(lineno)
                    glColor3f(1,1,1)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
                    glPopName()
                glTranslatef(*[x for x in self.center_view_offset ])
            else:
                glPushName(0)
                if self.get_show_rapids():
                    glCallList(self.dlist('select_rapids', gen=self.make_selection_list))
                glCallList(self.dlist('select_norapids', gen=self.make_selection_list))

            try:
                buffer = glRenderMode(GL_RENDER)
            except :
                buffer = []
            break

        if buffer:
            min_depth, max_depth, names = (buffer[0].near, buffer[0].far, buffer[0].names)
            for x in buffer:
                if min_depth < x.near:
                    min_depth, max_depth, names = (x.near, x.far, x.names)

            if True:#self.config_with_twp:
                self.set_twp_highlight_line(names[0])
            self.set_highlight_line(names[0])
        else:
            self.set_highlight_line(None)
            self.set_twp_highlight_line(None)
        # this is required to update the display right away (the values seem to be ignored)
        self.set_centerpoint(0,0,0)


        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)


    def set_twp_highlight_line(self, line):
        self.twp_highlight_line = []
        if line == None: return
        for seg_list in (self.traverse_transformed,
                         self.feed_transformed,
                         self.arcfeed_transformed):
            for lineno, start, end in seg_list: # lineno, start, end
                if lineno == line:
                    self.twp_highlight_line.append((start, end))

    def dlist(self, name, n=1, gen=lambda n: None):
        if name not in self._dlists:
            base = glGenLists(n)
            self._dlists[name] = base, n
            gen(base)
        return self._dlists[name][0]

    def stale_dlist(self, name):
        if name not in self._dlists: return
        base, count = self._dlists.pop(name)
        glDeleteLists(base, count)

    def __del__(self):
        for base, count in list(self._dlists.values()):
            glDeleteLists(base, count)

    def update_highlight_variable(self,line):
        self.highlight_line = line

    def set_current_line(self, line): pass
    def set_highlight_line(self, line):
        if line == self.get_highlight_line(): return
        self.update_highlight_variable(line)
        highlight = self.dlist('highlight')
        glNewList(highlight, GL_COMPILE)
        if line is not None and self.canon is not None:
            if self.is_foam():
                glPushMatrix()
                glTranslatef(0, 0, self.get_foam_z())
                x, y, z = self.canon.highlight(line, "XY")
                glTranslatef(0, 0, self.get_foam_w()-self.get_foam_z())
                u, v, w = self.canon.highlight(line, "UV")
                glPopMatrix()
                x = (x+u)/2
                y = (y+v)/2
                z = (self.get_foam_z() + self.get_foam_w())/2
            else:
                x, y, z = self.canon.highlight(line, self.get_geometry())
        
        elif self.canon is not None:
            x = (self.canon.min_extents[0] + self.canon.max_extents[0])/2
            y = (self.canon.min_extents[1] + self.canon.max_extents[1])/2
            z = (self.canon.min_extents[2] + self.canon.max_extents[2])/2
        else:
            x, y, z = 0.0, 0.0, 0.0   
        glEndList()
        self.set_centerpoint(x, y, z)

    @with_context_swap
    def redraw_perspective(self):
        w = self.winfo_width()
        h = self.winfo_height()
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(self.fovy, float(w)/float(h), self.near, self.far + self.distance)

        gluLookAt(0, 0, self.distance,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

        # Repeat again to properly update our custon kinematic switches
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(self.fovy, float(w)/float(h), self.near, self.far + self.distance)

        gluLookAt(0, 0, self.distance,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

    @with_context_swap
    def redraw_ortho(self):
        if not self.initialised: return

        w = self.winfo_width()
        h = self.winfo_height()
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        ztran = self.distance
        k = (abs(ztran or 1)) ** .55555
        l = k * h / w
        glOrtho(-k, k, -l, l, -1000, 1000.)

        gluLookAt(0, 0, 1,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

        # Repeat again to properly update our custon kinematic switches
        glViewport(0, 0, w, h)

        # Clear the background and depth buffer.
        glClearColor(*(self.colors['back'] + (0,)))
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        ztran = self.distance
        k = (abs(ztran or 1)) ** .55555
        l = k * h / w
        glOrtho(-k, k, -l, l, -1000, 1000.)

        gluLookAt(0, 0, 1,
            0, 0, 0,
            0., 1., 0.)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        try:
            self.redraw()
        finally:
            glFlush()                               # Tidy up
            glPopMatrix()                   # Restore the matrix

    def color_limit(self, cond):
        if cond:
            glColor3f(*self.colors['label_limit'])
        else:
            glColor3f(*self.colors['label_ok'])
        return cond


    def show_extents(self):
        s = self.stat
        g = self.canon

        if g is None: return

        # Dimensions
        x,y,z,p = 0,1,2,3
        view = self.get_view()
        is_metric = self.get_show_metric()
        dimscale = is_metric and 25.4 or 1.0
        fmt = is_metric and "%.1f" or "%.2f"

        machine_limit_min, machine_limit_max = self.soft_limits()

        pullback = max(g.max_extents[x] - g.min_extents[x],
                       g.max_extents[y] - g.min_extents[y],
                       g.max_extents[z] - g.min_extents[z],
                       2 ) * .1

        dashwidth = pullback/4
        charsize = dashwidth * 1.5
        halfchar = charsize * .5

        if view == z or view == p:
            z_pos = g.min_extents[z]
            zdashwidth = 0
        else:
            z_pos = g.min_extents[z] - pullback
            zdashwidth = dashwidth

        #draw dimension lines
        self.color_limit(0)
        glBegin(GL_LINES)

        # x dimension
        if view != x and g.max_extents[x] > g.min_extents[x]:
            y_pos = g.min_extents[y] - pullback
            #dimension line
            glVertex3f(g.min_extents[x], y_pos, z_pos)
            glVertex3f(g.max_extents[x], y_pos, z_pos)
            #line perpendicular to dimension line at min extent
            glVertex3f(g.min_extents[x], y_pos - dashwidth, z_pos - zdashwidth)
            glVertex3f(g.min_extents[x], y_pos + dashwidth, z_pos + zdashwidth)
            #line perpendicular to dimension line at max extent
            glVertex3f(g.max_extents[x], y_pos - dashwidth, z_pos - zdashwidth)
            glVertex3f(g.max_extents[x], y_pos + dashwidth, z_pos + zdashwidth)

        # y dimension
        if view != y and g.max_extents[y] > g.min_extents[y]:
            x_pos = g.min_extents[x] - pullback
            #dimension line
            glVertex3f(x_pos, g.min_extents[y], z_pos)
            glVertex3f(x_pos, g.max_extents[y], z_pos)
            #line perpendicular to dimension line at min extent
            glVertex3f(x_pos - dashwidth, g.min_extents[y], z_pos - zdashwidth)
            glVertex3f(x_pos + dashwidth, g.min_extents[y], z_pos + zdashwidth)
            #line perpendicular to dimension line at max extent
            glVertex3f(x_pos - dashwidth, g.max_extents[y], z_pos - zdashwidth)
            glVertex3f(x_pos + dashwidth, g.max_extents[y], z_pos + zdashwidth)

        # z dimension
        if view != z and g.max_extents[z] > g.min_extents[z]:
            x_pos = g.min_extents[x] - pullback
            y_pos = g.min_extents[y] - pullback
            #dimension line
            glVertex3f(x_pos, y_pos, g.min_extents[z])
            glVertex3f(x_pos, y_pos, g.max_extents[z])
            #line perpendicular to dimension line at min extent
            glVertex3f(x_pos - dashwidth, y_pos - zdashwidth, g.min_extents[z])
            glVertex3f(x_pos + dashwidth, y_pos + zdashwidth, g.min_extents[z])
            #line perpendicular to dimension line at max extent
            glVertex3f(x_pos - dashwidth, y_pos - zdashwidth, g.max_extents[z])
            glVertex3f(x_pos + dashwidth, y_pos + zdashwidth, g.max_extents[z])

        glEnd()

        # Labels
        # get_show_relative is True calculates extents from the local origin
        # get_show_relative is False calculates extents from the machine origin
        if self.get_show_relative():
            offset = self.to_internal_units(s.g5x_offset + s.g92_offset)
        else:
            offset = 0, 0, 0
        #Z extent labels
        if view != z and g.max_extents[z] > g.min_extents[z]:
            if view == x:
                x_pos = g.min_extents[x] - pullback
                y_pos = g.min_extents[y] - 6.0*dashwidth
            else:
                x_pos = g.min_extents[x] - 6.0*dashwidth
                y_pos = g.min_extents[y] - pullback
            #Z MIN extent
            bbox = self.color_limit(g.min_extents_notool[z] < machine_limit_min[z])
            glPushMatrix()
            f = fmt % ((g.min_extents[z]-offset[z]) * dimscale)
            glTranslatef(x_pos, y_pos, g.min_extents[z] - halfchar)
            glScalef(charsize, charsize, charsize)
            glRotatef(-90, 0, 1, 0)
            glRotatef(-90, 0, 0, 1)
            if view != x:
                glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()
            #Z MAX extent
            bbox = self.color_limit(g.max_extents_notool[z] > machine_limit_max[z])
            glPushMatrix()
            f = fmt % ((g.max_extents[z]-offset[z]) * dimscale)
            glTranslatef(x_pos, y_pos, g.max_extents[z] - halfchar)
            glScalef(charsize, charsize, charsize)
            glRotatef(-90, 0, 1, 0)
            glRotatef(-90, 0, 0, 1)
            if view != x:
                glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()
            self.color_limit(0)
            glPushMatrix()
            #Z Midpoint
            f = fmt % ((g.max_extents[z] - g.min_extents[z]) * dimscale)
            glTranslatef(x_pos, y_pos, (g.max_extents[z] + g.min_extents[z])/2)
            glScalef(charsize, charsize, charsize)
            if view != x:
                glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 1, 0)
            self.hershey.plot_string(f, .5, bbox)
            glPopMatrix()
        #Y extent labels
        if view != y and g.max_extents[y] > g.min_extents[y]:
            x_pos = g.min_extents[x] - 6.0*dashwidth
            #Y MIN extent
            bbox = self.color_limit(g.min_extents_notool[y] < machine_limit_min[y])
            glPushMatrix()
            f = fmt % ((g.min_extents[y] - offset[y]) * dimscale)
            glTranslatef(x_pos, g.min_extents[y] + halfchar, z_pos)
            glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()
            #Y MAX extent
            bbox = self.color_limit(g.max_extents_notool[y] > machine_limit_max[y])
            glPushMatrix()
            f = fmt % ((g.max_extents[y] - offset[y]) * dimscale)
            glTranslatef(x_pos, g.max_extents[y] + halfchar, z_pos)
            glRotatef(-90, 0, 0, 1)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            self.color_limit(0)
            glPushMatrix()
            #Y midpoint
            f = fmt % ((g.max_extents[y] - g.min_extents[y]) * dimscale)
            glTranslatef(x_pos, (g.max_extents[y] + g.min_extents[y])/2,
                        z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == x:
                glRotatef(-90, 1, 0, 0)
                glTranslatef(0, halfchar, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, .5)
            glPopMatrix()
        #X extent labels
        if view != x and g.max_extents[x] > g.min_extents[x]:
            y_pos = g.min_extents[y] - 6.0*dashwidth
            #X MIN extent
            bbox = self.color_limit(g.min_extents_notool[x] < machine_limit_min[x])
            glPushMatrix()
            f = fmt % ((g.min_extents[x] - offset[x]) * dimscale)
            glTranslatef(g.min_extents[x] - halfchar, y_pos, z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == y:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()
            #X MAX extent
            bbox = self.color_limit(g.max_extents_notool[x] > machine_limit_max[x])
            glPushMatrix()
            f = fmt % ((g.max_extents[x] - offset[x]) * dimscale)
            glTranslatef(g.max_extents[x] - halfchar, y_pos, z_pos)
            glRotatef(-90, 0, 0, 1)
            if view == y:
                glRotatef(90, 0, 1, 0)
                glTranslatef(dashwidth*1.5, 0, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, 0, bbox)
            glPopMatrix()

            self.color_limit(0)
            glPushMatrix()
            #X midpoint
            f = fmt % ((g.max_extents[x] - g.min_extents[x]) * dimscale)
            glTranslatef((g.max_extents[x] + g.min_extents[x])/2, y_pos,
                        z_pos)
            if view == y:
                glRotatef(-90, 1, 0, 0)
                glTranslatef(0, halfchar, 0)
            glScalef(charsize, charsize, charsize)
            self.hershey.plot_string(f, .5)
            glPopMatrix()

    def to_internal_linear_unit(self, v, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4
        return v/lu

    def to_internal_units(self, pos, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4

        lus = [lu, lu, lu, 1, 1, 1, lu, lu, lu]
        return [a/b for a, b in zip(pos, lus)]

    def soft_limits(self):
        def fudge(x):
            if abs(x) > 1e30: return 0
            return x

        ax = self.stat.axis
        return (
            self.to_internal_units([fudge(ax[i]['min_position_limit'])
                for i in range(3)]),
            self.to_internal_units([fudge(ax[i]['max_position_limit'])
                for i in range(3)]))


    def jnum_for_aletter(self,aletter,kinsmodule,trajcoordinates):
        aletter = aletter.upper()
        if "trivkins" in kinsmodule:
            return trajcoordinates.index(aletter)
        else:
            try:
                guess = trajcoordinates.index(aletter)
                return guess
            except:
                return "XYZABCUVW".index(aletter)

    def posstrs(self):
        s = self.stat
        limit = list(s.limit[:])
        homed = list(s.homed[:])
        spd = self.to_internal_linear_unit(s.current_vel)
        return self.joint_dro_format(s,spd,self.get_num_joints(),limit, homed)

    # N.B. no conversion here because joint positions are unitless
    #      joint_mode and display_joint
    # Note: this is overridden in other guis (then AXIS) for different dro behavior
    def joint_dro_format(self,s,spd,num_of_joints,limit, homed):
        posstrs = [' ']
        droposstrs = [' ']
        if self.limit_violation:
            posstrs.append(self.twp_error_msg)
        if self.config_with_twp:
            if self.twp_active and self.kins_type != 2:
                posstrs.append(" ")
                posstrs.append("! KINEMATIICS MANUAL OVERRIDE ! ")                    
            posstrs.append(" ")
            posstrs.append("Joint Pos:")
            for i in range(num_of_joints):
                # N.B. no conversion here because joint positions are unitless
                posstrs.append("J%s:% 9.4f" % (i, s.joint_actual_position[i]))
            droposstrs = posstrs

        if  self.rotation_xy:
            posstrs.append(" ")
            posstrs.append("Rotation:")
            posstrs.append("XY: %3.2f deg" % (self.rotation_xy))

        if  self.g92_offset[0] or self.g92_offset[1] or self.g92_offset[2]:
            posstrs.append(" ")
            posstrs.append("G52,G92 Offset:")
            posstrs.append("x:% 9.2fmm" % (self.g92_offset[0] * 25.4))
            posstrs.append("y:% 9.2fmm" % (self.g92_offset[1] * 25.4))
            posstrs.append("z:% 9.2fmm" % (self.g92_offset[2] * 25.4))

        if self.twp_active:
            posstrs.append(" ")
            posstrs.append("TWP Offset:")
            posstrs.append("x:% 9.2fmm" % (self.twp_x * 25.4))
            posstrs.append("y:% 9.2fmm" % (self.twp_y * 25.4))
            posstrs.append("z:% 9.2fmm" % (self.twp_z * 25.4))
            posstrs.append("r:% 9.2fdeg" % self.twp_r)
            posstrs.append("i:% 9.2fdeg" % self.twp_i)

            posstrs.append(" ")
            posstrs.append("Initial Offset: G" + str(53+self.saved_index))
            posstrs.append("x:% 9.2fmm" % (self.saved_x * 25.4))
            posstrs.append("y:% 9.2fmm" % (self.saved_y * 25.4))
            posstrs.append("z:% 9.2fmm" % (self.saved_z * 25.4))

        else:
            posstrs.append(" ")
            posstrs.append("Active Offset: G" + str(53 + self.g5x_index))
            posstrs.append("x:% 9.2fmm" % (self.g5x_offset[0] * 25.4))
            posstrs.append("y:% 9.2fmm" % (self.g5x_offset[1] * 25.4))
            posstrs.append("z:% 9.2fmm" % (self.g5x_offset[2] * 25.4))


        return limit, homed, posstrs, droposstrs


    def draw_small_origin(self, n):
        glNewList(n, GL_COMPILE)
        r = 2.0/25.4
        glColor3f(*self.colors['small_origin'])

        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(r*math.cos(theta),r*math.sin(theta),0.0)
        glEnd()
        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(0.0, r*math.cos(theta), r*math.sin(theta))
        glEnd()
        glBegin(GL_LINE_STRIP)
        for i in range(37):
            theta = (i*10)*math.pi/180.0
            glVertex3f(r*math.cos(theta),0.0, r*math.sin(theta))
        glEnd()

        glBegin(GL_LINES)
        glVertex3f(-r, -r, 0.0)
        glVertex3f( r,  r, 0.0)
        glVertex3f(-r,  r, 0.0)
        glVertex3f( r, -r, 0.0)

        glVertex3f(-r, 0.0, -r)
        glVertex3f( r, 0.0,  r)
        glVertex3f(-r, 0.0,  r)
        glVertex3f( r, 0.0, -r)

        glVertex3f(0.0, -r, -r)
        glVertex3f(0.0,  r,  r)
        glVertex3f(0.0, -r,  r)
        glVertex3f(0.0,  r, -r)
        glEnd()
        glEndList()

    def draw_axes(self, n, letters="XYZ"):
        glNewList(n, GL_COMPILE)
        x,y,z,p = 0,1,2,3
        s = self.stat
        view = self.get_view()

        glColor3f(*self.colors['axis_x'])
        glBegin(GL_LINES)
        glVertex3f(1.0,0.0,0.0)
        glVertex3f(0.0,0.0,0.0)
        glEnd()

        if view != x:
            glPushMatrix()
            glTranslatef(1.2, -0.1, 0)
            if view == y:
                glTranslatef(0, 0, -0.1)
                glRotatef(90, 1, 0, 0)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[0], 0.5)
            glPopMatrix()

        glColor3f(*self.colors['axis_y'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,1.0,0.0)
        glEnd()

        if view != y:
            glPushMatrix()
            glTranslatef(0, 1.2, 0)
            if view == x:
                glTranslatef(0, 0, -0.1)
                glRotatef(90, 0, 1, 0)
                glRotatef(90, 0, 0, 1)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[1], 0.5)
            glPopMatrix()

        glColor3f(*self.colors['axis_z'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,0.0,1.0)
        glEnd()

        if view != z:
            glPushMatrix()
            glTranslatef(0, 0, 1.2)
            if self.is_lathe():
                glRotatef(-90, 0, 1, 0)
            if view == x:
                glRotatef(90, 0, 1, 0)
                glRotatef(90, 0, 0, 1)
            elif view == y or view == p:
                glRotatef(90, 1, 0, 0)
            glScalef(0.2, 0.2, 0.2)
            self.hershey.plot_string(letters[2], 0.5)
            glPopMatrix()

        glEndList()

    def draw_axes_no_letters(self, n):
        glNewList(n, GL_COMPILE)
        x,y,z,p = 0,1,2,3
        s = self.stat
        view = self.get_view()

        glScalef(0.4,0.4,0.4)
        glColor3f(*self.colors['axis_x'])
        glBegin(GL_LINES)
        glVertex3f(1.0,0.0,0.0)
        glVertex3f(0.0,0.0,0.0)
        glEnd()

        glColor3f(*self.colors['axis_y'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,1.0,0.0)
        glEnd()

        glColor3f(*self.colors['axis_z'])
        glBegin(GL_LINES)
        glVertex3f(0.0,0.0,0.0)
        glVertex3f(0.0,0.0,1.0)
        glEnd()
        glEndList()

    def make_cone(self, n):
        q = gluNewQuadric()
        glNewList(n, GL_COMPILE)
        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)
        if self.twp_active and self.kins_type != 2:
            glColor3f(1.,1.,1.)
        else:
            glColor3f(*self.colors['cone'])
        gluCylinder(q, 0, .1, .25, 32, 1)
        glPushMatrix()
        glTranslatef(0,0,.25)
        gluDisk(q, 0, .1, 32, 1)
        glPopMatrix()
        glEnable(GL_COLOR_MATERIAL)
        glDisable(GL_LIGHTING)
        glEndList()
        gluDeleteQuadric(q)

    def cache_tool(self, current_tool):
        self.cached_tool = current_tool
        glNewList(self.dlist('tool'), GL_COMPILE)
        dia = current_tool.diameter
        r = self.to_internal_linear_unit(dia) / 2.
        q = gluNewQuadric()
        gluCylinder(q, r, r, 8*r, 32, 1)
        glPushMatrix()
        glRotatef(180, 1, 0, 0)
        gluDisk(q, 0, r, 32, 1)
        glPopMatrix()
        glTranslatef(0,0,8*r)
        gluDisk(q, 0, r, 32, 1)
        gluDeleteQuadric(q)
        glEndList()

    def extents_info(self):
        if self.canon:
            mid = [(a+b)/2 for a, b in zip(self.canon.max_extents, self.canon.min_extents)]
            size = [(a-b) for a, b in zip(self.canon.max_extents, self.canon.min_extents)]
        else:
            mid = [0, 0, 0]
            size = [3, 3, 3]
        return mid, size

    def make_selection_list(self, unused=None):
        select_rapids = self.dlist('select_rapids')
        select_program = self.dlist('select_norapids')
        glNewList(select_rapids, GL_COMPILE)
        if self.canon: self.canon.draw(1, False)
        glEndList()
        glNewList(select_program, GL_COMPILE)
        if self.canon: self.canon.draw(1, True)
        glEndList()

    def make_main_list(self, unused=None):
        program = self.dlist('program_norapids')
        rapids = self.dlist('program_rapids')

        glNewList(program, GL_COMPILE)
        if self.canon: self.canon.draw(0, True)
        glEndList()

        glNewList(rapids, GL_COMPILE)
        if self.canon: self.canon.draw(0, False)
        glEndList()

    def load_preview(self, f, canon, *args):
        self.set_canon(canon)
        result, seq = gcode.parse(f, canon, *args)

        if result <= gcode.MIN_ERROR:
            self.canon.progress.nextphase(1)
            canon.calc_extents()
            self.stale_dlist('program_rapids')
            self.stale_dlist('program_norapids')
            self.stale_dlist('select_rapids')
            self.stale_dlist('select_norapids')

        return result, seq

    def from_internal_units(self, pos, unit=None):
        if unit is None:
            unit = self.stat.linear_units
        lu = (unit or 1) * 25.4

        lus = [lu, lu, lu, 1, 1, 1, lu, lu, lu]
        return [a*b for a, b in zip(pos, lus)]

    def inject_initial_offsets(self):
        if not hasattr(GLCanon, 'traverse_exp') :
            return

        def do_inject_initial_offsets(obj):
            s = self.stat
            s.poll()
            new_list=[]
            for lineno, start, end, g92_offset, g5x_offset, xy_rot, tool_offset, twp_xyzri in obj:
                # When loading/reloading a gcode program with percent signs g92 and tool_offset are set to (0,0,0) on
                # the first line 

                # inject initial offset values and change to inches so we don't need to reconvert for the preview'
                if g92_offset[0] is False:
                    new_g92_offset = (s.g92_offset[0]/25.4, s.g92_offset[1]/25.4, s.g92_offset[2]/25.4)
                else:
                    new_g92_offset = (g92_offset[0], g92_offset[1], g92_offset[2])

                if tool_offset[0] is False:
                    new_tool_offset = (s.tool_offset[0]/25.4, s.tool_offset[1]/25.4, s.tool_offset[2]/25.4)
                else:
                    new_tool_offset = (tool_offset[0], tool_offset[1], tool_offset[2])

                if g5x_offset[0] is False:
                    new_g5x_offset = (s.g5x_offset[0]/25.4, s.g5x_offset[1]/25.4, s.g5x_offset[2]/25.4)
                else:
                    new_g5x_offset = (g5x_offset[0], g5x_offset[1], g5x_offset[2])

                if xy_rot is False:
                    new_xy_rot = s.rotation_xy 
                else:
                    new_xy_rot = xy_rot 

                if twp_xyzri[4] is False:
                    new_twp_xyzri = (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i)
                else:
                    new_twp_xyzri = (twp_xyzri[0]/25.4, twp_xyzri[1]/25.4, twp_xyzri[2]/25.4,
                                     twp_xyzri[3], twp_xyzri[4])

                new_list.append((lineno, start, end, new_g92_offset, new_g5x_offset, new_xy_rot,
                                 new_tool_offset, new_twp_xyzri))
            return new_list # in inches and degrees

        self.traverse_injected = do_inject_initial_offsets(GLCanon.traverse_exp)
        self.feed_injected = do_inject_initial_offsets(GLCanon.feed_exp)
        self.arcfeed_injected = do_inject_initial_offsets(GLCanon.arcfeed_exp)

    # forms a 4x4 transformation matrix from a given 1x3 point vector [x,y,z]
    def point_to_matrix(self, point):
        # start with a 4x4 identity matrix and add the point vector to the 4th column
        matrix = np.identity(4)
        [matrix[0, 3], matrix[1, 3], matrix[2, 3]] = point
        matrix = np.asmatrix(matrix)
        return matrix

    # extracts the translation (point) vector form a given 4x4 transformation matrix
    def matrix_to_point(self, matrix):
        point = (matrix[0, 3], matrix[1, 3], matrix[2, 3])
        return point

    # returns 4x4 transformation matrix for given angles and 4x4 input matrix
    def kins_tool_transformation(self, angle_b, angle_c, matrix_in, direction='fwd'):
        b = math.radians(angle_b)
        tc = math.radians(angle_c)
        T_in = matrix_in
        # substitutions as used in the xyzbu_st_man kinematic
        Sb = sin(b)
        Cb = cos(b)
        Stc = sin(tc)
        Ctc = cos(tc)
        # define rotation matrices
        Rb = np.matrix([[Cb, 0, Sb, 0], [0, 1, 0, 0], [-Sb, 0, Cb, 0], [0, 0, 0, 1]])
        # virtual rotation around tool-z to orient tool-x and -y
        Rtc = np.matrix([[Ctc, -Stc, 0, 0], [Stc, Ctc, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        matrix_tool_fwd = Rb * Rtc * T_in
        matrix_tool_inv = np.transpose(Rtc) * np.transpose(Rb) * T_in
        if direction == 'fwd':
            return matrix_tool_fwd
        elif direction == 'inv':
            return matrix_tool_inv
        else:
            return 0

    def transform_preview_segments(self):
        if not hasattr(GLCanon, 'traverse_exp'):
            return
        def do_transform_segments(obj):
            new_list = []
            for seg in obj:
                lineno = seg[0]
                start = self.point_to_matrix(seg[1][:3])
                end = self.point_to_matrix(seg[2][:3])
                T_g92_offset = self.point_to_matrix(seg[3][:3])
                T_g5x_offset = self.point_to_matrix(seg[4][:3])
                T_twp_offset = self.point_to_matrix(seg[7][:3])
                tot_xy_rot = seg[5] + seg[7][3]
                xz_rot = seg[7][4]
                T_saved_offset = self.point_to_matrix((self.saved_x, self.saved_y, self.saved_z))
                # 1. Translate start and end-points by g92_offset
                Ps1 = T_g92_offset * start
                Pe1 = T_g92_offset * end
                if tot_xy_rot or xz_rot:
                    # 2. Rotate according to the twp-orientation
                    Ps2 = (self.kins_tool_transformation(xz_rot, tot_xy_rot, Ps1))
                    Pe2 = (self.kins_tool_transformation(xz_rot, tot_xy_rot, Pe1))
                else:
                    Ps2 = Ps1
                    Pe2 = Pe1
                # 3. Translate by twp_offset
                Ps3 = T_twp_offset * Ps2
                Pe3 = T_twp_offset * Pe2             
                # 4. Translate by the saved g5x_offset if twp is active or current g5x_offset otherwise
                if self.twp_active:
                    Ps4 = T_saved_offset * Ps3
                    Pe4 = T_saved_offset * Pe3  
                else:
                    Ps4 = T_g5x_offset * Ps3
                    Pe4 = T_g5x_offset * Pe3                 
                Ps = Ps4
                Pe = Pe4
                new_list.append((lineno, (Ps[0,3],Ps[1,3],Ps[2,3]), (Pe[0,3],Pe[1,3],Pe[2,3])) )

            return new_list

        # self.<>_injected = 
        # (line number, (start position_mm), (end position_mm), (g92_x, g92_y, g92_z), (g5x_x, g5x_y, g5x_z), 
        #     [0]               [1]                  [2]                  [3]                   [4]             
        # xy_rot, (tlo x, tlo y, tlo z), (twp_x,twp_y,twp_z,twp_r,twp_i))
        #  [5]            [6]                        [7]     
        self.traverse_transformed =  do_transform_segments(self.traverse_injected)
        self.feed_transformed = do_transform_segments(self.feed_injected)
        self.arcfeed_transformed = do_transform_segments(self.arcfeed_injected)

    def calculate_joint_extents(self):
        if not hasattr(GLCanon, 'traverse_exp'): return
        g = self.canon
        if g is None: return
        min_extents_rotated = [9e99,9e99,9e99]
        max_extents_rotated = [-9e99,-9e99,-9e99]
        # Find the maximum and minimum values for x, y, z and check for limit violations
        self.limit_violation = False
        def extents_do_transform_segments(obj):
            def calc_kins_offset(angle_xz):
                # Add joint offset introduced by kinematic to calculate the joint extents
                # these are the distances the head needs to move to compensate the twp inclination in non-trivial 
                # kinematics, these values depend on the pivot length of the machine
                cb = math.cos(math.radians(angle_xz))
                sb = math.sin(math.radians(angle_xz))
                head_dx =   sb * self.pivot_z 
                head_dz = (cb - 1) * self.pivot_z
                return (head_dx, 0, head_dz)

            new_list = []
            for seg in obj:
                lineno = seg[0]
                start = self.point_to_matrix(seg[1][:3])
                end = self.point_to_matrix(seg[2][:3])
                T_g92_offset = self.point_to_matrix(seg[3][:3])
                T_g5x_offset = self.point_to_matrix(seg[4][:3])
                T_tool_offset = self.point_to_matrix(seg[6][:3])
                T_twp_offset = self.point_to_matrix(seg[7][:3])
                tot_xy_rot = seg[5] + seg[7][3]
                xz_rot = seg[7][4]
                T_saved_offset = self.point_to_matrix((self.saved_x, self.saved_y, self.saved_z))
                # 1. Add tool_offset to the start and end points
                Ps1 = T_tool_offset*start
                Pe1 = T_tool_offset*end
                # 2. Translate by g92_offset
                Ps2 = T_g92_offset * Ps1
                Pe2 = T_g92_offset * Pe1
                if tot_xy_rot or xz_rot:
                    # 3. Rotate the start and end points according to the twp-orientation
                    Ps3 = (self.kins_tool_transformation(xz_rot, tot_xy_rot, Ps2))
                    Pe3 = (self.kins_tool_transformation(xz_rot, tot_xy_rot, Pe2))
                else:
                    Ps3 = Ps2
                    Pe3 = Pe2
                # 4. Translate by twp_offset
                Ps4 = T_twp_offset * Ps3
                Pe4 = T_twp_offset * Pe3
                if xz_rot:
                    # 5. Translate by kinematic offset
                    T_kins_offset = self.point_to_matrix(calc_kins_offset(xz_rot))
                    Ps5 = T_kins_offset * Ps4
                    Pe5 = T_kins_offset * Pe4  
                else: 
                    Ps5 = Ps4
                    Pe5 = Pe4
                # 6. Translate by the saved g5x_offset if twp is active or current g5x_offset otherwise 
                if self.twp_active:
                    Ps6 = T_saved_offset * Ps5
                    Pe6 = T_saved_offset * Pe5  
                else:
                    Ps6 = T_g5x_offset * Ps5
                    Pe6 = T_g5x_offset * Pe5                 
                Ps = Ps6
                Pe = Pe6 
                tool_z = seg[6][2]
                new_list.append( (lineno, (Ps[0,3],Ps[1,3],Ps[2,3]),  (Pe[0,3],Pe[1,3],Pe[2,3]), xz_rot, tool_z ))       
            return new_list
        # self.<>_injected = 
        # (line number, (start position_mm), (end position_mm), (g92_x, g92_y, g92_z), (g5x_x, g5x_y, g5x_z), 
        #     [0]               [1]                  [2]                  [3]                   [4]         
        # xy_rot, (tlo x, tlo y, tlo z), (twp_x,twp_y,twp_z,twp_r,twp_i))
        #     [5]          [6]                        [7]    
        extents_traverse_transformed =  extents_do_transform_segments(self.traverse_injected)
        extents_feed_transformed = extents_do_transform_segments(self.feed_injected)
        extents_arcfeed_transformed = extents_do_transform_segments(self.arcfeed_injected)
        for seg_list in (extents_traverse_transformed, extents_feed_transformed, extents_arcfeed_transformed):
            for tup in seg_list: # lineno, start, end
                for i in (1,2): # start, end points ( 0 entry is lineno)
                    for j in range(3): # x,y,z
                        max_extents_rotated[j] = max(max_extents_rotated[j], tup[i][j])
                        if  max_extents_rotated[j] > self.ini_max_limits[j]/25.4:
                            self.twp_error_msg = ('Line ' + str(tup[0]) + ' would exeed maximum limit in joint '
                                                  + str(j) + ': ' 
                                                  + str("% 9.2fmm" % (max_extents_rotated[j]*25.4)) + ' >= '
                                                  + str(self.ini_max_limits[j]) + 'mm')
                            self.limit_violation = True
                            break
                        min_extents_rotated[j] = min(min_extents_rotated[j], tup[i][j])
                        if  min_extents_rotated[j] < self.ini_min_limits[j]/25.4:
                            self.twp_error_msg = ('Line ' + str(tup[0]) + ' would exeed minimum limit in joint ' 
                                                  + str(j) + ': ' 
                                                  + str("% 9.2fmm" % (min_extents_rotated[j]*25.4)) + ' <= '
                                                  + str(self.ini_min_limits[j]) + 'mm')
                            self.limit_violation = True
                            break
                    if self.limit_violation:
                        self.data_limit_error = (tup[3], tup[4])
                        break
                if self.limit_violation:
                    break
            if self.limit_violation:
                break
        print('max_extents: ',
                     (max_extents_rotated[0]*25.4,max_extents_rotated[1]*25.4,max_extents_rotated[2]*25.4))
        print('min_extents: ',
                     (min_extents_rotated[0]*25.4,min_extents_rotated[1]*25.4,min_extents_rotated[2]*25.4))


    def redraw(self):
        if self.canon is None: return
        s = self.stat
        s.poll()

        linuxcnc.gui_rot_offsets(s.g5x_offset[0] + s.g92_offset[0],
                                 s.g5x_offset[1] + s.g92_offset[1],
                                 s.g5x_offset[2] + s.g92_offset[2])
        self.g5x_index = s.g5x_index 
        self.g5x_offset = self.to_internal_units(s.g5x_offset)[:3]
        self.g92_offset = self.to_internal_units(s.g92_offset)[:3]
        self.tool_offset = self.to_internal_units(s.tool_offset)[2]
        self.rotation_xy = s.rotation_xy
        self.machine_homed = all(val == 1 for val in s.homed[:3])

        self.ini_max_limits = (hal.get_value('ini.0.max_limit'),
                               hal.get_value('ini.1.max_limit'), 
                               hal.get_value('ini.2.max_limit'))
        self.ini_min_limits = (hal.get_value('ini.0.min_limit'), 
                               hal.get_value('ini.1.min_limit'), 
                               hal.get_value('ini.2.min_limit'))
        self.first_move = True

        if self.config_with_twp:
            try:
                self.kins_type = hal.get_value('motion.switchkins-type')
                self.twp_active = hal.get_value('twp_helper_comp.twp-is-active-out')
                self.twp_i = hal.get_value('twp_helper_comp.twp-i-out')
                self.twp_r = hal.get_value('twp_helper_comp.twp-r-out')
                self.twp_x = hal.get_value('twp_helper_comp.twp-x-out') / 25.4
                self.twp_y = hal.get_value('twp_helper_comp.twp-y-out') / 25.4
                self.twp_z = hal.get_value('twp_helper_comp.twp-z-out') / 25.4
                self.saved_index = int((hal.get_value('twp_helper_comp.g5x-index')) - 53)
                self.saved_x = hal.get_value('twp_helper_comp.g5x-x') / 25.4
                self.saved_y = hal.get_value('twp_helper_comp.g5x-y') / 25.4
                self.saved_z = hal.get_value('twp_helper_comp.g5x-z') / 25.4
                self.pivot_z = hal.get_value('twp_helper_comp.pivot-z-out') / 25.4
            except Exception as e:
                pass

            if self.kins_type != self.last_kins_type:
                self.lp.clear()
                self.last_kins_type = self.kins_type
        
        # we don't want to run the custom preview and extents calculations evertime the window updates'
        # no calc if prog is running or GLCanon not ready
        if s.motion_line == 0 and self.machine_homed and hasattr(GLCanon, 'traverse_exp') :
            if not hasattr(self, 'traverse_injected'): # First call so self.<>_injected lists do not exist yet
                self.inject_initial_offsets()
                self.transform_preview_segments()
                self.calculate_joint_extents()
                # self.<>_injected = 
                #(lineno, (start position_mm), (end position_mm), (g92_x, g92_y, g92_z), (g5x_x, g5x_y, g5x_z), 
                #    [0]           [1]                  [2]                  [3]                   [4]             
                # xy_rot, (tlo x, tlo y, tlo z), (twp_x,twp_y,twp_z,twp_r,twp_i))
                #  [5]            [6]                         [6]    
                self.fingerprint_exp = (self.canon.traverse_exp, self.canon.feed_exp, self.canon.arcfeed_exp)
            else: # recalculate in these cases
                last_first_traverse = self.traverse_injected[0] 
                # TODO: if xy-rotation is changed with G10 then the Gui is not updated automatically as redraw()
                # is not called. Not sure how to fix this yet. 
                # 'changing gremlin.py line 275 to 's.motion_mode, s.current_vel, s.rotation_xy)' seems to fix it
                if self.fingerprint_exp != (self.canon.traverse_exp, self.canon.feed_exp, self.canon.arcfeed_exp) or\
                       last_first_traverse[3] != (self.g92_offset[0], self.g92_offset[1], self.g92_offset[2]) or\
                       last_first_traverse[4] != (self.g5x_offset[0], self.g5x_offset[1], self.g5x_offset[2]) or\
                       last_first_traverse[5] != (self.rotation_xy) or\
                       last_first_traverse[6] != (s.tool_offset[0], s.tool_offset[1], s.tool_offset[2]/25.4) or\
                       last_first_traverse[7] != (self.twp_x, self.twp_y, self.twp_z, self.twp_r, self.twp_i):
                    self.inject_initial_offsets()
                    self.transform_preview_segments()
                    self.calculate_joint_extents()
                    self.fingerprint_exp = (self.canon.traverse_exp, self.canon.feed_exp, self.canon.arcfeed_exp)
                    last_first_traverse = self.traverse_injected[0]
        # get the current G54 offset values to work around the preview positioning bug in gmoccapy
        if s.g5x_index == 1:
            self.g54_x = self.g5x_offset[0]
            self.g54_y = self.g5x_offset[1]
            self.g54_z = self.g5x_offset[2]

        # we only want to use the limit values when they are not loosend for tool kinematics
        if not self.twp_active:
                self.machine_limit_min, self.machine_limit_max = self.soft_limits()

        # coordinates symbol with letters is constructed here but drawn later
        alist = self.dlist(('axes', self.get_view()), gen=self.draw_axes)
        # coordinates symbol w/o letters is constructed here but drawn later
        anlist = self.dlist(('axes_no_letters', self.get_view()), gen=self.draw_axes_no_letters)
        # origin symbol is constructed here but drawn later
        olist = self.dlist('draw_small_origin', gen=self.draw_small_origin)

        glDisable(GL_LIGHTING)
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_BLEND)
        glBlendFunc(GL_ONE, GL_CONSTANT_ALPHA)

        # this should should set the view to  to the current origin 
        if self.machine_homed:
            if self.twp_active:
                R_g92 = (self.kins_tool_transformation(self.twp_i,
                                                        self.twp_r,
                                                        self.point_to_matrix([self.g92_offset[0],
                                                                              self.g92_offset[1], 
                                                                              self.g92_offset[2]]),
                                                        'fwd'))
                self.center_view_offset = ((self.saved_x - self.g5x_offset[0] + self.twp_x + R_g92[0,3]),
                                           (self.saved_y - self.g5x_offset[1] + self.twp_y + R_g92[1,3]),
                                           (self.saved_z - self.g5x_offset[2] + self.twp_z + R_g92[2,3]))
            else:
                self.center_view_offset = ((self.g92_offset[0]),
                                           (self.g92_offset[1]),
                                           (self.g92_offset[2]))

        else:
            self.center_view_offset = ((-self.g5x_offset[0]),
                                       (-self.g5x_offset[1]),
                                       (-self.g5x_offset[2]))
        glTranslatef(*[-x for x in self.center_view_offset ])

        if self.twp_active:
            cb = math.cos(math.radians(self.twp_i))
            sb = math.sin(math.radians(self.twp_i))
            self.jdx =   sb * (self.pivot_z + s.tool_offset[2]/25.4) 
            self.jdz = (cb - 1) *(self.pivot_z + s.tool_offset[2]/25.4)

        # TOOL And Tool Movement
        # pos = absolute position - tool_offset
        pos = self.lp.last(self.get_show_live_plot())
        if pos is None: pos = [0] * 6
        rx, ry, rz = pos[3:6]
        pos = self.to_internal_units(pos[:3])
        glPushMatrix()
        # NOTE: This is for the tool and the tool-MOVEMENT
        if self.twp_active:
            if self.kins_type == 2:
                # rotated by the twp inclination and rotation
                R_pos_transformed = (self.kins_tool_transformation(self.twp_i,
                                                                   self.twp_r,
                                                                   self.point_to_matrix(pos),
                                                                   'fwd'))
                glTranslatef(R_pos_transformed[0,3], R_pos_transformed[1,3], R_pos_transformed[2,3])
            else:
                glTranslatef(*pos)  
        else:
            glTranslatef(*pos)

        glEnable(GL_BLEND)
        glEnable(GL_CULL_FACE)
        glBlendFunc(GL_ONE, GL_CONSTANT_ALPHA)
        glBlendColor(0,0,0,1) #Note the last (alpha) value can really messup the whole preview (ie if set to 0 

        glPushMatrix()
        # NOTE: This is for the tool-CYLINDER
        if self.twp_active:
            glRotatef(self.twp_i,0,1,0) 
            if self.kins_type != 2:
                glTranslatef(-self.jdx, 0, self.jdz)    
        current_tool = self.get_current_tool()
        if current_tool is None or current_tool.diameter == 0:
            if self.canon:
                g = self.canon
                x,y,z = 0,1,2
                cone_scale = max(g.max_extents[x] - g.min_extents[x],
                               g.max_extents[y] - g.min_extents[y],
                               g.max_extents[z] - g.min_extents[z],
                               2 ) * self.cone_basesize
            else:
                cone_scale = 1
            cone = self.dlist("cone", gen=self.make_cone)
            glScalef(cone_scale, cone_scale, cone_scale)
            glCallList(cone)
        else:
            if current_tool != self.cached_tool:
                self.cache_tool(current_tool)   
            glEnable(GL_LIGHTING)
            glEnable(GL_COLOR_MATERIAL)
            if self.twp_active and self.kins_type != 2:
                glColor3f(1.,1.,1.)
            else:
                glColor3f(*self.colors['tool'])
            glCallList(self.dlist('tool'))
            glDisable(GL_COLOR_MATERIAL)
            glDisable(GL_LIGHTING)       
        glPopMatrix()
        glPopMatrix()
        # /TOOL
 
        # SPINDLE
        if self.show_live_plot:
            glEnable(GL_LIGHTING)
            glEnable(GL_COLOR_MATERIAL)
            pos = self.lp.last(self.get_show_live_plot())
            if pos is None: pos = [0] * 6
            rx, ry, rz = pos[3:6]
            pos = self.to_internal_units(pos[:3])
            glPushMatrix()
            # NOTE: This is for the spindle-MOVEMENT
            if self.twp_active: 
                if self.kins_type == 2:
                    # rotated by the twp inclination and rotation
                    R_pos_transformed = (self.kins_tool_transformation(self.twp_i,
                                                                       self.twp_r,
                                                                       self.point_to_matrix(pos),
                                                                       'fwd'))
                    glTranslatef(R_pos_transformed[0,3], R_pos_transformed[1,3], R_pos_transformed[2,3])
                else:
                    glTranslatef(*pos)
            else:
                glTranslatef(*pos)
            glPushMatrix()
            # NOTE: This is for the spindle-GEOMETRY
            if self.twp_active and self.kins_type != 2:
                # indicator for the control point
                glPushMatrix()
                glTranslatef(self.jdx, 0, self.jdz)    
                glTranslatef(-self.jdx, 0, -self.jdz)
                glColor3f(*self.colors['cone'])
                radius = 4/25.4
                sphere(0,0,0,radius) #sphere(x, y, z, r)
                glColor3f(0.7,0.7,0)
                cylinder_z(radius, 1/25.4, s.tool_offset[2]/25.4 + self.pivot_z, 1/25.4) #(z1, r1, z2, r2)
                glPopMatrix()
            # Spindle body
            glColor3f(0.7,0.7,0)
            if self.twp_active:
                # rotated by the twp inclination and rotation
                glRotatef(self.twp_i,0,1,0) 
                if self.kins_type != 2:
                    glTranslatef(-self.jdx, 0, self.jdz)
            cylinder_z(s.tool_offset[2]/25.4, 25/25.4, s.tool_offset[2]/25.4 + 30/25.4, 50/25.4)
            cylinder_z((s.tool_offset[2] + 30)/25.4, 60/25.4,
                       (s.tool_offset[2] + 30)/25.4 + self.pivot_z + (30/25.4 + self.pivot_z)/2 , 60/25.4)
            glPopMatrix()
            glPopMatrix()
            glDisable(GL_LIGHTING)
            glDisable(GL_COLOR_MATERIAL)
        # /SPINDLE

        # CLASSIC PREVIEW 
        if False: #not self.config_with_twp:
            glPushMatrix()
            if self.get_show_rapids():
                glCallList(self.dlist('program_rapids', gen=self.make_main_list))
            glCallList(self.dlist('program_norapids', gen=self.make_main_list))
            glCallList(self.dlist('highlight'))
            if self.get_program_alpha():
                glDisable(GL_BLEND)
                glEnable(GL_DEPTH_TEST)

            if self.get_show_extents():
                self.show_extents()
            glPopMatrix()
        # /CLASSIC PREVIEW

        # MYPREVIEW 
        if True: #self.config_with_twp and self.machine_homed:
            glPushMatrix()
            if hasattr(self, 'traverse_transformed'):
                for lineno, start, end in self.traverse_transformed:
                    glColor3f(.9,.9,0)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
                for lineno, start, end in self.feed_transformed:
                    glColor3f(1,1,1)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
                for lineno, start, end in self.arcfeed_transformed:
                    glColor3f(1,1,1)
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
            if hasattr(self, 'twp_highlight_line'):
                for start, end in self.twp_highlight_line:
                    glLineWidth(3)
                    glColor3f(*self.colors['selected'])
                    glBegin(GL_LINES)
                    glVertex3f(start[0], start[1], start[2])
                    glVertex3f(end[0], end[1], end[2])
                    glEnd()
            glPopMatrix()
        # /MYPREVIEW
            
        # TWP PLANE tilted work plane as a grid centered in the origin of the rotated work space (G59)
        if self.config_with_twp and self.machine_homed :
            glLineWidth(1)
            glPushMatrix()
            # rotated by the twp inclination and rotation
            glRotatef(self.twp_i,0,1,0) 
            glRotatef(self.twp_r,0,0,1) 
            # Drop twp slightly below z0 so the grid does not
            # cover up the lines from the coordinate system or the offset
            glTranslatef(0,0,-0.01)
            glTranslatef(*self.g5x_offset)
            # draw grid
            if self.twp_active:
                glColor3f(*self.colors['twp_active'])
                if self.kins_type != 2:
                    glColor3f(0.5,0,0)
            else:
                glColor3f(*self.colors['twp_inactive'])
            grid_distance = 10 # distance from one line to the next in mm 
            grid_size = 500 # distance from the center to the egde
            grid_scale = 10/25.4
            glScalef(grid_scale, grid_scale, grid_scale)
            glBegin(GL_LINES);
            mys= int(grid_size/grid_distance)
            for i in range (-mys,mys+1,1):
                # line 1 in x direction
                glVertex3f(-mys, i, 0)
                glVertex3f( mys, i, 0)
                # line 1 in y direction
                glVertex3f( i,-mys, 0)
                glVertex3f( i, mys, 0)
            glEnd()
            glPopMatrix()
        # /TWP PLANE

        # /MACHINE ORIGIN AND OFFSETS
        glPushMatrix()
        # draw the origin symbol
        glCallList(olist)
        glPopMatrix()
        if self.machine_homed:
            # CURRENT OFFSET G5x
            glPushMatrix()
            # rotated by the twp inclination and rotation
            if self.kins_type == 2:
                glRotatef(self.twp_i,0,1,0) 
            glRotatef(self.twp_r,0,0,1) 
            glBegin(GL_LINES)
            glVertex3f(0,0,0)
            glVertex3f(*self.g5x_offset)
            glEnd()
            i = s.g5x_index
            if i<7:
                label = "G5%d" % (i+3)
            else:
                label = "G59.%d" % (i-6)
            glTranslatef(self.g5x_offset[0] * 0.5, self.g5x_offset[1] * 0.5, self.g5x_offset[2] * 0.5)
            glScalef(0.2,0.2,0.2)
            g5xrot=math.atan2(self.g5x_offset[1], self.g5x_offset[0])
            glRotatef(math.degrees(g5xrot), 0, 0, 1)
            glTranslatef(0.5, 0.5, 0)
            self.hershey.plot_string(label, 0.1)
            glPopMatrix()
            # /CURRENT OFFSET G5x

            # SAVED TWP OFFSET G54..G58
            if self.show_live_plot and self.twp_active:
                glBegin(GL_LINES)
                glVertex3f(0,0,0)
                glVertex3f(self.saved_x, self.saved_y, self.saved_z)
                glEnd()
                i = self.saved_index
                if i<7:
                    label = "G5%d" % (i+3)
                else:
                    label = "G59.%d" % (i-6)
                glPushMatrix()
                glTranslatef(self.saved_x * 0.5, self.saved_y * 0.5, self.saved_z * 0.5)
                glScalef(0.2,0.2,0.2)
                g5xrot=math.atan2(self.saved_y, self.saved_x)
                glRotatef(math.degrees(g5xrot), 0, 0, 1)
                glTranslatef(0.5, 0.5, 0)
                self.hershey.plot_string(label, 0.1)
                glPopMatrix()
            # /SAVED TWP OFFSET G54..G58

            # TWP OFFSET G68.2
            if self.show_live_plot and self.twp_active:
                glPushMatrix()
                glTranslatef(self.saved_x, self.saved_y, self.saved_z)
                glBegin(GL_LINES)
                glVertex3f(0,0,0)
                glVertex3f(self.twp_x ,self.twp_y, self.twp_z)
                glEnd()
                label = "G68.2"
                glTranslatef(self.twp_x * 0.5, self.twp_y * 0.5, self.twp_z * 0.5)
                glScalef(0.2,0.2,0.2)
                g5xrot=math.atan2(self.twp_y, self.twp_x)
                glRotatef(math.degrees(g5xrot), 0, 0, 1)
                glTranslatef(0.5, 0.5, 0)
                self.hershey.plot_string(label, 0.1)
                glPopMatrix()
            # /TWP OFFSET G68.2

            # OFFSET G52/G92
            if  self.g92_offset[0] or self.g92_offset[1] or self.g92_offset[2]:
                glPushMatrix()
                # rotated by the twp inclination and rotation
                if self.kins_type == 2:
                    glRotatef(self.twp_i,0,1,0)  
                glRotatef(self.twp_r,0,0,1)
                glTranslatef(self.g5x_offset[0], self.g5x_offset[1], self.g5x_offset[2])
                glRotatef(self.rotation_xy,0,0,1) 
                glBegin(GL_LINES)
                glVertex3f(0,0,0)
                glVertex3f(self.g92_offset[0], self.g92_offset[1], self.g92_offset[2]) 
                glEnd()
                label = "G92.G52"
                glTranslatef(self.g92_offset[0] * 0.1, self.g92_offset[1] * 0.1, self.g92_offset[2] * 0.1)
                glScalef(0.2,0.2,0.2)
                g92rot=math.atan2(self.g92_offset[1], self.g92_offset[0])
                glRotatef(math.degrees(g92rot), 0, 0, 1)
                glTranslatef(0.5, 0.5, 0)
                self.hershey.plot_string(label, 0.1)
                glPopMatrix()
            # /OFFSET G52/G92

            # COORDS for the original G5x offset 
            if self.show_live_plot:
                glPushMatrix()
                glTranslatef(self.saved_x, self.saved_y, self.saved_z)
                glRotatef(self.rotation_xy,0,0,1) 
                # draw coordinates 
                glCallList(anlist)
                glPopMatrix()
                # /COORDS for the original G5x offset 

            # COORDS for the current WCS w/o G52/G92
            if self.show_live_plot:
                glPushMatrix()
                # rotated by the twp inclination and rotation
                if self.kins_type == 2:
                    glRotatef(self.twp_i,0,1,0) 
                glRotatef(self.twp_r,0,0,1) 
                glTranslatef(self.g5x_offset[0], self.g5x_offset[1], self.g5x_offset[2])
                glRotatef(self.rotation_xy,0,0,1) 
                # draw coordinates 
                glCallList(anlist)
                glPopMatrix()
                # /COORDS for the original G5x offset 

            # COORDS for the current origin with all offsets 
            glPushMatrix()
            # rotated by the twp inclination and rotation
            if self.kins_type == 2:
                glRotatef(self.twp_i,0,1,0) 
            glRotatef(self.twp_r,0,0,1) 
            glTranslatef(self.g5x_offset[0],
                         self.g5x_offset[1],
                         self.g5x_offset[2])
            glRotatef(self.rotation_xy,0,0,1) 
            glTranslatef(self.g92_offset[0],
                         self.g92_offset[1],
                         self.g92_offset[2])
            # draw coordinates with letters
            glCallList(alist)
            glPopMatrix()
            # /COORDS for the original G5x offset 


            # MACHINE LIMITS       
            glPushMatrix()
            if self.limit_violation:
                glColor3f(*self.colors['limits'])
                glLineWidth(2)
                glTranslatef(0,0, -self.data_limit_error[1])
                angle_xz = math.radians(self.data_limit_error[0])
                cb = math.cos(angle_xz)
                sb = math.sin(angle_xz)
                dx =   sb * (self.pivot_z + self.data_limit_error[1])
                dz = (cb - 1) * (self.pivot_z + self.data_limit_error[1])
                glTranslatef(- dx,
                             - 0,
                             - dz)
            else:
                glColor3f(0,0.25,0)
                glLineWidth(1)
                glTranslatef(0,0, -self.tool_offset)
                if self.twp_active:
                    angle_xz = math.radians(self.twp_i)
                    cb = math.cos(angle_xz)
                    sb = math.sin(angle_xz)
                    self.jdx =   sb * (self.pivot_z + self.tool_offset)
                    self.jdz = (cb - 1) * (self.pivot_z + self.tool_offset)
                    glTranslatef(- self.jdx,
                                 - 0,
                                 - self.jdz)

            glBegin(GL_LINES)

            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_max[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_max[2])


            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_max[2])

            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_max[2])


            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_min[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_max[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_max[1], self.machine_limit_max[2])

            glVertex3f(self.machine_limit_min[0], self.machine_limit_min[1], self.machine_limit_max[2])
            glVertex3f(self.machine_limit_max[0], self.machine_limit_min[1], self.machine_limit_max[2])
            glEnd()
            
            glColor3f(*self.colors['limits_fill'])
            glBegin(GL_QUADS)
            # bottom face
            glNormal3f(0,0,-1)
            a = 250 / 25.4
            glVertex3f(self.machine_limit_min[0] - 1.5 * a, self.machine_limit_min[1] - a, self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0] + 1.5 * a, self.machine_limit_min[1] - a, self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0] + 1.5 * a, self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_min[0] - 1.5 * a, self.machine_limit_max[1], self.machine_limit_min[2])
            glEnd()

            c = self.colors['limits_fill']
            glColor3f(c[0]-0.05, c[1]-0.05, c[2]-0.05 )
            glBegin(GL_QUADS)        # positive Y face
            glNormal3f(0,1,0)
            glVertex3f(self.machine_limit_min[0] - a, self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0] + a, self.machine_limit_max[1], self.machine_limit_min[2])
            glVertex3f(self.machine_limit_max[0] + a, self.machine_limit_max[1], self.machine_limit_max[2] + a)
            glVertex3f(self.machine_limit_min[0] - a, self.machine_limit_max[1], self.machine_limit_max[2] + a)
            glEnd()
            
            glPopMatrix()
            # /MACHINE LIMITS  

        # LIVE PLOT
        # this is required for the tool to move
        glDepthFunc(GL_LEQUAL)
        glLineWidth(3)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_BLEND)
        glPushMatrix()
        # rotated by the twp inclination and rotation
        if self.twp_active:
            if self.kins_type == 2:
                glRotatef(self.twp_i,0,1,0)
            else:
                glTranslatef(-self.jdx, 0, -self.jdz)    
        glRotatef(self.twp_r,0,0,1) 
        lu = 1/((s.linear_units or 1)*25.4)
        glScalef(lu, lu, lu)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glTranslatef(0,0,.003)

        self.lp.call()

        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        glDisable(GL_BLEND)
        glLineWidth(1)
        glDepthFunc(GL_LESS)
        # /LIVE PLOT

        # OVERLAY DRO
        limit, homed, posstrs, droposstrs = self.posstrs()
        maxlen = max([len(p) for p in posstrs])
        base, charwidth, linespace = glnav.use_pango_font('monospace 8', 0, 128)
        pixel_width = charwidth * max(len(p) for p in posstrs)
        glColor3f(*self.colors['overlay_foreground']) # not working
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        ypos = self.winfo_height()
        glOrtho(0.0, self.winfo_width(), 0.0, ypos, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        # allows showing/hiding overlay DRO readout
        if self.show_overlay:
            maxlen = 0
            ypos -= linespace + 5
            xpos = 15
            for string in posstrs:
                maxlen = max(maxlen, len(string))
                glRasterPos2i(xpos, ypos)
                for char in string:
                    glCallList(base + ord(char))
                ypos -= linespace
            self.show_overlay = True
        else:
            self.show_overlay = False
        glPopMatrix()
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        # /OVERLAY DRO
        
def cylinder_z(z1, r1, z2, r2):
    if z1 > z2:
        tmp = z1
        z1 = z2
        z2 = tmp
        tmp = r1
        r1 = r2
        r2 = tmp
    # need to translate the whole thing to z1
    glPushMatrix()
    glTranslatef(0,0,z1)
    # the cylinder starts out at Z=0
    gluCylinder(gluNewQuadric(), r1, r2, z2-z1, 32, 1)
    # bottom cap
    glRotatef(180,1,0,0)
    gluDisk(gluNewQuadric(), 0, r1, 32, 1)
    glRotatef(180,1,0,0)
    # the top cap needs flipped and translated
    glPushMatrix()
    glTranslatef(0,0,z2-z1)
    gluDisk(gluNewQuadric(), 0, r2, 32, 1)
    glPopMatrix()
    glPopMatrix()

def sphere(x, y, z, r):
    # need to translate the whole thing to x,y,z
    glPushMatrix()
    glTranslatef(x,y,z)
    # the sphere starts out at the origin
    gluSphere(gluNewQuadric(), r, 32, 16)
    glPopMatrix()
# vim:ts=8:sts=4:sw=4:et:
