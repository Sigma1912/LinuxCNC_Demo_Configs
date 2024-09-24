#!/usr/bin/env python3

from twp_vismach import *
import hal
import math
import sys
import os

PATH = os.environ.get('CONFIG_DIR')

try: # Expect files in working directory
    # NOTE to run this file as standalone python script absolute paths might have to be used to find the obj files

    stlfile = os.path.join(PATH, "obj files/head_notool.stl")
    axisz1 = AsciiSTL(filename=stlfile)
    axisz2 = AsciiSTL(filename=stlfile)
    axisz3 = AsciiSTL(filename=stlfile)

    objfile = os.path.join(PATH, "obj files/head.obj")
    head  = AsciiOBJ(filename=objfile)
    objfile = os.path.join(PATH, "obj files/gantri.obj")
    rack  = AsciiOBJ(filename=objfile)
    objfile = os.path.join(PATH, "obj files/rangka.obj")
    rangka  = AsciiOBJ(filename=objfile)
    objfile = os.path.join(PATH, "obj files/bed.obj")
    bed  = AsciiOBJ(filename=objfile)
    objfile = os.path.join(PATH, "obj files//atc.obj")
    atc  = AsciiOBJ(filename=objfile)


except Exception as detail:
    print(detail)
    raise SystemExit("3axis-multiheadgui requires obj files in working directory")



c = hal.component("xyz-multihead-gui")
c.newpin("joint0", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint1", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint2", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint3", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("joint4", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("kinstype", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_diameter", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_length", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("tool_x", hal.HAL_FLOAT, hal.HAL_IN)
c.ready()



class HalToolCylinder(CylinderZ):
    def __init__(self, comp, *args):
        CylinderZ.__init__(self, *args)
        self.comp = c

    def coords(self):
        r = 6 # default if hal pin not set
        if (c.tool_diameter > 0): r = c.tool_diameter/2
        return c.tool_length, r, 0, r






work = Capture()
tool = Capture()
tooltip = Capture()

# kinematic axis
# axis_y -> axis_x -> axis_z  -> tool
# base -> work

# tool
tool_cylinder = HalToolCylinder(c)
tool = Collection([tooltip, tool, tool_cylinder])
tool = Translate([tool],-200,0,0)
tool = HalTranslate([tool],c,"kinstype",200,0,0)
tool = HalTranslate([tool],c,"tool_length",0,0,-1)
tool = HalTranslate([tool],c,"joint0",1,0,0)
tool = HalTranslate([tool],c,"joint1",0,1,0)
tool = HalTranslate([tool],c,"joint2",0,0,1)
tool = HalTranslate([tool],c,"joint3",0,0,1)
tool = HalTranslate([tool],c,"joint4",0,0,1)

# axis z1
axisz1 = Color([1,1,1,1],[axisz1])
axisz1 = Rotate([axisz1],90,1,0,0)
axisz1 = Translate([axisz1],0,0,48)
axisz1 = HalTranslate([axisz1],c,"joint2",0,0,1)

# axis z2
axisz2 = Color([1,1,1,1],[axisz2])
axisz2 = Rotate([axisz2],90,1,0,0)
axisz2 = Translate([axisz2],0,0,48)
axisz2 = HalTranslate([axisz2],c,"joint3",0,0,1)

# axis z3
axisz3 = Color([1,1,1,1],[axisz3])
axisz3 = Rotate([axisz3],90,1,0,0)
axisz3 = Translate([axisz3],0,0,48)
axisz3 = HalTranslate([axisz3],c,"joint4",0,0,1)

# axis x1
head = Color([0.2,0.2,1,1],[head])
head = Rotate([head],90,1,0,0)
axisx1 = Collection([axisz1, head])
axisx1 = Translate([axisx1],-200,0,0)
axisx1 = HalTranslate([axisx1],c,"joint0",1,0,0)

# axis x2
head2 = Color([0.5,0.5,1,1],[head])
axisx2 = Collection([axisz2, head2])
axisx2 = Translate([axisx2],0,0,0)
axisx2 = HalTranslate([axisx2],c,"joint0",1,0,0)

# axis x3
head3 = Color([0.8,0.8,1,1],[head])
axisx3 = Collection([axisz3, head3])
axisx3 = Translate([axisx3],200,0,0)
axisx3 = HalTranslate([axisx3],c,"joint0",1,0,0)

# axis y
rack = Color([0.7,0.7,0.4,0.4],[rack])
rack = Rotate([rack],90,1,0,0)
axisy = Collection([axisx1, axisx2, axisx3, rack])
axisy = HalTranslate([axisy],c,"joint1",0,1,0)
machine = Collection([axisy,tool])

# base
rangka = Color([1,1,1,1],[rangka])
rangka = Rotate([rangka],90,1,0,0)


bed = Color([1,0.8,0.2,0.2],[bed])
bed = Rotate([bed],90,1,0,0)


atc = Color([0.5,0.5,0.5,0.5],[atc])
atc = Rotate([atc],90,1,0,0)

work = Capture()

base = Collection([rangka, bed, atc, work])

model = Collection([base, machine])

main(model, tooltip, work, size=4000, lat=-60, lon=25)

