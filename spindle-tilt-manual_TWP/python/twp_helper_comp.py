#!/usr/bin/env python3
import time
import hal
import linuxcnc
c = linuxcnc.command() # create a connection to the command channel
s = linuxcnc.stat() # create a connection to the status channel
h = hal.component("twp_helper_comp")

# these are fed directly from the remap.py script
h.newpin("saved-offset-index", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("saved-offset-x", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("saved-offset-y", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("saved-offset-z", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("saved-xy-rot", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("twp-i-in", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("twp-r-in", hal.HAL_FLOAT, hal.HAL_IN)

# expose these values as halpins
h.newpin("twp-i-out", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("twp-r-out", hal.HAL_FLOAT, hal.HAL_OUT)


h.newpin("g5x-index", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g5x-x", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g5x-y", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g5x-z", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("xy-rot", hal.HAL_FLOAT, hal.HAL_OUT)

h.newpin("rel-position-x", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("rel-position-y", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("rel-position-z", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("offset-index", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g52-g92-x", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g52-g92-y", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("g52-g92-z", hal.HAL_FLOAT, hal.HAL_OUT)

h.newpin("pivot-z-in", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("pivot-z-out", hal.HAL_FLOAT, hal.HAL_OUT)

h.newpin("twp-error", hal.HAL_FLOAT, hal.HAL_IO)

h.newpin("twp-x-in", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("twp-y-in", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("twp-z-in", hal.HAL_FLOAT, hal.HAL_IN)
h.newpin("twp-x-out", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("twp-y-out", hal.HAL_FLOAT, hal.HAL_OUT)
h.newpin("twp-z-out", hal.HAL_FLOAT, hal.HAL_OUT)

h.newpin("machine-is-on", hal.HAL_BIT, hal.HAL_IN)
h.newpin("twp-is-active-in", hal.HAL_BIT, hal.HAL_IN)
h.newpin("twp-is-active-out", hal.HAL_BIT, hal.HAL_OUT)

h.newpin("wrong-offset-with-twp-active", hal.HAL_BIT, hal.HAL_OUT)

h.ready()

cnt_rst = 0

try:
    while 1:

        # Handle error messages coming from the remap
        if h['twp-error'] == 0:
            pass
        elif h['twp-error'] == 1:
            c.error_msg('G68.2 ERROR: TWP already active.')  
        elif h['twp-error'] == 2:
            c.error_msg('G68.2 ERROR: Cannot define TWP from active G59 or G59.x offsets.')         
        elif h['twp-error'] == 3:
            c.error_msg("G68.2 ERROR: Can't enable TWP with G91 active.")
        h['twp-error'] = 0

        # passthrough the twp arguments
        h['twp-x-out'] = h['twp-x-in']
        h['twp-y-out'] = h['twp-y-in']
        h['twp-z-out'] = h['twp-z-in']
        h['twp-r-out'] = h['twp-r-in']
        h['twp-i-out'] = h['twp-i-in']

        # passthrough the twp state
        h['twp-is-active-out'] = h['twp-is-active-in']

        # passthrough the pivot length
        h['pivot-z-out'] = h['pivot-z-in']

        s.poll() # get current values
        current_index = s.g5x_index          
        current_offset = s.g5x_offset 
        current_xy_rotation = s.rotation_xy        
        g52_g92_offset = s.g92_offset
        actual_position = s.actual_position
        tool_offset = s.tool_offset
        # expose g52/g92 offset for vismach panel
        if current_index <= 6:
            h['offset-index'] = 53 + current_index 
        else:
            h['offset-index'] = 59 + (current_index -6)/10 
        # calculate relative position for vismach panel
        h['rel-position-x'] = actual_position[0]-current_offset[0]-g52_g92_offset[0]
        h['rel-position-y'] = actual_position[1]-current_offset[1]-g52_g92_offset[1]
        h['rel-position-z'] = actual_position[2]-current_offset[2]-g52_g92_offset[2]-tool_offset[2]
        # expose g52/g92 offset for vismach panel
        h['g52-g92-x'] = g52_g92_offset[0]
        h['g52-g92-y'] = g52_g92_offset[1]
        h['g52-g92-z'] = g52_g92_offset[2]

        # Bad things can happen if the user activates any other offset than G59 (G59.x) or
        # there is a gcode error, estop or other unusual behavior so we want to provide a pin that 
        # can can be used to reset twp if that happens
        # cnt_rst pulses the pin when active to make it more robust at machine startup/restart
        if h['machine-is-on'] and h['twp-is-active-in'] and current_index < 6 and cnt_rst <= 10000: 
            h['wrong-offset-with-twp-active'] = True
            cnt_rst += 1
        elif cnt_rst > 10000 : 
            h['wrong-offset-with-twp-active'] = False 
            cnt_rst += 1
            if cnt_rst > 20000:
                cnt_rst = 0   

        # we only want to expose offsets when twp is not active       
        if not h['twp-is-active-in']:

            h['g5x-index'] = current_index + 53
            h['g5x-x'] = current_offset[0]
            h['g5x-y'] = current_offset[1]
            h['g5x-z'] = current_offset[2]
            h['xy-rot'] = current_xy_rotation

        else : #freeze the output

            h['g5x-index'] = h['saved-offset-index'] + 53
            h['g5x-x'] = h['saved-offset-x']
            h['g5x-y'] = h['saved-offset-y']
            h['g5x-z'] = h['saved-offset-z']
            h['xy-rot'] = h['saved-xy-rot']

except KeyboardInterrupt:
    raise SystemExit
