#!/usr/bin/python
#
# Script to connect using vridge-api, and position the headset in VR space
#
# (c) Simon Wood, 16 Nov 2020

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *

import argparse
import zmq
import time
import math

# we will use OpenVR to sense HMD VSync timing
try:
	import openvr
	_hasOpenVR = True
except ImportError:
	_hasOpenVR = False
'''
_hasOpenVR = False
'''

# Commandline options
parser = argparse.ArgumentParser(prog='reposition.py')

# specific an initial position
parser.add_argument("-x", "--xpos", dest="X", type=float, default="0.0",
	help="Specify an initial X position (float in Meters)")
parser.add_argument("-y", "--ypos", dest="Y", type=float, default="1.2",
	help="Specify an initial Y position (float in Meters)")
parser.add_argument("-z", "--zpos", dest="Z", type=float, default="0.0",
	help="Specify an initial Z position (float in Meters)")

parser.add_argument("-P", "--pitch", dest="pitch", type=float, default="0.0",
	help="Specify an initial  pitch (+ve = up, float in degrees)")
parser.add_argument("-Y", "--yaw", dest="yaw", type=float, default="0.0",
	help="Specify an initial yaw (+ve = left, float in degrees)")
parser.add_argument("-R", "--roll", dest="roll", type=float, default="0.0",
	help="Specify an initial roll (+ve = left hand down, float in degrees)")

parser.add_argument("-s", "--server", dest="server", default="localhost",
	help="Use particular server (instead of localhost)" )
parser.add_argument("-f", "--fps", dest="fps", type=float, default="60.0",
	help="Specify an approximate fps, ie. set frequency that reports are sent")
parser.add_argument("-t", "--offset", dest="offset", type=float, default="0.0",
	help="Specify an time offset for the timing of HMD VSync" )
parser.add_argument("-S", "--spin", action="store_true", dest="spin",
	help="Do a spin test, turn around for 10s before exiting" )

options = parser.parse_args()

# -------------------------------
# Make a connection to VRidge-API server
# may need Windows firewall rules to allow

addr = 'tcp://' + options.server + ':38219'
print ("Connecting to", addr)

ctx = zmq.Context()

# -------------------------------
# connect to the control channel
control = ctx.socket(zmq.REQ)
control.connect(addr)

# Sleep to allow sockets to connect.
#time.sleep(1.0)

control.send_json({ "ProtocolVersion":1, "Code":2 })
answer = control.recv_json()

control.send_json({ "RequestedEndpointName":"HeadTracking", "ProtocolVersion":1, "Code":1 })
answer = control.recv_json()

# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)
headset.connect(answer['EndpointAddress'])

# SendRadRotationAndPosition
anglesposition = Struct(
	Const(2, Int32ul),  # Version
	Const(3, Int32ul),  # SendRadRotationAndPosition
	Const(24, Int32ul), # DataLength
	"data" / Padded(64, Array(6, Float32l)),
)

justposition = Struct(
	Const(2, Int32ul),  # Version
	Const(5, Int32ul),  # SendPosition
	Const(12, Int32ul), # DataLength
	"data" / Padded(64, Array(3, Float32l)),
)

# set the initial position
if options.pitch or options.yaw or options.roll:
    output = anglesposition.build(dict(data=[ \
		math.radians(options.pitch), \
		math.radians(options.yaw), \
		math.radians(options.roll), \
		options.X, \
		options.Y, \
		options.Z ]))
else:
	output = justposition.build(dict(data=[ \
		options.X, \
		options.Y, \
		options.Z ]))

headset.send(output)
answer = headset.recv()

if options.fps < 1.0:
	options.fps = 1.0

def float_range(start, stop, step):
	while start < stop:
		yield float(start)
		start += step

if options.spin:
	if _hasOpenVR:
		openvr.init(openvr.VRApplication_Overlay)

	# Spin 360 degrees, takes 10s, step size is calculated for FPS
	for swivel in float_range(0.0, 10.0, 1.0/options.fps):
		output = anglesposition.build(dict(data=[ \
                        math.radians(options.pitch),math.radians(36 * swivel), \
			math.radians(options.roll), \
			options.X, options.Y, options.Z ]))
		headset.send(output)
		answer = headset.recv()

		if _hasOpenVR:
			time_since = openvr.VRSystem().getTimeSinceLastVsync()
			to_sleep = (1.0/options.fps) - time_since[1] + options.offset
			#print("to_sleep", to_sleep, "frame", time_since[2])

			if to_sleep > 0.0:
				time.sleep(to_sleep)
		else:
			time.sleep(1.0/options.fps)

headset.close()
control.close()
ctx.destroy()
