#!/usr/bin/python
#
# Script to connect using vridge-api, create 2 controllers and swivel them
#
# (c) Simon Wood, 18 June 2017

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *

import zmq
import time
from math import radians
from pyrr import Quaternion, Matrix44, Vector3
import numpy as np

addr = 'tcp://localhost:38219'
print ("Connecting to", addr)

ctx = zmq.Context()

# -------------------------------
# connect to the control channel
master = ctx.socket(zmq.REQ)
master.connect(addr)

master.send_json({ "ProtocolVersion":1, "Code":2 })
answer = master.recv_json()

master.send_json({ "RequestedEndpointName":"HeadTracking", "ProtocolVersion":1, "Code":1 })
answer = master.recv_json()
headset_addr = answer['EndpointAddress']

master.send_json({ "RequestedEndpointName":"Controller", "ProtocolVersion":1, "Code":1 })
answer = master.recv_json()
controller_addr = answer['EndpointAddress']

print("Found:")
print("Headset", headset_addr)
print("Controller", controller_addr)

# -------------------------------
# Connect to controller
controller = ctx.socket(zmq.REQ)
controller.connect(controller_addr)

controller_packet = Struct(
	Const(Int32ul, 2),		# Version
	Padded(2, Const(Byte, 1)),	# SendFullState
	Padded(2, Const(Byte, 0)),	# ControllerOrigin

	"id" / Int32ul,			# ControllerId
	Const(Int32ul, 0),		# Status
	"orient" / Array(16, Float32l),	# OrientationMatrix
	"count" / Int32ul,		# PacketNum

	"pressed" / Int64ul,		# ButtonPressed
	"touched" / Int64ul,		# ButtonTouched
	"axis" / Padded(40, Array(3, Float32l)),	# we only use 1st 3 of 10 floats
)

# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)
headset.connect(headset_addr)

# SendRadRotationAndPosition
head_packet = Struct(
	Const(Int32ul, 2),		# Version
	Padded(4, Const(Byte, 3)),	# SendRadRotationAndPosition
	Const(Int32ul, 24),		# DataLength
	"data" / Padded(64, Array(6, Float32l)),
)

# -------------------------------
# position view point
output = head_packet.build(dict(data=[radians(0),radians(90),radians(0),2,1,0]))
headset.send(output)
answer = headset.recv()

count = 0
for swivel in range(0,960,2):
	# location [side, up, forward]
	matrix1 = Matrix44.from_translation(Vector3([0,1,0]))

	# rotate [pitch, roll, yaw]
	matrix1 = matrix1 * Matrix44.from_eulers([radians(0),radians(0),radians(swivel)])

	controller1 = controller_packet.build(dict(
		id = 1,
		orient = matrix1.transpose().reshape(16).tolist(),
		count = count,
		pressed = 1,
		touched = 1,
		axis = [0, 0, 0],
		))
	controller.send(controller1)
	answer = controller.recv()

	matrix2 = Matrix44.from_translation(Vector3([0,1.5,0]))
	matrix2 = matrix2 * Matrix44.from_eulers([radians(0),radians(0),radians(-swivel)])

	controller2 = controller_packet.build(dict(
		id = 2,
		orient = matrix2.transpose().reshape(16).tolist(),
		count = count,
		pressed = 3,
		touched = 3,
		axis = [0, 0, 0],
		))
	controller.send(controller2)
	answer = controller.recv()

	count = count + 1
	time.sleep(0.1)

headset.close()
controller.close()
master.close()
ctx.destroy()
