#!/usr/bin/python
#
# Script to implement crude fake controller using VRidge-API and SDL2
#
# (c) Simon Wood, 19 June 2017

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *

import zmq
import time
from math import radians
from pyrr import Quaternion, Matrix44, Vector3
import numpy as np
import sdl2

addr = 'tcp://localhost:38219'

# -------------------------------
# Start SDL and find gamecontroller

sdl2.SDL_Init(sdl2.SDL_INIT_TIMER | sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_GAMECONTROLLER)

if (sdl2.SDL_NumJoysticks() == 0):
	print("No Joysticks found")
	sdl2.SDL_Quit()
	exit(0)

index = 0
joystick = sdl2.SDL_JoystickOpen(index);
if joystick == None:
	print("Unable to open device")
	sdl2.SDL_Quit()
	exit(0)
else:
	mappings = sdl2.SDL_GameControllerAddMappingsFromFile(b'gamecontrollerdb.txt')

	if sdl2.SDL_IsGameController(index) == 0:
		print("Device", index, "does not support GameController Mode")
		sdl2.SDL_Quit()
		exit(0)

	name = sdl2.SDL_GameControllerNameForIndex(index)
	print("Using Device", index, ":", name)

	gamecontroller = sdl2.SDL_GameControllerOpen(index)
	if gamecontroller == None:
		print("Unable to open gamecontroller")
		sdl2.SDL_Quit()
		exit(0)

# -------------------------------
# connect to the control channel
print ("Connecting to", addr)
ctx = zmq.Context()

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
output = head_packet.build(dict(data=[radians(0),radians(0),radians(0),0,2,0]))
headset.send(output)
answer = headset.recv()

last = 0
count = 0

left_moved = None
left_pitch = 0
left_yaw = 0
left_trigger = 0
left_buttons = 0

right_moved = None
right_pitch = 0
right_yaw = 0
right_trigger = 0
right_buttons = 0

while True:
	sdl2.SDL_PumpEvents()

	# Controller assignments:
	# Controller1 (left)
	# SDL_CONTROLLER_AXIS_RIGHTX
	# SDL_CONTROLLER_AXIS_RIGHTY
	# SDL_CONTROLLER_BUTTON_RIGHTSTICK
	# SDL_CONTROLLER_AXIS_TRIGGERRIGHT
	# SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
	# SDL_CONTROLLER_BUTTON_GUIDE

	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_RIGHTSTICK):
		right_buttons |= (1<<0) # System
	else:
		right_buttons &= ~(1<<0)
	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_GUIDE):
		right_buttons |= (1<<1) # Menu
	else:
		right_buttons &= ~(1<<1)
	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER):
		right_buttons |= (1<<2) # Grip
	else:
		right_buttons &= ~(1<<2)

	right_pitch = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_RIGHTY)
	right_yaw = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_RIGHTX)
	right_trigger = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_TRIGGERRIGHT)
	if right_trigger > 327:
		right_buttons |= (1<<33) # Axis1
	else:
		right_buttons &= ~(1<<33)

	# Contoller2 (right)
	# SDL_CONTROLLER_AXIS_LEFTX
	# SDL_CONTROLLER_AXIS_LEFTY
	# SDL_CONTROLLER_BUTTON_LEFTSTICK
	# SDL_CONTROLLER_AXIS_TRIGGERLEFT
	# SDL_CONTROLLER_BUTTON_LEFTSHOULDER
	# SDL_CONTROLLER_BUTTON_GUIDE

	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_LEFTSTICK):
		left_buttons |= (1<<0) # System
	else:
		left_buttons &= ~(1<<0)
	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_GUIDE):
		left_buttons |= (1<<1) # Menu
	else:
		left_buttons &= ~(1<<1)
	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_LEFTSHOULDER):
		left_buttons |= (1<<2) # Grip
	else:
		left_buttons &= ~(1<<2)

	left_pitch = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_LEFTY)
	left_yaw = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_LEFTX)
	left_trigger = sdl2.SDL_GameControllerGetAxis(gamecontroller, sdl2.SDL_CONTROLLER_AXIS_TRIGGERLEFT)
	if left_trigger > 327:
		left_buttons |= (1<<33) # Axis1
	else:
		left_buttons &= ~(1<<33)

	# Unused...
	# SDL_CONTROLLER_BUTTON_A
	# SDL_CONTROLLER_BUTTON_B
	# SDL_CONTROLLER_BUTTON_X
	# SDL_CONTROLLER_BUTTON_Y
	# SDL_CONTROLLER_BUTTON_BACK
	# SDL_CONTROLLER_BUTTON_START
	# SDL_CONTROLLER_BUTTON_DPAD_UP
	# SDL_CONTROLLER_BUTTON_DPAD_DOWN
	# SDL_CONTROLLER_BUTTON_DPAD_LEFT
	# SDL_CONTROLLER_BUTTON_DPAD_RIGHT

	# Postion controller1
	matrix1 = Matrix44.from_translation(Vector3([0.2,1.5,0]))
	matrix1 = matrix1 * Matrix44.from_eulers([radians((right_pitch*(90/32768))+90),radians(right_yaw*(90/32768)),radians(0)])
	matrix1 = matrix1 * Matrix44.from_translation(Vector3([0,0.4,0]))
	matrix1 = matrix1 * Matrix44.from_eulers([radians((right_pitch*(45/32768))-45),radians(right_yaw*(-90/32768)),radians(0)])

	controller1 = controller_packet.build(dict(
		id = 1,
		orient = matrix1.transpose().reshape(16).tolist(),
		count = count,
		pressed = right_buttons,
		touched = right_buttons,
		axis = [0, 0, (right_trigger/32768)]
		))
	controller.send(controller1)
	answer = controller.recv()

	# Postion controller2
	matrix2 = Matrix44.from_translation(Vector3([-0.2,1.5,0]))
	matrix2 = matrix2 * Matrix44.from_eulers([radians((left_pitch*(90/32768))+90),radians(left_yaw*(90/32768)),radians(0)])
	matrix2 = matrix2 * Matrix44.from_translation(Vector3([0,0.4,0]))
	matrix2 = matrix2 * Matrix44.from_eulers([radians((left_pitch*(45/32768))-45),radians(left_yaw*(-90/32768)),radians(0)])

	controller2 = controller_packet.build(dict(
		id = 2,
		orient = matrix2.transpose().reshape(16).tolist(),
		count = count,
		pressed = left_buttons,
		touched = left_buttons,
		axis = [0, 0, (left_trigger/32768)]
		))
	controller.send(controller2)
	answer = controller.recv()

	count = count + 1
	time.sleep(0.1)

headset.close()
controller.close()
master.close()
ctx.destroy()

sdl2.SDL_Quit()
exit(0)
