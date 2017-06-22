#!/usr/bin/python
#
# Script to implement crude fake controller using VRidge-API and SDL2
#
# (c) Simon Wood, 19 June 2017

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *

# http://zeromq.org
# ZeroMQ-4.0.4-miru1.0-x86.exe
#
# https://www.nico-maas.de/?p=1081
# https://github.com/zeromq/pyzmq
# pyzmq-16.0.2.zip

import zmq

# http://libsdl.org/
# SDL2-2.0.5-win32-x64.zip
#
# https://pysdl2.readthedocs.io/en/rel_0_9_5/
# https://github.com/syntonym/pysdl2
# pysdl2-master.zip

import sdl2

import time
from math import radians
from pyrr import Quaternion, Matrix44, Vector3
import numpy as np

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

if answer['ProtocolVersion'] != 2 or answer['Code'] != 0:
	print("HeadTracking: incompatible protocol, or not available")
	exit(0)

headset_addr = answer['EndpointAddress']

master.send_json({ "RequestedEndpointName":"Controller", "ProtocolVersion":1, "Code":1 })
answer = master.recv_json()

if answer['ProtocolVersion'] != 2 or answer['Code'] != 0:
	print("Controller: incompatible protocol, or not available")
	exit(0)

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
	"status" / Int32ul,		# Status
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

head_SendRadRotationAndPosition = Struct(
	Const(Int32ul, 2),		# Version
	Padded(4, Const(Byte, 3)),	# SendRadRotationAndPosition
	Const(Int32ul, 24),		# DataLength
	"rotation" / Array(3, Float32l),
	"position" / Array(3, Float32l),
	Padding(40),
)

# -------------------------------
# position view point
output = head_SendRadRotationAndPosition.build(dict(
		rotation=[radians(0),radians(0),radians(0)],
		position=[0,2,0]
		))
headset.send(output)
answer = headset.recv()

last = 0
count = 0

left_visible = time.time()
left_pitch = 0
left_yaw = 0
left_trigger = 0
left_buttons = 0
left_touched = 0

right_visible = time.time()
right_pitch = 0
right_yaw = 0
right_trigger = 0
right_buttons = 0
right_touched = 0

while True:
	sdl2.SDL_PumpEvents()

	# Controller assignments:
	# Controller1 (right)
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

	if right_trigger > 3276: # 10% depressed
		right_touched |= (1<<33) # Axis1
	else:
		right_touched &= ~(1<<33)
	if right_trigger > 16384: # 50% depressed
		right_buttons |= (1<<33) # Axis1
	else:
		right_buttons &= ~(1<<33)

	# Make a controller visible only for 15 seconds after movement/button press
	if right_touched or right_buttons or abs(right_pitch) > 3276 or abs(right_yaw) > 3276:
		right_visible = time.time()

	if right_visible:
		if (right_visible + 15) < time.time():
			right_visible = None

	# Controller2 (left)
	# SDL_CONTROLLER_AXIS_LEFTX
	# SDL_CONTROLLER_AXIS_LEFTY
	# SDL_CONTROLLER_BUTTON_LEFTSTICK
	# SDL_CONTROLLER_AXIS_TRIGGERLEFT
	# SDL_CONTROLLER_BUTTON_LEFTSHOULDER
	# SDL_CONTROLLER_BUTTON_GUIDE

	if sdl2.SDL_GameControllerGetButton(gamecontroller, sdl2.SDL_CONTROLLER_BUTTON_LEFTSTICK):
		left_buttons |= (1<<0) # System
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

	if left_trigger > 3276: # 10% depressed
		left_touched |= (1<<33) # Axis1
	else:
		left_touched &= ~(1<<33)
	if left_trigger > 16384: # 50% depressed
		left_buttons |= (1<<33) # Axis1
	else:
		left_buttons &= ~(1<<33)

	# Make a controller visible only for 15 seconds after movement/button press
	if left_touched or left_buttons or abs(left_pitch) > 3276 or abs(left_yaw) > 3276:
		left_visible = time.time()

	if left_visible:
		if (left_visible + 15) < time.time():
			left_visible = None

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
		status = 0 if right_visible else 1,
		orient = matrix1.transpose().reshape(16).tolist(),
		count = count,
		pressed = right_buttons,
		touched = right_touched,
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
		status = 0 if left_visible else 1,
		orient = matrix2.transpose().reshape(16).tolist(),
		count = count,
		pressed = left_buttons,
		touched = left_touched,
		axis = [0, 0, (left_trigger/32768)]
		))
	controller.send(controller2)
	answer = controller.recv()

	# Packet counter
	count = count + 1
	time.sleep(0.1)

headset.close()
controller.close()
master.close()
ctx.destroy()

sdl2.SDL_Quit()
exit(0)
