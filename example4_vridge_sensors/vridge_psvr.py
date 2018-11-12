#!/usr/bin/env python

# Read the HID stream from the PSVR and extract
# the timestamp, gyro and accelerometer values.
#
# Use the to calculate the headset's rotation
# and send to VRidge via API.

import argparse
from sys import version_info
if version_info[0] < 3:
	import six

from construct import *
import numpy as np
import zmq

# https://github.com/morgil/madgwick_py
# Reused under GPLv3
#from quaternion import Quaternion
from madgwickahrs import MadgwickAHRS

from pyquaternion import Quaternion


# Commandline options
parser = argparse.ArgumentParser(prog='vridge_psvr.py')

parser.add_argument("-A", "--angle", action="store_true", dest="angle",
	help="Send data to VRidge as angles (rather than Quaternions" )
parser.add_argument("-S", "--sim", action="store_true", dest="sim",
	help="Simuluate the PSVR" )
parser.add_argument("-D", "--debug", action="store_true", dest="debug",
	help="Output extra Debug" )

parser.add_argument("-s", "--server", dest="server", default="localhost",
	help="Use particular server (instead of localhost)" )
parser.add_argument("-c", "--comp", action="store_true", dest="comp",
	help="Enable Gryo Compensation" )

options = parser.parse_args()


# Structure of the PSVR sensor                      MPU-9250
# Gyro 1 - yaw, +ve right hand forward              => Z
#      2 - pitch, +ve look up                       => X
#      3 - roll, +ve right hand down                => Y
#
# Acc  1 - head+, toes-                             => Z
#      2 - right+, left-                            => X
#      3 - front+, back-                            => Y
sensor = Struct(
   "timestamp" / Int32ul,      # only 24bits used, then rolls, usec
   "gyro" / Array(3, Int16sl), # +/-250 degree/second allegdely
   "acc" / Array(3, Int16sl),  # only 12bits used, >> 4
)

ACC_FACTOR = np.array((1, 1, -1))
GYRO_FACTOR = np.array((1, 1, -1)) * ((1998.0/(1<<15)) * (np.pi/180.0))

Sensor = None
position = (0,0,0)
gcomp = np.array((0,0,0))
time2 = 0

# -------------------------------
# Make a connection to VRidge-API server
addr = 'tcp://' + options.server + ':38219'
print ("Connecting to", addr)

ctx = zmq.Context()

# -------------------------------
# connect to the control channel
control = ctx.socket(zmq.REQ)
control.connect(addr)

control.send_json({ "ProtocolVersion":1, "Code":2 })
answer = control.recv_json()
if options.debug:
	print(answer)

control.send_json({ "RequestedEndpointName":"HeadTracking", "ProtocolVersion":1, "Code":1 })
answer = control.recv_json()
if options.debug:
	print(answer)

# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)

# Workaround VRdidge bug
endpoint = answer['EndpointAddress']
headset.connect('tcp://' + options.server + ":" + str(endpoint.split(':')[-1]))

# SendRadRotationAndPosition
anglesposition = Struct(
	Const(2, Int32ul),  # Version
	Const(3, Int32ul),  # SendRadRotationAndPosition
	Const(24, Int32ul), # DataLength
	"data" / Padded(64, Array(6, Float32l)),
)
# SendQuaternionRotationAndPosition
quaternionposition = Struct(
	Const(2, Int32ul),  # Version
	Const(4, Int32ul),  # SendQuaternionRotationAndPosition
	Const(24, Int32ul), # DataLength
	"data" / Padded(64, Array(7, Float32l)),
)
# SendPosition
justposition = Struct(
	Const(2, Int32ul),  # Version
	Const(5, Int32ul),  # SendPosition
	Const(24, Int32ul), # DataLength
	"data" / Padded(64, Array(3, Float32l)),
)

if not options.sim:
	# Use the first HID interface of the PSVR
	import hidapi
	device_list = hidapi.enumerate(0x054c, 0x09af)
	for device_info in device_list:
		Sensor = hidapi.Device(device_info)
		break

# -------------------------------
# Calculate Gryo compensation (headset should be static)
comptime = 0
while Sensor and options.comp:
	data = Sensor.read(64)
	if data == None:
		continue

	frame = sensor.parse(data[16:32])
	time1  = frame["timestamp"]
	if time1 < time2:
		time2 = time2 - (1 << 24)
	if time2:
		delta = (time1 - time2) / 1000000
	else:
		# Assume 500us for first sample
		delta = 500 / 1000000

	# Sum gyro movements
	gcomp = gcomp + np.array(frame["gyro"])
	comptime = comptime + delta

	frame = sensor.parse(data[32:48])
	time2  = frame["timestamp"]
	if time2 < time1:
		time1 = time1 - (1 << 24)
	delta = (time2 - time1) / 1000000

	gcomp = gcomp + np.array(frame["gyro"])
	comptime = comptime + delta

	if comptime > 5:
		gcomp = gcomp * GYRO_FACTOR / comptime
		print("Gcomp:", gcomp)
		break;


# -------------------------------
# Initiate AHRS

refq = Quaternion(1,0,0,0) # look front
'''
refq = Quaternion.from_angle_axis(np.pi/2,0,0,1) # look front, sideways
refq = Quaternion.from_angle_axis(np.pi/2,1,0,0) # look up
refq = Quaternion.from_angle_axis(np.pi/2,0,1,0) # look right
'''
#refq = Quaternion(axis=[0,0,1],angle=np.pi/2) # look right
#refq = Quaternion(axis=[1,0,0],angle=np.pi/2) # look front, roll 90' left-down
#refq = Quaternion(axis=[0,1,0],angle=np.pi/2) # look up
ahrs = MadgwickAHRS(sampleperiod=0.0005, beta=0.1 , quaternion=refq)

# Structure of the PSVR sensor                      MPU-9250
# Gyro 1 - yaw, +ve right hand forward              => Z
#      2 - pitch, +ve look up                       => X
#      3 - roll, +ve right hand down                => Y
#
# Acc  1 - head+, toes-                             => Z
#      2 - right+, left-                            => X
#      3 - front+, back-                            => Y
while Sensor or options.sim:
	if options.sim:
		# use fake data for testing
		# (yaw, pitch, roll) (up, side, side)
		a=1
		#ahrs.update_imu((1, 0, 0), (0, 0, 1)) # sideways, pitching
		#ahrs.update_imu((0, 0, 1), (1, 0, 0)) # looking front, rolling right up
		#ahrs.update_imu((0, 1, 0), (0, 1, 0)) # looking right, pitching up

		#ahrs.update_imu((1, 0, 0), (0, 0, 1)) # sideways, pitching
		#ahrs.update_imu((0, 0, 1), (1, 0, 0)) # looking front, yawing to right
		ahrs.update_imu((0, 1, 0), (0, 1, 0)) # looking right, pitching up
	else:
		data = Sensor.read(64)
		if data == None:
			continue

		frame = sensor.parse(data[16:32])
		time1  = frame["timestamp"]
		if time1 < time2:
			time2 = time2 - (1 << 24)
		if time2:
			delta = (time1 - time2) / 1000000
		else:
			# Assume 500us for first sample
			delta = 500 / 1000000

		# Compute headset rotation - first data
		ahrs.update_imu(np.array(frame["gyro"]-gcomp)*GYRO_FACTOR, \
				(np.array(frame["acc"])>>4)*ACC_FACTOR , delta)

		frame = sensor.parse(data[32:48])
		time2  = frame["timestamp"]
		if time2 < time1:
			time1 = time1 - (1 << 24)
		delta = (time2 - time1) / 1000000

		# Compute headset rotation - second data
		ahrs.update_imu(np.array(frame["gyro"]-gcomp)*GYRO_FACTOR, \
				(np.array(frame["acc"])>>4)*ACC_FACTOR , delta)

	if options.debug:
		'''
		print("Euler: ", ahrs.quaternion.to_euler_angles())
		print("Eu123: ", ahrs.quaternion.to_euler123())
		print("Quat:  ", ahrs.quaternion._get_q())
		'''
		print("Euler: ", ahrs.quaternion.yaw_pitch_roll)
		print("Quat:  ", ahrs.quaternion.elements)
		print()

	if options.angle:
		'''
		# Send as euler angles
		(p,r,y) = ahrs.quaternion.to_euler_angles()
		# Send as euler123 angles
		(r,p,y) = ahrs.quaternion.to_euler123()
		(r,y,p) = ahrs.quaternion.yaw_pitch_roll
		'''
		(y,p,r) = ahrs.quaternion.yaw_pitch_roll
		data = tuple((p,y,r)) + position
		output = anglesposition.build(dict(data=list(data)))
	else:
		# Send as Quaternion (note: VRidge uses 'x,y,z,w')
		'''
		(w,x,y,z) = tuple(ahrs.quaternion._get_q())
		data = tuple((x,y,z,w))+ position
		'''
		(w,x,y,z) = ahrs.quaternion.elements
		data = tuple((y,z,x,w))+ position
		output = quaternionposition.build(dict(data=list(data)))

	# Update position in VRidge
	headset.send(output)
	answer = headset.recv()

# clean up connections
headset.close()
control.close()
ctx.destroy()
