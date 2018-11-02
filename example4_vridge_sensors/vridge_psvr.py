#!/usr/bin/env python

# Read the HID stream from the PSVR and extract
# the timestamp, gyro and accelerometer values.
#
# Use the to calculate the headset's rotation
# and send to VRidge via API.

import hidapi
from construct import *
import numpy as np
import zmq

# https://github.com/morgil/madgwick_py
# Reused under GPLv3
from quaternion import Quaternion
from madgwickahrs import MadgwickAHRS

from sys import version_info
if version_info[0] < 3:
	import six

# Structure of the PSVR sensor
# Gyro 1 - yaw, +ve right hand forward
#      2 - pitch, +ve look up
#      3 - roll, +ve right hand down
#
# Acc  1 - head+, toes-
#      2 - right+, left-
#      3 - front+, back-
sensor = Struct(
   "timestamp" / Int32ul,      # only 24bits used, then rolls, usec
   "gyro" / Array(3, Int16sl), # +/-250 degree/second allegdely
   "acc" / Array(3, Int16sl),  # only 12bits used, >> 4
)

GYROSCOPE_FACTOR = ((1998.0/(1<<15)) * (np.pi/180.0))

time2 = 0
ahrs = MadgwickAHRS(sampleperiod=0.005)
Sensor = None

# -------------------------------
# Make a connection to VRidge-API server
#server = 'localhost'
server = '192.168.0.116'
addr = 'tcp://' + server + ':38219'
print ("Connecting to", addr)

ctx = zmq.Context()

# -------------------------------
# connect to the control channel
control = ctx.socket(zmq.REQ)
control.connect(addr)

control.send_json({ "ProtocolVersion":1, "Code":2 })
answer = control.recv_json()

control.send_json({ "RequestedEndpointName":"HeadTracking", "ProtocolVersion":1, "Code":1 })
answer = control.recv_json()

# -------------------------------
# Connect to headset
headset = ctx.socket(zmq.REQ)

# Workaround VRdidge bug
#headset.connect(answer['EndpointAddress'])
endpoint = answer['EndpointAddress']
headset.connect('tcp://' + server + ":" + str(endpoint.split(':')[-1]))

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

position = (0,0,0)


# Use the first HID interface of the PSVR
device_list = hidapi.enumerate(0x054c, 0x09af)
for device_info in device_list:
	Sensor = hidapi.Device(device_info)
	break

while Sensor:
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

	# Compute headset rotation
	ahrs.update_imu(np.array(frame["gyro"])*GYROSCOPE_FACTOR, \
 			(np.array(frame["acc"])>>4), delta)

	frame = sensor.parse(data[32:48])
	time2  = frame["timestamp"]
	if time2 < time1:
		time1 = time1 - (1 << 24)
	delta = (time2 - time1) / 1000000
	ahrs.update_imu(np.array(frame["gyro"])*GYROSCOPE_FACTOR, \
 			(np.array(frame["acc"])>>4), delta)

	'''
	# Send as euler angles
	data = ahrs.quaternion.to_euler_angles() + position
	output = anglesposition.build(dict(data=list(data)))
	'''
	# Send as Quaternion
	data = tuple(ahrs.quaternion._get_q()) + position
	output = quaternionposition.build(dict(data=list(data)))

	headset.send(output)
	answer = headset.recv()

# clean up connections
headset.close()
control.close()
ctx.destroy()
