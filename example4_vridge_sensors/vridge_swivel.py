#!/usr/bin/python
#
# Script to connect using vridge-api, and swivel the viewport around 360'
#
# (c) Simon Wood, 10 June 2017

# Note: uses Construct 2.8
# https://construct.readthedocs.io/en/latest/

from construct import *

import zmq
import time
import math

#server = 'localhost'
server = '192.168.0.116'
addr = 'tcp://' + server + ':38219'
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

# Workaround VRdidge bug
#headset.connect(answer['EndpointAddress'])
endpoint = answer['EndpointAddress']
headset.connect('tcp://' + server + ":" + str(endpoint.split(':')[-1]))

# SendRadRotationAndPosition
packet = Struct(
	Const(2, Int32ul),  # Version
	Const(3, Int32ul),  # SendRadRotationAndPosition
	Const(24, Int32ul), # DataLength
	"data" / Padded(64, Array(6, Float32l)),
)

for swivel in range(0,360,2):
	output = packet.build(dict(data=[math.radians(0),math.radians(swivel),math.radians(0),0,0,0]))
	headset.send(output)
	answer = headset.recv()

	time.sleep(0.1)

headset.close()
control.close()
ctx.destroy()
