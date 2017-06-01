#!/usr/bin/env python

# Read the HID stream from the PSVR and extract
# the timestamp, gyro and accelerometer values.

import hidapi
import struct

from sys import version_info
if version_info[0] < 3:
	import six

time2 = 0
Sensor = None

# Use the first HID interface of the PSVR
device_list = hidapi.enumerate(0x054c, 0x09af)
for device_info in device_list:
	Sensor = hidapi.Device(device_info)
	break

while Sensor:
	data = Sensor.read(64)
	if data == None:
		continue

	(time1, ) = struct.unpack("<I", data[16:20])
	if time1 < time2:
		time2 = time2 - (1 << 24)
	print("Time-Slot1:", time1, time1 - time2)
	print("Gyros:", struct.unpack("<hhh", data[20:26]))
	print("Accel:", struct.unpack("<hhh", data[26:32]))

	print("-")

	(time2, ) = struct.unpack("<I", data[32:36])
	if time2 < time1:
		time1 = time1 - (1 << 24)
	print("Time-Slot2:", time2, time2 - time1)
	print("Gyros:", struct.unpack("<hhh", data[36:42]))
	print("Accel:", struct.unpack("<hhh", data[42:48]))

	print("=")
