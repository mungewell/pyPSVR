#!/usr/bin/env python

# Read the HID stream from the PSVR and extract
# the timestamp, gyro and accelerometer values.

from sys import exit, platform, version_info
import struct

if platform == "linux" or platform == "linux2":
	import hidapi
else:
	# Requires:
	# https://github.com/trezor/cython-hidapi
	import hid

from sys import version_info
if version_info[0] < 3:
	import six

time2 = 0
Sensor = None

# Use the first HID interface of the PSVR
if platform == "linux" or platform == "linux2":
	device_list = hidapi.enumerate(0x054c, 0x09af)
	for device_info in device_list:
		Sensor = hidapi.Device(device_info)
		break
else:
	'''
	for d in hid.enumerate():
		keys = list(d.keys())
		keys.sort()
		for key in keys:
			print("%s : %s" % (key, d[key]))
		print()
	'''

	Sensor = hid.device()
	Sensor.open(0x054c, 0x09af)

	# enable non-blocking mode
	Sensor.set_nonblocking(1)

	print("Manufacturer: %s" % Sensor.get_manufacturer_string())
	print("Product: %s" % Sensor.get_product_string())
	print("Serial No: %s" % Sensor.get_serial_number_string())

while Sensor:
	data = bytes(Sensor.read(64))
	if data == None or len(data) == 0:
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
