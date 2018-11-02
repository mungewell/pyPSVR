#!/usr/bin/python
#
# Script control the PSVR and switch between Cinematic/VR Mode
# (c) Simon Wood, 25 May 2017
#

import usb
import argparse
from sys import exit, platform, version_info

import time
import binascii

if version_info[0] < 3:
	import six

# Commandline options
parser = argparse.ArgumentParser(prog='pyPSVR.py')

parser.add_argument("-S", "--shutdown", action="store_true", dest="shutdown",
	help="Shutdown the PSVR" )
parser.add_argument("-o", "--on", action="store_true", dest="on",
	help="Turn the PSVR on" )
parser.add_argument("-O", "--off", action="store_true", dest="off",
	help="Turn the PSVR off" )

parser.add_argument("-v", "--vrmode", action="store_true", dest="vrmode",
	help="Turn on VR mode" )
parser.add_argument("-c", "--cinemode", action="store_true", dest="cinemode",
	help="Turn on Cinematic mode" )

parser.add_argument("-l", "--leds", action="store_true", dest="leds",
	help="Turn on the LEDs" )
parser.add_argument("-L", "--ledval", type=int, dest="ledval",
	help="Set brightness values for the LEDs" )
parser.add_argument("-N", "--lednum", type=int, dest="lednum",
	help="Set brightness only for specific LED" )

parser.add_argument("-s", "--size", type=int, dest="size",
	help="Set the size of the Cinematic screen" )
parser.add_argument("-d", "--dist", type=int, dest="dist",
	help="Set the distance of the Cinematic screen" )
parser.add_argument("-m", "--mist", type=int, dest="mist",
	help="Set the mistery of the Cinematic screen" )
parser.add_argument("-b", "--bright", type=int, dest="bright",
	help="Set the brightness of the screen" )
parser.add_argument("-H", "--hdmi", type=int, dest="hdmi",
	help="Set the HDMI resolution of Social-Screen" )
parser.add_argument("-k", "--lock", action="store_true", dest="lock",
	help="Lock the Cinematic screen in position" )

parser.add_argument("-r", "--recenter", action="store_true", dest="recenter",
	help="Recenter the screen in Cinematic Mode" )

parser.add_argument("-i", "--id", action="store_true", dest="id",
	help="Request serial and revision" )
parser.add_argument("-C", "--cal", action="store_true", dest="cal",
	help="Request calibration data" )
parser.add_argument("-G", "--reg", type=int, dest="reg",
	help="Request register data" )
parser.add_argument("-R", "--read", action="store_true", dest="read",
	help="After writing command, listen for responses" )

parser.add_argument("-t", "--test", type=int, dest="test",
	help="test variable" )

options = parser.parse_args()

# ------------------
# Start doing stuff
psvr = usb.core.find(idVendor=0x054c, idProduct=0x09af)

if not psvr:
	exit("Could not locate PSVR")

reattach = bytearray(psvr[0].bNumInterfaces)
if platform == "linux" or platform == "linux2":
	for configuration in psvr:
		for interface in configuration:
			ifnum = interface.bInterfaceNumber
			# Only Detach the Control interface
			if ifnum == 5:
				reattach[ifnum] = True
				if not psvr.is_kernel_driver_active(ifnum):
					reattach[ifnum] = False
					continue
				try:
					print("Detaching: %s" % ifnum)
					#print("%s: %s\n" % (psvr, ifnum))
					psvr.detach_kernel_driver(ifnum)
				except usb.core.USBError as e:
					pass

if platform == "win32":
	psvr.set_configuration()

cfg = psvr.get_active_configuration() 

# Interface 5 -> Endpoint 0x4
ifnum = cfg[(5,0)].bInterfaceNumber 
alternate_settting = usb.control.get_interface(psvr, ifnum) 
intf = usb.util.find_descriptor(cfg, bInterfaceNumber = ifnum, 
		bAlternateSetting = alternate_settting) 

ep = usb.util.find_descriptor(intf,custom_match = \
		lambda e: \
		usb.util.endpoint_direction(e.bEndpointAddress) == \
		usb.util.ENDPOINT_OUT) 

rp = usb.util.find_descriptor(intf,custom_match = \
		lambda e: \
		usb.util.endpoint_direction(e.bEndpointAddress) == \
		usb.util.ENDPOINT_IN) 

if ep:
	# Known Commands:
	# Cmd 0x11, 0x13, 0x15, 0x17, 0x1b, 0x21, 0x23, 0x81

	# Shutdown PSVR, requires button press on chord to turn on again
	if options.shutdown:
		ep.write(b'\x13\x00\xaa\x04\x01\x00\x00\x00')
	'''
	else:
		# This seems to help prevent missed commands - reset parser?
		ep.write(b'\x13\x00\xaa\x04\x00\x00\x00\x00')
	'''

	# Headset on/off, Processing Unit stays on
	if options.on:
		ep.write(b'\x17\x00\xaa\x04\x01\x00\x00\x00')
	elif options.off:
		ep.write(b'\x17\x00\xaa\x04\x00\x00\x00\x00')

	# Enable VR mode
	if options.vrmode:
		if options.leds:
			# Front and Back LEDs on
			ep.write(b'\x11\x00\xaa\x08\x01\x11\x03\x11\x00\x00\x00\x00')
		else:
			# No LEDs
			ep.write(b'\x11\x00\xaa\x08\x01\x00\x03\x11\x00\x00\x00\x00')
	# Cinematic mode
	elif options.cinemode:
		if options.leds:
			# Just Front LEDs on
			ep.write(b'\x11\x00\xaa\x08\x02\x01\x03\x11\x00\x00\x00\x00')
		else:
			ep.write(b'\x11\x00\xaa\x08\x02\x00\x03\x11\x00\x00\x00\x00')
			#ep.write(b'\x11\x00\xaa\x08\x03\x00\x03\x11\x00\x00\x00\x00')

	'''
	# Prevent a shutdown
	# seen if 2 commands are sent too quickly
	if options.vrmode or options.cinemode:
		time.sleep(1)
	'''

	# Set LED brightness
	if options.ledval or options.lednum:
		if options.ledval > 99:
			options.ledval = 99
		if options.ledval < 0:
			options.ledval = 0

		if options.lednum:
			# Only adjust specific LED
			if options.lednum > 9:
				options.lednum = 9
			if options.lednum < 1:
				options.lednum = 1
			ledhi = (1 << options.lednum - 1) >> 8
			ledlo = (1 << options.lednum - 1) & 0xff
		else:
			# Adjust all LEDs
			ledhi = 0x01
			ledlo = 0xff

		ep.write(b'\x15\x00\xaa\x10' + 
			bytearray([ledlo, ledhi]) + \
			bytearray([options.ledval]*9) + \
			b'\x00\x00\x00\x00\x00')

	# Control 'Cinematic Screen' params
	if options.size or options.dist or options.mist \
			or options.bright or options.hdmi:
		# Limit size/dist/etc
		if options.size:
			if options.size > 100:
				options.size = 100
			if options.size < 26:
				options.size = 26
		else:
			options.size=52

		if options.dist:
			if options.dist > 50:
				options.dist = 50
			if options.dist < 20:
				options.dist = 20
		else:
			options.dist=35

		if options.mist:
			if options.mist > 40: # no 'bad value', but ignores if greater
				options.mist = 40
			if options.mist < 1:
				options.mist = 1
		else:
			options.mist=20

		if options.bright:
			if options.bright > 32:
				options.bright = 32
			if options.bright < 1:
				options.bright = 1
		else:
			options.bright=20

		if options.hdmi:
			if options.hdmi > 4:
				options.hdmi = 4
			if options.hdmi < 0:
				options.hdmi = 0
		else:
			options.hdmi=0

		if options.lock:
			lock = 0x00
		else:
			lock = 0x40

		if options.size:
			ep.write(b'\x21\x00\xaa\x10' + \
				bytearray([lock, options.size, options.dist, options.mist]) + \
				b'\x00\x00\x00\x00\x00\x00' + bytearray([options.bright]) + \
				b'\x00' + bytearray([options.hdmi]) + b'\x00\x00\x00')

	if options.recenter:
		ep.write(b'\x1b\x00\xaa\x04\x00\x00\x00\x00')

	if options.test:
		# Seems to affect the mode in report 0xf0, but don't know what else
		# forces VR->Cine without gamepad icon
		# -t   8 -> 0x00,0x12
		# -t  16 -> 0x00,0x12
		# -t  32 -> 0x00,0x12
		# -t  64 -> 0x08,0x12
		# -t 128 -> 0x10,0x12
		# -t 192 -> 0x18,0x12
		ep.write(b'\x23\x00\xaa\x04' + bytearray([options.test]) + '\x00\x00\x00')

	if options.read:
		response = 0
		while rp:
			# Request serial number
			if options.id and response == 0:
				ep.write(b'\x81\x00\xaa\x08\x80\x00\x00\x00\x00\x00\x00\x00')

			# Request calibration data
			if options.cal and response < 5:
				ep.write(b'\x81\x00\xaa\x08\x86' + \
					bytearray([response]) + \
					b'\x00\x00\x00\x00\x00\x00')

			# Request particular register
			if options.reg and response == 0:
				ep.write(b'\x81\x00\xaa\x08' + \
					bytearray([options.reg]) + \
					b'\x00\x00\x00\x00\x00\x00\x00')

			data = None
			try:
				data = rp.read(100, 100)
			except usb.core.USBError as e:
				pass
			if not data:
				continue
			
			# Header
			print(time.asctime())
			print(binascii.hexlify(data[0:4]))
			data = data[4:]

			# Body
			while len(data) >= 16:
				print(binascii.hexlify(data[0:16])) #, binascii.b2a_qp(data[0:16])
				data = data[16:]
			if data:
				print(binascii.hexlify(data)) #, binascii.b2a_qp(data)
			print("-")
			response = response + 1
else:
	print("Unable to locate Endpoint")

# Clean up, and re-attach endpoints if they were previous attached
usb.util.dispose_resources(psvr)

if platform == "linux" or platform == "linux2":
	for configuration in psvr:
		for interface in configuration:
			ifnum = interface.bInterfaceNumber
			if (reattach[ifnum]):
				print("Re-attaching: %s" % ifnum)
				psvr.attach_kernel_driver(ifnum)
