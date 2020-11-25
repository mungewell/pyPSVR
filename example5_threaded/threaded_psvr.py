#!/usr/bin/env python

# Read the HID stream from the PSVR and extract
# the timestamp, gyro and accelerometer values.
#
# Use the to calculate the headset's rotation
# and send to VRidge via API.

from sys import exit, platform, version_info
import numpy as np
import time

import signal
import threading

from sys import version_info
if version_info[0] < 3:
    import six

if platform == "linux" or platform == "linux2":
    import hidapi
else:
    # Requires:
    # https://github.com/trezor/cython-hidapi
    import hid

# Requires:
# https://github.com/construct/construct
from construct import *

# Requires:
# https://github.com/KieranWynn/pyquaternion
from pyquaternion import Quaternion

# Locally cloned and modified (Reused under GPLv3)
# https://github.com/morgil/madgwick_py
from madgwickahrs import MadgwickAHRS

options = None
headset = None

# =====================================================================
# Structure of the PSVR sensor
# Gyro 0 - yaw, +ve right hand forward
#      1 - pitch, +ve look up
#      2 - roll, +ve right hand down
#
# Acc  0 - head+, toes-
#      1 - right+, left-
#      2 - front+, back-

sensor = Struct(
    "timestamp" / Int32ul,  # only 24bits used, then rolls, usec
    "gyro" / Array(3, Int16sl),  # +/-250 degree/second allegdely
    "acc" / Array(3, Int16sl),  # only 12bits used, >> 4
)

ACC_FACTOR = np.array((1, 1, -1))
GYRO_FACTOR = np.array((1, 1, -1)) * ((1998.0 / (1 << 15)) * (np.pi / 180.0))

class psvr_sensor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        # control signals
        self.event_calibrate = threading.Event()
        self.event_terminate = threading.Event()
        self.event_terminated = threading.Event()
        self.lock_ahrs = threading.Semaphore()

        self.gcomp = np.array((0,0,0))
        self.gtime = 0

        refq = Quaternion(1, 0, 0, 0)  # look front
        self.ahrs = MadgwickAHRS(sampleperiod=0.0005, beta=0.1, quaternion=refq)

    def run(self):
        global options, headset #, ACC_FACTOR, GYRO_FACTOR

        time2 = 0
        Sensor = None

        if options.debug:
            print("starting sensor")

        # Use the first HID interface of the PSVR
        if platform == "linux" or platform == "linux2":
            device_list = hidapi.enumerate(0x054c, 0x09af)
            for device_info in device_list:
                Sensor = hidapi.Device(device_info)
                break
        else:
            Sensor = hid.device()
            Sensor.open(0x054c, 0x09af)
 
            # enable non-blocking mode
            Sensor.set_nonblocking(1)

        while not self.event_terminate.wait(0.0005):
            data = bytes(Sensor.read(64))
            if data == None or len(data) == 0:
                continue

            frame = sensor.parse(data[16:32])
            time1 = frame["timestamp"]
            if time1 < time2:
                time2 = time2 - (1 << 24)
            if time2:
                delta = (time1 - time2) / 1000000
            else:
                # Assume 500us for first sample
                delta = 500 / 1000000

            # Compute headset rotation - first data block
            self.lock_ahrs.acquire()
            if self.event_calibrate.isSet():
                self.gcomp = self.gcomp + np.array(frame["gyro"])
                self.gtime = self.gtime + delta
            else:
                self.ahrs.update_imu((np.array(frame["gyro"])-self.gcomp)*GYRO_FACTOR, \
                    (np.array(frame["acc"])>>4)*ACC_FACTOR , delta)

            frame = sensor.parse(data[32:48])
            time2 = frame["timestamp"]
            if time2 < time1:
                time1 = time1 - (1 << 24)
            delta = (time2 - time1) / 1000000

            # Compute headset rotation - second data block
            if self.event_calibrate.isSet():
                self.gcomp = self.gcomp + np.array(frame["gyro"])
                self.gtime = self.gtime + delta
            else:
                self.ahrs.update_imu((np.array(frame["gyro"])-self.gcomp)*GYRO_FACTOR, \
                    (np.array(frame["acc"])>>4)*ACC_FACTOR , delta)
            self.lock_ahrs.release()

            # complete calibration
            if self.event_calibrate.isSet():
                if self.gtime > 5:
                    self.gcomp = self.gcomp * GYRO_FACTOR / self.gtime
                    time2 = 0
                    self.event_calibrate.clear()

        self.event_terminate.clear()
        self.event_terminated.set()

    def calibrate(self):
        if not self.event_calibrate.isSet():
            self.lock_ahrs.acquire()
            self.gcomp = np.array((0,0,0))
            self.gcomp_time = 0
            self.event_calibrate.set()
            self.lock_ahrs.release()

            if options.debug:
                print("headset calibrating...")

    def yaw_pitch_roll(self):
        self.lock_ahrs.acquire()
        ahrs = self.ahrs
        self.lock_ahrs.release()

        return ahrs.quaternion.yaw_pitch_roll

    def elements(self):
        self.lock_ahrs.acquire()
        ahrs = self.ahrs
        self.lock_ahrs.release()

        return ahrs.quaternion.elements

    def terminate(self, widget=None, event=None, data=None):
        if options.debug:
            print("headset terminating...")

        self.event_terminate.set()
        while self.event_terminate.isSet():
            time.sleep(0.1)

        if options.debug:
            print("headset terminated")

    def terminated(self):
        return self.event_terminated.isSet()

# =====================================================================
def Run():
    import argparse

    '''
    # we will use OpenVR to sense HMD VSync timing
    try:
        import openvr
        _hasOpenVR = True
    except ImportError:
        _hasOpenVR = False
    '''
    _hasOpenVR = False

    global options, display

    # Commandline options
    parser = argparse.ArgumentParser(prog='threading.py')

    parser.add_argument("-D", "--debug", action="store_true", dest="debug",
        help="Output extra debug messages")
    parser.add_argument("-c", "--calibrate", action="store_true", dest="calibrate",
        help="Calibrate the AHRS for gyro drift in the headset")
    parser.add_argument("-f", "--fps", dest="fps", type=float, default="60.0",
        help="Specify an approximate fps, ie. set frequency that reports are sent")

    options = parser.parse_args()

    if options.fps < 1.0:
        options.fps = 1.0

    headset = psvr_sensor()
    headset.start()

    # capture Control-C to close application
    signal.signal(signal.SIGINT, headset.terminate)

    if _hasOpenVR:
        openvr.init(openvr.VRApplication_Overlay)

    if options.calibrate:
        headset.calibrate()

    while not headset.terminated():
        (y,p,r) = headset.yaw_pitch_roll()

        if options.debug:
            print("Angles: ", y, p, r)

        if _hasOpenVR:
            time_since = openvr.VRSystem().getTimeSinceLastVsync()
            to_sleep = (1.0/options.fps) - time_since[1]

            if to_sleep > 0.0:
                time.sleep(to_sleep)
        else:
                time.sleep(1.0/options.fps)

if __name__ == '__main__':
    Run()
