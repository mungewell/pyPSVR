#!/usr/bin/python

# Track the position and rotation of a printed April Tag

# April Tag libary and python code here:
# https://github.com/swatbotics/apriltag

from SimpleCV import *
import apriltag
import numpy as np
import pygame
import cv2
import argparse

# Commandline options
parser = argparse.ArgumentParser(prog='find_april.py')

parser.add_argument("-c", "--cam", type=int, dest="cam", default=0,
    help="Specify which camera to use")
parser.add_argument("-C", "--cal", type=str, dest="cal", default=0,
    help="Calibration file prefix")
parser.add_argument("-n", "--nocal", action="store_true", dest="nocal",
    help="Do not use calibrated camMatrix and distCoeff")
parser.add_argument("-D", "--dmtx", action="store_true", dest="dmtx",
    help="Only used the DMTX corners" )

parser.add_argument("-t", "--test", action="store_true", dest="test",
    help="Test using the 'original.png' image" )
parser.add_argument("-d", "--dump", action="store_true", dest="dump",
    help="Dump the DMTX, computed subcorners to term" )

options = parser.parse_args()

# 29mm squares on calibration sheet
scale = 29.28

# Predefined object = 50mm x 50mm April Tag PDF
objectPoints = np.array([[[-25, -25, 0], \
    [25, -25, 0], \
    [25, 25, 0], \
    [-25, 25, 0]]], \
    dtype=np.float32) / scale

rVec = None
tVec = None
iterate = False
calibrated = 0
if options.test == False or options.cal:
    cam = Camera(options.cam)
    if options.cal:
        calibrated = cam.loadCalibration(options.cal)
    else:
        calibrated = cam.loadCalibration("default")
    image = cam.getImage()

if options.test:
    print("Using 'original.png' for image source")
    image = Image("original.png")

if calibrated and options.nocal == 0:
    camMatrix = np.array(cam.getCameraMatrix(), dtype=np.float32)
    # Note: This requires a patched SimpleCV
    #distCoeff = np.array(cam.getDistCoeff(), dtype=np.float32)
    distCoeff = np.zeros((5,1))
else:
    print("No calibration")
    calibrated = 0
    focal_length = image.width
    center = (image.width/2, image.height/2)
    camMatrix = np.array(
        [[focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]], dtype = "double"
        )
    distCoeff = np.zeros((5,1))

if options.dump:
    print(camMatrix)
    print(distCoeff)

display = Display()

april_opt = apriltag.DetectorOptions()
april_opt.families = 'tag16h5'
april_opt.refine_pose = True

april_read = apriltag.Detector(april_opt)

while display.isNotDone():
    if options.test:
        # re-read image
        image = Image("original.png")
    else:
        if calibrated:
            image_orig = cam.getImage()
            image = cam.undistort(image_orig)
        else:
             image = cam.getImage()

    overlay = DrawingLayer((image.width, image.height))

    gray = np.array(image.getGrayscaleMatrix())
    detections = april_read.detect(gray)

    for detection in detections:
        if detection.decision_margin < 60:
            continue

        # Highlight barcode
        arrow = []
        arrow.append(detection.corners[0].reshape(2))
        arrow.append(detection.corners[1].reshape(2))
        arrow.append(detection.corners[2].reshape(2))
        arrow.append(detection.corners[3].reshape(2))
        overlay.lines(arrow, Color.RED, width=2)

        # April Tags have sub-pixel accuracy already
        imagePoints = detection.corners.reshape(1,4,2)
        if options.dump:
            print(imagePoints)

        good, rVec, tVec = cv2.solvePnP(objectPoints, imagePoints, \
            camMatrix, distCoeff, rVec, tVec, iterate, cv2.CV_ITERATIVE)

        if good:
            iterate = True
            overlay.text("tVec: %.3f, %.3f, %.3f" % (tVec[0][0], tVec[1][0], tVec[2][0]), \
                (10, image.height - 20), Color.RED)
            if options.dump:
                print("tVec %.3f, %.3f, %.3f" % (tVec[0][0], tVec[1][0], tVec[2][0]))

            # Can not be 'behind' barcode, or too far away
            if tVec[2][0] < 0 or tVec[2][0] > 1000:
                rVec = None
                tVec = None
                iterate = False
                continue

            dst, jacobian = cv2.Rodrigues(rVec)
            x = tVec[0][0]
            y = tVec[2][0]
            t = (math.asin(-dst[0][2]))

            Rx = y * (math.cos((math.pi/2) - t))
            Ry = y * (math.sin((math.pi/2) - t))

            overlay.text("Rx, Ry: %.3f, %.3f" % (Rx, Ry), \
                (image.width/2, image.height - 20), Color.RED)

            # Project a 3D point (-25.0, -25.0, 50.0) onto the image plane.
            (point2D, jacobian) = cv2.projectPoints(np.array([(-25, -25, -50)])/scale, \
                rVec, tVec, camMatrix, distCoeff)
 
            arrow = []
            arrow.append(detection.corners[0].reshape(2))
            arrow.append(point2D[0][0])

            # Add a box in front of barcode
            (point2D, jacobian) = cv2.projectPoints(np.array([(25, -25, -50)])/scale, \
                rVec, tVec, camMatrix, distCoeff)
            arrow.append(point2D[0][0])
            (point2D, jacobian) = cv2.projectPoints(np.array([(25, 25, -50)])/scale, \
                rVec, tVec, camMatrix, distCoeff)
            arrow.append(point2D[0][0])
            (point2D, jacobian) = cv2.projectPoints(np.array([(-25, 25, -50)])/scale, \
                rVec, tVec, camMatrix, distCoeff)
            arrow.append(point2D[0][0])
            (point2D, jacobian) = cv2.projectPoints(np.array([(-25, -25, -50)])/scale, \
                rVec, tVec, camMatrix, distCoeff)
            arrow.append(point2D[0][0])

            overlay.lines(arrow, Color.GREEN, width=4)
        else:
            # Clear iteration if SolvePNP is 'bad'
            rVec = None
            tVec = None
            iterate = False

    # Clear iteration if no barcode found
    if len(detections) == 0:
        rVec = None
        tVec = None
        iterate = False

    if( pygame.key.get_pressed()[pygame.K_SPACE] != 0 ):
        image.save("original.png")

    image.addDrawingLayer(overlay)
    image.applyLayers()
    image.save(display)

    if( pygame.key.get_pressed()[pygame.K_SPACE] != 0 ):
        image.save("annotated.png")

    if( pygame.key.get_pressed()[pygame.K_ESCAPE] != 0 ):
        break

    if( pygame.key.get_pressed()[pygame.K_r] != 0 ):
        rVec = None
        tVec = None
        iterate = False
