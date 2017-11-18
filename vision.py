#!/usr/bin/env python

from __future__ import division
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables
import argparse


def picamvidopencv():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = int(options.shutter)  # 0 to 31163; 0 is auto
    rawCapture = PiRGBArray(camera, size=(640, 480))
    crosshair = [320, 240]
    toggle_rectangles = True
    contours_big = 0
    contours_small = 0
    distance = 0
    target_width = 12
    target_height = 12
    tape = float(options.tape_width)  # tape width

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.25')
    nettable = NetworkTables.getTable("Vision")

    # allow the camera to warmup
    time.sleep(0.1)

    # (1) Position of tapeEdgeX, tapeEdgeY
    # (2) Position of tapeEdgeX + largeCropWidth, tapeEdgeY + largeCropHeight
    #     largeCrop is a crop from (1) to (2)
    #     smallCrop is a crop from (3) to (4)

    # (1)---------------------
    #  |   (3)-----------    |
    #  |    |           |    |
    #  |    |           |    |
    #  |    -----------(4)   |
    #  ---------------------(2)

    # capture frames from the camera

    def findTapePixels(originalPixels, dimensionInInches):
        #Returns the number of pixels that make up the tape
        ratio = (dimensionInInches - tape * 2)/dimensionInInches
        return int((originalPixels - (originalPixels * ratio))/2)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, np.array([50, 50, 150]), np.array([100, 255, 255]))
        phase1 = mask.copy()
        phase2 = mask.copy()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        largeCrop = phase1[100:300, 100:500]

        if len(contours) > 0:
            contours_big = 0
            contours_small = 0
            for contour in contours:
                tapeEdgeX, tapeEdgeY, largeCropWidth, largeCropHeight = cv2.boundingRect(contour)
                if (largeCropWidth * largeCropHeight) > 100: #if the area is greater than 100, used to remove single pixels
                    if (largeCropWidth / largeCropHeight) < 1.2 and (largeCropWidth / largeCropHeight) > 0.8: #ratios used when determining angles
                        largeCrop = phase1[tapeEdgeY:(tapeEdgeY + largeCropHeight), tapeEdgeX:(tapeEdgeX + largeCropWidth)]
                        tapeWidthPixels = findTapePixels(largeCropWidth, target_width)
                        tapeHeightPixels = findTapePixels(largeCropHeight, target_height)
                        smallCropWidth = largeCropWidth - tapeWidthPixels * 2 # cw is smallCrop width
                        smallCropHeight = largeCropHeight - tapeHeightPixels * 2  #int(tapeEdgeHeight * (target_height - tape - tape) / target_height)   # cw is largeCrop height
                        smallCrop = largeCrop[tapeHeightPixels:smallCropHeight + tapeHeightPixels, tapeWidthPixels:smallCropWidth + tapeWidthPixels,]
                        whitePixelsInLarge = cv2.countNonZero(largeCrop)
                        whitePixelsInSmall = cv2.countNonZero(smallCrop)
                        # print "act", (whitePixelsInLarge - whitePixelsInSmall)
                        # print "bxh", largeCropWidth * largeCropHeight
                        # print "cxh", smallCropWidth*smallCropHeight
                        if (whitePixelsInLarge - whitePixelsInSmall) > (0.25 * (largeCropWidth*largeCropHeight - smallCropWidth*smallCropHeight)):   # tape is 20% filled
                            if (whitePixelsInSmall) < (0.1 * (smallCropWidth*smallCropHeight)):   # non-tape part is less than 10% filled
                                if toggle_rectangles:
                                       cv2.rectangle(image, (tapeEdgeX, tapeEdgeY), (tapeEdgeX + largeCropWidth, tapeEdgeY + largeCropHeight), (0, 0, 255), 1)
                                       cv2.rectangle(image, (tapeEdgeX + tapeWidthPixels, tapeEdgeY + tapeHeightPixels), (tapeEdgeX + tapeWidthPixels + smallCropWidth, tapeEdgeY + tapeHeightPixels + smallCropHeight), (255,0,0), 1)
                                mid = tapeEdgeX + (largeCropWidth/2)
                                offset = 320 - mid
                                angle = offset * (62.2 / 640)
                                distance = 3770 / largeCropWidth
                                contours_big += 1
                                nettable.putNumber('angle', float(angle))
                #else:
                #    sx,sy,sw,sh = cv2.boundingRect(contour)
                #    if toggle_rectangles:
                #        cv2.rectangle(image, (sx, sy), (sx + sw, sy + sh), (255, 0, 0), 2)
                #    contours_small += 1

        if contours_big == 1:
            cv2.putText(image, str(largeCropWidth) + "," + str(largeCropHeight), (320, 420), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.putText(image, str(angle), (320, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.putText(image, str(distance), (320, 440), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)

        key = cv2.waitKey(1) & 0xFF
        # print time.time()

        # Shutter
        if key == ord("S"):
            camera.shutter_speed += 1000
        if key == ord("s"):
            camera.shutter_speed += -1000

        # Toggle
        if key == ord("t"):
            toggle_rectangles = not toggle_rectangles

        # Marker
        if key == ord("i"): #up
            crosshair[1] += -1
        if key == ord("k"): #down
            crosshair[1] += 1
        if key == ord("j"): #left
            crosshair[0] += -1
        if key == ord("l"): #right
            crosshair[0] += 1
        if key == ord("I"):
            crosshair[1] += -10
        if key == ord("K"):
            crosshair[1] += 10
        if key == ord("J"):
            crosshair[0] += -10
        if key == ord("L"):
            crosshair[0] += 10

        cv2.putText(image, "Contours:" + str(len(contours)), (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
        cv2.putText(image, "(S)hutter: " + str(camera.shutter_speed), (0, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
        cv2.line(image, (crosshair[0]-10, crosshair[1]), (crosshair[0]+10, crosshair[1]), (255, 255, 255), thickness=1)
        cv2.line(image, (crosshair[0], crosshair[1]-10), (crosshair[0], crosshair[1]+10), (255, 255, 255), thickness=1)
        cv2.putText(image, "HSV: " + str(hsv[crosshair[0], crosshair[1]]), (0, 60), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)


        # show the frame
        cv2.imshow("Image", image)
        cv2.imshow("Mask", phase2)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


def main():
    picamvidopencv()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', action='store', dest='shutter',
                        default=20000,
                        help='Select the shutter speed (0-31163)')
    parser.add_argument('-t', action='store', dest='tape_width',
                        default=2.25,
                        help='Select the tape width to detect')
    options = parser.parse_args()
    main()
