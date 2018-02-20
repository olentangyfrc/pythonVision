#!/usr/bin/env python

from __future__ import division
import time
import math
import cv2
import numpy as np
import argparse
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables
from datetime import datetime


def adjust_hsv(name, val, delta, nettable):
    val += delta
    val = min(255, max(0, val))
    nettable.putNumber(name, val)
    return val


def main():
    # convenience variables
    red = (0, 0, 255)
    blue = (255, 0, 0)
    white = (255, 255, 255)

    # Camera Variables
    fieldOfView = math.radians(48.8 / 2)  # input degrees
    heightBox = 11
    #offset = 3 # inches
    #Position X should be negative for offset
    #Position Y should be positive for offset
    #  /-X-----Y-\
    #  |  FRONT  |/\
    #  |         |\/
    #  |         |
    #  |         |/\
    #  |   BACK  |\/
    #  \---------/

    # Cube Variables
    color_hue_min = 0
    color_hue_max = 100
    color_sat_min = 180
    color_sat_max = 255
    color_val_min = 75
    color_val_max = 255
    minSize = 150  # of contour, in case there are two

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = 21000  # 18000 for cube #2448 for Tape
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.2')
    nettable = NetworkTables.getTable("Vision")
    nettable.putNumber('color_hue_min', color_hue_min)
    nettable.putNumber('color_hue_max', color_hue_max)
    nettable.putNumber('color_sat_min', color_sat_min)
    nettable.putNumber('color_sat_max', color_sat_max)
    nettable.putNumber('color_val_min', color_val_min)
    nettable.putNumber('color_val_max', color_val_max)
    nettable.putNumber('shutter_speed', camera.shutter_speed)

    # allow the camera to warmup
    time.sleep(0.1)


    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        nettable.putString('current_ts', datetime.now())
        # Get image
        image = frame.array

        # analysis for findCube
        angle = 0.0
        horizontalDistance = 0.0
        distance = 0.0
        found = False

        camera.shutter_speed = int(nettable.getNumber('shutter_speed', camera.shutter_speed))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(
                               [nettable.getNumber('color_hue_min', color_hue_min),
                                nettable.getNumber('color_sat_min', color_sat_min),
                                nettable.getNumber('color_val_min', color_val_min)]),
                           np.array(
                               [nettable.getNumber('color_hue_max', color_hue_max),
                                nettable.getNumber('color_sat_max', color_sat_max),
                                nettable.getNumber('color_val_max', color_val_max)])
                           )
        mask = cv2.blur(mask, (10, 10))
        mask = cv2.erode(mask, None, iterations=20)
        mask = cv2.dilate(mask, None, iterations=20)
        ret, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
        phase1 = mask.copy()

        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            best_contour = {'contour': None, 'size': 0}
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                # If facing head on the ratio of h/w is 11/13    ~= 0.84615384615
                # If facing at 45 degrees the ratio of h/w is 11/(13 * sqrt(2)) ~= 0.59832112254
                size = w * h
                if size >= minSize and abs(h / w) > 0.5:  # and abs(h/w) < 0.9: #no reason to be too tall, if there is a stack it will line up regardless:
                    if size > best_contour['size']:
                        best_contour = {'contour': contour, 'size': size}

            if best_contour['contour'] is not None:
                x, y, w, h = cv2.boundingRect(best_contour['contour'])
                cv2.rectangle(image, (x, y), (x + w, y + h), red, 3)
                # box = image[y:(y + h), x:(x + w)] #show a frame only of the box
                cx, cy = x + w / 2, y + h / 2

                offset = 320 - cx
                angle = offset * (62.2 / 640)
                straightDistance = ((heightBox * 480 / 2) / h) / (
                math.tan(fieldOfView))  # Perpendicular distance
                distance = straightDistance / math.cos(abs(math.radians(angle)))
                horizontalDistance = math.sqrt(distance * distance - straightDistance * straightDistance)
                # if angle < 0:
                #     horizontalDistance = horizontalDistance * -1
                #horizontalDistance = horizontalDistance - offset

                found = True


        # Publish Angle & Distance
        nettable.putNumber('angle', float(angle))
        nettable.putNumber('horizontalDistance', float(horizontalDistance))
        nettable.putNumber('distance', float(distance))
        nettable.putBoolean('found', found)

        # show the frame
        if options.show:
            hsvtext = str(color_hue_min) + "," + str(color_sat_min) + "," + str(color_val_min) + " " +\
                      str(color_hue_max) + "," + str(color_sat_max) + "," + str(color_val_max)
            cv2.putText(image, "HSV: " + hsvtext, (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, white, 1, 8)
            cv2.putText(image, "Contours:" + str(len(contours)), (0, 40), cv2.FONT_HERSHEY_PLAIN, 1, white, 1, 8)
            cv2.putText(image, "(S)hutter: " + str(camera.shutter_speed), (0, 60), cv2.FONT_HERSHEY_PLAIN, 1, white, 1, 8)
            cv2.putText(image, "Angle: " + str(angle), (20, 400), cv2.FONT_HERSHEY_PLAIN, 1.5, white, 2, 8)
            cv2.putText(image, "Distance: " + str(distance), (20, 420), cv2.FONT_HERSHEY_PLAIN, 1.5, white, 2, 8)
            cv2.putText(image, "H Distance: " + str(horizontalDistance), (20, 380), cv2.FONT_HERSHEY_PLAIN, 1.5, white, 2, 8)
            cv2.putText(image, str(found), (20, 440), cv2.FONT_HERSHEY_PLAIN, 1.5, white, 2, 4)
            cv2.imshow("Image", image)
            cv2.imshow("Mask", phase1)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # Check key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord("S"):
            camera.shutter_speed += 1000
            nettable.putNumber('shutter_speed', camera.shutter_speed)
        if key == ord("s"):
            camera.shutter_speed += -1000
            nettable.putNumber('shutter_speed', camera.shutter_speed)
        if key == ord("I"):
            color_hue_max = adjust_hsv('color_hue_max', color_hue_max, 5, nettable)
        if key == ord("J"):
            color_hue_max = adjust_hsv('color_hue_max', color_hue_max, -5, nettable)
        if key == ord("O"):
            color_sat_max = adjust_hsv('color_sat_max', color_sat_max, 5, nettable)
        if key == ord("K"):
            color_sat_max = adjust_hsv('color_sat_max', color_sat_max, -5, nettable)
        if key == ord("P"):
            color_val_max = adjust_hsv('color_val_max', color_val_max, 5, nettable)
        if key == ord("L"):
            color_val_max = adjust_hsv('color_val_max', color_val_max, -5, nettable)
        if key == ord("i"):
            color_hue_min = adjust_hsv('color_hue_min', color_hue_min, 5, nettable)
        if key == ord("j"):
            color_hue_min = adjust_hsv('color_hue_min', color_hue_min, -5, nettable)
        if key == ord("o"):
            color_sat_min = adjust_hsv('color_sat_min', color_sat_min, 5, nettable)
        if key == ord("k"):
            color_sat_min = adjust_hsv('color_sat_min', color_sat_min, -5, nettable)
        if key == ord("p"):
            color_val_min = adjust_hsv('color_val_min', color_val_min, 5, nettable)
        if key == ord("l"):
            color_val_min = adjust_hsv('color_val_min', color_val_min, -5, nettable)

        if key == ord("q"):  # if the `q` key was pressed, break from the loop
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--show', action='store_true', dest='show', default=False)
    options = parser.parse_args()
    main()