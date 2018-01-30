#!/usr/bin/env python

from __future__ import division
import time
import math
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables

# At 12 feet away..
# 3.684 pixels per inch
# 2 in width is 7.368 pixels
# 56.3 pixel for 15.3 viewable inches

def picamvidopencv():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = 2448 #18000  # 0 to 31163; 0 is auto
    rawCapture = PiRGBArray(camera, size=(640, 480))
    crosshair = [320, 240]
    toggle_rectangles = True

    angle = 0.0
    realDistance = 0.0
    found = False

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.2')
    nettable = NetworkTables.getTable("Vision")

    # allow the camera to warmup
    time.sleep(0.1)


    minArea = 200 # ~350 pixels
    maxDist = 3 # distance between 1 contour and another in widths
    allow = 3 # number of widths it can vary



    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        centerX = 0
        centerY = 0

        angle = 0.0
        realDistance = 0.0
        found = False

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Adjust image
        #mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        mask = cv2.erode(hsv, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.inRange(mask, np.array([50, 50, 125]), np.array([100, 255, 255]))

        phase1 = mask.copy()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        correctContours = []
        largeCrop = phase1[100:300, 100:500]

        numContours = 0
        if len(contours) > 0:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if (w * h) > minArea: #if the area is greater than 100, used to remove single pixels
                    if (h / w) > 2.5: # ideally, h = 15.3 and w = 2, so the ratio would be ~3
                        correctContours.append(contour)
                        #cv2.rectangle(image, (x, y),(x + w, y + h), (0, 0, 255), 3)
                        numContours += 1

            #Loop through filtered contours
            for contour in correctContours:
                x1, y1, w1, h1 = cv2.boundingRect(contour)
                for contour2 in correctContours:
                    x2, y2, w2, h2 = cv2.boundingRect(contour2)
                    if x2 - x1 != 0:
                        if abs(x2 - x1) < maxDist * w1:
                            if abs(x2 - (x1 + w1 * 3)) < allow * w1:
                                h3 = 0 #the longer height of the two
                                if h1 >= h2:
                                    h3 = h1
                                else:
                                    h3 = h2

                                cv2.rectangle(image, (x1, y1), (x2 + w2, y2 + h3), (0, 0, 255), 3)
                                centerX = (x2 + w2 + x1)/2;
                                centerY = (y2 + h2 + y2)/2;

                                # Draw center of box
                                cv2.circle(image, (int(centerX), int(centerY)), 3, (0,0,255), 3)

                                #Find angle and distance
                                offset = 320 - centerX
                                angle = offset * (62.2 / 640)

                                heightTape = 15.3 # in inches
                                fieldOfView = math.radians(48.8/2) # input degrees

                                length = ((heightTape * 480/2)/h3)
                                distance = ((heightTape * 480/2)/h3)/(math.tan(fieldOfView)) #Perpendicular distance
                                realDistance = distance/math.cos(abs(math.radians(angle))) #Accounts for if the tape is off to the side (hypotenuse distance)

                                found = True


        # Publish Angle & Distance
        nettable.putNumber('angle', float(angle))
        nettable.putNumber('distance', float(realDistance))
        nettable.putBoolean('found', found)
        cv2.putText(image, "Angle: " + str(angle), (200, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3, 8)
        cv2.putText(image, "Distance: " + str(realDistance), (200, 420), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3, 8)
        cv2.putText(image, str(found), (200, 440), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3, 8)


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
        # cv2.imshow("Image", image)
        # cv2.imshow("Mask", phase1)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


def main():
    picamvidopencv()

if __name__ == '__main__':
    main()