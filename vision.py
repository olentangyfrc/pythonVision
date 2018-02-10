#!/usr/bin/env python

#TODO use networktables to see which one to look for#


from __future__ import division
import time
import math
import cv2
import numpy as np
import argparse
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables


# At 12 feet away..
# 3.684 pixels per inch
# 2 in width is 7.368 pixels
# 56.3 pixel for 15.3 viewable inches

#Camera Variables
fieldOfView = math.radians(48.8/2) # input degrees
heightBox = 11

#Cube Variables
vary = 75
yellowLower = (27 - vary,255 - vary, 150 - vary)#(29, 86, 6)
yellowUpper = (27 + vary,255 + vary, 150 + vary)##(35 + vary,255 + vary,255 + vary)#(64, 255, 255)
minSize = 150 # of contour, incase there are two

#Tape Variables
minArea = 200  # ~350 pixels
maxDist = 3  # distance between 1 contour and another in widths
allow = 3  # number of widths it can vary

def findCube(image):
    angle = 0.0
    horizontalDistance = 0.0
    distance = 0.0
    found = False

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    mask = cv2.blur(mask, (10, 10))
    mask = cv2.erode(mask, None, iterations=20)
    mask = cv2.dilate(mask, None, iterations=25)
    ret, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
    phase1 = mask.copy()

    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            #If facing head on the ratio of h/w is 11/13    ~= 0.84615384615
            #If facing at 45 degrees the ratio of h/w is 11/(13 * sqrt(2)) ~= 0.59832112254
            if w * h >= minSize:
                if abs(h/w) > 0.7: #and abs(h/w) < 0.9: #no reason to be too tall, if there is a stack it will line up regardless
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    #box = image[y:(y + h), x:(x + w)] #show a frame only of the box
                    cx, cy = x + w / 2, y + h / 2

                    offset = 320 - cx
                    angle = offset * (62.2 / 640)

                    straightDistance = ((heightBox * 480 / 2) / h) / (math.tan(fieldOfView))  # Perpendicular distance
                    distance = straightDistance / math.cos(abs(math.radians(angle)))
                    horizontalDistance = math.sqrt(distance * distance - straightDistance * straightDistance)

                    found = True

                    return found, angle, distance, horizontalDistance, len(contours), hsv, phase1
                else:
                    #Blue means it sees it but ratios are off. Is there another box aside, or behind? "we may never know"
                     cropped = image[y:(y + h), x:(x + w)]
                     cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 3)

                     #failed to get most recent point
                     #extBot = tuple(cropped[cropped[:, :, 1].argmax()][0])
                     #cv2.circle(image, extBot, 10, (255, 255, 255), -1)


    return found, angle, distance, horizontalDistance, len(contours), hsv, phase1

def findTape(image):
    angle = 0.0
    distance = 0.0
    horizontalDistance = 0.0
    found = False

    # Adjust image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(mask, np.array([50, 50, 125]), np.array([100, 255, 255]))
    mask = cv2.erode(hsv, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)


    phase1 = mask.copy()
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    correctContours = [] #a list of contours with the correct size

    numContours = 0
    if len(contours) > 0:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w * h) > minArea:  # if the area is greater than 100, used to remove single pixels
                if (h / w) > 2.5:  # ideally, h = 15.3 and w = 2, so the ratio would be ~3
                    correctContours.append(contour)
                    numContours += 1

        # Loop through filtered contours
        for contour in correctContours:
            x1, y1, w1, h1 = cv2.boundingRect(contour)
            for contour2 in correctContours:
                x2, y2, w2, h2 = cv2.boundingRect(contour2)
                if x2 - x1 != 0:
                    if abs(x2 - x1) < maxDist * w1:
                        if abs(x2 - (x1 + w1 * 3)) < allow * w1:
                            h3 = 0  # the longer height of the two
                            if h1 >= h2:
                                h3 = h1
                            else:
                                h3 = h2

                            cv2.rectangle(image, (x1, y1), (x2 + w2, y2 + h3), (0, 0, 255), 3)
                            centerX = (x2 + w2 + x1) / 2;
                            centerY = (y2 + h2 + y2) / 2;

                            # Draw center of box
                            cv2.circle(image, (int(centerX), int(centerY)), 3, (0, 0, 255), 3)

                            # Find angle and distance
                            offset = 320 - centerX
                            angle = offset * (62.2 / 640)

                            heightTape = 15.3  # in inches
                            fieldOfView = math.radians(48.8 / 2)  # input degrees

                            length = ((heightTape * 480 / 2) / h3)
                            straightDistance = ((heightTape * 480 / 2) / h3) / (math.tan(fieldOfView))  # Perpendicular distance
                            distance = straightDistance / math.cos(abs(math.radians(angle)))  # Accounts for if the tape is off to the side (hypotenuse distance)
                            horizontalDistance = math.sqrt(distance * distance - straightDistance * straightDistance)

                            found = True
                            return found, angle, distance, horizontalDistance, len(contours), hsv, phase1
    return found, angle, distance, horizontalDistance, len(contours), hsv, phase1

def picamvidopencv():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = 21000 #18000 for cube #2448 for Tape
    rawCapture = PiRGBArray(camera, size=(640, 480))
    crosshair = [320, 240]
    toggle_rectangles = True

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.2')
    nettable = NetworkTables.getTable("Vision")

    # allow the camera to warmup
    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #Get image
        image = frame.array

        #Determine what to look for based on networktables
        if True:
            found, angle, distance, horizontalDistance, contoursLength, hsv, phase1 = findCube(image)
        else:
            found, angle, distance, horizontalDistance, contoursLength, hsv, phase1 = findTape(image)

        # Publish Angle & Distance
        nettable.putNumber('angle', float(angle))
        nettable.putNumber('horizontalDistance', float(horizontalDistance))
        nettable.putNumber('distance', float(distance))
        nettable.putBoolean('found', found)

        if options.show:
            cv2.putText(image, "Angle: " + str(angle), (200, 400), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3, 8)
            cv2.putText(image, "Distance: " + str(distance), (200, 420), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255),3, 8)
            cv2.putText(image, "H Distance: " + str(horizontalDistance), (200, 380), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255),3, 8)
            cv2.putText(image, str(found), (200, 440), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 3, 8)
            cv2.putText(image, "Contours:" + str(contoursLength), (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1,8)
            cv2.putText(image, "(S)hutter: " + str(camera.shutter_speed), (0, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.line(image, (crosshair[0] - 10, crosshair[1]), (crosshair[0] + 10, crosshair[1]), (255, 255, 255), thickness=1)
            cv2.line(image, (crosshair[0], crosshair[1] - 10), (crosshair[0], crosshair[1] + 10), (255, 255, 255), thickness=1)
            cv2.putText(image, "HSV: " + str(hsv[crosshair[0], crosshair[1]]), (0, 60), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)

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


        # show the frame
        if options.show:
             cv2.imshow("Image", image)
             cv2.imshow("Mask", phase1)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

def main():
    picamvidopencv()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--show', action='store_true', dest='show', default=False)
    options = parser.parse_args()
    main()