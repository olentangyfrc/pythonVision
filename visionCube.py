#!/usr/bin/env python

from __future__ import division
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables


# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
vary = 60
yellowLower = (27 - vary,255 - vary, 150 - vary)#(29, 86, 6)
yellowUpper = (27 + vary,255 + vary, 150 + vary)##(35 + vary,255 + vary,255 + vary)#(64, 255, 255)

minSize = 150 # of contour, incase there are two



def mainMethod():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = 18000  # 0 to 31163; 0 is auto
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.25')
    nettable = NetworkTables.getTable("Vision")

    # allow the camera to warmup
    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the current frame
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        mask = cv2.blur(mask, (10,10))
        mask = cv2.erode(mask, None, iterations=20)
        mask = cv2.dilate(mask, None, iterations=20)
        ret, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
        realmask = mask.copy()
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


        x = -1
        y = -1

        if len(contours) > 0:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h >= minSize:
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    box = image[y:(y + h), x:(x + w)]
                    cx, cy = x + w/2, y + h/2

                    offset = 320 - cx
                    angle = offset * (62.2 / 640)

                    nettable.putNumber('angle', float(angle))
                    cv2.imshow("Box", box)
                    break

                    #http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html
                    #dst = cv2.cornerHarris(gray, 2, 3, 0.04)







        # show the frame
        cv2.imshow("Image", image)
        #cv2.imshow("RealMask", realmask)
        #cv2.imshow("Box", box)
        #if box is not None:
            #print("not none found")
            #cv2.imshow("Box", box)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

def main():
    mainMethod()

if __name__ == '__main__':
    main()
