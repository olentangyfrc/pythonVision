#!/usr/bin/env python

import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from networktables import NetworkTables


def picamvidopencv():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    camera.shutter_speed = 5000  # 0 to 31163; 0 is auto
    rawCapture = PiRGBArray(camera, size=(640, 480))
    crosshair = [320, 240]
    toggle_rectangles = True
    contours_big = 0
    contours_small = 0
    distance = 0

    # initialize network tables
    NetworkTables.initialize(server='10.46.11.25')
    nettable = NetworkTables.getTable("Vision")

    # allow the camera to warmup
    time.sleep(0.1)

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, np.array([50, 50, 150]), np.array([100, 255, 255]))
        phase1 = mask.copy()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contours_big = 0
            contours_small = 0
            for contour in contours:
                if cv2.contourArea(contour) > 2000:
                    bx,by,bw,bh = cv2.boundingRect(contour)
                    if toggle_rectangles:
                        cv2.rectangle(image, (bx, by), (bx + bw, by + bh), (0, 0, 255), 5)
                    mid = bx + (bw/2)
                    offset = 320 - mid
                    angle = offset * (62.2 / 640)
                    distance = 3770 / bw
                    contours_big += 1
                    nettable.putNumber('angle', float(angle))
                else:
                    sx,sy,sw,sh = cv2.boundingRect(contour)
                    if toggle_rectangles:
                        cv2.rectangle(image, (sx, sy), (sx + sw, sy + sh), (255, 0, 0), 2)
                    contours_small += 1

        if contours_big == 0:
            camera.shutter_speed += 0
        elif contours_big == 1:
            cv2.putText(image, str(bw) + "," + str(bh), (320, 420), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.putText(image, str(angle), (320, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            cv2.putText(image, str(distance), (320, 440), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
            if contours_small > 20:
                camera.shutter_speed += 0
        elif contours_big >= 2:
            camera.shutter_speed += 0

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
        if key == ord("i"):
            crosshair[1] += -1
        if key == ord("m"):
            crosshair[1] += 1
        if key == ord("j"):
            crosshair[0] += -1
        if key == ord("k"):
            crosshair[0] += 1
        if key == ord("I"):
            crosshair[1] += -10
        if key == ord("M"):
            crosshair[1] += 10
        if key == ord("J"):
            crosshair[0] += -10
        if key == ord("K"):
            crosshair[0] += 10

        cv2.putText(image, "Contours:" + str(len(contours)), (0, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
        cv2.putText(image, "(S)hutter: " + str(camera.shutter_speed), (0, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
        cv2.line(image, (crosshair[0]-10, crosshair[1]), (crosshair[0]+10, crosshair[1]), (255, 255, 255), thickness=1)
        cv2.line(image, (crosshair[0], crosshair[1]-10), (crosshair[0], crosshair[1]+10), (255, 255, 255), thickness=1)
        cv2.putText(image, "HSV: " + str(hsv[crosshair[0], crosshair[1]]), (0, 60), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)


        # show the frame
        cv2.imshow("Frame", image)
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
