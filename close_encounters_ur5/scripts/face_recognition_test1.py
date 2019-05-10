#!/usr/bin/env python

# ros stuff
import sys
import rospy
from std_msgs.msg import Int16

# vision stuff
import numpy as np
import cv2
# other
import time
import math

class Constants:
    res_x = 640
    res_y = 480

class FaceRecognition:
    def __init__(self):
        # init vision stuff for face recognition
        self.face_cascade = cv2.CascadeClassifier('/home/ubuntu/Documents/vision/opencv/data/haarcascades/haarcascade_frontalface_default.xml')
        self.cap = cv2.VideoCapture(0)
        # decide where the center is
        self.cen_x = constants.res_x / 2
        self.cen_y = constants.res_y / 2
    def run(self):
        # take a frame
        ret, img = self.cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        # reset values
        x1, x2, x3, x_chosen = 0, 0, 0, 0
        y1, y2, y3, y_chosen = 0, 0, 0, 0
        d1, d2, d3, d_chosen = 0, 0, 0, 0
        i = 0
        for (x,y,w,h) in faces:
            # draw a rectangle around the face
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),5)
            # overwrite coordinates and distance to center
            i += 1
            if i == 1:
                x1 = x + w / 2
                y1 = y + h / 2
                d1 = math.sqrt(math.pow((self.cen_x - x),2) + math.pow((self.cen_y - y),2))
            elif i == 2:
                x2 = x + w / 2
                y2 = y + h / 2
                d2 = math.sqrt(math.pow((self.cen_x - x),2) + math.pow((self.cen_y - y),2))
            else:
                x3 = x + w / 2
                y3 = y + h / 2
                d3 = math.sqrt(math.pow((self.cen_x - x),2) + math.pow((self.cen_y - y),2))
        # decide which face to track
        if x1 != 0:
            x_chosen, y_chosen, d_chosen = x1, y1, d1
        if d2 < d_chosen and x2 != 0:
            x_chosen, y_chosen, d_chosen = x2, y2, d2
        if d3 < d_chosen and x3 != 0:
            x_chosen, y_chosen, d_chosen = x3, y3, d3
        # draw cross over the the chosen face
        if x_chosen != 0:
            cv2.line(img,(x_chosen, 0),(x_chosen, constants.res_y),(0,0,255),5)
            cv2.line(img,(0, y_chosen),(constants.res_x, y_chosen),(0,0,255),5)
        # show image
        cv2.imshow('img',img)
        cv2.waitKey(30)
        # print values
        print("x")
        print(x1, x2, x3)
        print("chosen x")
        print(x_chosen)
        print("y")
        print(y1, y2, y3)
        print("chosen y")
        print(y_chosen)
        print("d")
        print(d1, d2, d3)
        print("chosen d")
        print(d_chosen)
        print(" ")
    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('vision_node', anonymous=False)
        rospy.loginfo("vision node starting")

        # init publishers for the function nodes that need it, like the trackers
        face_x_publisher = rospy.Publisher('/vision_face_x', Int16, queue_size=1)
        face_y_publisher = rospy.Publisher('/vision_face_y', Int16, queue_size=1)

        # init constants
        constants = Constants()

        # init face recognition
        faceRecognition = FaceRecognition()

        # run all
        while not rospy.is_shutdown():
            faceRecognition.run()
            # wait a bit
            time.sleep(0.05)

        # shutdown
        faceRecognition.shutdown()

    except rospy.ROSInterruptException:
        pass