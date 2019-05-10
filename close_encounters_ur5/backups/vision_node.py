#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int8, Int16
import time
import math
import numpy as np
import cv2

class Constants:
    # the program checks no more than 1/sleep_time per second
    sleep_time = 0.1
    # resolution values, to determine where the center is
    res_x, res_y = 640, 480
    # values for distance determining
    distance_threshold_1 = 90
    distance_threshold_2 = 50

class Publishers:
    def faces(_, face_x, face_y, face_d):
        face_x_publisher.publish(face_x)
        face_y_publisher.publish(face_y)
        face_d_publisher.publish(face_d)
    def dice(_, score):
        dice_score_publisher.publish(score)

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
        # (re)set values
        i = 0
        x_list = []
        y_list = []
        d_list = []
        h_list = []
        for (x,y,w,h) in faces:
            # draw a rectangle around the face
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),5)
            # append to the lists
            x_list.append(1)
            y_list.append(1)
            d_list.append(1)
            h_list.append(1)
            # insert coordinates and distance to center
            x_list.insert(i, x + w / 2)
            y_list.insert(i, y + h / 2)
            d_list.insert(i, math.sqrt((self.cen_x - x) * (self.cen_x - x) + (self.cen_y - y) * (self.cen_y - y)))
            h_list.insert(i, h)
            # raise index for the appending and inserting
            i += 1
        # check if the list has at least one component
        if d_list != []:
            # set values to the first component of the lists
            x_chosen, y_chosen, d_chosen, h_chosen = x_list[0], y_list[0], d_list[0], h_list[0]
            # decide which face to track
            for j in range(0, i):
                #info = "\nface number: " + str(j) + "\nx: " + str(x_list[j]) + "\ny: " + str(y_list[j]) + "\nd: " + str(d_list[j]) + "\nheight: " + str(h_list[j])
                #rospy.loginfo(info)
                if d_list[j] < d_chosen:
                    x_chosen, y_chosen, d_chosen = x_list[j], y_list[j], d_list[j]
                if h_list[j] < h_chosen:
                    h_chosen = h_list[j]
            # determine distance value
            distance_value = 0 # no face within 2 meters
            if h_chosen > constants.distance_threshold_1:
                distance_value = 1 # face within 1 meter
            elif h_chosen > constants.distance_threshold_2:
                distance_value = 2 # face between 1 and 2 meters
            #info = "Vision: face distance value: " + str(distance_value)
            #rospy.loginfo(info)
            # publish face values
            publishers.faces(x_chosen, y_chosen, distance_value)
            # draw cross over the the chosen face
            cv2.line(img,(x_chosen, 0),(x_chosen, constants.res_y),(0,0,255),5)
            cv2.line(img,(0, y_chosen),(constants.res_x, y_chosen),(0,0,255),5)
        else:
            # publish that there's no face at the moment
            publishers.faces(-1, -1, -1)
            #rospy.loginfo("Vision: no face")
        # show image
        cv2.imshow('img',img)
        cv2.waitKey(30)
    def shutdown(self):
        self.cap.release()

class DiceRecognition:
    def __init__(self):
        # init vision stuff for dice recognition
        self.min_threshold = 10                      # these values are used to filter our detector.
        self.max_threshold = 200                     # they can be tweaked depending on the camera distance, camera angle, ...
        self.min_area = 100                          # ... focus, brightness, etc.
        self.min_circularity = .3                    # default .3
        self.min_inertia_ratio = .5
        self.cap = cv2.VideoCapture(0)               # '0' is the webcam's ID. usually it is 0 or 1. 'cap' is the video object.
        self.cap.set(15, -4)                         # '15' references video's brightness. '-4' sets the brightness.
        self.counter = 0                             # script will use a counter to handle FPS.
        self.readings = [0, 0]                       # lists are used to track the number of pips.
        self.display = [0, 0]
    def run(self):
        # check for dice and number of pips
        if self.counter >= 90000:                # set maximum sizes for variables and lists to save memory.
            self.counter = 0
            self.readings = [0, 0]
            self.display = [0, 0]
        ret, im = self.cap.read()                                    # 'im' will be a frame from the video.
        params = cv2.SimpleBlobDetector_Params()                # declare filter parameters.
        params.filterByArea = True
        params.filterByCircularity = True
        params.filterByInertia = True
        params.minThreshold = self.min_threshold
        params.maxThreshold = self.max_threshold
        params.minArea = self.min_area
        params.minCircularity = self.min_circularity
        params.minInertiaRatio = self.min_inertia_ratio
        detector = cv2.SimpleBlobDetector_create(params)        # create a blob detector object.
        keypoints = detector.detect(im)                         # keypoints is a list containing the detected blobs.
        # here we draw keypoints on the frame.
        #im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255),
        #                                        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #cv2.imshow("Dice Reader", im_with_keypoints)            # display the frame with keypoints added.
        self.reading = len(keypoints)                                # 'reading' counts the number of keypoints (pips).
        if self.counter % 10 == 0:                                   # enter this block every X frames.
            self.readings.append(self.reading)                            # note the reading from this frame.
            if self.readings[-1] == self.readings[-2] == self.readings[-3]:    # if the last 3 readings are the same...
                self.display.append(self.readings[-1])                    # ... then we have a valid reading.
            # if the most recent valid reading has changed, and it's something other than zero, then print it.
            if self.display[-1] != self.display[-2] and self.display[-1] != 0:
                info = str(self.display[-1]) + "\n****"
                rospy.loginfo(info)
                publishers.dice(self.display[-1])
        self.counter += 1
        cv2.waitKey(30)
    def shutdown(self):
        self.cap.release()

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('vision_node', anonymous=True)
        rospy.loginfo("vision node starting")
        # init publishers for the function nodes that need it, like the trackers
        face_x_publisher = rospy.Publisher('/vision_face_x', Int16, queue_size=1) # x in video
        face_y_publisher = rospy.Publisher('/vision_face_y', Int16, queue_size=1) # y in video
        face_d_publisher = rospy.Publisher('/vision_face_d', Int8, queue_size=1) # h in video --> distance
        dice_score_publisher = rospy.Publisher('/vision_score', Int8, queue_size=1) # number of pips in frame --> score
        publishers = Publishers()
        # init constants
        constants = Constants()
        # init face recognition
        faceRecognition = FaceRecognition()
        #diceRecognition = DiceRecognition()
        # run all
        while not rospy.is_shutdown():
            # run everything
            faceRecognition.run()
            #diceRecognition.run()
            # (run other functionalities like hand/cup recognition)
            # wait a bit before taking the next frame
            time.sleep(constants.sleep_time)
        # shutdown
        faceRecognition.shutdown()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass