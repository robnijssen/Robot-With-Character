#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int8, Int16
import time
import math
import numpy as np
import cv2
from close_encounters_ur5.srv import SetVisionMode, SetVisionModeResponse
from close_encounters_ur5.srv import SetPosition, SetPositionRequest

class Constants:
    # the program checks no more than 1/sleep_time per second
    sleep_time = 0.01
    # resolution values, to determine where the center is
    res_x, res_y = 640, 480
    # values for distance determining
    distance_threshold_1 = 90
    distance_threshold_2 = 50
    # prepare response for set mode service
    result = SetVisionModeResponse()
    result.result = True

class Publishers:
    def faces(_, face_x, face_y, face_d):
        face_x_publisher.publish(face_x)
        face_y_publisher.publish(face_y)
        face_d_publisher.publish(face_d)
    def dice(_, score):
        dice_score_publisher.publish(score)

class Callbacks:
    def set_vision_checks(self, mode):
        if mode.mode == 0:
            faceRecognition.mode = False
        elif mode.mode == 1:
            faceRecognition.mode = True
        else:
            rospy.logwarn("Vision: Tried to set unknown mode.")
        return visionConstants.result

class FaceRecognition:
    def __init__(self):
        # keep track of the mode
        self.mode = False
        # init vision stuff for face recognition
        self.face_cascade = cv2.CascadeClassifier('/home/ubuntu/Documents/vision/opencv/data/haarcascades/haarcascade_frontalface_default.xml')
        self.cap = cv2.VideoCapture(0)
        # decide where the center is
        self.cen_x = visionConstants.res_x / 2
        self.cen_y = visionConstants.res_y / 2
    def run(self):
        # take a frame
        ret, img = self.cap.read()
        if self.mode == True:
            # only do face recognition when mode is set to face recognition
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            # (re)set values
            x_list = []
            y_list = []
            d_list = []
            h_list = []
            for (x,y,w,h) in faces:
                # draw a rectangle around the face
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),5)
                # append to the lists
                x_list.append(x + w / 2)
                y_list.append(y + h / 2)
                d_list.append(math.sqrt((self.cen_x - x) * (self.cen_x - x) + (self.cen_y - y) * (self.cen_y - y)))
                h_list.append(math.sqrt((self.cen_x - x) * (self.cen_x - x) + (self.cen_y - y) * (self.cen_y - y)))
            # check if the list has at least one component
            if d_list != []:
                # set values to the first component of the lists
                x_chosen, y_chosen, d_chosen, h_chosen = x_list[0], y_list[0], d_list[0], h_list[0]
                # decide which face to track
                for j in range(0, len(d_list)):
                    #info = "\nface number: " + str(j) + "\nx: " + str(x_list[j]) + "\ny: " + str(y_list[j]) + "\nd: " + str(d_list[j]) + "\nheight: " + str(h_list[j])
                    #rospy.loginfo(info)
                    if d_list[j] < d_chosen:
                        x_chosen, y_chosen, d_chosen = x_list[j], y_list[j], d_list[j]
                    if h_list[j] < h_chosen:
                        h_chosen = h_list[j]
                # determine distance value
                distance_value = 0 # no face within 2 meters
                if h_chosen > visionConstants.distance_threshold_1:
                    distance_value = 1 # face within 1 meter
                elif h_chosen > visionConstants.distance_threshold_2:
                    distance_value = 2 # face between 1 and 2 meters
                #info = "Vision: face distance value: " + str(distance_value)
                #rospy.loginfo(info)
                # publish face values
                publishers.faces(x_chosen, y_chosen, distance_value)
                # draw cross over the the chosen face
                cv2.line(img,(x_chosen, 0),(x_chosen, visionConstants.res_y),(0,0,255),5)
                cv2.line(img,(0, y_chosen),(visionConstants.res_x, y_chosen),(0,0,255),5)
                # send x and y to the move queue for processing
                coordinates = SetPositionRequest()
                coordinates.x = x_chosen
                coordinates.y = y_chosen
                visionSetFacePosition(coordinates)
            else:
                # publish that there's no face at the moment
                publishers.faces(-1, -1, -1)
                # return -1 to the result
                distance_value = -1
                #rospy.loginfo("Vision: no face")
        # show complete image
        cv2.imshow('img',img)
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

        visionConstants = Constants()
        visionCallbacks = Callbacks()

        # init face recognition
        faceRecognition = FaceRecognition()

        # init services
        rospy.Service('/vision_checks', SetVisionMode, visionCallbacks.set_vision_checks)

        # wait for services to start
        rospy.wait_for_service('/set_face_position')
        rospy.wait_for_service('/set_cup_position')
        visionSetFacePosition = rospy.ServiceProxy('/set_face_position', SetPosition)
        visionSetCupPosition = rospy.ServiceProxy('/set_cup_position', SetPosition)

        # run all
        while not rospy.is_shutdown():
            # run everything
            faceRecognition.run()
            # (run other functionalities like dice/cup recognition)
            # wait a bit before taking the next frame
            time.sleep(visionConstants.sleep_time)

        # shutdown
        faceRecognition.shutdown()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        faceRecognition.shutdown()