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
    # values for determining distance to face
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
        # prepare a request for set_position
        self.coordinates = SetPositionRequest()
    def run(self):
        # take a frame
        ret, img = self.cap.read()
        if self.mode == True:
            # only do face recognition when mode is set to face recognition
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            # create lists for face coordinates, distance from the camera, and distance to the center of the frame
            x_list = []
            y_list = []
            d_list = []
            e_list = []
            for (x,y,w,h) in faces:
                # draw a rectangle around the face
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),5)
                # determine position of the center of the face
                x_face_center = x + w / 2
                y_face_center = y + h / 2
                # determine distance value of this face
                if h > visionConstants.distance_threshold_1:
                    d = 1 # this face is within 1 meter
                elif h > visionConstants.distance_threshold_2:
                    d = 2 # this face is between 1 and 2 meters
                else:
                    d = 0 # this face isn't within 2 meters
                # determine this face's distance to the center of the screen
                e = math.sqrt((self.cen_x - x_face_center) * (self.cen_x - x_face_center) + (self.cen_y - y_face_center) * (self.cen_y - y_face_center))
                # append determined data to the lists
                x_list.append(x_face_center)
                y_list.append(y_face_center)
                d_list.append(d)
                e_list.append(e)
            # check if there's at least one face in the list
            if len(d_list) > 0:
                # set value to the first component of the lists
                chosen_face = 0
                # decide which face to track
                for j in range(0, len(d_list)):
                    if d_list[chosen_face] > d_list[j]:
                        if e_list[j] < e_list[chosen_face]:
                            chosen_face = j
                # draw cross over the the chosen face
                cv2.line(img,(x_list[chosen_face], 0),(x_list[chosen_face], visionConstants.res_y),(0,0,255),5)
                cv2.line(img,(0, y_list[chosen_face]),(visionConstants.res_y, y_list[chosen_face]),(0,0,255),5)
                # publish face values
                visionPublishers.faces(x_list[chosen_face], y_list[chosen_face], d_list[chosen_face])
                # send x and y to the move queue for processing
                self.coordinates.x = x_list[chosen_face]
                self.coordinates.y = y_list[chosen_face]
                visionSetFacePosition(self.coordinates)
            else:
                # publish that there's no face at the moment
                visionPublishers.faces(-1, -1, -1)
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
        
        visionConstants = Constants()
        visionCallbacks = Callbacks()

        # init publishers for the function nodes that need it, like the trackers
        face_x_publisher = rospy.Publisher('/vision_face_x', Int16, queue_size=1)
        face_y_publisher = rospy.Publisher('/vision_face_y', Int16, queue_size=1)
        face_d_publisher = rospy.Publisher('/vision_face_d', Int8, queue_size=1)
        dice_score_publisher = rospy.Publisher('/vision_score', Int8, queue_size=1)
        visionPublishers = Publishers()

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
            #(run other functionalities like dice/cup recognition)
            # wait a bit before taking the next frame
            time.sleep(visionConstants.sleep_time)

        # shutdown
        faceRecognition.shutdown()
        cv2.destroyAllWindows()

    except rospy.ROSInterruptException:
        faceRecognition.shutdown()