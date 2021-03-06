#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int8, Int16
import math
import numpy as np
import cv2
from close_encounters_ur5.srv import *
from close_encounters_ur5.msg import *

class Constants:
    # the program checks no more than 1/sleep_time per second
    sleep_time = 0.01
    # default resolution values, to determine where the center is
    res_x, res_y = 640, 480
    # center of the frame
    cen_x, cen_y = 320, 240
    # values for determining distance to face
    distance_threshold_1 = 90
    distance_threshold_2 = 50
    # prepare response for set mode service
    result = SetVisionModeResponse()
    result.result = True

class Variables:
    # to do: add x and y of the dice to these lists to draw squares over them in the original frane (for feedback)
    dice_x_list = []
    dice_y_list = []

class Callbacks:
    def set_vision_checks(self, mode):
        # set the modes to True/False
        if mode.mode == 1:
            faceRecognition.mode = True
        else:
            faceRecognition.mode = False
        if mode.mode == 2:
            diceMain.mode = True
        else:
            diceMain.mode = False
        if mode.mode == 3:
            cupRecognition.mode = True
        else:
            cupRecognition.mode = False
        # check if the mode was out of range
        if mode.mode < 0 or mode.mode > 3:
            rospy.logwarn("\t\t\tVision: Tried to set unknown mode.")
        return visionConstants.result

class FaceRecognition:
    def __init__(self):
        # keep track of the mode
        self.mode = False
        # init vision stuff for face recognition
        self.face_cascade = cv2.CascadeClassifier('/home/ubuntu/Documents/vision/opencv/data/haarcascades/haarcascade_frontalface_default.xml')
        # decide where the center is
        self.cen_x = visionConstants.res_x / 2
        self.cen_y = visionConstants.res_y / 2
        # prepare a request for set_position
        self.coordinates = FaceCoordinates()
    def run(self, frame):
        if self.mode == True:
            # only do face recognition when mode is set to face recognition
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            # create lists for face coordinates, distance from the camera, and distance to the center of the frame
            x_list = []
            y_list = []
            d_list = []
            e_list = []
            # set these values to the first components of the lists
            chosen_face = 0
            j = 0
            for (x,y,w,h) in faces:
                # draw a rectangle around the face
                cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,0),5)
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
                # check if the chosen face should be updated
                if d_list[chosen_face] > d_list[j]:
                    # face is closer than the other ones in the list
                    chosen_face = j
                elif d_list[chosen_face] == d_list[j] and e_list[j] < e_list[chosen_face]:
                    # the face is as close and another face in the list, but it is closer to the center of the frame
                    chosen_face = j
                j += 1
            # check if there's at least one face in the list
            if len(d_list) > 0:
                # draw cross over the the chosen face
                cv2.line(frame,(x_list[chosen_face], 0),(x_list[chosen_face], visionConstants.res_y),(0,0,255),5)
                cv2.line(frame,(0, y_list[chosen_face]),(visionConstants.res_x, y_list[chosen_face]),(0,0,255),5)
                # prepare coordinates for publishing
                self.coordinates.x = x_list[chosen_face]
                self.coordinates.y = y_list[chosen_face]
                self.coordinates.d = d_list[chosen_face]
                self.coordinates.e = e_list[chosen_face]
                rospy.loginfo("\t\t\tVision: Face found at coordinates: " + str(x_list[chosen_face]) + " " + str(y_list[chosen_face]) + " " + str(d_list[chosen_face])+ " " + str(e_list[chosen_face]))
            else:
                # no face found --> don't draw a cross on the frame
                # prepare coordinates for publishing
                self.coordinates.x, self.coordinates.y, self.coordinates.d, self.coordinates.e = -1, -1, -1, -1
            # publish coordinates for the move queue to check the position and send that to the position memory
            face_coordinates_publisher.publish(self.coordinates)

class DiceMain():
    def __init__(self):
        self.mode = False
        self.saveCount = 0
    def main(self, frame):
        if self.mode == True:
            # execute the code
            self.frameRes = cv2.resize(frame, (900, 900))
            self.mask, self.result, self.contourFrame = diceFunctions.contours(self.frameRes)
            self.keypoints = dicePipCount.amount_of_pips(self.result)
            self.amount = dicePipCount.amount_of_dice(frame)
            # show results
            #self.showImages()
    def showImages(self):
        cv2.imshow("Mask", self.mask)
        cv2.imshow("Result", self.result)
        cv2.imshow("keypoints", self.keypoints)
        cv2.imshow("contours", self.contourFrame)
        """if cv2.waitKey(1) &0xFF == ord("s"):
            cv2.imwrite("trayWithDiceRes" + str(self.saveCount)+".jpg", self.result)
            cv2.imwrite("Keypoints" + str(self.saveCount)+ ".jpg", self.keypoints)
            cv2.imwrite("Contours" + str(self.saveCount)+".jpg", self.contourFrame)
            cv2.imwrite("FrameRaw" + str(self.saveCount) + ".jpg", self.frameRaw)
            cv2.imwrite("FrameResized_900X900" + str(self.saveCount) + ".jpg", self.frameRes)
            self.saveCount += 1
            rospy.loginfo("saved image: trayWithDice" + str(self.saveCount) + ".jpg")
            """

class DiceFunctions():
    def contours(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert to HSV values
        # Create the upper and lower green values thanks to trackbars
        Upper_H = 200 #cv2.getTrackbarPos("Upper_H", "trackbars")
        Upper_S = 100 #cv2.getTrackbarPos("Upper_S", "trackbars")
        Upper_V = 255 #cv2.getTrackbarPos("Upper_V", "trackbars")
        Lower_H = 0 #cv2.getTrackbarPos("Lower_H", "trackbars")
        Lower_S = 0 #cv2.getTrackbarPos("Lower_S", "trackbars")
        Lower_V = 200 #cv2.getTrackbarPos("Lower_V", "trackbars")
        upper_green = np.array([Upper_H, Upper_S, Upper_V])
        lower_green = np.array([Lower_H, Lower_S, Lower_V])
        # Filter the color between upper and lower value
        mask = cv2.inRange(hsv, lower_green, upper_green)
        #maskCircle = np.zeros((900, 900, 3), dtype="uint8")
        res = cv2.bitwise_and(frame, frame, mask=mask)
        resGray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        #edge_detected_image = cv2.Canny(resGray, 75, 200)
        img, contours, hierarchy = cv2.findContours(resGray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_list = []
        for c in contours:
            length = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*length, True)
            if len(approx == 4) and length > 100 and length < 250:
                # print length
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                diceCalculate.center(box)
                cv2.drawContours(frame, [box], 0, (0,255,255), 5) # Rotated box
        return mask, res, frame

class DiceCalculate():
    centervar = []
    orientationvar = []
    def center(self, box):
        if len(self.centervar) >1000:
            self.centervar = [0.0]
        point0X = box[0][0]
        point0Y = box[0][1]
        point1X = box[1][0]
        point1Y = box[1][1]
        point2X = box[2][0]
        point2Y = box[2][1]
        point3X = box[3][0]
        point3Y = box[3][1]
        centerX = int((point3X-point1X)/2 + point1X)
        centerY = int((point0Y - point2Y)/2 + point2Y)
        self.centervar.append({"x": centerX, "y": centerY})
        self.centervar = self.centervar[-2:]
        self.orientation(box)
    def orientation(self, boxPoints):
        if len(self.orientationvar) >1000:
            self.orientationvar = [0.0]
        point0X = boxPoints[0][0]
        point0Y = boxPoints[0][1]
        point1X = boxPoints[1][0]
        point1Y = boxPoints[1][1]
        point2X = boxPoints[2][0]
        point2Y = boxPoints[2][1]
        point3X = boxPoints[3][0]
        point3Y = boxPoints[3][1]
        deltaX = point3X - point0X
        deltaY = point0Y - point3Y
        angleRad = math.atan2(deltaY, deltaX)
        angle = math.degrees(angleRad)
        self.orientationvar.append(angleRad)
        self.orientationvar = self.orientationvar[-2:]

class DicePipCount():
    dice_results = [-1, -1]
    previous_dice_result = -1
    pip_results = [0, 0]
    previous_pip_result = 0
    kernel_dice_count = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    kernel = (5, 5)
    kernelOpen = np.ones((5,5))
    kernelClose = np.ones((20,20))
    def amount_of_dice(self, frame):
        # Emty list to keep track of amount of dices.
        self.contours_list_dice = []
        # Apply blur to smooth out noise and convert to hsv colorspace.
        blur = cv2.GaussianBlur(frame, (15, 15), 2)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        # Ranges for green.
        upper_green = np.array([101 , 255, 255])
        lower_green = np.array([50, 60, 16])
        # Filter the color between upper and lower value.
        mask = cv2.inRange(hsv, lower_green, upper_green)
        # Image enhancement. 
        maskClosed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_dice_count)
        maskOpen = cv2.morphologyEx(maskClosed, cv2.MORPH_OPEN, self.kernel_dice_count)
        # Apply edge detection.
        edge_detected_image = cv2.Canny(maskClosed, 75, 175)
        # Find contours.
        img, contours, hierarchy = cv2.findContours(edge_detected_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Look in the contours that are found.
        for c in contours:
            area = cv2.contourArea(c)
            length = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*length, True)
            x,y,w,h = cv2.boundingRect(c)
            # Check if contour is dice and keep track of the amount. 
            if len(approx > 4) and length > 30  and w >25 and h > 25 and w <100 and h < 100 and area >500:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                self.contours_list_dice.append(c)
                cv2.drawContours(frame, c, 0, (0, 0, 255), 1)
        #cv2.imshow('mask', maskClosed)
        #cv2.imshow("Dice detected", frame)
        #cv2.imshow("edge", edge_detected_image)
        # Return the amount of dice to the main function. 
        dice_result = (len(self.contours_list_dice)/2)
        # make sure the list doesn't grow longer than this value
        if len(self.dice_results) > 5:
            del self.dice_results[0]
        self.dice_results.append(dice_result)
        try:
            if self.dice_results[-1] == self.dice_results[-2] == self.dice_results[-3] and self.previous_dice_result != dice_result:
                self.previous_dice_result = dice_result
                dice_amount_publisher.publish(dice_result)
                rospy.loginfo("\t\t\tVision: Amount of dice found: " + str(dice_result))
                return dice_result
            else:
                return -1
        except:
            return -1
    def amount_of_pips(self, frame):
        # Create parameters for blobdetection
        parameters = cv2.SimpleBlobDetector_Params()
        parameters.minDistBetweenBlobs = 5 #minDistBlob
        parameters.filterByColor = True
        parameters.blobColor = 0
        parameters.filterByInertia = True
        parameters.minInertiaRatio = 0.5 #minInertia/10
        parameters.maxInertiaRatio = 1.0 #maxInertia/10
        parameters.filterByArea = True
        parameters.minArea = 50
        parameters.maxArea = 200
        parameters.filterByCircularity = True
        parameters.filterByConvexity = False
        # create blob detector
        detector = cv2.SimpleBlobDetector_create(parameters) 
        # Filter frame
        frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, self.kernel) 
        frame = cv2.dilate(frame, (self.kernel))
        # Detect the keypoints
        keypoints = detector.detect(frame) 
        # Draw the keypoints on frame
        img_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
        # amount of keypoints
        pip_result = len(keypoints) 
        # update the output value if the amount is the same for 3 times and different from the previous output
        self.pip_results.append(pip_result)
        if self.pip_results[-1] == self.pip_results[-2] == self.pip_results[-3] and self.previous_pip_result != pip_result:
            self.previous_pip_result = pip_result
            rospy.loginfo("\t\t\tVision: amount of pips: " + str(pip_result))
            dice_score_publisher.publish(pip_result)
        # make sure the length doesn't become to long
        if len(self.pip_results) > 5:
            del self.pip_results[0]
        text = cv2.putText(img_with_keypoints, str(pip_result), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
        return img_with_keypoints

class Cup():
    def __init__(self):
        # keep track of the mode
        self.mode = False
        # needed variables for the opening and closing masks and making the contour
        self.kernelOpen = np.ones((5,5))
        self.kernelClose = np.ones((20,20))
        self.contour_list = []
        # determine what part of the frame needs to be taken for the cup detection
        h = 400
        w = 400
        self.top = int(visionConstants.cen_y - (h / 2))
        self.bottom = int(visionConstants.cen_y + (h / 2))
        self.left = int(visionConstants.cen_x - (w / 2))
        self.right = int(visionConstants.cen_x + (w / 2))
        # keep track of the previous result
        self.previous_cup_result = 0
    def main(self, frame):
        if self.mode == True:
            # show where the detection is looking
            cv2.rectangle(frame,(self.left,self.top),(self.right,self.bottom),(0,255,0),1)
            # take the center part of the frame
            cropped_frame = frame[self.top:self.bottom, self.left:self.right]
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
            # define range of blue color in HSV
            lower_color = np.array([100,110,0])
            upper_color = np.array([140,255,255])
            # Threshold the HSV image to get only green colors
            mask = cv2.inRange(hsv, lower_color, upper_color)
            # reduce noise
            maskOpen = cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernelOpen)
            maskClose = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,self.kernelClose)
            dilated = cv2.dilate(maskClose, self.kernelOpen)
            bilateral_filtered_image = cv2.bilateralFilter(dilated, 5, 175, 175)
            # apply edge detection
            edge_detected_image = cv2.Canny(bilateral_filtered_image, 75, 200)
            # find contours
            _,contours,hier = cv2.findContours(edge_detected_image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            # (re)set value for setting and checking later
            cup_result = 0
            # check contrours for the cup
            for c in contours:
                # determine area of the contour
                area = cv2.contourArea(c)
                x,y,w,h=cv2.boundingRect(c)
                approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
                if (len(approx) > 8) and (len(approx) < 23) and (area > 30):
                    self.contour_list.append(c)
                # check if the width, height, and area are all viable
                if w < 200 and w > 100 and h < 350 and h > 200 and area < 70000 and area > 20000:
                    # cup found
                    cv2.rectangle(frame,(self.left+x,self.top+y),(self.left+x+w,self.top+y+h),(0,255,0),5)
                    cup_result = 1
                    break
            if self.previous_cup_result != cup_result:
                self.previous_cup_result = cup_result
                # print and publish the result
                if cup_result == 1:
                    rospy.loginfo("\t\t\tVision: Cup detected.")
                else:
                    rospy.loginfo("\t\t\tVision: Cup lost.")
                cup_detected_publisher.publish(cup_result)

if __name__ == '__main__':
    try:
        # start a new node
        rospy.init_node('vision_node', anonymous=True)
        rospy.loginfo("\t\t\tVision: Node starting.")
        
        visionConstants = Constants()
        visionVariables = Variables()
        visionCallbacks = Callbacks()

        # init publishers for the function nodes that need it, like the trackers
        face_coordinates_publisher = rospy.Publisher('/vision_face_coordinates', FaceCoordinates, queue_size=1)
        dice_score_publisher = rospy.Publisher('/vision_score', Int8, queue_size=1)
        dice_amount_publisher = rospy.Publisher('/vision_amount', Int8, queue_size=1)
        cup_detected_publisher = rospy.Publisher('/vision_cup_detected', Int8, queue_size=1)

        # init video feed
        cap = cv2.VideoCapture(-1)

        # take a frame from the video feed for determiniing some initial values
        _, frame = cap.read()

        if cap.isOpened():
            # determine resolution and center of the camera frame
            visionConstants.res_x, visionConstants.res_y = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            visionConstants.cen_x = int(visionConstants.res_x / 2)
            visionConstants.cen_y = int(visionConstants.res_y / 2)
            rospy.loginfo("\t\t\tVision: Camera resolution: " + str(visionConstants.res_x) + ", " + str(visionConstants.res_y) + ".")
        else:
            rospy.logfatal("\t\t\tVision: No camera found. Please make sure it is connected and try again.")

        # init face recognition
        faceRecognition = FaceRecognition()

        # init dice recognition
        diceMain = DiceMain()
        diceFunctions = DiceFunctions()
        diceCalculate = DiceCalculate()
        dicePipCount = DicePipCount()

        # init cup recognition
        cupRecognition = Cup()

        # init mode setting service
        rospy.Service('/vision_checks', SetVisionMode, visionCallbacks.set_vision_checks)

        if cap.isOpened():
            # run all
            while not rospy.is_shutdown():
                # take a frame from the video feed
                _, frame = cap.read()
                # run everything using the frame from the video feed
                faceRecognition.run(frame)
                diceMain.main(frame)
                cupRecognition.main(frame)
                # show the altered frame
                cv2.imshow('frame', frame)
                cv2.waitKey(30)
                # wait a bit before taking the next frame
                rospy.sleep(visionConstants.sleep_time)
        else:
            rospy.logfatal("Vision: No camera found. Please make sure it is connected and try again.")

        # shutdown
        rospy.loginfo("Vision: Shutting down.")
        cv2.destroyAllWindows()
        cap.release()

    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        cap.release()
