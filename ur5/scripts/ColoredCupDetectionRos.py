#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time 

 
#needed variables for the opening and closing masks and making the contour
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))
contour_list = []


#take frame from device to check if the device opened correctly 
#_, frame = cap.read()
class Cup():
    def __init__(self):
        pass
    
    def main(self):
        # Take each frame
        _, frame = cap.read()

        #resize if necessary
        #frame = cv2.resize(frame, (480,360))

        #copy frame to output
        output = frame.copy()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # define range of blue color in HSV
        lower_color = np.array([100,110,0])
        upper_color = np.array([140,255,255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_color, upper_color)

        #remove background noise 
        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)

        #fill in empty gaps inside object 
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        #clone the final mask into frame 
        maskFinal=maskClose


        
        #increase white border around object
        dilated = cv2.dilate(maskFinal,kernelOpen)

        #smooth the frame with a bilateral filter
        bilateral_filtered_image = cv2.bilateralFilter(dilated, 5, 175, 175)

        #apply edge detection on the framee
        edge_detected_image = cv2.Canny(bilateral_filtered_image, 75, 200)

        #find contours in canny detected image
        contours,hier,_ = cv2.findContours(edge_detected_image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            #Contour area is taken
            area = cv2.contourArea(c)
            x,y,w,h=cv2.boundingRect(c)

            approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
        

            if ((len(approx) > 8) & (len(approx) < 23) & (area > 30) ):
                contour_list.append(c)

            if area >5000 and area<20000:
                cv2.rectangle(output,(5,40),(400,100),(255,255,255),cv2.FILLED)
                cv2.putText(output, "Cup detected in range", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 1, 1, 3)

            # discard areas that are too large or too small 
            if (h>400 and w>400) or (h<50 or w<50):
                continue

            #draw a box around the object
            cv2.rectangle(output    ,(x,y),(x+w,y+h),(0,0,255), 2)

            #calculate center of box 
            centerX = x+(w/2)
            centerY = y+(h/2)

            #draw a dot on the center of the box
            cv2.circle(output, (centerX, centerY), 2, (0,255,255), 5)
        
        #show output frame
        cv2.imshow('output', output)
        cv2.imshow('trackbar', img)
        
        #exit program when pressing 'q' button 
    if cv2.waitKey(25) & 0xFF == ord('q'):
        self.shutdown()


    def shutdown(self):
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        cap = cv2.VideoCapture(0)
        time.sleep(2)

        if cap.isOpened():
            CupProgram = Cup()
            while not rospy.is_shutdown():
                CupProgram.main() 
                time.sleep(0.1)
    except rospy.ROSInterruptException:
        CupProgram.shutdown()

