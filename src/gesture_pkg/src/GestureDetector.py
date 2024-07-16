#!/usr/bin/python
#from itertools import count

__author__ = 'wessi'
import cv2
import math
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class GestureRecognizer:
    NODE_NAME = 'gesture_node'
    CX_INITIAL_VALUE = 0
    flag = True
    X = 0
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.publisher = rospy.Publisher('/gesture_channel', String, queue_size=10)
        self.img_subscriber = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        cv2.namedWindow("Gesture and Contour Window")
        self.rate = rospy.Rate(15)

    def getInitialCx(self, cx):

        if GestureRecognizer.flag:
            GestureRecognizer.CX_INITIAL_VALUE = cx
            GestureRecognizer.flag = False

        return GestureRecognizer.CX_INITIAL_VALUE

    def callback(self, data):
        try:
            img = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")  # Convert ROS image to CVMat

            cv2.rectangle(img,(2,10),(650,650),(0,255,0),0)
            crop_img = img[2:650, 2:650]
            grey = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            value = (35, 35)
            blurred = cv2.GaussianBlur(grey, value, 0)
            _, thresh1 = cv2.threshold(blurred, 200, 255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            _, contours, hierarchy = cv2.findContours(thresh1.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            max_area = -1
            for i in range(len(contours)):
                cnt=contours[i]
                area = cv2.contourArea(cnt)
                if(area>max_area):
                    max_area=area
                    ci=i
            cnt=contours[ci]

        #######################This line draws the center#####################
            moments = cv2.moments(cnt)
            if moments['m00']!=0:
                        cx = int(moments['m10']/moments['m00']) # cx = M10/M00
                        cy = int(moments['m01']/moments['m00']) # cy = M01/M00

            centr=(cx,cy)
            cv2.circle(img,centr,5,[0,0,255],2)
        ######################This line draws the center######################

            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(crop_img,(x,y),(x+w,y+h),(0,0,255),0)
            hull = cv2.convexHull(cnt)
            drawing = np.zeros(crop_img.shape,np.uint8)
            cv2.drawContours(drawing,[cnt],0,(0,255,0),0)
            cv2.drawContours(drawing,[hull],0,(0,0,255),0)
            hull = cv2.convexHull(cnt,returnPoints = False)
            defects = cv2.convexityDefects(cnt,hull)
            count_defects = 0

            for i in range(defects.shape[0]):
                s,e,f,d = defects[i,0]
                start = tuple(cnt[s][0])
                end = tuple(cnt[e][0])
                far = tuple(cnt[f][0])
                a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
                if angle <= 90:
                    count_defects += 1
                    cv2.circle(crop_img,far,1,[0,0,255],-1)
                elif angle >= 180 and angle >= 90:
                    cv2.putText(img, str, (30,100), cv2.FONT_HERSHEY_DUPLEX, 1.6, 1.6)
                cv2.line(crop_img, start, end, [0, 0, 0], 2)
            if count_defects == 1:
                str = "Two fingers up"
            elif count_defects == 2:
                str = "Three fingers up"
            elif count_defects == 3:
                str = "Four fingers up"
            elif count_defects == 4:

                if abs(self.CX_INITIAL_VALUE-cx)>3:#if the gap in two direction of cx > 3
                    str= "\"Hi hi...\" Hand Waving..."#str= "Waving..%d" % cx
                    self.CX_INITIAL_VALUE = cx
                else:
                    str= "\"Hi hi...\" "

            else:
                str = "Recognizing Hand Gesture..."
            cv2.putText(img, str, (30,100), cv2.FONT_HERSHEY_COMPLEX, 1.2, 1.2)

            self.publisher.publish(str)
            # self.rate.sleep()
            all_img = np.hstack((drawing, crop_img))
            cv2.imshow('Gesture and Contour Window', all_img)
            cv2.waitKey(3)
            #print(0115516777)tembirawo eskemizor| yesew girgida eskemiyankuakua/shua bitn binil min...
        except CvBridgeError, e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node(GestureRecognizer.NODE_NAME, anonymous=False)
    rospy.loginfo("Starting " + GestureRecognizer.NODE_NAME+"...")
    GestureRecognizer()
    try:
        rospy.loginfo(GestureRecognizer.NODE_NAME+" started succefully!")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Stopping " + GestureRecognizer.NODE_NAME)
