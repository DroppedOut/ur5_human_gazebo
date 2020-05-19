#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_notebook.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()
lower = np.array([ 30,  150, 60])    #lower = np.array([ 0,  0, 163])  white
upper = np.array([80, 255, 255])      #upper = np.array([0, 0, 255])  

def trigger_response(request):
    global lower 
    lower = np.array([ 30,  70, 60])
    global upper 
    upper= np.array([90, 255, 255])

    return TriggerResponse(success=True,message="OK.")

class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)

    def detect(self, c):
		shape = 0 #sphere
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		if 6>len(approx) >3:
			shape=1 #cube
		else:
			shape = 0
		return shape

    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV
        # BEGIN FILTER
        global lower
        global upper
        mask = cv2.inRange(hsv,lower,upper)
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        # print h, w, d  (800,800,3)
        #BEGIN FINDER

        # cx range (55,750) cy range( 55, ~ )
        # END FINDER
        # Isolate largest contour
        #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
        for i, c in enumerate(cnts):
            area = cv2.contourArea(c)
            if area > 500:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                if cx<100 or cx>700 or cy<100:
                    break
                self.track_flag = True
                self.cx = cx
                self.cy = cy
                self.error_x = self.cx - w/2
                self.error_y = self.cy - (h/2+195)
                tracker.x = cx
                tracker.y = cy
                tracker.flag1 = self.track_flag
                tracker.error_x = self.error_x
                tracker.error_y = self.error_y
                tracker.flag3 = self.detect(c)
                #(_,_,w_b,h_b)=cv2.boundingRect(c)
                #print w_b,h_b
                # BEGIN circle
                cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
                cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.drawContours(image, cnts, -1, (0, 0, 255),1)
                #BGIN CONTROL
                break
            else:
                self.track_flag = False
                tracker.flag1 = self.track_flag


        self.cxy_pub.publish(tracker)
        cv2.namedWindow("camera", 1)
        cv2.imshow("camera", image )
        cv2.waitKey(1)

switch_mask_service = rospy.Service('/mask_color', Trigger, trigger_response)
follower=ur5_vision()
rospy.spin()
