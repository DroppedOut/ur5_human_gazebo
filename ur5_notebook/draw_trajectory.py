#!/usr/bin/env python

import rospy
import moveit_commander
from threading import Thread
from gazebo_msgs.msg import ModelState
from cob_srvs.srv import *
from gazebo_msgs.srv import GetModelState
import cv2
import numpy as np

class pen:
    def __init__(self):
        self.alive=True
        rospy.init_node("human_g_left", anonymous=False)
        rospy.on_shutdown(self.cleanup)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander('left_arm')
        self.arm.set_pose_reference_frame("Pelvis")
        self.end_effector_link = self.arm.get_end_effector_link()
        self.paint()



    def paint(self):
        cv2.namedWindow("image", 1)
        self.img = np.zeros((512,512,3), np.uint8)
        cv2.rectangle(self.img,(256-37,256-25+30),(256+37,256+25+30),(165,165,165),-1)
        cv2.rectangle(self.img,(256+60-20,256-20+30),(256+60+20,256+20+30),(255,0,0),-1)
        cv2.rectangle(self.img,(256-60-20,256-20+30),(256-60+20,256+20+30),(0,0,255),-1)
        while self.alive:
            pose = self.arm.get_current_pose(self.end_effector_link).pose
            x=int(pose.position.y*100+256)
            y=int(pose.position.x*100+256)
            cv2.rectangle(self.img,(x,y),(x+1,y+1),(0,255,0),-1)
            cv2.imshow("image",self.img)
            cv2.waitKey(1)
 
    def cleanup(self):
        rospy.loginfo("Stopping the human")
        self.alive=False
        self.arm.stop()
        cv2.destroyAllWindows()
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


mypen=pen()

rospy.spin()



