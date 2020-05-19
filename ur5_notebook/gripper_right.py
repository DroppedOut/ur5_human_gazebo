#!/usr/bin/env python

import rospy
import moveit_commander
from threading import Thread
from gazebo_msgs.msg import ModelState
from cob_srvs.srv import *
from gazebo_msgs.srv import GetModelState

class grasper:
    def __init__(self):
        self.alive=True
        rospy.init_node("human_g_right", anonymous=False)
        rospy.on_shutdown(self.cleanup)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander('right_arm')
        self.arm.set_pose_reference_frame("Pelvis")
        self.end_effector_link = self.arm.get_end_effector_link()

        self.taken_item="x"
        self.grasp_thread=Thread(target=self.grasp)
        self.grasp_thread.start()

    def grasp(self):
        pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        model_state=ModelState()
        while self.alive:
            if self.taken_item !="x":
                item_pose = self.arm.get_current_pose(self.end_effector_link).pose
                item_pose.position.z-=0.05
                model_state.model_name=self.taken_item
                model_state.pose=item_pose
                model_state.reference_frame="Pelvis"   
                pose_pub.publish(model_state)   
 
    def cleanup(self):
        rospy.loginfo("Stopping the human")
        self.alive=False
        self.arm.stop()
        self.grasp_thread.join()
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

def trigger_response(request):
    global mygrasper
    mygrasper.taken_item=request.data
    return SetStringResponse(success=True,message="OK.")  

service = rospy.Service('/grasp_human_right', SetString, trigger_response)
global mygrasper
mygrasper=grasper()

rospy.spin()



