#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5_notebook.msg import Tracker
from rosgraph_msgs.msg import Clock
from cob_srvs.srv import *
from gazebo_msgs.srv import GetModelState
import moveit_msgs.msg
from time import sleep
from threading import Thread
import math
tracker = Tracker()


def grasp(obj):
    rospy.wait_for_service('grasp_human_left')
    try:
        response = rospy.ServiceProxy('grasp_human_left', SetString)
        req = SetStringRequest()
        req.data=obj
        return response(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Blocks:
    
    _targets={}

    _color="g"

    _blockListDict = {
        'b0': Block('/blocks_green_spawnerblock_0', 'Pelvis'),
        'b1': Block('/blocks_green_spawnerblock_1', 'Pelvis'),
        'b2': Block('/blocks_green_spawnerblock_2', 'Pelvis'),
        'b3': Block('/blocks_green_spawnerblock_3', 'Pelvis'),
        'b4': Block('/blocks_green_spawnerblock_4', 'Pelvis'),
        's0': Block('/spheres_green_spawnerblock_0', 'Pelvis'),
        's1': Block('/spheres_green_spawnerblock_1', 'Pelvis'),
        's2': Block('/spheres_green_spawnerblock_2', 'Pelvis'),
        's3': Block('/spheres_green_spawnerblock_3', 'Pelvis'),
        's4': Block('/spheres_green_spawnerblock_4', 'Pelvis'),

    }
    _blockListDict_white = {
        'b0': Block('/blocks_white_spawnerblock_0', 'Pelvis'),
        'b1': Block('/blocks_white_spawnerblock_1', 'Pelvis'),
        'b2': Block('/blocks_white_spawnerblock_2', 'Pelvis'),
        'b3': Block('/blocks_white_spawnerblock_3', 'Pelvis'),
        'b4': Block('/blocks_white_spawnerblock_4', 'Pelvis'),
        's0': Block('/spheres_white_spawnerblock_0', 'Pelvis'),
        's1': Block('/spheres_white_spawnerblock_1', 'Pelvis'),
        's2': Block('/spheres_white_spawnerblock_2', 'Pelvis'),
        's3': Block('/spheres_white_spawnerblock_3', 'Pelvis'),
        's4': Block('/spheres_white_spawnerblock_4', 'Pelvis'),

    }
    def get_gazebo_models(self):
        self.all_green={}
        try:
            if self._color=="g":
                dic=self._blockListDict
            else:
                dic=self._blockListDict_white
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self._targets.clear()
            self.all_green.clear()
            for block in dic.itervalues():
                blockName = str(block._name)
                resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                z=resp_coordinates.pose.position.z
                x=resp_coordinates.pose.position.x
                y=resp_coordinates.pose.position.y
                if z>-0.04 and 0.35>y>0:
                    self._targets[str(block._name)]=[resp_coordinates.pose.position.x,resp_coordinates.pose.position.y]
                if z>-0.04:
                    self.all_green[str(block._name)]=[resp_coordinates.pose.position.x,resp_coordinates.pose.position.y]
            if len(self.all_green)==0:
                self._color="b"
                for block in self._blockListDict_white.itervalues():
                    blockName = str(block._name)
                    resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                    z=resp_coordinates.pose.position.z
                    x=resp_coordinates.pose.position.x
                    if z>-0.04 and 0.35>y>0:
                        self._targets[str(block._name)]=[resp_coordinates.pose.position.x,resp_coordinates.pose.position.y]
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def get_targets(self):
        return self._targets
  
    
class human_movement:
    def __init__(self):
        rospy.init_node("human_mp_left", anonymous=False)
        self.cx = 400.0
        self.cy = 400.0
        self.wpts=[]
        self.blocks = Blocks()
        self.alive=True

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        print(self.robot.get_group_names())
        self.arm = moveit_commander.MoveGroupCommander('left_arm')

        reference_frame = "Pelvis"

        self.arm.set_pose_reference_frame(reference_frame)

        self.end_effector_link = self.arm.get_end_effector_link()

        self.arm.allow_replanning(True)

        self.arm.set_goal_position_tolerance(0.1)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.2)
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        self.scene = moveit_commander.PlanningSceneInterface(ns="/human_robotl")
        
        rospy.sleep(2)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.00
        box_pose.pose.position.y = 0.0
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(1, 2, 0.02))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.7 
        box_pose.pose.position.y = 0.0
        box_name = "box2"
        self.scene.add_box(box_name, box_pose, size=(1, 2, 0.02))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.0 
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.x = -0.3
        box_name = "box3"
        self.scene.add_box(box_name, box_pose, size=(0.02, 2 , 2))
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.0 
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.x = 0.0
        box_name = "box4"
        self.scene.add_box(box_name, box_pose, size=(0.2, 0.2 , 1))

        self.move()

    def cleanup(self):
        rospy.loginfo("Stopping the human")
        self.arm.stop()
        self.alive=False
        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
       

    def spin(self):
        rospy.spin()
      
    #CHECK ALL POSES
    def move(self):
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        print(start_pose)
        pose_goal = deepcopy(start_pose)
        pose_goal.position.x = 0.404565865893
        pose_goal.position.y = 0.207285631433
        pose_goal.position.z = 0.177146627918
        pose_goal.orientation.x=-0.0562389250618
        pose_goal.orientation.y=-0.00608242085117
        pose_goal.orientation.z=-0.997779533885
        pose_goal.orientation.w=0.035159482714
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose_goal)
        plan = self.arm.go(wait=False)
        self.arm.stop()
        sleep(10)
        print("START POINT")
        self.arm.clear_pose_targets()
        self.blocks.get_gazebo_models()
        while self.alive:
            self.blocks.get_gazebo_models()
            targets=self.blocks.get_targets().copy()
            try:
                nearest_tg=targets[list(targets)[0]]
            except IndexError:
                print("NO BLOCKS")
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                take_pose = deepcopy(start_pose)
                take_pose.position.x = 0.422628155526
                take_pose.position.y = 0.587655243535
                take_pose.position.z = 0.314984490975
                take_pose.orientation.x=-0.0327990454713
                take_pose.orientation.y=0.0448338109666
                take_pose.orientation.z=-0.446287652431
                take_pose.orientation.w=0.893163749431
                self.wpts.append(deepcopy(take_pose))
                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.00, True)
                self.arm.execute(plan,wait=False)
                sleep(10)
                self.wpts.pop(0)
                #if len(self.blocks.all_green)==0:
                    #self.cleanup()

            minimum=0
            nearest_tg=""
            for tg in targets.keys():
                if abs(targets[tg][1])>abs(minimum):
                    minimum=targets[tg][1]
                    nearest_tg=tg
            if nearest_tg=="":
                print("no blocks, waiting")
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                take_pose = deepcopy(start_pose)
                take_pose.position.x = 0.422628155526
                take_pose.position.y = 0.587655243535
                take_pose.position.z = 0.314984490975
                take_pose.orientation.x=-0.0327990454713
                take_pose.orientation.y=0.0448338109666
                take_pose.orientation.z=-0.446287652431
                take_pose.orientation.w=0.893163749431
                self.wpts.append(deepcopy(take_pose))
                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.00, True)
                self.arm.execute(plan,wait=False)
                sleep(10)
                self.wpts.pop(0)
            else:
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose

                take_pose = deepcopy(start_pose)
                take_pose.position.x = targets[nearest_tg][0]+0.05
                take_pose.position.y = targets[nearest_tg][1]
                take_pose.position.z = 0.1
                self.wpts.append(deepcopy(take_pose))
                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.00, True)
                self.arm.execute(plan,wait=False)
                sleep(10)
                self.wpts.pop(0)
                print("Taking block")
                check_pose = self.arm.get_current_pose(self.end_effector_link).pose
                if abs(check_pose.position.x-targets[nearest_tg][0])<0.2 and abs(check_pose.position.y-targets[nearest_tg][1])<0.2:
                    if nearest_tg[1]=="b":
                        grasp(nearest_tg)
                        print("took block")
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        take_pose = deepcopy(start_pose)
                        take_pose.position.x = 0.422628155526
                        take_pose.position.y = 0.587655243535
                        take_pose.position.z = 0.314984490975
                        take_pose.orientation.x=-0.0327990454713
                        take_pose.orientation.y=0.0448338109666
                        take_pose.orientation.z=-0.446287652431
                        take_pose.orientation.w=0.893163749431
                        self.wpts.append(deepcopy(take_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.00, True)
                        self.arm.execute(plan,wait=False)
                        sleep(10)
                        self.wpts.pop(0)
                        grasp("x")
                        print("dropped block")
                    else:
                        grasp(nearest_tg)
                        print("took sphere")
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        take_pose = deepcopy(start_pose)
                        take_pose.position.x = 0.250705672725
                        take_pose.position.y = -0.101496676632
                        take_pose.position.z = 0.484566062209
                        take_pose.orientation.x=0.0549008313445
                        take_pose.orientation.y=0.00709543061992
                        take_pose.orientation.z=0.998426074423
                        take_pose.orientation.w=0.00899597100743
                        self.wpts.append(deepcopy(take_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.00, True)
                        self.arm.execute(plan,wait=False)
                        sleep(10)
                        self.wpts.pop(0)
                        grasp("x")
                        print("dropped sphere")
                    

movement=human_movement()

rospy.spin()
