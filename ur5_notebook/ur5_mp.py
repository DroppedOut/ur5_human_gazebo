#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur5_notebook.msg import Tracker
from std_srvs.srv import Trigger, TriggerRequest

from time import sleep
tracker = Tracker()


def switch_mask():
    rospy.wait_for_service('mask_color')
    try:
        response = rospy.ServiceProxy('mask_color', Trigger)
        req = TriggerRequest()
        return response(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.phase = 1
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()
        self.pre_shape=0
        
        self.wpts=[]
        self.side=0

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        # wpose.position.x = 0.1052
        # wpose.position.y = -0.4271
        # wpose.position.z = 0.4005
        #
        # wpose.orientation.x = 0.4811
        # wpose.orientation.y = 0.5070
        # wpose.orientation.z = -0.5047
        # wpose.orientation.w = 0.5000

        # self.waypoints.append(deepcopy(wpose))


        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # self.arm.set_pose_target(wpose)




        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266                 #1 
        self.default_joint_states[3] = -1.67721                       #-0.88455
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()

        self.arm.execute(plan)
        """
        # Specify end states for blue bin(drop object)
        self.end_joint_states_blue = deepcopy(self.default_joint_states)
        self.end_joint_states_blue[0] = -3.65
        self.end_joint_states_blue[2] = 1.79266
        self.end_joint_states_blue[3] = -1.67721
        # self.end_joint_states_blue[1] = -1.3705

        # Specify end states for red bin(drop object)
        self.end_joint_states_red = deepcopy(self.default_joint_states)
        self.end_joint_states_red[0] = 0.0
        self.end_joint_states_red[2] = 1.79266
        self.end_joint_states_red[3] = -1.67721
        # self.end_joint_states_red[1] = -1.3705

        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -3.65
        self.transition_pose[2] = 1.79266
        self.transition_pose[3] = -1.67721
        self.transition_pose[4] = -1.57

        self.transition_pose_red = deepcopy(self.transition_pose)
        self.transition_pose_red[0] = 0.0
        """

        # Specify end states for blue bin(drop object)
        self.end_joint_states_blue = deepcopy(self.default_joint_states)
        self.end_joint_states_blue[0] = -2.5
        self.end_joint_states_blue[2] = 1.79266
        self.end_joint_states_blue[3] = -1.67721
        # self.end_joint_states_blue[1] = -1.3705

        # Specify end states for red bin(drop object)
        self.end_joint_states_red = deepcopy(self.default_joint_states)
        self.end_joint_states_red[0] = -0.77
        self.end_joint_states_red[2] = 1.79266
        self.end_joint_states_red[3] = -1.67721
        # self.end_joint_states_red[1] = -1.3705

        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -2.5
        self.transition_pose[2] = 1.79266
        self.transition_pose[3] = -1.67721
        self.transition_pose[4] = -1.57

        self.transition_pose_red = deepcopy(self.transition_pose)
        self.transition_pose_red[0] = -0.77
        
        self.buff_pose=self.arm.get_current_pose(self.end_effector_link).pose

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        self.pre_shape=msg.flag3
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        if (self.track_flag ):  #and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6
            self.execute()
        else:
            self.track_flag = False
            self.execute()


    def execute(self):
        if self.track_flag:


            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            print(start_pose)
            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102


            if len(self.pointx)>8:
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 2 * x_speed
                    wpose.position.z = 0.07
                    self.shape=self.pre_shape

                else:
                    if len(self.pointx)==11:
                        tracker.flag2 = 1
                        self.cxy_pub.publish(tracker)

                    if len(self.pointx)<12:
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)

                    else:
                        if tracker.flag2:
                            self.track_flag=False
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.z = 0.4000
                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.arm.set_max_acceleration_scaling_factor(.01)
                        self.arm.set_max_velocity_scaling_factor(.08)
                        print(self.shape)
                        if self.shape:
                            self.arm.set_joint_value_target(self.transition_pose)
                            self.arm.set_start_state_to_current_state()
                            plan = self.arm.plan()
                            self.arm.execute(plan)

                            self.arm.set_joint_value_target(self.end_joint_states_blue)
                            self.arm.set_start_state_to_current_state()
                            plan = self.arm.plan()
                            self.arm.execute(plan)

                            self.arm.set_max_acceleration_scaling_factor(.05)
                            self.arm.set_max_velocity_scaling_factor(.05)
                            self.waypoints = []
                            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                            transition_pose = deepcopy(start_pose)
                            transition_pose.position.x -= 0.20 #0.15
                            transition_pose.position.y -= 0.20
                        else:
                            self.arm.set_joint_value_target(self.transition_pose_red)
                            self.arm.set_start_state_to_current_state()
                            plan = self.arm.plan()
                            self.arm.execute(plan)

                            self.arm.set_joint_value_target(self.end_joint_states_red)
                            self.arm.set_start_state_to_current_state()
                            plan = self.arm.plan()
                            self.arm.execute(plan)                    

                            self.arm.set_max_acceleration_scaling_factor(.05)
                            self.arm.set_max_velocity_scaling_factor(.05)
                            self.waypoints = []
                            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                            transition_pose = deepcopy(start_pose)
                            transition_pose.position.x += 0.30 #0.20
                            transition_pose.position.y -= 0.30

                        transition_pose.position.z = 0.1 #0.0
                        self.waypoints.append(deepcopy(transition_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        print("DROP")
                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)
                        print("RETURN")

                        transition_pose = deepcopy(start_pose)
                        self.waypoints.append(deepcopy(transition_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)
                        print("CONTINUE SEARCH")


            # Set the next waypoint to the right 0.5 meters
            else:
                wpose.position.x -= self.error_x*0.02/105
                wpose.position.y += self.error_y*0.04/105
                wpose.position.z = 0.15
                #wpose.position.z = 0.4005

            if self.phase == 1:
                self.waypoints.append(deepcopy(wpose))


                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                # self.arm.set_pose_target(wpose)

                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints

                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


                # plan = self.arm.plan()

                # If we have a complete plan, execute the trajectory
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else:
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            self.phase=1
            wpose = deepcopy(self.buff_pose)
            if self.side%5==0:
                wpose.position.x=-0.2                #-0.40
                self.side+=1
            if self.side%5==1:
                if wpose.position.x<0.10:            #0.40
                    wpose.position.x+=0.1
                    wpose.position.y=-0.45
                    self.wpts.append(deepcopy(wpose))
                    plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.0, True)
                    self.arm.execute(plan)
                    self.wpts.pop(0)
                    self.buff_pose=deepcopy(wpose)
                else:
                    self.side+=1
            if self.side%5==2:
                if wpose.position.y>-0.5:
                    wpose.position.y-=0.1
                    self.wpts.append(deepcopy(wpose))
                    plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.0, True)
                    self.arm.execute(plan)
                    self.wpts.pop(0)
                    self.buff_pose=deepcopy(wpose)
                else:
                    self.side+=1
            if self.side%5==3:
                if wpose.position.x>-0.1:          #-0.40
                    wpose.position.x-=0.1
                    self.wpts.append(deepcopy(wpose))
                    plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.0, True)
                    self.arm.execute(plan)
                    self.wpts.pop(0)
                    self.buff_pose=deepcopy(wpose)
                else:
                    self.side+=1
            if self.side%5==4:
                if wpose.position.y<-0.5:
                    wpose.position.y+=0.1
                    self.wpts.append(deepcopy(wpose))
                    plan, fraction = self.arm.compute_cartesian_path(self.wpts, 0.02, 0.0, True)
                    self.arm.execute(plan)
                    self.wpts.pop(0)
                    self.buff_pose=deepcopy(wpose)
                else:
                    self.side+=1
                    print(switch_mask())

            
mp=ur5_mp()

rospy.spin()
