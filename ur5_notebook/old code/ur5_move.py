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

from time import sleep
tracker = Tracker()



class ur5_move:
    def __init__(self):
        rospy.init_node("ur5_move", anonymous=False)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.points=[]
        self.state_change_time = rospy.Time.now()
        self.error_x=0
        self.error_y=0

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

        #-----------------------------------------------------------------------
        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1                 #1.79266
        self.default_joint_states[3] = -0.88455
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        # Specify end states (drop object)
        self.end_joint_states = deepcopy(self.default_joint_states)
        self.end_joint_states[0] = -3.65
        self.end_joint_states[2] = 1.79266
        self.end_joint_states[3] = -1.67721
        # self.end_joint_states[1] = -1.3705
        
        # Specify transition states (move object)
        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -1.57691
        self.transition_pose[2] = 1.79266
        self.transition_pose[3] = -1.67721
        self.transition_pose[4] = -1.57
        #-----------------------------------------------------------------------

        #Get to start pose
        self.exec_transit_pose()
        self.search()

    def tracking_callback(self, msg):
        self.track_flag = msg.flag1
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        """
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            self.find_and_take()
            self.default_pose_flag = False
        else:
            self.track_flag = False
            self.exec_start_pose()
            self.default_pose_flag = True
        """
    
    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def exec_start_pose(self):
        print("Executing Start pose")
        #target
        self.arm.set_joint_value_target(self.default_joint_states)
        #current
        self.arm.set_start_state_to_current_state()
        #plan current->target
        plan = self.arm.plan()
        #execute
        self.arm.execute(plan)

    def exec_transit_pose(self):
        print("Executing Transit pose")
        #target
        self.arm.set_joint_value_target(self.transition_pose)
        #current
        self.arm.set_start_state_to_current_state()
        #plan current->target
        plan = self.arm.plan()
        #execute
        self.arm.execute(plan)

    def exec_drop_pose(self):
        print("Executing Drop pose")
        #target
        self.arm.set_joint_value_target(self.end_joint_states)
        #current
        self.arm.set_start_state_to_current_state()
        #plan current->target
        plan = self.arm.plan()
        #execute
        self.arm.execute(plan)

    def search(self):
        print("Starting search...")
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        waypoints=[]
        wpose = deepcopy(start_pose)
        wpose.position.x=-0.45

        while wpose.position.x<0.45:
            wpose.position.x+=0.1
            wpose.position.y=-0.3
            waypoints.append(deepcopy(wpose))
            plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.02, 0.0, True)
            self.arm.execute(plan)
            if self.track_flag:
                self.find_and_take()
                return 0
                plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.02, 0.0, True)
                self.arm.execute(plan)
            waypoints.pop(0)
        """
        while wpose.position.y>-0.6:
            wpose.position.y-=0.1
            waypoints.append(deepcopy(wpose))
            plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.02, 0.0, True)
            self.arm.execute(plan)
            waypoints.pop(0)
        while wpose.position.x>-0.35:
            wpose.position.x-=0.1
            waypoints.append(deepcopy(wpose))
            plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.02, 0.0, True)
            self.arm.execute(plan)
            waypoints.pop(0)
        while wpose.position.y<-0.3:
            print(wpose.position.y)
            wpose.position.y+=0.1
            waypoints.append(deepcopy(wpose))
            plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.02, 0.0, True)
            self.arm.execute(plan)
            waypoints.pop(0)
        """

    def find_and_take(self):
        print("Found somth")
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        self.waypoints= []
        wpose = deepcopy(start_pose)
        #px=10.5/800
        #wpose.position.z =0.15
        #self.waypoints.append(deepcopy(wpose))
        #plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
        #self.arm.execute(plan)
        print(self.error_x*0.78*0.01)
        wpose.position.x-=self.error_x*0.78*0.01
        wpose.position.y+=self.error_y*0.78*0.01
        self.waypoints.append(deepcopy(wpose))
        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
        self.arm.execute(plan)    

    """

    def execute(self):

            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

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

                        self.arm.set_max_acceleration_scaling_factor(.006)
                        self.arm.set_max_velocity_scaling_factor(.08)



                        self.arm.set_joint_value_target(self.transition_pose)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan)

                        self.arm.set_joint_value_target(self.end_joint_states)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan)

                        if -0.1+0.05*self.object_cnt<0.2:
                            self.object_cnt += 1

                        self.waypoints = []
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose)
                        transition_pose.position.x -= 0.1
                        transition_pose.position.z = -0.1 + self.object_cnt*0.05
                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)



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
                    print(fraction)



"""

move=ur5_move()

rospy.spin()
