#!/usr/bin/env python

import rospy
import moveit_commander
from threading import Thread
from gazebo_msgs.msg import ModelState
from cob_srvs.srv import *
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class Blocks:
    
    _targets={}

    _color="g"

    _blockListDict = {
        'b0': Block('/blocks_green_spawnerblock_0', 'base_link'),
        'b1': Block('/blocks_green_spawnerblock_1', 'base_link'),
        'b2': Block('/blocks_green_spawnerblock_2', 'base_link'),
        'b3': Block('/blocks_green_spawnerblock_3', 'base_link'),
        'b4': Block('/blocks_green_spawnerblock_4', 'base_link'),
        's0': Block('/spheres_green_spawnerblock_0', 'base_link'),
        's1': Block('/spheres_green_spawnerblock_1', 'base_link'),
        's2': Block('/spheres_green_spawnerblock_2', 'base_link'),
        's3': Block('/spheres_green_spawnerblock_3', 'base_link'),
        's4': Block('/spheres_green_spawnerblock_4', 'base_link'),
        'b5': Block('/blocks_white_spawnerblock_0', 'base_link'),
        'b6': Block('/blocks_white_spawnerblock_1', 'base_link'),
        'b7': Block('/blocks_white_spawnerblock_2', 'base_link'),
        'b8': Block('/blocks_white_spawnerblock_3', 'base_link'),
        'b9': Block('/blocks_white_spawnerblock_4', 'base_link'),
        's5': Block('/spheres_white_spawnerblock_0', 'base_link'),
        's6': Block('/spheres_white_spawnerblock_1', 'base_link'),
        's7': Block('/spheres_white_spawnerblock_2', 'base_link'),
        's8': Block('/spheres_white_spawnerblock_3', 'base_link'),
        's9': Block('/spheres_white_spawnerblock_4', 'base_link'),

    }

    def __init__(self):
        self.alive=True
        rospy.init_node("respawn_blocks", anonymous=False)
        rospy.on_shutdown(self.cleanup)

    def get_gazebo_models(self):
        while self.alive:
            self.all_green={}
            pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
            model_state=ModelState()
            try:
                dic=self._blockListDict
                model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self._targets.clear()
                for block in dic.itervalues():
                    blockName = str(block._name)
                    resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                    z=resp_coordinates.pose.position.z
                    if z<-0.5:
                        item_pose = Pose()
                        item_pose.position.z=0.1
                        item_pose.position.x=0.3
                        item_pose.position.y=0.0
                        item_pose.orientation.w=1.0
                        model_state.model_name=str(block._name)
                        model_state.pose=item_pose
                        model_state.reference_frame="Pelvis"   
                        pose_pub.publish(model_state)  
            except rospy.ServiceException as e:
                rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def cleanup(self):
        self.alive=False

block=Blocks()
block.get_gazebo_models()
rospy.spin()



