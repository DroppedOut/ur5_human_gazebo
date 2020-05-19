#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock
from time import sleep
from gazebo_msgs.srv import GetModelState

def time_count(data): 
    global secs,nsecs
    secs=data.clock.secs
    nsecs=data.clock.nsecs  

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
        'b0': Block('/blocks_white_spawnerblock_0', 'base_link'),
        'b1': Block('/blocks_white_spawnerblock_1', 'base_link'),
        'b2': Block('/blocks_white_spawnerblock_2', 'base_link'),
        'b3': Block('/blocks_white_spawnerblock_3', 'base_link'),
        'b4': Block('/blocks_white_spawnerblock_4', 'base_link'),
        's0': Block('/spheres_white_spawnerblock_0', 'base_link'),
        's1': Block('/spheres_white_spawnerblock_1', 'base_link'),
        's2': Block('/spheres_white_spawnerblock_2', 'base_link'),
        's3': Block('/spheres_white_spawnerblock_3', 'base_link'),
        's4': Block('/spheres_white_spawnerblock_4', 'base_link'),

    }
    def get_gazebo_models(self):
        alive=True
        while alive:
            try:
                dic=self._blockListDict
                model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                self.count=0
                for block in dic.itervalues():
                    blockName = str(block._name)
                    resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                    z=resp_coordinates.pose.position.z
                    if z>=0:
                        self.count+=1
                if self.count == 0:
                    global start_secs,start_nsecs
                    print("------------------------ALL BLOCKS SORTED------------------------")
                    print("Seconds: "+str(((secs*1000+nsecs/1000000)-(start_secs*1000+start_nsecs/1000000))/1000)+" MSeconds: "+str(((secs*1000+nsecs/1000000)-(start_secs*1000+start_nsecs/1000000))%1000))
                    print("-----------------------------------------------------------------")
                    alive=False
            except rospy.ServiceException as e:
                rospy.loginfo("Get Model State service call failed:  {0}".format(e))

rospy.init_node("timer", anonymous=False)
rospy.Subscriber("clock", Clock, time_count)

sleep(15)
print("start_timer")

global start_secs,start_nsecs,secs,nsecs
start_secs=secs
start_nsecs=nsecs

blocks=Blocks()
blocks.get_gazebo_models()
rospy.spin()
