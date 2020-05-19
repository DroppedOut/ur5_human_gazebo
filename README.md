# ur5_human_gazebo
Gazebo simulation of human and Universal Robots ur5

Quick start

0) Open terminal and enter following commands one by one

1) sudo apt-get install ros-melodic-moveit ros-melodic-cob-srvs ros-melodic-ros-controllers

2) cd ~/catkin_ws/src

3) git clone https://github.com/DroppedOut/ur5_human_gazebo

4) cd ..

5) rosdep install --from-paths src --ignore-src --rosdistro melodic

6) mkdir devel/include/ur5_notebook

7) cp src/ur5_human_gazebo/ur5_notebook/blocks_poses.h devel/include/ur5_notebook/

8) catkin_make

9) source devel/setup.bash

10) cd src/ur5_human_gazebo/ur5_notebook chmod +x *.py

11) To launch simulation enter "roslaunch ur5_notebook simulation.launch"

12) To launch robot enter "roslaunch ur5_notebook robot.launch"

13) To launch human enter "roslaunch ur5_notebook human.launch"
