#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

from setting_parameters import *
from pick_place_main import *




def main():
   
    
    ( arm, gripper, scene, scene_pub,
      cube_pose, cube_size,
      table_pose, table_size,
      end_effector_link, end_effector_current_pose ) = setting_default_parameters()

    pose_down, attach_gazebo, attach_movit = pick(arm, gripper, scene, scene_pub, cube_pose, cube_size, end_effector_link ,end_effector_current_pose)
   

    if attach_gazebo and attach_movit: 
        place(arm, gripper, scene, scene_pub, pose_down, table_pose, table_size, cube_size, end_effector_link)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
