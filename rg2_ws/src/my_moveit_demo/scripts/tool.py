#!/usr/bin/env python3
import sys
from place.place_main import place
import rospy
import moveit_commander

from setting_parameters import *
# from pick_place_main import *
from pick.pick_main import pick
# from place.place_function import 


def main():
    
    # 1) 一定要先初始化 ROS 節點
    rospy.init_node("pick_place_demo", anonymous=True)
    # 2) 再初始化 MoveIt! 的 C++ backend
    moveit_commander.roscpp_initialize(sys.argv)

    
    ( arm, gripper, scene, scene_pub,
      cube_pose, cube_size,
      table_pose, table_size,
      end_effector_link, end_effector_current_pose ) = setting_default_parameters()
    

    # pick(arm, gripper, scene, scene_pub, cube_pose, cube_size, end_effector_link ,end_effector_current_pose)

    pose_down, attach_gazebo, attach_movit = pick(arm, scene, cube_pose, end_effector_link)
   

    if attach_gazebo and attach_movit: 
        place(arm, gripper, scene, scene_pub, pose_down, table_pose, table_size, cube_size, end_effector_link)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
