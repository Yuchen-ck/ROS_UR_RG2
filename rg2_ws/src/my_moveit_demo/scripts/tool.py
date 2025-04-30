#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene

from attach_and_detach import *

from pick_place_main import * 

gripper_tip_offset = 0.12


def main():
    rospy.init_node("pick_place_demo", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.Subscriber("/ur5/joint_states", JointState, joint_states_callback)
    rospy.sleep(5)  # 等待 TF 與 joint_states 更新

    # 1. 定義 cube_green 的位姿（請依據你的 world 設定調整）
    cube_pose = Pose()
    location = (0.72121 ,-0.1274 ,0.05)
    cube_pose.position.x = location[0]
    cube_pose.position.y = location[1]
    cube_pose.position.z = location[2]

    cube_size = (0.07, 0.1 ,0.05)
    cube_size_x = cube_size[0]
    cube_size_y = cube_size[1]
    cube_size_z = cube_size[2]
    
    # 2. 將 cube_green 加入 MoveIt! 規劃場景
    scene = moveit_commander.PlanningSceneInterface()
    add_object_to_scene(scene, "cube_green", cube_pose, size_x= cube_size_x, size_y = cube_size_y, size_z = cube_size_z)

    ### no collision ###
    scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
    set_allowed_collision(
        scene_pub,
        "cube_green",
        [   
            "rg2_base_link", # 一定要有這個，不然不能夾到物件
            "l_finger_link",
            "l_moment_arm_link",
            "l_truss_arm_link",
            "r_finger_link",
            "r_moment_arm_link",
            "r_truss_arm_link"
        ],
        True
    )
    rospy.loginfo("已設定允許 'rg2_base_link' 與 'cube_green' 發生碰撞")


    table_pose = Pose()
    table_location = (0.128493 ,0.8905 ,-0.025557)
    table_pose.position.x = table_location[0]
    table_pose.position.y = table_location[1]
    table_pose.position.z = table_location[2]

    table_size = (0.4 ,0.4 ,0.2)
    table_size_x = table_size[0]
    table_size_y= table_size[1]
    table_size_z = table_size[2]

    scene = moveit_commander.PlanningSceneInterface()
    add_object_to_scene(scene, "yellow_table", table_pose, size_x = table_size_x, size_y = table_size_y, size_z = table_size_z)


    # 3. 初始化 MoveIt! 控制介面
    arm = moveit_commander.MoveGroupCommander("manipulator", ns=rospy.get_namespace())
    gripper = moveit_commander.MoveGroupCommander("gripper", ns=rospy.get_namespace())
    rospy.sleep(2)
    
    # 設定規劃參數
    arm.set_planning_time(30)
    arm.set_num_planning_attempts(100)
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    

    # 先取得目前所有關節值
    end_effector_link = arm.get_end_effector_link()
    rospy.loginfo("End effector link: %s", end_effector_link)
    rospy.loginfo("Using tf2 to get current end-effector pose...")

    end_effector_current_pose = get_current_end_effector_pose(end_effector_link)
    
    rospy.loginfo("Current pose: x=%.3f, y=%.3f, z=%.3f",
                  end_effector_current_pose.position.x, end_effector_current_pose.position.y, end_effector_current_pose.position.z)
    rospy.loginfo("Current orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              end_effector_current_pose.orientation.x,
              end_effector_current_pose.orientation.y,
              end_effector_current_pose.orientation.z,
              end_effector_current_pose.orientation.w)
    
    service = rospy.Service('pick_object', Trigger, handle_pick_service)
    

    # pose_down, attach_gazebo, attach_movit = pick(arm, gripper, scene, scene_pub, cube_pose, cube_size, end_effector_link ,end_effector_current_pose)
   

    # if attach_gazebo and attach_movit: 
    #     place(arm, gripper, scene, scene_pub, pose_down, table_pose, table_size, cube_size, end_effector_link)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
