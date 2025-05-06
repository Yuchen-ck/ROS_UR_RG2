#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene

# 你的工具函数都在 object.py 里
from object import (
    get_current_end_effector_pose,
    add_object_to_scene,
    set_allowed_collision,
)

# joint_states_callback 在 gripper.py 里
from gripper import joint_states_callback

gripper_tip_offset = 0.12

def setting_default_parameters():
    """
    初始化 ROS、MoveIt!、TF 订阅、场景对象、规划器和抓取/放置所需的各项
    返回：
        arm: MoveGroupCommander("manipulator")
        gripper: MoveGroupCommander("gripper")
        scene: PlanningSceneInterface()
        scene_pub: Publisher(to /planning_scene)
        cube_pose: Pose  # cube_green 在 Gazebo/MoveIt! 中的目标
        cube_size: tuple(float x, float y, float z)
        table_pose: Pose
        table_size: tuple(float x, float y, float z)
        end_effector_link: str
        end_effector_current_pose: Pose
    """
    moveit_commander.roscpp_initialize(sys.argv)

    # # 2) 订阅关节状态，用于 gripper 的位置检测
    # rospy.Subscriber("/ur5/joint_states", JointState, joint_states_callback)
    # rospy.sleep(5)  # 等待 TF 和 joint_states 都到位

    # 3) 定义 cube_green 的位姿与尺寸
    cube_pose = Pose()
    cube_pose.position.x = 0.72121
    cube_pose.position.y = -0.1274
    cube_pose.position.z = 0.05
    cube_size = (0.07, 0.10, 0.05)

    # 4) 把 cube_green 加入 MoveIt! 规划场景
    scene = moveit_commander.PlanningSceneInterface()
    add_object_to_scene(
        scene, "cube_green",
        cube_pose,
        size_x=cube_size[0],
        size_y=cube_size[1],
        size_z=cube_size[2]
    )

    # 5) 允許 /planning_scene 更新中的 no-collision 设置
    scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
    set_allowed_collision(
        scene_pub,
        "cube_green",
        [
            "rg2_base_link",
            "l_finger_link", "l_moment_arm_link", "l_truss_arm_link",
            "r_finger_link", "r_moment_arm_link", "r_truss_arm_link"
        ],
        True
    )
    rospy.loginfo("已允許 'rg2_base_link' 與 'cube_green' 碰撞")

    # 6) 定义 table 的位姿与尺寸，并加入场景
    table_pose = Pose()
    table_pose.position.x = 0.128493
    table_pose.position.y = 0.8905
    table_pose.position.z = -0.025557
    table_size = (0.40, 0.40, 0.20)

    add_object_to_scene(
        scene, "yellow_table",
        table_pose,
        size_x=table_size[0],
        size_y=table_size[1],
        size_z=table_size[2]
    )

    # 7) 初始化 MoveGroupCommander
    arm = moveit_commander.MoveGroupCommander("manipulator",
                                              ns=rospy.get_namespace())
    gripper = moveit_commander.MoveGroupCommander("gripper",
                                                  ns=rospy.get_namespace())
    rospy.sleep(2)

    # 8) 规划参数
    arm.set_planning_time(30)
    arm.set_num_planning_attempts(100)
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)

    # 9) 取得当前末端状态
    end_effector_link = arm.get_end_effector_link()
    rospy.loginfo("End effector link: %s", end_effector_link)

    end_effector_current_pose = get_current_end_effector_pose(end_effector_link)
    # rospy.loginfo(
    #     "Current EE pose: x=%.3f y=%.3f z=%.3f",
    #     end_effector_current_pose.position.x,
    #     end_effector_current_pose.position.y,
    #     end_effector_current_pose.position.z
    # )
    ori = end_effector_current_pose.orientation
    # rospy.loginfo("Current EE ori: x=%.3f y=%.3f z=%.3f w=%.3f",
    #     ori.x, ori.y, ori.z, ori.w
    # )

    # 10) 返回所有用得到的句柄和參數
    return (
        arm, gripper, scene, scene_pub,
        cube_pose, cube_size,
        table_pose, table_size,
        end_effector_link, end_effector_current_pose
    )
