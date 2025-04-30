#!/usr/bin/env python3
# coding: utf-8

import rospy
import moveit_commander
import sys

def main():
    # 初始化 MoveIt Commander 與 ROS 節點
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_fk_demo', anonymous=True)

    # 初始化機器人控制器
    arm = moveit_commander.MoveGroupCommander('manipulator')  # UR5 的 Move Group 名稱為 "manipulator"

    # 設定機械手臂的容許誤差
    arm.set_goal_joint_tolerance(0.001) #弧度

    # 設定最大加速度與速度比例（0~1 之間）
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    # ========= 第一段：讓手臂回到初始位置（up） =========
    rospy.loginfo("Moving to up position...")
    arm.set_named_target('up')  # SRDF 裡預設的 named target
    arm.go() 
    rospy.sleep(1)

    # ========= 第二段：設定正向運動目標（關節角度） =========
    rospy.loginfo("Moving to specified joint position...")
    joint_positions = [
        0.391410,   # shoulder_pan_joint
       -0.676384,   # shoulder_lift_joint
       -0.376217,   # elbow_joint
        0.0,        # wrist_1_joint
        1.052834,   # wrist_2_joint
        0.454125    # wrist_3_joint
    ]

    arm.set_joint_value_target(joint_positions)
    arm.go()
    rospy.sleep(1)

    # ========= 第三段：回到 up =========
    rospy.loginfo("Returning to up position...")
    arm.set_named_target('up')
    arm.go()
    rospy.sleep(1)

    # 關閉 MoveIt Commander
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    main()
