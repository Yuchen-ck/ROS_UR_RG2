#!/usr/bin/env python3
# coding: utf-8

import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped

def main():
    # 初始化 MoveIt 與 ROS 節點
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_ik_demo', anonymous=True)

    # 初始化 MoveGroup（UR5 的 group 名稱一般是 "manipulator"）
    arm = moveit_commander.MoveGroupCommander('manipulator')

    # 允許當規劃失敗時重新規劃
    arm.allow_replanning(True)

    # 設定容許誤差、速度與加速度比例
    arm.set_goal_position_tolerance(0.001)
    arm.set_goal_orientation_tolerance(0.001)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    # 取得末端執行器的連結名稱（通常為 ee_link）
    end_effector_link = arm.get_end_effector_link()

    # 設定參考坐標系（通常使用 base_link）
    reference_frame = 'base_link'
    arm.set_pose_reference_frame(reference_frame)

    # 設定目前狀態為起始狀態
    arm.set_start_state_to_current_state()

    # ========== 建立目標姿態 ==========
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()

    # 目標位置（XYZ）
    target_pose.pose.position.x = 0.2593
    target_pose.pose.position.y = 0.0636
    target_pose.pose.position.z = 0.1787

    # 目標姿態（Quaternion 四元數）
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(-2e-06, -2e-06, -0.000269)
    target_pose.pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # 設定手臂的目標位置
    arm.set_pose_target(target_pose, end_effector_link)

    # ========== 規劃與執行 ==========
    success, plan, planning_time, error_code = arm.plan()

    if success:
        rospy.loginfo("Path planned successfully. Executing...")
        # 根據規劃的運動路徑去控制機械臂
        arm.execute(plan, wait=True)
        rospy.sleep(1)
    else:
        rospy.logwarn("Motion planning failed.")

    # ========== 回到 up ==========
    arm.set_named_target("up")
    arm.go()
    rospy.sleep(1)

    # 關閉 MoveIt
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)



if __name__ == '__main__':
    main()

