#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    # 初始化 ROS 節點
    rospy.init_node("moveit_demo", anonymous=True)
    rospy.loginfo("Initializing moveit_commander ...")
    moveit_commander.roscpp_initialize(sys.argv)

    # 建立 MoveGroupCommander 物件
    arm = moveit_commander.MoveGroupCommander("manipulator")
    rospy.loginfo("Ready to take commands for planning group %s", arm.get_name())

    # 設定目標位姿 (請根據你的機械臂需求來調整座標與四元數)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0

    arm.set_pose_target(target_pose)
    rospy.loginfo("Pose target set: position(%.2f, %.2f, %.2f)", 
                  target_pose.position.x, target_pose.position.y, target_pose.position.z)

    # 進行路徑規劃
    rospy.loginfo("Planning motion...")
    plan_result = arm.plan()

    # 檢查規劃結果的結構並擷取正確的 RobotTrajectory 物件
    rospy.loginfo("plan_result type: %s", type(plan_result))
    if isinstance(plan_result, tuple):
        rospy.loginfo("Plan result is a tuple with length: %d", len(plan_result))
        trajectory = None
        # 嘗試尋找包含 joint_trajectory 屬性的元素
        for i, item in enumerate(plan_result):
            rospy.loginfo("Element %d type: %s", i, type(item))
            if hasattr(item, "joint_trajectory"):
                trajectory = item
                rospy.loginfo("Trajectory found at index %d", i)
                break
        if trajectory is None:
            rospy.logerr("No valid trajectory (with joint_trajectory) found in plan result!")
            return
    else:
        trajectory = plan_result

    # 執行規劃好的路徑
    rospy.loginfo("Executing planned trajectory ...")
    arm.execute(trajectory, wait=True)
    rospy.loginfo("Movement execution finished!")

    # 清除並關閉 MoveIt! 相關資源
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
