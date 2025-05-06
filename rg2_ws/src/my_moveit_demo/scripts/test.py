#!/usr/bin/env python3
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize
import sys
import rospy

roscpp_initialize(sys.argv)
rospy.init_node("gripper_test", anonymous=True)

robot = RobotCommander()
gripper = MoveGroupCommander("gripper")

# 印出目前 joint 的上下限
joint = robot.get_joint("gripper_joint")
print("gripper_joint limit:", joint.bounds())

# 設定 joint 值
try:
    gripper.set_joint_value_target([0.5])  # 或用 {"gripper_joint": 0.5}
    gripper.go(wait=True)
    gripper.stop()
except Exception as e:
    print(f"❌ set_joint_value_target failed: {e}")