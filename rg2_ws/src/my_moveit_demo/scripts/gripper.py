#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState

# 設定目標數值與容忍誤差
# 根據你提供的資訊：
# 當前 gripper 關閉時數值大約是 -0.29
# 當夾爪打開時應為約 0.523
TARGET_OPEN =  1.125
TARGET_CLOSE = -0.05
OPEN_GOAL_TOLERANCE = 0.1  # 如果差距小於此值，認定成功
CLOSE_GOAL_TOLERANCE = 0.5
gripper_position = None  # 用來記錄夾爪實際位置


def joint_states_callback(msg):
    global gripper_position
    if "gripper_joint" in msg.name:
        idx = msg.name.index("gripper_joint")
        gripper_position = msg.position[idx]


def open_gripper(gripper):
    gripper.set_joint_value_target({"gripper_joint": TARGET_OPEN})
    success = gripper.go(wait=True)
    gripper.stop()
    
    if success:
        rospy.loginfo("夾爪打開命令已下達，檢查是否到達目標 (%.3f)", TARGET_OPEN)
    else:
        rospy.logwarn("打開夾爪命令發送失敗")

    if check_success(TARGET_OPEN ,OPEN_GOAL_TOLERANCE):
        rospy.loginfo("夾爪成功打開至目標 (%.3f)！", TARGET_OPEN)
        return True
    else:
        rospy.logerr("夾爪未達到打開目標 (%.3f)！", TARGET_OPEN)
        return False

def close_gripper(gripper):
    gripper.set_joint_value_target({"gripper_joint": TARGET_CLOSE})
    success = gripper.go(wait=True)
    gripper.stop()

    if success:
        rospy.loginfo("夾爪閉合命令已下達，檢查是否到達目標 (%.3f)", TARGET_CLOSE)
    else:
        rospy.logwarn("閉合夾爪命令發送失敗")

    if check_success(TARGET_CLOSE ,CLOSE_GOAL_TOLERANCE ):
        rospy.loginfo("夾爪成功閉合至目標 (%.3f)！", TARGET_CLOSE)
        return True
    else:
        rospy.logerr("夾爪未達到閉合目標 (%.3f)！", TARGET_CLOSE)
        return False

def check_success(target_value, tolerance, timeout=10):
    rate = rospy.Rate(5)  # 10Hz
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < timeout:
        if gripper_position is not None:
            diff = abs(gripper_position - target_value)
            rospy.loginfo("檢查中: gripper_joint = %.3f, 目標 = %.3f, 差距 = %.3f", gripper_position, target_value, diff)
            if diff < tolerance:
                return True
        rate.sleep()
    return False



def main():
    rospy.init_node("check_gripper_joint_limits", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    gripper = moveit_commander.MoveGroupCommander("gripper", ns=rospy.get_namespace())
    close_gripper(gripper)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    # 1) 啟動 ROS node
    rospy.init_node('print_planning_frame', anonymous=True)
    # 2) 啟動 MoveIt! C++ 介面
    moveit_commander.roscpp_initialize(sys.argv)

    # 3) 建立 MoveGroupCommander
    arm = moveit_commander.MoveGroupCommander("manipulator",
                                              ns=rospy.get_namespace())
    # 4) 讀出並印出 planning frame
    planning_frame = arm.get_planning_frame()
    rospy.loginfo(">>> MoveIt planning frame is: %s", planning_frame)

    # 5) 清理
    moveit_commander.roscpp_shutdown()
