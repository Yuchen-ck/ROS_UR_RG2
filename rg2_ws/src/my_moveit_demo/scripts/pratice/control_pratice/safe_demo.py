#!/usr/bin/env python3
# coding: utf-8

import rospy
import sys
import moveit_commander
from sensor_msgs.msg import JointState
import tf2_ros
from copy import deepcopy
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from moveit_commander import RobotTrajectory
from geometry_msgs.msg import PoseStamped

latest_joint_state = None

def joint_state_callback(msg):
    global latest_joint_state
    latest_joint_state = msg

def is_close(target_list, current_list, tol=0.05):
    return all(abs(t - c) < tol for t, c in zip(target_list, current_list))

def get_current_end_effector_pose(end_effector_link, reference_frame="base_link", timeout=rospy.Duration(1.0)):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    try:
        trans = tf_buffer.lookup_transform(reference_frame, end_effector_link, rospy.Time(0), timeout)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = reference_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation
        return pose.pose
    except Exception as e:
        rospy.logerr("TF lookup failed: %s", e)
        return None

def add_obstacle_below_end_effector(scene, robot, ee_pose):
    box_pose = PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = ee_pose.position.x
    box_pose.pose.position.y = ee_pose.position.y
    box_pose.pose.position.z = ee_pose.position.z - 0.10  # 加在下方 10cm

    scene.add_box("obstacle_below_ee", box_pose, size=(0.1, 0.1, 0.1))
    rospy.loginfo("已加入障礙物 obstacle_below_ee！")
    rospy.sleep(2.0)

def main():
    global latest_joint_state

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("set_safe_pose", anonymous=True)

    rospy.Subscriber("/ur5/joint_states", JointState, joint_state_callback)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("manipulator")
    arm.set_goal_joint_tolerance(0.01)
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    arm.set_planning_time(20.0)

    safe_joint_pose = {
        'shoulder_pan_joint': 0.0,
        'shoulder_lift_joint': -1.57,
        'elbow_joint': 1.57,
        'wrist_1_joint': -1.57,
        'wrist_2_joint': -1.57,
        'wrist_3_joint': 0.0
    }

    rospy.loginfo("Moving to safe joint pose...")
    max_attempts = 5
    attempts = 0
    success = False
    rospy.sleep(1.0)

    while attempts < max_attempts and not success:
        arm.set_joint_value_target(safe_joint_pose)
        result = arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()
        rospy.sleep(1.0)

        if latest_joint_state is None:
            rospy.logwarn("No joint state received yet!")
            attempts += 1
            continue

        compare_keys = list(safe_joint_pose.keys())
        current_values = []
        for key in compare_keys:
            if key in latest_joint_state.name:
                index = latest_joint_state.name.index(key)
                current_values.append(latest_joint_state.position[index])
            else:
                current_values.append(0.0)

        target_values = list(safe_joint_pose.values())
        if is_close(target_values, current_values, tol=0.05):
            success = True
            rospy.loginfo("Attempt %d: Joints are close enough.", attempts + 1)
        attempts += 1

    if not success:
        rospy.logerr("Failed to move to safe pose.")
        return

    # ✅ 取得末端位姿並加入障礙物
    ee_link = arm.get_end_effector_link()
    rospy.loginfo("取得末端位置中...")
    ee_pose = get_current_end_effector_pose(ee_link)
    if ee_pose:
        add_obstacle_below_end_effector(scene, robot, ee_pose)
    else:
        rospy.logwarn("無法取得末端位置，略過障礙物加入")

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == '__main__':
    main()
