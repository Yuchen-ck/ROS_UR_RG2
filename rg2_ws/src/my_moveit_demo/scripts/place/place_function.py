#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from std_srvs.srv import Trigger, TriggerResponse
from copy import deepcopy

from gripper import open_gripper, close_gripper, JointState, joint_states_callback
from object import add_object_to_scene, set_allowed_collision, get_current_end_effector_pose ,move_to_pose
from attach_and_detach import attach_links_gazebo, attach_links_moveit, detach_links_gazebo, detach_links_moveit


def place(arm, gripper, scene, scene_pub, pose_start, table_pose, table_size, cube_size, end_effector_link):
    rospy.loginfo("\n[PLACE] 開始 place 流程...")
    cube_size_x, cube_size_y, cube_size_z = cube_size
    table_size_x, table_size_y, table_size_z = table_size

    pose_table = deepcopy(pose_start)
    pose_table.position.z = table_size_z + cube_size_z + 0.5
    pose_table.position.x = table_pose.position.x
    pose_table.position.y = table_pose.position.y + 0.2

    plan_lift, fraction_lift = arm.compute_cartesian_path([pose_start, pose_table], 0.03, True)
    lift_success = False

    if fraction_lift >= 0.5:
        for i, point in enumerate(plan_lift.joint_trajectory.points):
            point.time_from_start = rospy.Duration((i + 1) * 0.05)
        arm.execute(plan_lift, wait=True)
        lift_success = True
    else:
        arm.set_start_state_to_current_state()
        arm.set_pose_target(pose_table)
        lift_success = arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

    if lift_success:
        x_diff = abs(pose_table.position.x - table_pose.position.x)
        y_diff = abs(pose_table.position.y - (table_pose.position.y + 0.2))
        
        # 物件在桌面上方
        if x_diff < 0.01 and y_diff < 0.05:
            
            rospy.loginfo("✅ 手臂往下移動，再放爪")
            pose_down_again = deepcopy(pose_table)
            pose_down_again.position.z = table_pose.position.z + cube_size_z / 2 + 0.01

            waypoints = [deepcopy(pose_table), deepcopy(pose_down_again)]

            (plan_down_again, fraction_down_again) = arm.compute_cartesian_path( waypoints, 0.02, True)

            if fraction_down_again >= 0.5:
                for i, point in enumerate(plan_down_again.joint_trajectory.points):
                    point.time_from_start = rospy.Duration((i + 1) * 0.05)
                arm.execute(plan_down_again, wait=True)
                lift_success = True
            else:
                arm.set_start_state_to_current_state()
                arm.set_pose_target(pose_down_again)
                lift_success = arm.go(wait=True)
                arm.stop()
                arm.clear_pose_targets()

            open_gripper(gripper)
            
            rospy.sleep(1.0)

            detach_gazebo = detach_links_gazebo("cube_green")
            detach_movit = detach_links_moveit(scene, end_effector_link, "cube_green")
            
            if detach_gazebo and detach_movit:
                scene.remove_attached_object(end_effector_link, name="cube_green")
                set_allowed_collision(scene_pub, "cube_green", [
                    "rg2_base_link", "l_finger_link", "l_moment_arm_link", "l_truss_arm_link",
                    "r_finger_link", "r_moment_arm_link", "r_truss_arm_link"], False)
                rospy.loginfo("✅ 物體成功釋放")
            else:
                rospy.logwarn("⚠️ Detach 失敗")
        else:
            rospy.loginfo("❌ 尚未靠近桌面，取消釋放")
    else:
        rospy.logwarn("❌ 抬高手臂失敗")
