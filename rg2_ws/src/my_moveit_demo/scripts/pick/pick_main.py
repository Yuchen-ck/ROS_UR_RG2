#!/usr/bin/env python3
import rospy
import sys
import moveit_commander
from std_srvs.srv import Trigger, TriggerResponse
from copy import deepcopy

from gripper import open_gripper, close_gripper, JointState, joint_states_callback
from object import add_object_to_scene, set_allowed_collision, get_current_end_effector_pose ,move_to_pose
from attach_and_detach import attach_links_gazebo, attach_links_moveit, detach_links_gazebo, detach_links_moveit



def pick(arm, gripper, scene, scene_pub, cube_pose, cube_size, end_effector_link ,end_effector_current_pose ,gripper_tip_offset = 0.02):
    rospy.loginfo("[PICK] 嘗試打開夾爪...")
    if open_gripper(gripper):
        rospy.loginfo("RG2 Gripper 已打開")

    rospy.sleep(1)
    arm.stop()
    arm.clear_pose_targets()  # ✅ 正確的方法名稱
    arm.set_start_state_to_current_state()

    waypoints = [deepcopy(end_effector_current_pose)]

    cube_size_x, cube_size_y, cube_size_z = cube_size
    pose1 = deepcopy(end_effector_current_pose)
    pose1.position.z = cube_pose.position.z + cube_size_z + gripper_tip_offset
    pose1.position.x = cube_pose.position.x 
    pose1.position.y = cube_pose.position.y - (cube_size_y //2)
    waypoints.append(deepcopy(pose1))

    (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.02, True)
    rospy.loginfo("[PICK] Cartesian path coverage: %.2f%%", fraction * 100)
    move_to_pose(arm, plan, fraction)

    rospy.sleep(1)
    arm.stop()
    arm.clear_pose_targets()  # ✅ 修正錯誤
    arm.set_start_state_to_current_state()

    if abs(pose1.position.x - cube_pose.position.x) <= 0.01 and abs(pose1.position.y - cube_pose.position.y) <= 0.01:
        rospy.loginfo("[PICK] 夾爪已在物體正上方，下移中...")
        
        
        pose_down = deepcopy(pose1)
        gripper_pad_thickness = 0.01
        pose_down.position.z = cube_pose.position.z + (cube_size_z / 2) - gripper_pad_thickness

        waypoints = [deepcopy(pose1), deepcopy(pose_down)]
        (plan_down, fraction_down) = arm.compute_cartesian_path(waypoints, 0.02, True)
        move_to_pose(arm, plan_down, fraction_down, criterion=0.5)

        rospy.sleep(2)

        set_allowed_collision(
            scene_pub, "cube_green",
            ["rg2_base_link", "l_finger_link", "l_moment_arm_link", "l_truss_arm_link",
             "r_finger_link", "r_moment_arm_link", "r_truss_arm_link"], True)


        rospy.loginfo("Target pose: x=%.3f, y=%.3f, z=%.3f",
                        cube_pose.position.x, cube_pose.position.y, cube_pose.position.z)
        rospy.loginfo("After moveing pose: x=%.3f, y=%.3f, z=%.3f",
                        pose1.position.x, pose1.position.y, pose1.position.z)



        rospy.loginfo("[PICK] 嘗試閉合夾爪...")
        close_success = close_gripper(gripper)

        if close_success:
            rospy.loginfo("[PICK] 夾爪閉合成功，附著物體")
            attach_gazebo = attach_links_gazebo("cube_green")
            attach_movit = attach_links_moveit(scene, end_effector_link, "cube_green")
            return pose_down, attach_gazebo, attach_movit
        else:
            rospy.logwarn("[PICK] 閉合失敗")
            return None, False, False
    else:
        rospy.logwarn("[PICK] 夾爪未對準物體，上方移動失敗")
        return None, False, False
