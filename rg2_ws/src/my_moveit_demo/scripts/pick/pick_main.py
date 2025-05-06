#!/usr/bin/env python3
import rospy

from std_srvs.srv import Trigger
from pick_place_msgs.srv import PickStep 

from pick.pick_function import *
from attach_and_detach import *

def call_gripper_srv(service_name: str) -> bool:
    """
    共用的小工具：呼叫 pick/open 或 pick/close
    回傳 True 代表成功，False 代表失敗
    """
    rospy.wait_for_service(service_name)
    try:
        srv = rospy.ServiceProxy(service_name, Trigger)
        resp = srv()          # Trigger 無需 request 內容
        if resp.success:
            rospy.loginfo("%s → %s", service_name, resp.message)
        else:
            rospy.logerr("%s → %s", service_name, resp.message)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("%s service call failed: %s", service_name, e)
        return False

def call_pick_step_srv(step_name: str):
    """
    呼叫 pick_step service，例如 step_name = "move_above_object"
    回傳成功與否、訊息與目標姿態
    """
    rospy.wait_for_service("/pick_step")
    try:
        srv = rospy.ServiceProxy("/pick_step", PickStep)
        resp = srv(step_name)
        if resp.success:
            rospy.loginfo("Step '%s' 成功：%s", step_name, resp.message)
        else:
            rospy.logwarn("Step '%s' 失敗：%s", step_name, resp.message)
        return resp.success, resp.pose
    except rospy.ServiceException as e:
        rospy.logerr("呼叫 pick_step '%s' 失敗：%s", step_name, e)
        return False, None

def pick(arm, scene, cube_pose, end_effector_link):
    
    arm.set_start_state_to_current_state()

    # Step 1 - open gripper
    # open_gripper(gripper)
    if call_gripper_srv("gripper/open"):
        rospy.loginfo("RG2 Gripper 已打開")
    else:
        rospy.logerr("無法打開夾爪，終止 Pick 流程")
        return None, False, False

    rospy.sleep(1)
    arm.stop()
    arm.clear_pose_targets()  # ✅ 正確的方法名稱
    arm.set_start_state_to_current_state()


    # Step 2 - 移動到物體上方
    # move_above_object(arm, cube_pose, cube_size, end_effector_current_pose, gripper_tip_offset=0.2)
    success, pose_above_object = call_pick_step_srv("move_above_object")
    if not success:
        rospy.logerr("移動到物體上方失敗，終止 Pick 流程")
        return None, False, False

    rospy.sleep(1)
    arm.stop()
    arm.clear_pose_targets()  # ✅ 正確的方法名稱
    arm.set_start_state_to_current_state()


    if abs(pose_above_object.position.x - cube_pose.position.x) <= 0.01 and abs(pose_above_object.position.y - cube_pose.position.y) <= 0.01:
        rospy.loginfo("[PICK] 夾爪已在物體正上方，下移中...")
        
        # Step 3 - 夾爪往下移
        # down_to_object(arm, pose_above_object, cube_pose, cube_size[2])
        success, pose_down = call_pick_step_srv("move_lower_to_object")
        if not success:
            rospy.logerr("移動到物體上方失敗，終止 Pick 流程")
            return None, False, False
        
        rospy.sleep(1)
        arm.stop()
        arm.clear_pose_targets()  # ✅ 正確的方法名稱
        arm.set_start_state_to_current_state()

        # Step 4 - close_gripper
        # close_gripper(gripper)
        if call_gripper_srv("gripper/close"):
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
