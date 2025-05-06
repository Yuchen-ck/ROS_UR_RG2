# #!/usr/bin/env python3
import rospy
# import sys
# import moveit_commander
from std_srvs.srv import Trigger
from pick_place_msgs.srv import PickStep 

# from gripper import open_gripper, close_gripper, JointState, joint_states_callback
# from object import add_object_to_scene, set_allowed_collision, get_current_end_effector_pose ,move_to_pose
# from attach_and_detach import attach_links_gazebo, attach_links_moveit, detach_links_gazebo, detach_links_moveit

# from place.place_function import move_above_table

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

def call_place_step_srv(step_name: str):
    """
    呼叫 pick_step service，例如 step_name = "move_above_object"
    回傳成功與否、訊息與目標姿態
    """
    rospy.wait_for_service("/place_step")
    try:
        srv = rospy.ServiceProxy("/place_step", PickStep)
        resp = srv(step_name)
        if resp.success:
            rospy.loginfo("Step '%s' 成功：%s", step_name, resp.message)
        else:
            rospy.logwarn("Step '%s' 失敗：%s", step_name, resp.message)
        return resp.success, resp.pose
    except rospy.ServiceException as e:
        rospy.logerr("呼叫 pick_step '%s' 失敗：%s", step_name, e)
        return False, None
    
def place(arm, gripper, scene, scene_pub, pose_start, table_pose, table_size, cube_size, end_effector_link):
    rospy.loginfo("\n[PLACE] 開始 place 流程...")
    arm.set_start_state_to_current_state()

    #[PLACE] Step 1 : move above table 
    # move_above_table(arm, pose_start, table_pose, table_size, cube_size)
    success, pose_above_object = call_place_step_srv("move_above_table")
    if not success:
        rospy.logerr("移動到物體上方失敗，終止 Pick 流程")

    rospy.sleep(1)
    arm.stop()
    arm.clear_pose_targets()  # ✅ 正確的方法名稱
    arm.set_start_state_to_current_state()

    if abs(pose_above_object.position.x - table_pose.position.x)< 0.01 and abs(pose_above_object.position.y - (table_pose.position.y + 0.2)) < 0.05:
        rospy.loginfo("✅ 手臂往下移動，再放爪")

        if call_gripper_srv("gripper/close"):
            rospy.loginfo("[PICK] 夾爪閉合成功，附著物體")
        else:
            rospy.logwarn("[PICK] 閉合失敗")
            
    #     pose_down_again = deepcopy(pose_table)
    #     pose_down_again.position.z = table_pose.position.z + cube_size_z / 2 + 0.01

    #     waypoints = [deepcopy(pose_table), deepcopy(pose_down_again)]

    #     (plan_down_again, fraction_down_again) = arm.compute_cartesian_path( waypoints, 0.02, True)

    #     if fraction_down_again >= 0.5:
    #         for i, point in enumerate(plan_down_again.joint_trajectory.points):
    #             point.time_from_start = rospy.Duration((i + 1) * 0.05)
    #         arm.execute(plan_down_again, wait=True)
    #         lift_success = True
    #     else:
    #         arm.set_start_state_to_current_state()
    #         arm.set_pose_target(pose_down_again)
    #         lift_success = arm.go(wait=True)
    #         arm.stop()
    #         arm.clear_pose_targets()

    #     open_gripper(gripper)
        
    #     rospy.sleep(1.0)

    #     detach_gazebo = detach_links_gazebo("cube_green")
    #     detach_movit = detach_links_moveit(scene, end_effector_link, "cube_green")
        
    #     if detach_gazebo and detach_movit:
    #         scene.remove_attached_object(end_effector_link, name="cube_green")
    #         set_allowed_collision(scene_pub, "cube_green", [
    #             "rg2_base_link", "l_finger_link", "l_moment_arm_link", "l_truss_arm_link",
    #             "r_finger_link", "r_moment_arm_link", "r_truss_arm_link"], False)
    #         rospy.loginfo("✅ 物體成功釋放")
    #     else:
    #         rospy.logwarn("⚠️ Detach 失敗")
    # else:
    #     rospy.logwarn("❌ 抬高手臂失敗")
