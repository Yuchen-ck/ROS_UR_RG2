#!/usr/bin/env python3
# coding: utf-8

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from moveit_commander import PlanningSceneInterface

gripper_links = [
    "l_moment_arm_link",
    "l_finger_link",
    "l_truss_arm_link",
    "r_moment_arm_link",
    "r_finger_link",
    "r_truss_arm_link"
    ]

def wait_for_model(model_name, timeout=10.0):
    """
    等待 Gazebo 中指定模型出現
    """
    found = False
    def callback(msg):
        nonlocal found
        if model_name in msg.name:
            found = True

    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.loginfo(f"等待模型 {model_name} 出現在 Gazebo 中...")
    rate = rospy.Rate(10)
    t = 0
    while not found and t < timeout:
        rate.sleep()
        t += 0.1
    sub.unregister()
    if found:
        rospy.loginfo(f"模型 {model_name} 已出現")
    else:
        rospy.logwarn(f"模型 {model_name} 未在 {timeout} 秒內出現")
    return found

def attach_links_gazebo(object_model_name, object_link_name="link", robot_model_name="ur5"):
    """
    將物體 Attach 到 gripper 的多個 link 上，
    如果全部連結都成功，回傳 True，否則回傳 False。
    """

    rospy.wait_for_service("/link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)

    if not wait_for_model(object_model_name, timeout=10.0):
        rospy.logerr(f"模型 {object_model_name} 未出現，終止")
        return False

    if not wait_for_model(robot_model_name, timeout=10.0):
        rospy.logerr(f"機器人 {robot_model_name} 未出現，終止")
        return False

    all_success = True  # 初始設為 True，若中途任一失敗會改為 False

    for gripper_link in gripper_links:
        req = AttachRequest()
        req.model_name_1 = object_model_name
        req.link_name_1 = object_link_name
        req.model_name_2 = robot_model_name
        req.link_name_2 = gripper_link
        try:
            resp = attach_srv.call(req)
            if resp.ok:
                rospy.loginfo(f"✅ 成功連結 {object_model_name}:{object_link_name} <--> {robot_model_name}:{gripper_link}")
            else:
                rospy.logwarn(f"⚠️ 服務呼叫成功，但連結失敗：{object_model_name}:{object_link_name} <--> {robot_model_name}:{gripper_link}")
                all_success = False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ 服務呼叫失敗，無法連結 {robot_model_name}:{gripper_link}，錯誤：{e}")
            all_success = False

    return all_success


def detach_links_gazebo(object_model_name, object_link_name="link", robot_model_name="ur5"):
    """
    將物體從 gripper 的多個 link 拆除，
    若全部成功分離則回傳 True，否則回傳 False。
    """

    rospy.wait_for_service("/link_attacher_node/detach")
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    if not wait_for_model(object_model_name, timeout=10.0):
        rospy.logerr(f"模型 {object_model_name} 未出現，終止")
        return False

    if not wait_for_model(robot_model_name, timeout=10.0):
        rospy.logerr(f"機器人 {robot_model_name} 未出現，終止")
        return False

    all_success = True

    for gripper_link in gripper_links:
        req = AttachRequest()
        req.model_name_1 = object_model_name
        req.link_name_1 = object_link_name
        req.model_name_2 = robot_model_name
        req.link_name_2 = gripper_link
        try:
            resp = detach_srv.call(req)
            if resp.ok:
                rospy.loginfo(f"✅ 成功分離 {object_model_name}:{object_link_name} <--> {robot_model_name}:{gripper_link}")
            else:
                rospy.logwarn(f"⚠️ 服務回傳分離失敗：{object_model_name}:{object_link_name} <--> {robot_model_name}:{gripper_link}")
                all_success = False
        except rospy.ServiceException as e:
            rospy.logerr(f"❌ 服務呼叫失敗，無法分離 {robot_model_name}:{gripper_link}，錯誤：{e}")
            all_success = False

    return all_success

def attach_links_moveit(scene, end_effector_link, object_model_name):
    """
    將物件附著到 MoveIt 的 end_effector_link 上，並指定 gripper_links 為 touch_links。
    若附著成功，回傳 True，否則回傳 False。
    """
    rospy.loginfo(f"嘗試將 {object_model_name} 附著到 {end_effector_link}，使用 touch_links={gripper_links}")
    scene.attach_box(
        end_effector_link,
        object_model_name,
        touch_links=gripper_links
    )
    rospy.sleep(1.0)  # 給 MoveIt 一點時間同步

    attached_objects = scene.get_attached_objects()
    if object_model_name in attached_objects:
        rospy.loginfo(f"✅ 成功將 {object_model_name} 附著至 {end_effector_link}")
        return True
    else:
        rospy.logwarn(f"⚠️ 附著失敗：{object_model_name} 不在 AttachedObjects 中")
        return False

def detach_links_moveit(scene, end_effector_link, object_model_name):
    """
    將物件從 MoveIt 的 end_effector_link 上解除附著。
    若成功解除，回傳 True，否則回傳 False。
    """
    rospy.loginfo(f"嘗試從 {end_effector_link} 分離 {object_model_name} ...")
    scene.remove_attached_object(end_effector_link, name=object_model_name)
    rospy.sleep(1.0)  # 給 MoveIt 一點時間同步

    attached_objects = scene.get_attached_objects()
    if object_model_name not in attached_objects:
        rospy.loginfo(f"✅ 已從 {end_effector_link} 成功分離 {object_model_name}")
        return True
    else:
        rospy.logwarn(f"⚠️ 分離失敗：{object_model_name} 仍存在於 AttachedObjects")
        return False




# if __name__ == "__main__":
#     rospy.init_node("safe_gripper_attach_demo")
#     test_detach_gripper_links(object_model_name="cube_green")
