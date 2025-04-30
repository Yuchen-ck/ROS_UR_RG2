#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os, rospkg, rospy
from std_srvs.srv import Trigger, TriggerResponse

# 把 my_moveit_demo/scripts 加到 module 路径，这样才能 import gripper.py
rospack     = rospkg.RosPack()
demo_path   = rospack.get_path("my_moveit_demo")
scripts_dir = os.path.join(demo_path, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from setting_parameters import setting_default_parameters
from gripper               import open_gripper, close_gripper

# 全局拿到 gripper 对象
arm = gripper = scene = scene_pub = cube_pose = cube_size = None
table_pose = table_size = end_effector_link = end_effector_current_pose = None

def handle_open_trigger(req):
    """Trigger 服务：打开爪子"""
    ok = open_gripper(gripper)
    msg = "gripper opened" if ok else "gripper failed to open"
    return TriggerResponse(ok, msg)

def handle_close_trigger(req):
    """Trigger 服务：关闭爪子"""
    ok = close_gripper(gripper)
    msg = "gripper closed" if ok else "gripper failed to close"
    return TriggerResponse(ok, msg)

if __name__ == "__main__":
    rospy.init_node("gripper_service")

    # 先初始化所有 MoveIt!/场景/抓取参数
    arm, gripper, scene, scene_pub, \
    cube_pose, cube_size, \
    table_pose, table_size, \
    end_effector_link, end_effector_current_pose = setting_default_parameters()

    # 注册两个服务： pick/open 和 pick/close
    rospy.Service("pick/open",  Trigger, handle_open_trigger)
    rospy.Service("pick/close", Trigger, handle_close_trigger)

    rospy.loginfo("gripper_service ready: advertise pick/open and pick/close")
    rospy.spin()
