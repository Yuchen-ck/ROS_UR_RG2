#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rospkg
import rospy
from copy import deepcopy
from geometry_msgs.msg import Pose
from pick_place_msgs.srv import PickStep, PickStepResponse

# 1) 將 my_moveit_demo/scripts 以及 pick 子目錄加到 PYTHONPATH
rospack    = rospkg.RosPack()
demo_path  = rospack.get_path("my_moveit_demo")
base_scripts = os.path.join(demo_path, "scripts")
pick_pkg_dir = os.path.join(base_scripts, "pick")

for p in (base_scripts, pick_pkg_dir):
    if p not in sys.path:
        sys.path.insert(0, p)

place_pkg_dir = os.path.join(base_scripts, "place")

for p in (base_scripts, place_pkg_dir):
    if p not in sys.path:
        sys.path.insert(0, p)

# 2) 載入你自己的功能模組
from setting_parameters import setting_default_parameters
from pick_function      import move_above_object, down_to_object
from place_function import move_above_table

# 3) 全域變數（由 setting_default_parameters() 填值）
arm                     = None
cube_pose               = None
cube_size               = None
end_effector_pose       = None
move_above_pose         = None 
table_pose              = None
table_size              = None  # 暫存第一步計算出來的 pose

# 4) pick 階段的 Service callback
def handle_pick_step(req):
    global move_above_pose
    step = req.step_name

    # Step A: 移到物體上方
    if step == "move_above_object":
        pose1 = move_above_object(
            arm,
            cube_pose,
            cube_size,
            end_effector_pose,
            gripper_tip_offset=0.2
        )
        if pose1:
            move_above_pose = deepcopy(pose1)
            return PickStepResponse(True, "moved above OK", pose1)
        else:
            return PickStepResponse(False, "move_above failed", Pose())

    # Step B: 往下移 0.5m 到抓取高度
    elif step == "move_lower_to_object":
        if move_above_pose is None:
            return PickStepResponse(False, "no above-pose to lower from", Pose())
        pose_down = down_to_object(
            arm,
            move_above_pose,
            cube_pose,
            cube_size[2]
        )
        if pose_down:
            move_above_pose = deepcopy(pose_down)
            return PickStepResponse(True, "lowered OK", pose_down)
        else:
            return PickStepResponse(False, "lower failed", Pose())

    # Unknown step
    return PickStepResponse(False, f"Unknown pick step '{step}'", Pose())


# 5) place 階段的 Service callback（範例先重用 pick 的邏輯，如要不同可自行改）
def handle_place_step(req):
    global move_above_pose
    step = req.step_name

    # Step A: 移到物體上方
    if step == "move_above_table":
        pose_above_table = move_above_table(arm, move_above_pose, table_pose, table_size, cube_size)
        if pose_above_table:
            move_above_pose = deepcopy(pose_above_table)
            return PickStepResponse(True, "moved above OK", pose_above_table)
        else:
            return PickStepResponse(False, "move_above failed", Pose())

    # # Step B: 往下移 0.5m 到放置高度
    # elif step == "move_lower_to_object":
    #     if move_above_pose is None:
    #         return PickStepResponse(False, "no above-pose to lower from", Pose())
        
    #     pose_down = down_to_object(
    #         arm,
    #         move_above_pose,
    #         cube_pose,
    #         cube_size[2]
    #     )
        
    #     if pose_down:
    #         move_above_pose = deepcopy(pose_down)
    #         return PickStepResponse(True, "lowered OK", pose_down)
    #     else:
    #         return PickStepResponse(False, "lower failed", Pose())

    return PickStepResponse(False, f"Unknown place step '{step}'", Pose())


# 6) Main：初始化、註冊兩個 Service
if __name__ == "__main__":
    rospy.init_node("pick_place_steps_server")

    # setting_default_parameters() 返回：
    # arm, gripper, scene, scene_pub,
    # cube_pose, cube_size,
    # table_pose, table_size,
    # end_effector_link, end_effector_current_pose
    # 但我們只需要其中部分，用 _ 略過不需要的
    (
        arm, _gripper, _scene, _scene_pub,
        cube_pose, cube_size,
        _table_pose, _table_size,
        _ee_link, end_effector_pose
    ) = setting_default_parameters()

    # 註冊 Service
    rospy.Service("pick_step",  PickStep, handle_pick_step)
    rospy.Service("place_step", PickStep, handle_place_step)

    rospy.loginfo("pick_place_steps_server ready")
    rospy.spin()
