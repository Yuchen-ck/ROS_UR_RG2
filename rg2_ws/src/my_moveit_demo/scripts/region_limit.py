#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def is_object_in_grasp_region(object_pose,
                              ideal_point=(0.138, 0.0, 0.010),
                              tolerances=(0.02, 0.06, 0.03)):
    """
    檢查物件是否在夾爪抓取區域內。
    
    參數:
      object_pose: geometry_msgs/Pose，表示物件在相同座標系（例如 hand_link）下的位姿。
      ideal_point: 理想抓取點 (x, y, z)。根據 TF 查詢，假設左右指尖中點為 (0.138, 0.0, 0.010)。
      tolerances: 每個軸允許的偏差 (dx, dy, dz)，例如：
                  - x 軸容忍 ±0.02 m
                  - y 軸容忍 ±0.06 m (由於兩指分離較大)
                  - z 軸容忍 ±0.03 m
    回傳:
      True：物件在抓取範圍內
      False：物件超出抓取範圍
    """
    dx = abs(object_pose.position.x - ideal_point[0])
    dy = abs(object_pose.position.y - ideal_point[1])
    dz = abs(object_pose.position.z - ideal_point[2])
    rospy.loginfo("物件相對於理想抓取點的偏移: dx=%.3f, dy=%.3f, dz=%.3f", dx, dy, dz)
    if dx <= tolerances[0] and dy <= tolerances[1] and dz <= tolerances[2]:
        return True
    return False

def main():
    rospy.init_node("check_grasp_region_demo", anonymous=True)
    # 假設這裡 object_pose 為你從世界資訊或檢測系統獲取的物件位姿，
    # 例如 cube_red 的位姿（根據之前範例：位置 (0.912166, -0.555631, 0.05)，朝向無旋轉）
    object_pose = Pose()
    object_pose.position.x = 0.912166
    object_pose.position.y = -0.555631
    object_pose.position.z = 0.05
    object_pose.orientation.x = 0.0
    object_pose.orientation.y = 0.0
    object_pose.orientation.z = 0.0
    object_pose.orientation.w = 1.0

    # 假設我們期望夾爪抓取點（例如以 hand_link 為參考的抓取點）定義為 (0.138, 0, 0.010)
    ideal_grasp_point = (0.138, 0.0, 0.010)
    # 定義允許的容忍誤差（根據實際測量，你可以做微調）
    tolerances = (0.02, 0.06, 0.03)

    if is_object_in_grasp_region(object_pose, ideal_point=ideal_grasp_point, tolerances=tolerances):
        rospy.loginfo("物件在夾爪抓取範圍內！")
    else:
        rospy.logwarn("物件超出夾爪抓取範圍！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
