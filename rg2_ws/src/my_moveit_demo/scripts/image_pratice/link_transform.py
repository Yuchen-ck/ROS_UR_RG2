#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped


def transform_pose_to_gripper():
    rospy.init_node("link_transform_example")

    # 等待 tf buffer 準備好
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("等待 TF 資訊可用...")
    tf_buffer.can_transform("rg2_base_link", "world", rospy.Time(0), rospy.Duration(5.0))

    # 定義原始物體 Pose（在 world 座標系）
    world_pose = Pose()
    world_pose.position.x = 0.721
    world_pose.position.y = -0.127
    world_pose.position.z = 0.050
    world_pose.orientation.w = 1.0  # 單位四元數，沒有旋轉

    # 包裝成 PoseStamped（需要 header.frame_id）
    world_pose_stamped = PoseStamped()
    world_pose_stamped.header.stamp = rospy.Time(0)
    world_pose_stamped.header.frame_id = "world"
    world_pose_stamped.pose = world_pose

    try:
        # 查詢 transform：從 world 到 rg2_base_link
        transform = tf_buffer.lookup_transform(
            target_frame="rg2_base_link",
            source_frame="world",
            time=rospy.Time(0),
            timeout=rospy.Duration(3.0)
        )

        # 執行變換
        gripper_pose = tf2_geometry_msgs.do_transform_pose(world_pose_stamped, transform)

        rospy.loginfo("轉換後的 Pose（在 rg2_base_link 下）：")
        rospy.loginfo("Position: x=%.3f, y=%.3f, z=%.3f",
                      gripper_pose.pose.position.x,
                      gripper_pose.pose.position.y,
                      gripper_pose.pose.position.z)
        rospy.loginfo("Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                      gripper_pose.pose.orientation.x,
                      gripper_pose.pose.orientation.y,
                      gripper_pose.pose.orientation.z,
                      gripper_pose.pose.orientation.w)

    except Exception as e:
        rospy.logerr("轉換失敗：%s", str(e))


if __name__ == "__main__":
    try:
        transform_pose_to_gripper()
    except rospy.ROSInterruptException:
        pass
