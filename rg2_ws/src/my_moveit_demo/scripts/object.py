#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from std_srvs.srv import Trigger, TriggerResponse


table_size = (0.4 ,0.4 ,0.2)
table_size_x = table_size[0]
table_size_y= table_size[1]
table_size_z = table_size[2]


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


def add_object_to_scene(scene, cube_id, pose, size_x ,size_y ,size_z):
    """
    使用 PlanningSceneInterface 將 cube 加入 MoveIt! 的規劃場景
    """
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    co = CollisionObject()
    co.id = cube_id
    co.header.frame_id = "base_link"  # 根據你的 robot frame 調整

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]
    co.primitives = [box]
    co.primitive_poses = [pose]
    co.operation = co.ADD

    scene.add_object(co)
    rospy.loginfo("已將物體 %s 加入規劃場景", cube_id)
    rospy.sleep(2)


def set_allowed_collision(scene_pub, object_name, link_names, allowed):
    """
    取得目前的 planning scene，更新允許碰撞矩陣（ACM），
    將指定的物件（object_name）與連結（link_names）設定為允許或不允許碰撞，
    並發佈更新後的 diff 到 /planning_scene 主題。

    :param scene_pub: rospy.Publisher，發佈 planning scene diff 的 Publisher。
    :param object_name: str，要更新的物件名稱（例如 "cube_red"）。
    :param link_names: list of str，要更新允許碰撞的連結名稱（例如 ["rg2_base_link"]）。
    :param allowed: bool，若為 True 則允許碰撞；若為 False 則禁止碰撞。
    """

    # 等待 /get_planning_scene 服務可用
    rospy.loginfo("等待 /get_planning_scene 服務...")
    rospy.wait_for_service('/get_planning_scene')
    try:
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        req = GetPlanningSceneRequest()
        # 請求 ACM 組件
        req.components.components = req.components.ALLOWED_COLLISION_MATRIX
        resp = get_planning_scene(req)
    except rospy.ServiceException as e:
        rospy.logerr("取得 planning scene 失敗: %s", e)
        return

    # 取得目前的允許碰撞矩陣
    current_acm = resp.scene.allowed_collision_matrix

    # 檢查目前的 entry_names 是否已有該物件
    if object_name in current_acm.entry_names:
        idx_obj = current_acm.entry_names.index(object_name)
    else:
        # 如果不存在則新增該物件名稱，並同步更新其他 entry 的長度
        current_acm.entry_names.append(object_name)
        # 為新物件建立一筆 AllowedCollisionEntry，預設值根據現有 entry_names 的數量設定 False
        new_entry = AllowedCollisionEntry()
        new_entry.enabled = [False] * len(current_acm.entry_names)
        # 同時需要在現有的其他 entry 裡面增加新欄位 (預設為 False)
        for entry in current_acm.entry_values:
            entry.enabled.append(False)
        current_acm.entry_values.append(new_entry)
        idx_obj = len(current_acm.entry_names) - 1

    # 對每個要更新的連結進行處理
    for link in link_names:
        if link in current_acm.entry_names:
            idx_link = current_acm.entry_names.index(link)
        else:
            # 如果連結名稱不存在，也加入 ACM 中
            current_acm.entry_names.append(link)
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(current_acm.entry_names)
            for entry in current_acm.entry_values:
                entry.enabled.append(False)
            current_acm.entry_values.append(new_entry)
            idx_link = len(current_acm.entry_names) - 1

        # 更新 object 與 link 互相間的 collision 設定
        current_acm.entry_values[idx_obj].enabled[idx_link] = allowed
        current_acm.entry_values[idx_link].enabled[idx_obj] = allowed

    # 建立 planning scene diff 訊息，並將修改後的 ACM 帶入
    ps_diff = PlanningScene()
    ps_diff.is_diff = True
    ps_diff.allowed_collision_matrix = current_acm

    # 發佈更新訊息
    scene_pub.publish(ps_diff)
    rospy.loginfo("已更新 ACM：物件 '%s' 與連結 %s 的 collision 設定為 %s", object_name, link_names, allowed)

def move_to_pose(arm, plan, fraction , criterion = 0.8):
    if fraction >= criterion:
        # 手動補上時間資訊
        duration_per_point = 0.05
        for i, point in enumerate(plan.joint_trajectory.points):
            point.time_from_start = rospy.Duration((i + 1) * duration_per_point)

        rospy.loginfo("Total points in trajectory: %d", len(plan.joint_trajectory.points))
        for i, point in enumerate(plan.joint_trajectory.points):
            rospy.loginfo("Point %d:", i)
            rospy.loginfo("  Time from start: %.4f s", point.time_from_start.to_sec())
            rospy.loginfo("  Positions: %s", [round(p, 4) for p in point.positions])

        rospy.loginfo("Executing Cartesian path...")
        arm.execute(plan, wait=True)
    else:
        rospy.logwarn("Cartesian path planning failed. Fraction too low.")



# if __name__ == '__main__':
#     rospy.init_node('utility_services_node')
#     # 初始化 MoveIt! planning scene publisher
#     scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
#     # 初始化 PlanningSceneInterface
#     from moveit_commander import PlanningSceneInterface
#     scene = PlanningSceneInterface()

#     # 註冊 Services
#     rospy.Service('utility/get_pose', Trigger, handle_get_pose)
#     rospy.Service('utility/add_object', Trigger, handle_add_object)
#     rospy.Service('utility/update_acm', Trigger, handle_update_acm)
#     rospy.Service('utility/execute_move', Trigger, handle_execute_move)

#     rospy.loginfo('Utility services ready: get_pose, add_object, update_acm, execute_move')
#     rospy.spin()
