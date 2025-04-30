#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
from geometry_msgs.msg import Point

class GreenObject3DDetector:
    def __init__(self):
        # 建立 CV Bridge，用於 ROS Image ↔ OpenCV 圖片互轉
        self.bridge = CvBridge()
        # 建立 publisher，發布偵測到的綠色物件的 (u, v, depth) 作為 Point 訊息
        self.pub = rospy.Publisher("green_object_detection", Point, queue_size=1)

        # 相機內參：fx, fy, cx, cy，初值設 None
        self.fx = self.fy = self.cx = self.cy = None
        # 訂閱 RGB 相機的 CameraInfo，以讀取內參
        rospy.Subscriber("/rgbd_camera/rgb/camera_info",
                         CameraInfo, self.camera_info_callback)

        # 使用 message_filters 同步訂閱 RGB 影像和 Depth 影像
        rgb_sub = message_filters.Subscriber("/rgbd_camera/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber("/rgbd_camera/depth/image_raw", Image)

        # ApproximateTimeSynchronizer：允許前後 0.1 秒內做時間對齊
        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.sync_callback)

    def camera_info_callback(self, info):
        # 只在第一次 callback 時抓取內參
        if self.fx is None:
            # K 矩陣展開：K = [fx, 0, cx; 0, fy, cy; 0,0,1]
            self.fx, self.fy = info.K[0], info.K[4]
            self.cx, self.cy = info.K[2], info.K[5]
            rospy.loginfo(
                f"相機內參: fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}")

    def sync_callback(self, rgb_msg, depth_msg):
        # 如果還沒讀到內參就先跳過
        if self.fx is None:
            return

        # 將 ROS Image 轉為 OpenCV 圖片
        cv_rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding="bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # 1) BGR → HSV，篩出綠色區域
        hsv = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2HSV)
        # 綠色範圍 [H:40~80, S:50~255, V:50~255]
        lower = np.array([40, 50, 50])
        upper = np.array([80,255,255])
        mask = cv2.inRange(hsv, lower, upper)

        # 2) 開關運算清理雜訊
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 3) 找到所有輪廓，並選出面積最大的那一個
        contours, _ = cv2.findContours(mask,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        best_rect = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100 and area > max_area:
                max_area = area
                # 最小面積矩形 (中心點, (寬,高), 旋轉角度)
                best_rect = cv2.minAreaRect(cnt)

        # 4) 如果有找到合適的矩形，計算它的中心像素 (u,v)
        if best_rect:
            (u, v), _, _ = best_rect  # u = 橫向 pixel, v = 豎向 pixel
            # 5) 從深度圖在 (v,u) 取深度值 (單位：公尺)
            depth_val = cv_depth[int(v), int(u)]
            # 濾掉 nan 或 <=0 的無效深度
            if not np.isnan(depth_val) and depth_val > 0:
                # 6) 將 (u,v,depth) 包成 Point 並發布
                msg = Point(x=u, y=v, z=float(depth_val))
                self.pub.publish(msg)
                rospy.loginfo(
                    f"Published green object @ u={u:.1f}, v={v:.1f}, depth={depth_val:.3f}m")
                # u={u:.1f}, v={v:.1f}, depth={depth_val:.3f}m
                # u = 綠色物件中心在影像中的水平方向 pixel 座標
                # v = 綠色物件中心在影像中的垂直方向 pixel 座標
                # depth = 該像素點在相機前方的深度距離 (m)

                # 7) 繪製偵測結果在 debug 影像上
                box = cv2.boxPoints(best_rect)
                box = np.int0(box)
                cv2.polylines(cv_rgb, [box], True, (255,0,0), 2)
                cv2.circle(cv_rgb, (int(u), int(v)), 4, (0,255,0), -1)
                cv2.putText(cv_rgb, f"Z={depth_val:.2f}m",
                            (int(u)+5, int(v)-5),
                            cv2.FONT_HERSHEY_PLAIN, 1.2, (0,255,0), 2)

        # 8) 顯示 debug 視窗
        cv2.imshow("Green Detection", cv_rgb)
        cv2.waitKey(1)


def main():
    rospy.init_node("green_object_detector_node")
    GreenObject3DDetector()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()