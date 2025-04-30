#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs    # ← 一定要加！
from geometry_msgs.msg import PointStamped

class GreenBoxCam:
    def __init__(self):
        rospy.init_node('green_box_cam', anonymous=True)
        self.bridge    = CvBridge()
        self.K         = None
        self.cam_frame = None
        self.rgb       = None
        self.depth     = None

        # TF2 buffer & listener
        self.tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buf)

        # Subscribe to depth‐camera_info (use the depth camera's frame/origin!)
        rospy.Subscriber('/rgbd_camera/depth/camera_info', CameraInfo,
                         self.cb_info, queue_size=1)
        # RGB for color segmentation
        rospy.Subscriber('/rgbd_camera/rgb/image_raw', Image,
                         self.cb_rgb, queue_size=1)
        # Depth for distance
        rospy.Subscriber('/rgbd_camera/depth/image_raw', Image,
                         self.cb_depth, queue_size=1)

    def cb_info(self, msg, *args):
        if self.K is None:
            # use depth camera intrinsics & frame
            self.K = np.array(msg.K).reshape(3,3)
            self.cam_frame = msg.header.frame_id
            rospy.loginfo("Got depth camera intrinsics, frame: %s",
                          self.cam_frame)

    def cb_rgb(self, msg, *args):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_calc()

    def cb_depth(self, msg, *args):
        # assume 32FC1
        self.depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        self.try_calc()

    def try_calc(self):
        if self.K is None or self.rgb is None or self.depth is None:
            return

        # 1) HSV mask to find green contour
        hsv = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (40,50,50), (80,255,255))
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return
        M = cv2.moments(max(cnts, key=cv2.contourArea))
        if M['m00']==0:
            return
        u = int(M['m10']/M['m00'])
        v = int(M['m01']/M['m00'])

        # 2) read depth
        d = float(self.depth[v,u])
        if np.isnan(d) or d<=0:
            return

        # 3) project to camera frame
        fx,fy = self.K[0,0], self.K[1,1]
        cx,cy = self.K[0,2], self.K[1,2]
        Xc = (u - cx) * d / fx
        Yc = (v - cy) * d / fy
        Zc = d

        # 4) stamp as PointStamped in cam_frame
        p_cam = PointStamped()
        p_cam.header.frame_id = self.cam_frame
        p_cam.header.stamp = rospy.Time(0)
        p_cam.point.x, p_cam.point.y, p_cam.point.z = Xc, Yc, Zc

        # 5) transform to 'world'
        try:
            p_world = self.tf_buf.transform(p_cam, 'world',
                                            rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn("TF transform to world failed: %s", e)
            return

        # 6) output – this IS the box's true world coords
        rospy.loginfo("Box true @ world: x=%.3f y=%.3f z=%.3f",
                      p_world.point.x,
                      p_world.point.y,
                      p_world.point.z)

        rospy.signal_shutdown("done")

if __name__=='__main__':
    GreenBoxCam()
    rospy.spin()