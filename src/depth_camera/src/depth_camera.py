#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2

# Camera intrinsics for 360° panorama camera (1920x640, HFOV=360°, VFOV=120°)
fx = 305.6
fy = 305.6
cx = 960.0
cy = 320.0

HFOV = 2 * np.pi
VFOV = np.deg2rad(120.0)

image_width = 1920
image_height = 640

bridge = CvBridge()

def pointcloud_callback(msg):
    try:
        # Transform from LiDAR frame (e.g., sensor_at_scan) to camera frame
        transform = tf_buffer.lookup_transform(
            "camera", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )

        # Create empty depth image
        depth_image = np.zeros((image_height, image_width), dtype=np.float32)

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pt = tf2_geometry_msgs.PointStamped()
            pt.header.frame_id = msg.header.frame_id
            pt.point.x, pt.point.y, pt.point.z = point

            # Transform to camera frame
            pt_transformed = tf_buffer.transform(pt, "camera", rospy.Duration(1.0))
            X = pt_transformed.point.x
            Y = pt_transformed.point.y
            Z = pt_transformed.point.z

            if Z <= 0.1 or Z > 200.0:  # Ignore invalid ranges
                continue
            
            r = np.sqrt(X**2 + Y**2 + Z**2)
            theta = np.arctan2(Y, X)   # [-π, π] - 수평각
            phi   = np.arcsin(Z / r)

            # Project to image plane
            u = int((theta + np.pi) / HFOV * image_width)
            v = int((np.pi/2 - phi) / VFOV * image_height)
            
            if 0 <= u < image_width and 0 <= v < image_height:
                # Use nearest Z (smallest depth)
                if depth_image[v, u] == 0 or Z < depth_image[v, u]:
                    depth_image[v, u] = Z
        
        # depth_for_dilate = np.copy(depth_image)
        # depth_for_dilate[depth_for_dilate == 0] = np.nan

        # mask = np.isnan(depth_for_dilate).astype(np.uint8)  # 빈 공간을 마스크로
        # dilated = cv2.dilate(np.nan_to_num(depth_for_dilate, nan=0), np.ones((3, 3), np.uint8), iterations=1)

        # depth_filled = np.where(mask == 1, dilated, depth_image)

        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.header.frame_id = "camera"
        depth_pub.publish(depth_msg)

    except Exception as e:
        rospy.logwarn("Transform or projection failed: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("depth_image_from_lidar")
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber("/registered_scan", PointCloud2, pointcloud_callback)
    depth_pub = rospy.Publisher("/depth_image", Image, queue_size=1)

    rospy.spin()
