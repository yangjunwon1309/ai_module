#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import numpy as np
import tf
from sensor_msgs import point_cloud2
import ctypes, rospkg
import os
import cv2


bridge = CvBridge()


try:
    rp = rospkg.RosPack()
    pkg_path = rp.get_path("depth_camera")
    so_path = os.path.join(pkg_path, "../..", "devel", "lib", "libdepth_projector.so")
    so_path = os.path.abspath(so_path)

    print("Loading:", so_path)
    depth_projector = ctypes.CDLL(so_path)
    depth_projector.project_lidar_to_depth.restype = None
    print("loading success")
except Exception as e:
    try:
        print(f"loding failed as {e}, retrying load the library")
        so_path = os.path.join(pkg_path, "../../../../../devel/lib/libdepth_projector.so")
        so_path = os.path.abspath(so_path)
        print("Loading:", so_path)
        depth_projector = ctypes.CDLL(so_path)
        depth_projector.project_lidar_to_depth.restype = None
        print("loading success")
    except Exception as e2 :
        print(f"loding failed as {e2}..")

# Buffers
stack_num = 400
odom_stack = np.zeros((stack_num, 7))  # x, y, z, roll, pitch, yaw, time
odom_id_pointer = -1
image_id_pointer = 0

image_init = False
image_time = 0.0
laser_cloud_time = 0.0
new_laser_cloud = False
laser_cloud = []
seg_image_cv = None
depth_pub = None
semantic_pub = None
camera_offset_z = 0.0

def odom_handler(msg):
    global odom_id_pointer
    q = msg.pose.pose.orientation
    (r, p, y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    odom_id_pointer = (odom_id_pointer + 1) % stack_num
    odom_stack[odom_id_pointer] = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        r, p, y,
        msg.header.stamp.to_sec()
    ]

def image_handler(msg):
    global seg_image_cv, image_time, image_init
    image_time = msg.header.stamp.to_sec()
    seg_image_cv = bridge.imgmsg_to_cv2(msg, "bgr8")
    image_init = True

def laser_handler(msg):
    global laser_cloud, new_laser_cloud, laser_cloud_time
    laser_cloud_time = msg.header.stamp.to_sec()
    laser_cloud = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    new_laser_cloud = True

def visualize_depth_overlay(rgb_image, depth_image):
    if np.max(depth_image) == 0:
        return rgb_image
    
    depth_threshold = np.percentile(valid_depth, 95) # 7 m
    clipped_depth = np.clip(depth_image, 0, depth_threshold)
    depth_mask = (depth_image > 0) & (depth_image <= depth_threshold)

    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=255.0 / depth_threshold), cv2.COLORMAP_JET
    )

    overlay = rgb_image.copy()
    overlay[depth_mask] = cv2.addWeighted(rgb_image[depth_mask], 0.5, depth_colormap[depth_mask], 0.5, 0)
    return overlay

def process():
    global image_id_pointer, new_laser_cloud
    if not image_init or not new_laser_cloud:
        #print("Skipping: image not ready or no new laser data")
        return
    new_laser_cloud = False

    if len(laser_cloud) == 0:
        print("Skipping: laser_cloud is empty")
        return
    if odom_id_pointer < 0:
        print("Skipping: odom not received yet")
        return

    print(f"Trying to sync image_time={image_time:.3f}, odom_stack time={odom_stack[image_id_pointer][6]:.3f}")

    min_diff = float("inf")
    best_index = -1
    for i in range(stack_num):
        diff = abs(odom_stack[i][6] - image_time)
        if diff < min_diff:
            min_diff = diff
            best_index = i

    if min_diff > 10:
        print(f"Skipping: best odom sync diff too large ({min_diff:.3f}s)")

    image_id_pointer = best_index
    #print(f"Using odom[{image_id_pointer}] with time diff: {min_diff:.3f}s")
    #print("Running depth projection")


    h, w = seg_image_cv.shape[:2]
    depth_image = np.zeros((h, w), dtype=np.float32)
    before = depth_image.copy()

    # Prepare data for C++ call
    num_points = len(laser_cloud)
    cloud_arr = np.array(laser_cloud, dtype=np.float32).flatten()
    depth_ptr = depth_image.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    cloud_ptr = cloud_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
    odom = odom_stack[image_id_pointer][:6].astype(np.float32)
    odom_ptr = odom.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

    # Call C++ function
    try:
        depth_projector.project_lidar_to_depth(cloud_ptr, num_points, w, h, odom_ptr, ctypes.c_float(camera_offset_z), depth_ptr)
    except Exception as e:
        print(f"depth projector error : {e}")
    
    # Publish
    msg = bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
    msg.header.stamp = rospy.Time.from_sec(laser_cloud_time)
    msg.header.frame_id = "camera"
    depth_pub.publish(msg)

    overlay_image = visualize_depth_overlay(seg_image_cv, depth_image)
    overlay_msg = bridge.cv2_to_imgmsg(overlay_image, encoding="bgr8")
    overlay_msg.header.stamp = rospy.Time.from_sec(laser_cloud_time)
    overlay_msg.header.frame_id = "camera"
    semantic_pub.publish(overlay_msg)

    print(f"publish success ") #{np.sum(np.abs(depth_image - before))}

def main():
    global depth_pub, semantic_pub, camera_offset_z
    rospy.init_node("depth_projector_py")
    camera_offset_z = rospy.get_param("~cameraOffsetZ", 0.0)

    rospy.Subscriber("/state_estimation", Odometry, odom_handler, queue_size=50)
    rospy.Subscriber("/camera/image", Image, image_handler, queue_size=2)
    rospy.Subscriber("/registered_scan", PointCloud2, laser_handler, queue_size=2)
    depth_pub = rospy.Publisher("/depth_image", Image, queue_size=1)
    semantic_pub = rospy.Publisher("/semantic_depth_image", Image, queue_size=1)

    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        process()
        rate.sleep()

if __name__ == '__main__':
    main()
