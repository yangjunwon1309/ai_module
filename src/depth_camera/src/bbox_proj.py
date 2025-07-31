import rospy
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import tf
import math

class BoundingBox3DExtractor:
    def __init__(self):
        rospy.init_node("bbox_3d_extractor")
        
        self.stack_num = 100
        self.odom_stack = np.zeros((self.stack_num, 8))  # x, y, z, x, y, z, w, time
        self.odom_id_pointer = -1
        self.image_id_pointer = 0

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_pose = None
        self.image_time = 0

        self.camera_offset_z = 0

        self.bbox_sub = rospy.Subscriber("/bbox", Int32MultiArray, self.bbox_callback)
        self.depth_sub = rospy.Subscriber("/depth_image", Image, self.depth_callback)
        self.pose_sub = rospy.Subscriber("/state_estimation", Odometry, self.odom_callback)
        self.pub = rospy.Publisher("/bbox_points", PointCloud2, queue_size=1)
    
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.image_time = msg.header.stamp.to_sec()
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position

        self.odom_id_pointer = (self.odom_id_pointer + 1) % self.stack_num
        self.odom_stack[self.odom_id_pointer] = [
            pos.x,
            pos.y,
            pos.z,
            q.x, q.y, q.z, q.w,
            msg.header.stamp.to_sec()
        ]

    def bbox_callback(self, msg):
        if self.latest_depth is None :
            rospy.loginfo("Waiting for depth and pose...")
            return
        
        # 1. Extract bounding box pixel range (assumed Int32MultiArray format: [x_min, y_min, x_max, y_max] )
        if len(msg.data) != 4:
            rospy.logwarn("Expected 4-element bbox")
            return
        x_min, y_min, x_max, y_max = msg.data

        cropped_depth = self.latest_depth[y_min:y_max, x_min:x_max].copy()
        h, w = cropped_depth.shape
        if h == 0 or w == 0:
            rospy.logwarn("Empty bounding box area")
            return
        
        u_coord, v_coord = np.meshgrid(np.arange(x_min, x_max), np.arange(y_min, y_max))
        u = u_coord.flatten()
        v = v_coord.flatten()
        d = cropped_depth.flatten()
        
        mask = d > 0
        u = u[mask]
        v = v[mask]
        d = d[mask]

        image_width = self.latest_depth.shape[1]
        image_height = self.latest_depth.shape[0]

        theta = math.pi - 2 * math.pi * u / image_width
        phi = math.pi/2 - 2 * math.pi * v / image_height

        X = d * np.cos(phi) * np.cos(theta)
        Y = d * np.cos(phi) * np.sin(theta)
        Z = d * np.sin(phi)

        points_cam = np.vstack([X, Y, Z, np.ones_like(X)])  # 4xN

        # sensor frame to map frame (cam offset = 0)
        min_diff = float("inf")
        image_id_pointer = -1
        for i in range(self.stack_num):
            if self.odom_stack[i][7] == 0:
                continue
            diff = abs(self.odom_stack[i][7] - self.image_time)
            if diff < min_diff:
                min_diff = diff
                image_id_pointer = i

        pos_ori = self.odom_stack[image_id_pointer]
        trans = tf.transformations.translation_matrix([pos_ori[0], pos_ori[1], pos_ori[2] + self.camera_offset_z])
        rot = tf.transformations.quaternion_matrix([pos_ori[3], pos_ori[4], pos_ori[5], pos_ori[6]])
        T = np.dot(trans, rot)  

        points_map = T @ points_cam

        header = Header()
        header.stamp = rospy.Time.from_sec(self.image_time)
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        cloud_data = list(zip(points_map[0], points_map[1], points_map[2]))
        pc2_msg = pc2.create_cloud(header, fields, cloud_data)
        self.pub.publish(pc2_msg)

if __name__ == '__main__':
    try:
        node = BoundingBox3DExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
