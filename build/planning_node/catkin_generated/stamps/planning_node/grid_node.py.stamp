#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
import numpy as np
import hashlib

class GridNodePublisher:
    def __init__(self):
        rospy.init_node('grid_node_publisher')
        self.grid_size = rospy.get_param("~grid_size", 2.0)  # in meters
        self.a_star_node_size = rospy.get_param("~a_star_node_size", 0.2) 
        self.min_points_per_grid = rospy.get_param("~min_points", 1)

        self.origin = np.array([0.0, 0.0, 0.0])
        self.received_pose = False
        
        self.prev_pc_hash = None
        self.prev_grid_msg = None
        self.prev_a_star_msg = None

        rospy.Subscriber("/state_estimation", Odometry, self.pose_callback)
        rospy.Subscriber("/traversable_area", PointCloud2, self.pc_callback)
        self.grid_pub = rospy.Publisher("/node_list", PointCloud2, queue_size=1)
        self.a_node_pub = rospy.Publisher("/a_node_list", PointCloud2, queue_size=1)

    def pose_callback(self, msg):
        if not self.received_pose:
            self.origin = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.received_pose = True
            rospy.loginfo(f"Using initial origin from /state_estimation: {self.origin}")

    def hash_pointcloud(self, msg):
        # 바이너리 buffer 기준 hash 계산
        return hashlib.md5(msg.data).hexdigest()

    def pc_callback(self, msg):
        if not self.received_pose:
            rospy.logwarn("Origin not yet received from /state_estimation.")
            return

        # 현재 PointCloud의 hash와 이전 것을 비교
        current_hash = self.hash_pointcloud(msg)
        if current_hash == self.prev_pc_hash:
            # 동일한 데이터 -> 이전 메시지 재사용
            if self.prev_grid_msg and self.prev_a_star_msg:
                self.grid_pub.publish(self.prev_grid_msg)
                self.a_node_pub.publish(self.prev_a_star_msg)
                rospy.loginfo("Re-published cached node messages (no change in input).")
            return

        # 새로운 PointCloud -> 계산 수행
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        points = np.array(points)

        if points.shape[0] == 0:
            rospy.logwarn("Received empty point cloud.")
            return

        points -= self.origin
        grid_indices = np.floor(points[:, :2] / self.grid_size).astype(int)
        a_star_node_indices = np.floor(points[:, :2] / self.a_star_node_size).astype(int)

        unique_grids = {}
        for idx, grid_idx in enumerate(grid_indices):
            key = tuple(grid_idx)
            if key not in unique_grids:
                unique_grids[key] = []
            unique_grids[key].append(points[idx])

        node_points = []
        for pts in unique_grids.values():
            if len(pts) >= self.min_points_per_grid:
                mean_xyz = np.mean(np.array(pts), axis=0)
                node_points.append(mean_xyz + self.origin)

        unique_a_star_nodes = {}
        for idx, grid_idx in enumerate(a_star_node_indices):
            key = tuple(grid_idx)
            if key not in unique_a_star_nodes:
                unique_a_star_nodes[key] = []
            unique_a_star_nodes[key].append(points[idx])

        a_star_node_points = []
        for pts in unique_a_star_nodes.values():
            if len(pts) >= self.min_points_per_grid:
                mean_xyz = np.mean(np.array(pts), axis=0)
                a_star_node_points.append(mean_xyz + self.origin)

        if len(node_points) == 0 or len(a_star_node_points) == 0:
            rospy.logwarn("No valid node points found.")
            return

        # 메시지 생성
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        grid_msg = pc2.create_cloud(header, fields, node_points)
        a_star_msg = pc2.create_cloud(header, fields, a_star_node_points)

        # publish 및 저장
        self.grid_pub.publish(grid_msg)
        self.a_node_pub.publish(a_star_msg)
        self.prev_grid_msg = grid_msg
        self.prev_a_star_msg = a_star_msg
        self.prev_pc_hash = current_hash

        rospy.loginfo(f"Published {len(node_points)} grid node(s), {len(a_star_node_points)} a-star node(s).")

if __name__ == '__main__':
    try:
        GridNodePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
