#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
import numpy as np
##
class GridNodePublisher:
    def __init__(self):
        rospy.init_node('grid_node_publisher')
        self.grid_size = rospy.get_param("~grid_size", 2.0)  # in meters
        self.min_points_per_grid = rospy.get_param("~min_points", 1)

        self.origin = np.array([0.0, 0.0, 0.0])
        self.received_pose = False

        rospy.Subscriber("/state_estimation", PoseStamped, self.pose_callback)
        rospy.Subscriber("/traversable_area", PointCloud2, self.pc_callback)
        self.pub = rospy.Publisher("/node_list", PointCloud2, queue_size=1)

    def pose_callback(self, msg):
        if not self.received_pose:
            self.origin = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            self.received_pose = True
            rospy.loginfo(f"Using initial origin from /state_estimation: {self.origin}")

    def pc_callback(self, msg):
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        points = np.array(points)

        if points.shape[0] == 0:
            return

        # Offset 기준 위치
        points -= self.origin

        # 각 point의 grid 인덱스를 계산
        grid_indices = np.floor(points[:, :2] / self.grid_size).astype(int)

        # 같은 인덱스를 가지는 포인트 그룹핑
        unique_grids = {}
        for idx, grid_idx in enumerate(grid_indices):
            key = tuple(grid_idx)
            if key not in unique_grids:
                unique_grids[key] = []
            unique_grids[key].append(points[idx])

        # 각 grid 내 중심점을 구해 저장
        node_points = []
        for pts in unique_grids.values():
            if len(pts) >= self.min_points_per_grid:
                pts_arr = np.array(pts)
                mean_xyz = np.mean(pts_arr, axis=0)
                node_points.append(mean_xyz + self.origin)

        if len(node_points) == 0:
            rospy.logwarn("No valid node points found.")
            return

        # node_points를 PointCloud2로 변환하여 퍼블리시
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, node_points)
        self.pub.publish(pc2_msg)
        rospy.loginfo(f"Published {len(node_points)} grid node(s) to /node_list")

if __name__ == '__main__':
    try:
        GridNodePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
