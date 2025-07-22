#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray, Header, String, Float32
import numpy as np
import hashlib, struct
import pickle
import os

class AStarNodePublisher:
    def __init__(self):
        rospy.init_node('a_star_node_publisher')
        
        rospy.Subscriber("/state_estimation", Odometry, self.pose_callback)
        rospy.Subscriber("/edge_list", Int32MultiArray, self.list_callback)
        rospy.Subscriber("/node_list", PointCloud2, self.node_callback)
        rospy.Subscriber("/a_node_list", PointCloud2, self.a_node_callback)
        rospy.Subscriber("/edge_key", String, self.edge_key_callback)

        self.received_pose = False
        self.origin = None

        self.new_data = True

        self.edge_list = None
        self.edge_dict = {}
        self.selected_edge_key = None
        self.nodes = None
        self.a_nodes = None
        self.prev_edge_hash = None
        self.a_nodes_penalty = None
        self.a_star_node_size = rospy.get_param("~a_star_node_size", 0.2)

        self.route_pub = rospy.Publisher("/route_for_key", PointCloud2, queue_size=1)
        self.dis_pub = rospy.Publisher("/dis_for_key", Float32, queue_size=1)
        self.mode_pub = rospy.Publisher("/mode", String, queue_size=1)
    
    def save_edge_dict(self, edge_dict, filename='edge_data.pkl'):
        edge_routes = {}
        edge_distances = {}

        for key, val in edge_dict.items():
            #key_ = tuple(key)
            if val['route']:  # route가 비어있지 않은 경우만 저장
                edge_routes[key] = val['route']  # key는 (i, j)
                edge_distances[key] = val['dis']
        
        data = {
            'routes': edge_routes,
            'distances': edge_distances
        }
        os.makedirs("data", exist_ok=True)
        with open(f'data/{filename}', 'wb') as f:
            pickle.dump(data, f)

        print(f"[INFO] Saved {len(edge_routes)} edge routes to {filename}")

    def pose_callback(self, msg):
        if not self.received_pose:
            self.origin = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.received_pose = True
            rospy.loginfo(f"Using initial origin from /state_estimation: {self.origin}")

    def edge_key_callback(self, msg):
        self.selected_edge_key = str(msg.data.strip())
        rospy.loginfo(f"Selected edge key: {self.selected_edge_key}")

        # 바로 publish 시도
        self.publish_selected_route()
    
    def hash_list(self, msg):
        # 바이너리 buffer 기준 hash 계산
        packed = struct.pack(f'{len(msg.data)}i', *msg.data)
        return hashlib.md5(packed).hexdigest()

    def publish_selected_route(self):
        if not self.selected_edge_key or self.selected_edge_key not in self.edge_dict:
            rospy.logwarn("Selected edge key not in edge_dict.")
            rospy.loginfo(list(self.edge_dict.keys()))
            return

        route = self.edge_dict[self.selected_edge_key]['route']
        dis = self.edge_dict[self.selected_edge_key]['dis']

        if not route:
            rospy.logwarn("Route is empty.")
            return

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # 또는 실제 사용 중인 frame

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, route)
        self.route_pub.publish(pc2_msg)
        self.dis_pub.publish(Float32(data=dis))
        rospy.loginfo(f"Published route and distant for edge {self.selected_edge_key}")

    def list_callback(self, msg):
        current_hash = self.hash_list(msg)
        if current_hash == self.prev_edge_hash:
            rospy.loginfo("Same edge_list received. Skipping A* computation.")
            msg = String()
            msg.data = "tsp"
            self.mode_pub.publish(msg)
            self.new_data = False
            return
        self.new_data = True
        self.prev_edge_hash = current_hash

        data = msg.data
        self.edge_list = [(data[i], data[i + 1]) for i in range(0, len(data), 2)]
        
        for edge in self.edge_list:
            dis, route = self.a_star_compute(edge)
            self.edge_dict[str(edge)] = {'dis': dis, 'route': route}
            rospy.loginfo(f"Computed edge {edge} with distance {dis} and route length {len(route)}")
        
        self.save_edge_dict(self.edge_dict)
    
    def node_callback(self, msg):
        if self.new_data :
            self.nodes = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

    def a_node_callback(self, msg):
        if self.new_data :
            self.a_nodes = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            self.a_nodes_penalty = [0] * len(self.a_nodes)
            for idx, node in enumerate(self.a_nodes):
                node_mean =  tuple(np.round(np.array(node[:2]) / self.a_star_node_size) * self.a_star_node_size)
                neighbor_list = self.get_neighbors(tuple(node))
                if len(neighbor_list) == 0:
                    edge_penalty = float('inf')
                    skew_penalty = float('inf')
                else:
                    edge_penalty = (4 / len(neighbor_list))
                    skew_penalty = 0
                    # if len(neighbor_list) == 4:
                    #     for neightbor in neighbor_list : 
                    #         skew_penalty += np.abs(np.linalg.norm(np.array(neightbor[:2]) - np.array(node[:2])))
                    
                self.a_nodes_penalty[idx] = edge_penalty + skew_penalty

    def heuristic(self, p1, p2):
        p1_idx = next((i for i, n in enumerate(self.a_nodes) if tuple(n) == p1), 0)
        return np.linalg.norm(np.array(p1) - np.array(p2)) + self.a_nodes_penalty[p1_idx]

    def a_star_compute(self, edge):
        if not self.nodes or not self.a_nodes:
            return float('inf'), []

        start_node = self.nodes[edge[0]]
        goal_node = self.nodes[edge[1]]

        a_nodes_np = np.array(self.a_nodes)
        start_idx = np.argmin(np.linalg.norm(a_nodes_np[:, :2] - np.array(start_node[:2]), axis=1))
        goal_idx = np.argmin(np.linalg.norm(a_nodes_np[:, :2] - np.array(goal_node[:2]), axis=1))

        start = tuple(a_nodes_np[start_idx])
        goal = tuple(a_nodes_np[goal_idx])

        open_set = {start}
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        visited = set()
        

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                total_distance = sum(
                    self.heuristic(path[i], path[i + 1]) for i in range(len(path) - 1)
                )
                return total_distance, path

            open_set.remove(current)
            visited.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in visited:
                    continue
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    open_set.add(neighbor)

        return float('inf'), []
    
    def get_neighbors(self, current, threshold=0.22):
        neighbors = []
        for i, n in enumerate(self.a_nodes):
            if tuple(n) == current:
                continue
            diff = np.abs(np.array(current[:2]) - np.array(n[:2]))
            if max(diff[0], diff[1]) <= threshold:
                neighbors.append(tuple(n))
        return neighbors

if __name__ == '__main__':
    try:
        AStarNodePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
