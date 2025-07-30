#!/usr/bin/env python3
import rospy
import pickle
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import String, Int32MultiArray
import hashlib, struct
from collections import defaultdict
import subprocess, signal
import os

class ExplorationNode:
    def __init__(self):
        rospy.init_node('exploration_node')
        
        self.mst_edges = []
        self.nodes = []
        self.new_data = True
        self.triggered = False
        self.prev_edge_hash = None

        self.graph = defaultdict(list)
        self.traversal_order = []
        
        self.position = None
        self.node_to_travel = []

        self.edge_routes = {}
        self.edge_distances = {}

        self.cur_node_idx = None
        self.next_node_idx = None
        self.route_idx = 0

        self.dis_ths = 0.45
        
        self.bag_process = None
        self.explore_stop = False

        rospy.Subscriber("/state_estimation", Odometry, self.pose_callback)
        rospy.Subscriber("/mst_edges_marker", Marker, self.mst_callback)
        rospy.Subscriber("/node_list", PointCloud2, self.node_callback)
        rospy.Subscriber("/edge_list", Int32MultiArray, self.list_callback)
        rospy.Subscriber("/mode", String, self.mode_callback)

        self.pose_pub = rospy.Publisher('/way_point_with_heading', Pose2D, queue_size=1)
        
        rospy.Timer(rospy.Duration(0.2), self.timer_callback)

        rospy.loginfo("Exploration node initialized. Listening to /mst_edges_marker")
    
    def timer_callback(self, event):
        if not self.explore_stop:
            self.waypoint_planning()

    def node_callback(self, msg):
        if self.new_data :
            self.nodes = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    
    def node_idx(self, node):
        if len(self.nodes) < 1:
            return None
        
        node_tuple = (round(node.x, 3), round(node.y, 3), round(node.z, 3))

        for i, n in enumerate(self.nodes):
            n_tuple = (round(n[0], 3), round(n[1], 3), round(n[2], 3))
            if node_tuple == n_tuple:
                return i
        
        rospy.logwarn(f"[WARNING] Node {node_tuple} not found in self.nodes.")
        return None
    
    def pose_callback(self, msg):
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
    
    def hash_list(self, msg):
        packed = struct.pack(f'{len(msg.data)}i', *msg.data)
        return hashlib.md5(packed).hexdigest()
    
    def list_callback(self, msg):
        current_hash = self.hash_list(msg)
        if current_hash == self.prev_edge_hash:
            self.new_data = False
            return
        rospy.loginfo("New edge_list received. load and process agian")
        self.new_data = True
        self.graph = defaultdict(list)
        self.prev_edge_hash = current_hash
    
    def mode_callback(self, msg):
        if msg.data == 'tsp' and not self.triggered:
            self.triggered = True
            self.load_and_process_data()

    def load_and_process_data(self):
        if not self.triggered :
            return
        
        filename = "edge_data.pkl"
        try:
            with open(f'data/{filename}', 'rb') as f:
                data = pickle.load(f)
            
            self.edge_routes = {
                eval(k): v for k, v in data['routes'].items()
            }
            self.edge_distances = {
                eval(k): v for k, v in data['distances'].items()
            }

        except Exception as e:
            rospy.logerr(f"Failed to load {filename}: {e}")
            #rospy.logerr(f"[DEBUG] edge_routes keys sample: {list(self.edge_routes.keys())[:5]}")
            #rospy.logerr(f"[DEBUG] key type: {type(list(self.edge_routes.keys())[0])}")

    def mst_callback(self, msg):
        if msg.type != Marker.LINE_LIST:
            rospy.logwarn("Received non LINE_LIST marker")
            return
        
        if len(msg.points) % 2 != 0:
            rospy.logwarn("LINE_LIST marker does not contain an even number of points")
            return
        
        if len(self.graph) < 1 :
            for i in range(0, len(msg.points), 2):
                p1 = msg.points[i]
                p2 = msg.points[i+1]

                # MST에서 정점 중심으로 한 방향의 간선만 추가
                self.graph[self.node_idx(p1)].append(self.node_idx(p2))
                self.graph[self.node_idx(p2)].append(self.node_idx(p1))
        
        if len(self.node_to_travel) < 1:
            self.explore_with_mst()
    
    def explore_with_mst(self):
        if len(self.graph) < 1 :
            return
        
        max_degree = max(len(self.graph[n]) for n in self.graph)
        candidate_roots = [n for n in self.graph if len(self.graph[n]) == max_degree]

        def distance(n1, n2):
            return np.linalg.norm(np.array(n1[:2]) - np.array(n2[:2]))

        root = min(candidate_roots, key=lambda n: distance(self.nodes[n], self.position))

        def dfs_traversal(graph, start_node):
            visited = set()
            traversal_order = []

            def dfs(node):
                visited.add(node)
                traversal_order.append(node)
                for neighbor in graph[node]:
                    if neighbor not in visited:
                        dfs(neighbor)
                        traversal_order.append(node)    # backtracking

            dfs(start_node)
            return traversal_order
        
        self.traversal_order = dfs_traversal(self.graph, root)
        self.node_to_travel = self.traversal_order.copy()

        print(self.node_to_travel)
        # 대충 순회 순서를 구했으므로 각 순회 순서 별로 way point를 하나 씩 출력하고, state estimation과 비교해서 update 해야 함
    
    def waypoint_planning(self):
        
        def distance_okay(n1, n2):
            return (np.linalg.norm(np.array(n1[:2]) - np.array(n2[:2])) < self.dis_ths)
        
        # 새로운 데이터가 들어왔다면, 위의 코드를 처리할 때까지 대기..
        if self.new_data:
            return
        
        if len(self.node_to_travel) < 1 :
            rospy.logwarn("No node to travel. publishing stop")
            return
        
        # initial planning to root node
        if len(self.node_to_travel) == len(self.traversal_order) :
            init_node = self.nodes[self.node_to_travel[0]]
            
            if not distance_okay(self.position, init_node):
                dx = init_node[0] - self.position[0]
                dy = init_node[1] - self.position[1]

                yaw = np.arctan2(dy, dx)

                pose2d = Pose2D()
                pose2d.x = init_node[0]
                pose2d.y = init_node[1]
                pose2d.theta = yaw

                self.pose_pub.publish(pose2d)
            else:
                self.cur_node_idx = self.node_to_travel.pop(0)
                self.next_node_idx = self.node_to_travel[0]
                # rosbag record start as below code (entire route)
                self.start_recording(f'{self.cur_node_idx}.bag')
                rospy.logwarn("initial node arrived, send waypoint following MST")
        
        else:

            route_cn = self.edge_routes.get((self.cur_node_idx, self.next_node_idx), [])
            
            if not route_cn and self.edge_routes.get((self.next_node_idx, self.cur_node_idx), []):
                route_cn = list(reversed(self.edge_routes[(self.next_node_idx, self.cur_node_idx)]))
            
            if not route_cn:
                rospy.logwarn(f"No route found between {self.cur_node_idx} and {self.next_node_idx}")
                return
            
            if self.route_idx < len(route_cn):
                waypoint = route_cn[self.route_idx]

                if not distance_okay(self.position, waypoint):
                    dx = waypoint[0] - self.position[0]
                    dy = waypoint[1] - self.position[1]

                    yaw = np.arctan2(dy, dx)

                    pose2d = Pose2D()
                    pose2d.x = waypoint[0]
                    pose2d.y = waypoint[1]
                    pose2d.theta = yaw
                    
                    self.pose_pub.publish(pose2d)
                
                else:
                    self.route_idx += 1
                    if self.route_idx == 3:
                        self.stop_recording()
                    if self.route_idx == len(route_cn) - 4 :
                        self.start_recording(f'{self.next_node_idx}.bag')
                    if self.route_idx >= len(route_cn) :
                        self.route_idx = 0
                        if len(self.node_to_travel) > 1 :
                            self.cur_node_idx = self.node_to_travel.pop(0)
                            self.next_node_idx = self.node_to_travel[0]
                            rospy.logwarn(f"{self.cur_node_idx} node reached! move to {self.next_node_idx} node")
                        else :
                            self.cur_node_idx = self.node_to_travel.pop(0)
                            # rosbag record stop as below code (entire route)
                            self.stop_recording()
                            self.explore_stop = True
                            rospy.logwarn("all nodes reached! arrived at initial root node")
    
    def start_recording(self, filename='whole_0.bag'):
        
        bag_dir = os.path.join(os.path.dirname(__file__), 'data')
        os.makedirs(bag_dir, exist_ok=True)
        filepath = os.path.join(bag_dir, filename)

        topics = ['/camera/image/compressed', '/state_estimation', '/object_markers', '/registered_scan']
        command = ['rosbag', 'record', '-O', filepath] + topics
        self.bag_process = subprocess.Popen(command) #, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        rospy.loginfo("rosbag recording started.")
    
    def stop_recording(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            rospy.loginfo("rosbag recording stopped.")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ExplorationNode()
    node.run()