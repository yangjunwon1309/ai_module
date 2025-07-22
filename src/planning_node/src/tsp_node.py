#!/usr/bin/env python3
import rospy
import pickle
import networkx as nx
import heapq
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32MultiArray
import hashlib, struct

def prim_mst_edges(G, nodes):
    N = len(nodes)
    idx_to_node = {i: n for i, n in enumerate(nodes)}
    node_to_idx = {n: i for i, n in enumerate(nodes)}

    visited = [False] * N
    hq = []
    mst_edges = []

    visited[0] = True
    for j in range(N):
        if G.has_edge(nodes[0], nodes[j]):
            w = G[nodes[0]][nodes[j]]['weight']
            heapq.heappush(hq, (w, 0, j))

    while hq:
        w, u, v = heapq.heappop(hq)
        if visited[v]:
            continue
        visited[v] = True
        mst_edges.append((nodes[u], nodes[v]))
        for j in range(N):
            if not visited[j] and G.has_edge(nodes[v], nodes[j]):
                w2 = G[nodes[v]][nodes[j]]['weight']
                heapq.heappush(hq, (w2, v, j))

    return mst_edges


class MSTVisualizer:
    def __init__(self):
        rospy.init_node('mst_marker_visualizer')
        self.mode = None
        self.triggered = False
        self.G = None
        self.edge_routes = None
        self.largest = None
        self.mst_edges = None

        self.prev_edge_hash = None

        rospy.Subscriber("/mode", String, self.mode_callback)
        rospy.Subscriber("/edge_list", Int32MultiArray, self.list_callback)
        rospy.loginfo("Waiting for /mode == 'tsp'...")

        self.marker_pub = rospy.Publisher('/mst_edges_marker', Marker, queue_size=1)

    def hash_list(self, msg):
        # 바이너리 buffer 기준 hash 계산
        packed = struct.pack(f'{len(msg.data)}i', *msg.data)
        return hashlib.md5(packed).hexdigest()
    
    def list_callback(self, msg):
        current_hash = self.hash_list(msg)
        if current_hash == self.prev_edge_hash:
            return
        rospy.loginfo("New edge_list received. load and process agian")
        self.triggered = False
        self.prev_edge_hash = current_hash
    
    def mode_callback(self, msg):
        if msg.data == 'tsp' and not self.triggered:
            rospy.loginfo("Received mode 'tsp'. Loading data and starting visualization.")
            self.triggered = True
            self.load_and_process_data()

    def load_and_process_data(self):
        if not self.triggered :
            return
        
        filename = "edge_data.pkl"
        try:
            with open(f'data/{filename}', 'rb') as f:
                data = pickle.load(f)
            
            #self.edge_routes = data['routes']
            #edge_distances = data['distances']
            
            self.edge_routes = {
                eval(k): v for k, v in data['routes'].items()
            }
            edge_distances = {
                eval(k): v for k, v in data['distances'].items()
            }

            rospy.loginfo(f"Loaded {len(self.edge_routes)} edge routes from {filename}")

            nodes = set()
            for (i, j) in self.edge_routes:
                nodes.add(i)
                nodes.add(j)
            nodes = sorted(list(nodes))

            self.G = nx.Graph()
            for (i, j), route in self.edge_routes.items():
                if route:
                    dist = edge_distances.get((i, j), 1.0)
                    self.G.add_edge(i, j, weight=dist)

            components = list(nx.connected_components(self.G))
            if not components:
                rospy.logerr("No connected components!")
                exit()
            self.largest = sorted(list(max(components, key=len)))
            self.mst_edges = prim_mst_edges(self.G, self.largest)
            
        except Exception as e:
            rospy.logerr(f"Failed to load {filename}: {e}")
            rospy.logerr(f"[DEBUG] edge_routes keys sample: {list(self.edge_routes.keys())[:5]}")
            rospy.logerr(f"[DEBUG] key type: {type(list(self.edge_routes.keys())[0])}")
    
    def publish_mst(self):
        if self.mst_edges is None:
            return

        print(f"MST edge 개수: {len(self.mst_edges)}")

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mst_edges"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.8
        marker.color.a = 1.0

        for u, v in self.mst_edges:
            pts_uv = self.edge_routes.get((u, v), [])
            if not pts_uv and self.edge_routes.get((v, u), []):
                pts_uv = list(reversed(self.edge_routes[(v, u)]))

            if pts_uv:
                start = pts_uv[0]
                end = pts_uv[-1]
                marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
                marker.points.append(Point(x=end[0], y=end[1], z=end[2]))
            else:
                rospy.logwarn(f"No route for edge {u} <-> {v}")
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    visualizer = MSTVisualizer()
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        visualizer.publish_mst()
        rate.sleep()
