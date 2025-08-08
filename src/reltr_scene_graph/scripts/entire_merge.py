#!/usr/bin/env python3
import sys
#sys.path.append(os.path.dirname(os.path.abspath(__file__)), "..")

import os
import glob
import json
import math
from pathlib import Path
from collections import Counter
import numpy as np
import cv2
import tf
from sklearn.neighbors import NearestNeighbors
import rospkg

STATIC_LABELS   = {"room", "building"}

class SceneGraphMerger:
    def __init__(self, merged_dir, data_root, out_json, voxel_size=0.05, nn_radius=0.1):
        self.merged_dir = merged_dir
        self.data_root = data_root
        self.out_json = out_json
        self.voxel_size = voxel_size
        self.nn_radius = nn_radius

        self.nodes = {}
        self.edges = []
        self.idp_pair = {}
        self.dp_pair = {}

    def load_merged_graphs(self):
        graphs = []
        for fp in sorted(glob.glob(os.path.join(self.merged_dir, "merged_sg_*.json"))):
            with open(fp, 'r') as f:
                graphs.append(json.load(f))
        return graphs

    def build_global_sg(self, graphs):
        for gi, g in enumerate(graphs):
            for n in g["nodes"]:
                uid = f"{n['id']}_{gi}"
                self.nodes[uid] = {
                    "id": uid,
                    "orig_id": n["id"],
                    "label": n["label"],
                    "bbox": n["bbox"],
                    "pc": None,
                    "img_stat" : None,
                }
            for e in g["edges"]:
                sid = f"{e['subject']}_{gi}"
                oid = f"{e['object']}_{gi}"
                self.edges.append({
                    "subject": sid,
                    "object": oid,
                    "predicate": e["predicate"],
                    "confidence": e.get("confidence", 1.0)
                })

    def align_depth_pose(self, node_idx):
        img_dirs = sorted(glob.glob(os.path.join(self.data_root, str(node_idx), "image", "*.png")))
        depth_dirs = sorted(glob.glob(os.path.join(self.data_root, str(node_idx), "depth", "*.png")))
        pose_paths = sorted(glob.glob(os.path.join(self.data_root, str(node_idx), "pose", "*.json")))
        if not depth_dirs or not pose_paths:
            return None
        if len(depth_dirs) != len(pose_paths):
            print(f"[Warning] Depth/pose count mismatch for node {node_idx}")
        idp = {"image":[], "depth": [], "pose": []}
        for i_path, d_path, p_path in zip(img_dirs, depth_dirs, pose_paths):
            img = cv2.imread(i_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
            depth = cv2.imread(d_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
            with open(p_path, 'r') as f:
                pose = json.load(f)
            idp["image"].append(img)
            idp["depth"].append(depth)
            idp["pose"].append(pose)
        self.idp_pair[node_idx] = idp
        self.dp_pair[node_idx] = idp

    def extract_pointcloud_from_bbox(self, depth, pose, bbox, camera_offset_z=0.0):
        xmin, ymin, xmax, ymax = map(int, bbox)
        patch = depth[ymin:ymax, xmin:xmax]
        h, w = patch.shape
        if h == 0 or w == 0:
            return np.empty((0, 3), dtype=np.float32)

        u_coord, v_coord = np.meshgrid(np.arange(xmin, xmax), np.arange(ymin, ymax))
        u = u_coord.flatten()
        v = v_coord.flatten()
        d = patch.flatten() / 1000.0  # mm to meters

        mask = d > 0
        u, v, d = u[mask], v[mask], d[mask]
        image_width, image_height = depth.shape[1], depth.shape[0]

        theta = math.pi - 2 * math.pi * u / image_width
        phi = math.pi/2 - math.pi * v / image_height

        X = d * np.cos(phi) * np.cos(theta)
        Y = d * np.cos(phi) * np.sin(theta)
        Z = d * np.sin(phi)

        pts_cam = np.vstack([X, Y, Z, np.ones_like(X)])
        trans = tf.transformations.translation_matrix(
            [pose["position"]["x"], pose["position"]["y"], pose["position"]["z"] + camera_offset_z]
        )
        rot = tf.transformations.quaternion_matrix([
            pose["orientation"]["x"], pose["orientation"]["y"],
            pose["orientation"]["z"], pose["orientation"]["w"]
        ])
        T = trans.dot(rot)
        pts_map = T.dot(pts_cam)
        return pts_map[:3, :].T.astype(np.float32)

    def extract_pc(self, node_id):
        orig, idx = node_id.rsplit('_', 1)
        #idx = node_id[-1]
        
        idx = int(idx)
        dp = self.dp_pair.get(idx)
        if dp is None:
            self.nodes[node_id]["pc"] = np.empty((0, 3), dtype=np.float32)
            return
        pcs = []
        for depth, pose in zip(dp["depth"], dp["pose"]):
            pc = self.extract_pointcloud_from_bbox(depth, pose, self.nodes[node_id]["bbox"])
            if pc.size:
                pcs.append(pc)
        if not pcs:
            self.nodes[node_id]["pc"] = np.empty((0, 3), dtype=np.float32)
            return
        all_pc = np.vstack(pcs)
        # Outlier removal
        mu, sig = all_pc.mean(0), all_pc.std(0)
        inliers = all_pc[np.all(np.abs(all_pc - mu) <= 2 * sig, axis=1)]
        # Voxel downsample
        keys = np.floor(inliers / self.voxel_size).astype(np.int32)
        _, idxs = np.unique(keys, axis=0, return_index=True)
        self.nodes[node_id]["pc"] = inliers[idxs]

    def nnratio(self, pc1, pc2):
        if pc1.size == 0 or pc2.size == 0:
            return 0.0
        nbrs = NearestNeighbors(radius=self.nn_radius).fit(pc2)
        dists, _ = nbrs.kneighbors(pc1)
        return float(np.mean(dists[:, 0] <= self.nn_radius))
    
    def extract_statistics_patch(self, node_id,
                             hist_bins: int = 8) -> np.ndarray:
        
        orig, idx = node_id.rsplit('_', 1)
        idx = int(idx)
        dp = self.dp_pair.get(idx)

        if dp is None:
            self.nodes[node_id]["img_stat"] = np.zeros(1 + 3*hist_bins + 2, dtype=np.float32)
            return
        
        bbox = self.nodes[node_id]["bbox"]

        feat_list = []
        for img in dp["image"]:
            xmin, ymin, xmax, ymax = map(int, bbox)
            patch = img[ymin:ymax, xmin:xmax]

            if patch.size == 0:
                # [entropy, 3*hist_bins, meanV, meanS]
                return np.zeros(1 + 3*hist_bins + 2, dtype=np.float32)

            gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)

            hist_gray, _ = np.histogram(gray, bins=hist_bins, range=(0, 256))
            p = hist_gray.astype(np.float32)
            p_sum = p.sum()
            if p_sum > 0:
                p = p / p_sum
                p_nonzero = p[p > 0]
                entropy = -np.sum(p_nonzero * np.log2(p_nonzero))
            else:
                entropy = 0.0

            hist_features = []
            for c in range(3):  # B, G, R
                channel = patch[:, :, c]
                hist_c, _ = np.histogram(channel, bins=hist_bins, range=(0, 256))
                # normalize
                if hist_c.sum() > 0:
                    hist_c = hist_c.astype(np.float32) / hist_c.sum()
                hist_features.append(hist_c)
            hist_features = np.concatenate(hist_features, axis=0)  # shape (3*hist_bins,)

            hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).astype(np.float32)
            # OpenCV HSV: H[0-179], S[0-255], V[0-255]
            S = hsv[:, :, 1] / 255.0
            V = hsv[:, :, 2] / 255.0
            mean_s = float(np.mean(S))
            mean_v = float(np.mean(V))

            feat = np.hstack([
                np.array([entropy], dtype=np.float32),
                hist_features.astype(np.float32),
                np.array([mean_v, mean_s], dtype=np.float32)
            ])

            feat_list.append(feat)

        img_stat = np.mean(np.stack(feat_list, axis=0), axis=0).astype(np.float32)
        self.nodes[node_id]["img_stat"] = img_stat


    def node_sim(self, id1, id2):
        sim = 0

        pc1 = self.nodes[id1]["pc"]
        pc2 = self.nodes[id2]["pc"]
        
        sim += self.nnratio(pc1, pc2)

        # another similarity metric sholud be added
        return sim

    def update_node_features(self):
        for node_id in list(self.nodes.keys()):
            self.extract_pc(node_id)
            # should add another method to update other features

    def compute_all_sim(self):
        sim = {}
        ids = list(self.nodes.keys())
        for i, id_i in enumerate(ids):
            for id_j in ids[i+1:]:
                score = self.node_sim(id_i, id_j)
                sim[(id_i, id_j)] = score
                sim[(id_j, id_i)] = score
        return sim

    def merge_pair(self, ui, uj):
        # merge bbox
        b1, b2 = self.nodes[ui]["bbox"], self.nodes[uj]["bbox"]
        self.nodes[ui]["bbox"] = [(a + b) / 2 for a, b in zip(b1, b2)]
        # update edges
        new_edges = []
        for e in self.edges:
            s, o = e["subject"], e["object"]
            if s == uj: s = ui
            if o == uj: o = ui
            if s != o:
                new_edges.append({"subject": s, "object": o,
                                  "predicate": e["predicate"],
                                  "confidence": e["confidence"]})
        self.edges = new_edges
        # remove node
        del self.nodes[uj]

    def iterative_merge(self, threshold=0.9):
        sim = self.compute_all_sim()
        while True:
            candidates = [(pair, score) for pair, score in sim.items() if score >= threshold]
            if not candidates: break
            (ui, uj), _ = max(candidates, key=lambda x: x[1])
            self.merge_pair(ui, uj)
            self.update_node_features()
            sim = self.compute_all_sim()

    def save_graph(self):
        # re assign node idx for all node id (flower1_0 => flower1)
        label_counter = Counter(n["label"] for n in self.nodes.values())

        next_idx = {label: 0 for label in label_counter}
        old2new = {}

        new_nodes = []
        for old_id, node in self.nodes.items():
            label = node["label"]
            idx   = next_idx[label]
            new_id = f"{label}{idx}"
            next_idx[label] += 1
            old2new[old_id] = new_id

            new_nodes.append({
                "id": new_id,
                "orig_id": node["label"],
                #"bbox": node["bbox"]
            })
        
        new_edges = []
        for e in self.edges:
            s_new = old2new.get(e["subject"], e["subject"])
            o_new = old2new.get(e["object"],  e["object"])
            new_edges.append({
                "subject":   s_new,
                "object":    o_new,
                "predicate": e["predicate"],
                "confidence": e.get("confidence", 1.0)
            })

        final = {"nodes": new_nodes, "edges": new_edges}
        with open(self.out_json, 'w') as f:
            json.dump(final, f, indent=2)

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('reltr_scene_graph')

    merged_dir = os.path.join(pkg_path, "sgg", "merged_sg")
    data_root = os.path.join(pkg_path, "..", "..", "data")
    out_json = os.path.join(pkg_path, "sgg", "final_sg.json")

    # merged_dir = "sgg/merged_sg"
    # data_root = "ai_module/data"
    # out_json = "sgg/final_sg.json"

    merger = SceneGraphMerger(merged_dir, data_root, out_json)
    graphs = merger.load_merged_graphs()
    merger.build_global_sg(graphs)

    # align image & depth & pose
    for idx in os.listdir(merged_dir):
        if idx.isdigit():
            merger.align_depth_pose(idx)

    merger.update_node_features()
    # merger.iterative_merge(threshold=0.9)  # 필요 시 호출
    merger.save_graph()

    print(f"Scene graph saved to {out_json}")





# def merge_depth_for_node(node_idx, depth_root, ext=".png"):
#     depth_dir = os.path.join(depth_root, str(node_idx), "depth")
#     paths = sorted(glob.glob(os.path.join(depth_dir, f"*{ext}")))
#     if not paths:
#         return None

#     sum_d = None
#     cnt_d = None
#     for p in paths:
#         d = cv2.imread(p, cv2.IMREAD_UNCHANGED).astype(np.float32)
#         valid = (d > 0).astype(np.float32)
#         if sum_d is None:
#             sum_d = d * valid
#             cnt_d = valid
#         else:
#             sum_d += d * valid
#             cnt_d += valid

#     # 평균 depth 계산. cnt_d는 누적한 횟수, sum_d는 누적 값에 해당
#     avg = np.zeros_like(sum_d)
#     mask = cnt_d > 0
#     avg[mask] = sum_d[mask] / cnt_d[mask]
#     return avg