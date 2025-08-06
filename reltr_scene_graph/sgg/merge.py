#!/usr/bin/env python
import json, glob, os, math
from pathlib import Path
from collections import Counter
import numpy as np

JSON_DIR        = "sg_json"
OUT_JSON        = "merged_sg.json"
OUT_PNG         = "merged_sg.png"
IMG_W, IMG_H    = 1920, 640
CTR_T           = 300
STATIC_LABELS   = {"room", "building"}

def ctr_dist_cyclic(b1, b2):
    c1x = (b1[0] + b1[2]) / 2.
    c2x = (b2[0] + b2[2]) / 2.
    c1y = (b1[1] + b1[3]) / 2.
    c2y = (b2[1] + b2[3]) / 2.
    dx  = abs(c1x - c2x)
    dx  = min(dx, IMG_W - dx)
    dy  = abs(c1y - c2y)
    return math.hypot(dx, dy)

global_nodes, global_edges = [], []
label_cnt                   = Counter()
id2idx                      = {}

def match_node(label, bbox):
    best_idx, best_dist = None, float('inf')
    for idx, n in enumerate(global_nodes):
        if n["label"] != label:
            continue
        dist = ctr_dist_cyclic(n["bbox"], bbox)
        if dist <= CTR_T and dist < best_dist:
            best_dist, best_idx = dist, idx
    return best_idx

def upsert_node(label, bbox):
    mi = match_node(label, bbox)
    if mi is not None:
        node = global_nodes[mi]
        if label not in STATIC_LABELS:
            node["_sum"] += np.array(bbox)
            node["_cnt"] += 1
            node["bbox"]  = (node["_sum"] / node["_cnt"]).tolist()
        return node["id"]

    label_cnt[label] += 1
    nid = f"{label}{label_cnt[label]}"
    node = {"id": nid, "label": label,
            "bbox": bbox, "_sum": np.array(bbox), "_cnt": 1}
    id2idx[nid] = len(global_nodes)
    global_nodes.append(node)
    return nid

def merge_one_triplet(trip):
    sid = upsert_node(trip["subject"], trip["subject_box"])
    oid = upsert_node(trip["object"],  trip["object_box"])
    global_edges.append({
        "subject": sid,
        "object":  oid,
        "predicate": trip["predicate"],
        "confidence": trip.get("confidence", 1.0)
    })

def merge_folder(json_dir=JSON_DIR):
    for fp in sorted(glob.glob(os.path.join(json_dir, "sg*.json"))):
        with open(fp, encoding="utf-8") as f:
            cur = json.load(f)
        for t in cur:
            merge_one_triplet(t)

    connected = {e["subject"] for e in global_edges} | \
                {e["object"]  for e in global_edges}
    pruned_nodes = [n for n in global_nodes if n["id"] in connected]
    for n in pruned_nodes:
        n.pop("_sum", None); n.pop("_cnt", None)

    merged = {"nodes": pruned_nodes, "edges": global_edges}
    Path(OUT_JSON).write_text(json.dumps(merged, indent=2))
    print(f"✅ merged graph → {OUT_JSON} | "
          f"nodes:{len(pruned_nodes)} edges:{len(global_edges)}")

    visualize(merged)

def visualize(g):
    try:
        import matplotlib.pyplot as plt
        import networkx as nx
    except ImportError:
        print("🔸 networkx / matplotlib 설치 시 PNG 시각화가 생성됩니다.")
        return

    G = nx.DiGraph()
    for n in g["nodes"]:
        G.add_node(n["id"], label=n["label"])
    for e in g["edges"]:
        G.add_edge(e["subject"], e["object"], label=e["predicate"])

    pos = nx.spring_layout(G, seed=0)
    plt.figure(figsize=(10, 7))
    nx.draw(G, pos,
            with_labels=True,
            node_size=1800,
            node_color="#fee8c8",
            edge_color="#636363",
            font_size=8,
            arrowsize=12)

    edge_labels = {(u, v): d["label"] for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos,
                                 edge_labels=edge_labels,
                                 font_size=7, font_color="blue")
    plt.axis("off"); plt.tight_layout()
    plt.savefig(OUT_PNG, dpi=300)
    print(f"preview saved → {OUT_PNG}")

if __name__ == "__main__":
    merge_folder()
