#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batch SGG:  data/{node_idx}/{topic}/0000.png ...  →  merged_scene_graph.json
Publishes on /scene_graph/merged_json  (latched)
"""

import os
import sys

this_dir = os.path.dirname(os.path.realpath(__file__))       # .../reltr_scene_graph/scripts
root_dir = os.path.abspath(os.path.join(this_dir, '..'))     # .../reltr_scene_graph
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)


import glob, json, time, rospy
from PIL import Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from reltr_scene_graph.sgg.RelTR_SGG import build_model, load_checkpoint, infer_one_image
from reltr_scene_graph.sgg.merge import merge_folder   # 기존 merge.py 함수


def run_one(model, path, topk, conf_th, device):
    img = Image.open(path).convert("RGB")
    return infer_one_image(model, img, topk=topk, conf_th=conf_th, device=device)

def main():
    rospy.init_node("batch_sgg_node")
    done_pub = rospy.Publisher("~done", Bool, queue_size=1, latch=True)

    root_dir   = rospy.get_param("~root_dir")                 # ai_module/data
    weights    = rospy.get_param("~weights")
    device     = rospy.get_param("~device", "cuda")
    topk       = rospy.get_param("~topk_triplets", 10)
    conf_th    = rospy.get_param("~confidence_thresh", 0.3)
    out_dir    = rospy.get_param("~save_dir",
                  os.path.expanduser("~/.ros/scene_graph_results"))
    out_dir = os.path.expanduser(out_dir)

    rospy.loginfo(f"[batch_sgg] START root_dir={root_dir}, save_dir={out_dir}, device={device}")
    
    model = build_model()
    model = load_checkpoint(model, ckpt_path=weights)

    node_dirs = [d for d in glob.glob(os.path.join(root_dir, "*"))
             if os.path.isdir(d) and os.path.basename(d).isdigit()]

    pub_dict = {}  # node_idx → publisher

    for ndir in node_dirs:
        node_idx = os.path.basename(ndir)
        tmp_json_dir = os.path.join(out_dir, f"tmp_node{node_idx}")
        os.makedirs(tmp_json_dir, exist_ok=True)

        image_dir = os.path.join(ndir, "image")

        # 1) 이미지 → triplets json 저장
        img_paths = sorted(
            glob.glob(os.path.join(image_dir, "*.jpg")) +
            glob.glob(os.path.join(image_dir, "*.png"))
        )
        for img_path in img_paths:
            trips = run_one(model, img_path, topk, conf_th, device)
            stamp = int(time.time()*1e9)
            json_path = os.path.join(tmp_json_dir, f"sg_{stamp}.json")
            with open(json_path, "w") as f:
                json.dump(trips, f)

        # 2) 폴더 내부 merge
        merged = merge_folder(tmp_json_dir)   # merge.py 함수 사용
        
        if merged is None:
            rospy.logwarn("[batch_sgg] node %s: merge empty → skip", node_idx)
            continue

        if not merged:
            rospy.logwarn("[batch_sgg] node %s merge result empty, skipping", node_idx)
            continue

        merged["node_idx"] = int(node_idx)
        merged["stamp_nsec"] = rospy.Time.now().to_nsec()

        # 3) 저장
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, f"merged_sg_{node_idx}.json"), "w") as f:
            json.dump(merged, f, indent=2)

        # 4) ROS 퍼블리시 (latch)
        topic = f"/scene_graph/node{node_idx}/json"
        if node_idx not in pub_dict:
            pub_dict[node_idx] = rospy.Publisher(topic, String,
                                                 queue_size=1, latch=True)
        pub_dict[node_idx].publish(String(json.dumps(merged)))
        rospy.loginfo("[batch_sgg] node %s → %s (%d tri)",
                      node_idx, topic, merged.get("num", 0))

    # ⭐ 옵션: all_nodes 묶어서 한 토픽에도 발행
    all_payload = {}

    for ndir in node_dirs:
        n = os.path.basename(ndir)
        fpath = os.path.join(out_dir, f"merged_sg_{n}.json")
        if os.path.exists(fpath):
            all_payload[f"node{n}"] = json.load(open(fpath))
    rospy.Publisher("/scene_graph/all_nodes/json", String,
                    queue_size=1, latch=True).publish(String(json.dumps(all_payload)))

    done_pub.publish(True)
    rospy.loginfo("[batch_sgg] FINISHED. saved to %s", out_dir)
    rospy.sleep(0.5)  # 로그/파일 flush
    rospy.signal_shutdown("batch_sgg done"); sys.exit(0)

if __name__ == "__main__":
    main()
