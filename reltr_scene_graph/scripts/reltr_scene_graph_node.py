#!/usr/bin/python3
import os
import json
import rospy
import rospkg
import torch
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import sys
from pathlib import Path

PKG_ROOT = Path(rospkg.RosPack().get_path('reltr_scene_graph'))
sys.path.append(str(PKG_ROOT))

from reltr_sg.wrapper import (
    build_reltr_model, preprocess_bgr, infer_reltr, decode_scene_graph
)

class RelTRSceneGraphNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic   = rospy.get_param("~image_topic", "/camera/image")
        self.use_compressed= rospy.get_param("~use_compressed", False)
        self.save_dir      = os.path.expanduser(rospy.get_param("~save_dir", "~/.ros/scene_graph_results"))
        self.weights       = rospy.get_param("~weights")
        self.device        = rospy.get_param("~device", "cuda")
        self.conf_th       = float(rospy.get_param("~confidence_thresh", 0.3))
        os.makedirs(self.save_dir, exist_ok=True)

        self.model = build_reltr_model(device=self.device, ckpt_path=self.weights)
        
        # pubs/subs
        self.pub_json = rospy.Publisher("scene_graph/json", String, queue_size=10)
        if self.use_compressed:
            rospy.Subscriber(self.image_topic, CompressedImage, self._on_image_compressed, queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(self.image_topic, Image, self._on_image_raw, queue_size=1, buff_size=2**24)
        rospy.loginfo(f"[reltr_scene_graph] Subscribed to {self.image_topic}")

    def _on_image_raw(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._process_frame(frame, msg.header.stamp.to_nsec())

    def _on_image_compressed(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self._process_frame(frame, msg.header.stamp.to_nsec())

    @torch.no_grad()
    def _process_frame(self, frame_bgr, stamp_nsec: int):
        try:
            tensor = preprocess_bgr(frame_bgr)
            outputs = infer_reltr(self.model, tensor, device=self.device)

            # (한 번만) 키 로그 찍기
            if not hasattr(self, "_printed"):
                self._printed = True
                if isinstance(outputs, dict):
                    rospy.loginfo(f"[reltr] outputs keys: {list(outputs.keys())}")

            triplets = decode_scene_graph(outputs, conf_th=self.conf_th)

            sg = {"stamp_nsec": int(stamp_nsec), "num": len(triplets), "triplets": triplets}
            out_path = os.path.join(self.save_dir, f"{stamp_nsec}.json")
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(sg, f, ensure_ascii=False)
            self.pub_json.publish(String(data=json.dumps(sg, ensure_ascii=False)))
            rospy.loginfo_throttle(2.0, f"[reltr_scene_graph] {len(triplets)} triplets -> {out_path}")

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[reltr_scene_graph] frame failed: {type(e).__name__}: {e}")
        
        if not hasattr(self, "_dbg"):
            self._dbg = True
            # shapes
            try:
                def shape(x): 
                    import torch
                    return tuple(x.shape) if isinstance(x, torch.Tensor) else None
                rospy.loginfo(f"[reltr][dbg] shapes sub_logits={shape(outputs.get('sub_logits'))} "
                            f"obj_logits={shape(outputs.get('obj_logits')) or shape(outputs.get('pred_logits'))} "
                            f"rel_logits={shape(outputs.get('rel_logits'))}")
                # top5 of rel (첫 1개만)
                import torch
                rel = outputs.get('rel_logits')
                if isinstance(rel, (list, tuple)):
                    rel = rel[0]
                if hasattr(rel, 'dim') and rel.dim() >= 2:
                    rel0 = rel[0] if rel.size(0) > 0 else rel
                    probs = torch.softmax(rel0, dim=-1).detach().cpu()
                    vals, idxs = torch.topk(probs, k=min(5, probs.shape[-1]), dim=-1)
                    rospy.loginfo(f"[reltr][dbg] rel top5 probs={vals.tolist()} idxs={idxs.tolist()}")
            except Exception as e:
                rospy.logwarn(f"[reltr][dbg] print failed: {e}")



def main():
    rospy.init_node("reltr_scene_graph_node")
    RelTRSceneGraphNode()
    rospy.spin()

if __name__ == "__main__":
    main()