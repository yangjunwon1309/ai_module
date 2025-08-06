#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, sys
import rospy, rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import torch
from pathlib import Path
from PIL import Image as PILImage
import numpy as np

PKG_ROOT = Path(rospkg.RosPack().get_path('reltr_scene_graph'))
# RelTR 루트를 파이썬 경로에 등록 (RelTR_SGG가 내부에서 RelTR.models.* import 함)
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))
# sgg 모듈 추가
sys.path.insert(0, str(PKG_ROOT / 'sgg'))

from RelTR_SGG import build_model, load_checkpoint, infer_one_image  # 동료 코드 사용

class RelTRSGGNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic   = rospy.get_param("~image_topic", "/camera/image")
        self.use_compressed= rospy.get_param("~use_compressed", False)
        self.weights       = rospy.get_param("~weights")
        self.device        = rospy.get_param("~device", "cuda")
        self.conf_th       = float(rospy.get_param("~confidence_thresh", 0.2))
        self.topk          = int(rospy.get_param("~topk_triplets", 10))
        self.save_dir      = os.path.expanduser(rospy.get_param("~save_dir", "~/.ros/scene_graph_results"))
        os.makedirs(self.save_dir, exist_ok=True)

        # 모델 로드 (RelTR_SGG 방식)
        self.model = build_model()
        self.model = load_checkpoint(self.model, ckpt_path=self.weights)
        if self.device == "cuda" and torch.cuda.is_available():
            self.model = self.model.cuda()
        self.model.eval()

        self.pub_json = rospy.Publisher("scene_graph/json", String, queue_size=10)

        if self.use_compressed:
            rospy.Subscriber(self.image_topic, CompressedImage, self._on_image_compressed, queue_size=1, buff_size=2**24)
        else:
            rospy.Subscriber(self.image_topic, Image,           self._on_image_raw,        queue_size=1, buff_size=2**24)
        rospy.loginfo(f"[reltr_sgg] Subscribed to {self.image_topic}")

    def _on_image_raw(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._process_frame(frame, msg.header.stamp.to_nsec())

    def _on_image_compressed(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self._process_frame(frame, msg.header.stamp.to_nsec())

    @torch.no_grad()
    def _process_frame(self, frame_bgr, stamp_nsec: int):
        try:
            # OpenCV BGR -> PIL RGB
            img_rgb = frame_bgr[..., ::-1]
            pil_img = PILImage.fromarray(img_rgb)

            dev = next(self.model.parameters()).device
            trips = infer_one_image(self.model, pil_img, topk=self.topk, conf_th=self.conf_th, device=dev)

            # trips = infer_one_image(self.model, pil_img, topk=self.topk, conf_th=self.conf_th)

            # 저장 + 퍼블리시
            out = {"stamp_nsec": int(stamp_nsec), "num": len(trips), "triplets": trips}
            out_path = os.path.join(self.save_dir, f"{stamp_nsec}.json")
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)
            self.pub_json.publish(String(data=json.dumps(out, ensure_ascii=False)))
            rospy.loginfo_throttle(2.0, f"[reltr_sgg] {len(trips)} triplets -> {out_path}")
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[reltr_sgg] frame failed: {type(e).__name__}: {e}")

def main():
    rospy.init_node("reltr_sgg_node")
    RelTRSGGNode()
    rospy.spin()

if __name__ == "__main__":
    main()