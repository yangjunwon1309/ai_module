import sys
import os
import glob
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import json
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as T

from PIL import Image
import requests
import matplotlib.pyplot as plt

CLASSES = [ 'N/A', 'airplane', 'animal', 'arm', 'bag', 'banana', 'basket', 'beach', 'bear', 'bed', 'bench', 'bike',
    'bird', 'board', 'boat', 'book', 'boot', 'bottle', 'bowl', 'box', 'boy', 'branch', 'building',
    'bus', 'cabinet', 'cap', 'car', 'cat', 'chair', 'child', 'clock', 'coat', 'counter', 'cow', 'cup',
    'curtain', 'desk', 'dog', 'door', 'drawer', 'ear', 'elephant', 'engine', 'eye', 'face', 'fence',
    'finger', 'flag', 'flower', 'food', 'fork', 'fruit', 'giraffe', 'girl', 'glass', 'glove', 'guy',
    'hair', 'hand', 'handle', 'hat', 'head', 'helmet', 'hill', 'horse', 'house', 'jacket', 'jean',
    'kid', 'kite', 'lady', 'lamp', 'laptop', 'leaf', 'leg', 'letter', 'light', 'logo', 'man', 'men',
    'motorcycle', 'mountain', 'mouth', 'neck', 'nose', 'number', 'orange', 'pant', 'paper', 'paw',
    'people', 'person', 'phone', 'pillow', 'pizza', 'plane', 'plant', 'plate', 'player', 'pole', 'post',
    'pot', 'racket', 'railing', 'rock', 'roof', 'room', 'screen', 'seat', 'sheep', 'shelf', 'shirt',
    'shoe', 'short', 'sidewalk', 'sign', 'sink', 'skateboard', 'ski', 'skier', 'sneaker', 'snow',
    'sock', 'stand', 'street', 'surfboard', 'table', 'tail', 'tie', 'tile', 'tire', 'toilet', 'towel',
    'tower', 'track', 'train', 'tree', 'truck', 'trunk', 'umbrella', 'vase', 'vegetable', 'vehicle',
    'wave', 'wheel', 'window', 'windshield', 'wing', 'wire', 'woman', 'zebra']

REL_CLASSES = ['__background__', 'above', 'across', 'against', 'along', 'and', 'at', 'attached to', 'behind',
    'belonging to', 'between', 'carrying', 'covered in', 'covering', 'eating', 'flying in', 'for',
    'from', 'growing on', 'hanging from', 'has', 'holding', 'in', 'in front of', 'laying on',
    'looking at', 'lying on', 'made of', 'mounted on', 'near', 'of', 'on', 'on back of', 'over',
    'painted on', 'parked on', 'part of', 'playing', 'riding', 'says', 'sitting on', 'standing on',
    'to', 'under', 'using', 'walking in', 'walking on', 'watching', 'wearing', 'wears', 'with']

from RelTR.models.backbone import Backbone, Joiner
from RelTR.models.position_encoding import PositionEmbeddingSine
from RelTR.models.transformer import Transformer
from RelTR.models.reltr import RelTR

def build_model():
    position_embedding = PositionEmbeddingSine(128, normalize=True)
    backbone = Backbone('resnet50', False, False, False)
    backbone = Joiner(backbone, position_embedding)
    backbone.num_channels = 2048

    transformer = Transformer(
        d_model=256, dropout=0.1, nhead=8,
        dim_feedforward=2048,
        num_encoder_layers=6,
        num_decoder_layers=6,
        normalize_before=False,
        return_intermediate_dec=True
    )

    model = RelTR(
        backbone, transformer,
        num_classes=151, num_rel_classes=51,
        num_entities=100, num_triplets=200
    )

    return model

def load_checkpoint(model, ckpt_path="$(find reltr_scene_graph)/checkpoints/checkpoint0149.pth"):
    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
    
    ckpt = torch.load(ckpt_path, map_location='cpu')
    model.load_state_dict(ckpt['model'])
    print("V Checkpoint loaded successfully")
    model.eval()
    return model

transform = T.Compose([
    T.Resize(800),
    T.ToTensor(),
    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

def box_cxcywh_to_xyxy(x):
    x_c, y_c, w, h = x.unbind(1)
    b = [(x_c - 0.5 * w), (y_c - 0.5 * h),
          (x_c + 0.5 * w), (y_c + 0.5 * h)]
    return torch.stack(b, dim=1)

def rescale_bboxes(out_bbox, size):
    img_w, img_h = size
    b = box_cxcywh_to_xyxy(out_bbox)
    b = b * torch.tensor([img_w, img_h, img_w, img_h], dtype=torch.float32)
    return b

def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    inter = max(0, xB - xA) * max(0, yB - yA)
    if inter == 0:
        return 0
    areaA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    areaB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return inter / float(areaA + areaB - inter)

IOU_THRESHOLD = 0.5
node_infos = {}

def get_node(label, box):
    for name, info in node_infos.items():
        if info['label'] == label and iou(info['box'], box) > IOU_THRESHOLD:
            return name
    idx = sum(1 for info in node_infos.values() if info['label'] == label) + 1
    new_name = f"{label}{idx}"
    node_infos[new_name] = {'label': label, 'box': box}
    return new_name

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import math

def circular_layout_equal(nodes, radius=1.0):
    n = len(nodes)
    thetas = np.linspace(0, 2*np.pi, n, endpoint=False)
    return {node: (radius*np.cos(t), radius*np.sin(t)) for node, t in zip(nodes, thetas)}

# @torch.no_grad()
# def infer_one_image(model, pil_img, topk=10, conf_th=0.30):
#     node_infos.clear()
#     img   = transform(pil_img).unsqueeze(0)
#     output = model(img)

#     p_rel  = output["rel_logits"].softmax(-1)[0, :, :-1]
#     p_sub  = output["sub_logits"].softmax(-1)[0, :, :-1]
#     p_obj  = output["obj_logits"].softmax(-1)[0, :, :-1]

#     keep = torch.logical_and(
#         p_rel.max(-1).values > conf_th,
#         torch.logical_and(p_sub.max(-1).values > conf_th,
#                           p_obj.max(-1).values > conf_th)
#     )
#     sub_boxes = rescale_bboxes(output["sub_boxes"][0, keep], pil_img.size)
#     obj_boxes = rescale_bboxes(output["obj_boxes"][0, keep], pil_img.size)

#     keep_q   = torch.nonzero(keep, as_tuple=True)[0]
#     scores   = (p_rel[keep_q].max(-1)[0] *
#                 p_sub[keep_q].max(-1)[0] *
#                 p_obj[keep_q].max(-1)[0])
#     best_idx = torch.argsort(-scores)[:topk]
#     keep_q   = keep_q[best_idx]

#     triplets = []
#     for i, q in enumerate(keep_q):
#         triplets.append({
#             "subject"     : CLASSES     [p_sub[q].argmax()],
#             "predicate"   : REL_CLASSES [p_rel[q].argmax()],
#             "object"      : CLASSES     [p_obj[q].argmax()],
#             "subject_box" : sub_boxes[best_idx][i].tolist(),
#             "object_box"  : obj_boxes [best_idx][i].tolist(),
#             "confidence"  : float(scores[best_idx][i])
#         })
#     return triplets

@torch.no_grad()
def infer_one_image(model, pil_img, topk=10, conf_th=0.30, device=None):
    # 1) 모델 디바이스 결정
    if device is None:
        device = next(model.parameters()).device

    # 2) 전처리 → 배치차원 → 모델 디바이스로 올리기 (핵심!)
    #    transform 은 파일 상단/전역에 이미 정의되어 있다고 가정
    img = transform(pil_img).unsqueeze(0).to(device)

    # 3) 추론
    out = model(img)

    # 4) 후처리는 CPU에서 (안전/호환성)
    rel = out["rel_logits"][0].detach().cpu()   # (Q, R)
    sub = out["sub_logits"][0].detach().cpu()   # (Q, C)
    obj = out["obj_logits"][0].detach().cpu()   # (Q, C)
    sub_b = out["sub_boxes"][0].detach().cpu()  # (Q, 4)
    obj_b = out["obj_boxes"][0].detach().cpu()  # (Q, 4)

    # 5) softmax & 배경 제거(마지막 클래스 가정)
    p_rel = F.softmax(rel, dim=-1)[..., :-1]
    p_sub = F.softmax(sub, dim=-1)[..., :-1]
    p_obj = F.softmax(obj, dim=-1)[..., :-1]

    # 6) 스코어 임계치로 1차 필터
    keep = (p_rel.max(-1).values > conf_th) & \
           (p_sub.max(-1).values > conf_th) & \
           (p_obj.max(-1).values > conf_th)

    if keep.sum().item() == 0:
        return []

    # 7) 박스 리스케일 (이미지 사이즈 기준)
    sub_boxes = rescale_bboxes(sub_b[keep], pil_img.size)  # Tensor on CPU
    obj_boxes = rescale_bboxes(obj_b[keep], pil_img.size)

    # 8) top-k 선정
    keep_q   = torch.nonzero(keep, as_tuple=True)[0]
    scores   = (p_rel[keep_q].max(-1).values *
                p_sub[keep_q].max(-1).values *
                p_obj[keep_q].max(-1).values)

    k = min(int(topk), scores.numel())
    best_idx = torch.topk(scores, k=k, largest=True).indices
    keep_q   = keep_q[best_idx]  # 선택된 쿼리 인덱스

    # 9) 트리플렛 구성 (라벨 인덱스는 .item()으로 파이썬 int 변환)
    triplets = []
    for i, q in enumerate(keep_q.tolist()):
        triplets.append({
            "subject"     : CLASSES     [p_sub[q].argmax(-1).item()],
            "predicate"   : REL_CLASSES [p_rel[q].argmax(-1).item()],
            "object"      : CLASSES     [p_obj[q].argmax(-1).item()],
            "subject_box" : sub_boxes[best_idx[i]].tolist(),
            "object_box"  : obj_boxes [best_idx[i]].tolist(),
            "confidence"  : float(scores[best_idx[i]].item())
        })
    return triplets


def save_json(frame_idx: int, trips, out_dir="sg_json"):
    Path(out_dir).mkdir(exist_ok=True, parents=True)
    out_path = Path(out_dir) / f"sg{frame_idx:04d}.json"
    with open(out_path, "w") as f:
        json.dump(trips, f, indent=2)

def run_on_folder(model, img_folder, topk=10):
    paths = sorted(glob.glob(os.path.join(img_folder, "*.jpg")))
    for idx, p in enumerate(paths):
        img = Image.open(p).convert("RGB")
        trips = infer_one_image(model, img, topk=topk)
        save_json(idx, trips, out_dir="sg_json")

if __name__ == '__main__':
    print("V Building model...")
    model = build_model()

    print("V Loading checkpoint...")
    model = load_checkpoint(model, ckpt_path='checkpoint0149.pth')

    IMG_FOLDER = "/home/aailab/cwh316/INTERN/matthewwk/CMU_VLA/SGG/node_0"
    run_on_folder(model, IMG_FOLDER, topk=10)
