# -*- coding: utf-8 -*-
from pathlib import Path
import sys, os, json
import torch
import torchvision.transforms as T
import numpy as np
import rospkg
from types import SimpleNamespace

# RelTR를 패키지로 import
from RelTR import __path__ as RELTR_PKG 

PKG_ROOT = Path(rospkg.RosPack().get_path('reltr_scene_graph'))
RELTR_ROOT = PKG_ROOT / 'RelTR'
if str(RELTR_ROOT) not in sys.path:
    sys.path.insert(0, str(RELTR_ROOT))

# A) 보편적 DETR류 전처리 (RelTR가 별도 transforms를 제공하면 교체)
_DEFAULT_TRANSFORM = T.Compose([
    T.ToTensor(),  # (H,W,C)->(C,H,W), [0,1]
    T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
])

def _args_from_ckpt(ckpt):
    a = ckpt.get('args', None)
    if a is None:
        return None
    if isinstance(a, dict):
        return SimpleNamespace(**a)
    # argparse.Namespace일 수도 있음
    return a

def _default_reltr_args(device="cuda"):
    # 레포마다 조금씩 다르지만, 대부분 DETR 계열 + VG용 기본값으로 안전하게 구성
    # 필요시 추후 실제 레포에 맞춰 미세조정(3단계) 한다~
    return SimpleNamespace(
        device=device if (device=="cuda" and torch.cuda.is_available()) else "cpu",
        backbone='resnet50',
        dilation=False,
        position_embedding='sine',
        hidden_dim=256,
        dropout=0.0,
        nheads=8,
        num_encoder_layers=6,
        num_decoder_layers=6,
        dim_feedforward=2048,
        pre_norm=False,
        aux_loss=False,
        # RelTR이 요구하는 클래스 수(예: Visual Genome 기준)
        # 체크포인트 args가 있으면 그걸 우선 사용하므로 여기 값은 fallback임
        num_obj_classes=151,   # (배경 포함 151을 쓰는 레포가 흔함)
        num_rel_classes=51,    # 관계 클래스 수
        # 기타 자주 쓰는 필드 (모델 내부에서 접근 가능성)
        num_queries=100
    )

def _ensure_args_defaults(args, device="cuda"):
    use_cuda = (device == "cuda" and torch.cuda.is_available())

    def setdef(k, v):
        if not hasattr(args, k):
            setattr(args, k, v)

    # 필수 공통
    setdef("device", "cuda" if use_cuda else "cpu")
    setdef("dataset", "vg")                 # OI 체크포인트면 "oi"로 바꾸기

    # 모델 구조
    setdef("backbone", "resnet50")
    setdef("dilation", False)
    setdef("position_embedding", "sine")
    setdef("hidden_dim", 256)
    setdef("dim_feedforward", 2048)
    setdef("dropout", 0.0)
    setdef("nheads", 8)

    # 레포가 enc/dec를 이렇게 부르는 듯 → 기본 6
    setdef("enc_layers", 6)
    setdef("dec_layers", 6)

    # 학습 옵션(추론만 할 거라 안전 기본값)
    setdef("pre_norm", False)
    setdef("aux_loss", False)
    setdef("return_interm_layers", False)
    setdef("lr_backbone", 0.0)

    # 매처/손실 계수 (DETR 계열 관용값)
    setdef("set_cost_class", 1.0)
    setdef("set_cost_bbox", 5.0)
    setdef("set_cost_giou", 2.0)
    setdef("set_iou_threshold", 0.5)

    setdef("bbox_loss_coef", 5.0)
    setdef("giou_loss_coef", 2.0)
    setdef("rel_loss_coef", 1.0)
    setdef("eos_coef", 0.1)

    # 클래스 개수(레포 필드명에 맞춤)
    setdef("num_entities", 151)   # VG commonly uses 151 incl. background
    setdef("num_triplets", 51)    # 관계 클래스 수(= predicates)

def build_reltr_model(device="cuda", ckpt_path:str=None):
    # 0) 먼저 ckpt를 읽어서 args를 뽑아보자
    ckpt = None
    if ckpt_path is not None and os.path.exists(ckpt_path):
        ckpt = torch.load(ckpt_path, map_location="cpu")
    args = _args_from_ckpt(ckpt) if ckpt is not None else None
    if args is None:
        args = _default_reltr_args(device=device)
    else:
        # ckpt의 args에도 device만큼은 보정
        if hasattr(args, "device"):
            args.device = device if (device=="cuda" and torch.cuda.is_available()) else "cpu"

    _ensure_args_defaults(args, device=device)

    # 1) models만
    from models import build_model as _build_model
    built = _build_model(args)
    model = built[0] if isinstance(built, tuple) else built

    # 2) 가중치 로딩
    if ckpt is not None:
        state = ckpt.get("model", ckpt)
        missing, unexpected = model.load_state_dict(state, strict=False)
        print(f"[RelTR] loaded ckpt: {ckpt_path}")
        if missing:    print(f"[RelTR] missing keys: {len(missing)} → {missing[:10]} ...")
        if unexpected: print(f"[RelTR] unexpected keys: {len(unexpected)} → {unexpected[:10]} ...")

    device_t = args.device if hasattr(args, "device") else ("cuda" if torch.cuda.is_available() and device=="cuda" else "cpu")
    model.to(device_t).eval()
    torch.backends.cudnn.benchmark = True
    return model

def preprocess_bgr(img_bgr: np.ndarray):
    """OpenCV BGR -> PIL-like tensor (C,H,W), float normalized"""
    # BGR -> RGB
    img_rgb = img_bgr[..., ::-1].copy()
    tensor = _DEFAULT_TRANSFORM(img_rgb)  # float tensor
    return tensor.unsqueeze(0)  # (1,C,H,W)

@torch.no_grad()
def infer_reltr(model: torch.nn.Module, img_tensor: torch.Tensor, device="cuda"):
    """
    모델 출력(raw)을 그대로 반환.
    출력 포맷은 레포 구현에 따라 다름(dict/tuple 등) → 후처리에서 해석
    """
    device_t = next(model.parameters()).device
    return model(img_tensor.to(device_t))

@torch.no_grad()
def decode_scene_graph(outputs, conf_th=0.3):
    """
    RelTR 출력(dict) -> (subject, predicate, object) triplets 리스트
    - 배치 차원/리스트 래핑/shape 불일치에 방어적으로 대응
    """
    triplets = []
    if not isinstance(outputs, dict):
        return triplets

    # 1) 안전하게 텐서 꺼내기 (list/tuple일 경우 첫 요소)
    def _pick(x):
        if isinstance(x, (list, tuple)):
            x = x[0] if len(x) > 0 else None
        return x

    sub_logits = _pick(outputs.get("sub_logits", None))
    obj_logits = _pick(outputs.get("obj_logits", None))
    if obj_logits is None:
        obj_logits = _pick(outputs.get("pred_logits", None))
    rel_logits = _pick(outputs.get("rel_logits", None))
    sub_boxes  = _pick(outputs.get("sub_boxes",  None))
    obj_boxes  = _pick(outputs.get("obj_boxes",  None))

    # 2) 배치 차원(squeeze) 처리: (B,N,C) -> (N,C)
    def _squeeze_bc(t):
        if t is None:
            return None
        if isinstance(t, torch.Tensor):
            # (1,N,C) 또는 (N,C) 만듦
            if t.dim() == 3 and t.size(0) == 1:
                t = t.squeeze(0)
            elif t.dim() == 1:
                t = t.unsqueeze(0)  # (C,) -> (1,C)
        return t

    sub_logits = _squeeze_bc(sub_logits)
    obj_logits = _squeeze_bc(obj_logits)
    rel_logits = _squeeze_bc(rel_logits)
    sub_boxes  = _squeeze_bc(sub_boxes)
    obj_boxes  = _squeeze_bc(obj_boxes)

    # 필수 텐서 없으면 종료
    if (sub_logits is None) or (obj_logits is None) or (rel_logits is None):
        return triplets

    # 3) 마지막 차원(-1) 기준 softmax → max
    sub_probs = torch.softmax(sub_logits, dim=-1)
    obj_probs = torch.softmax(obj_logits, dim=-1)
    rel_probs = torch.softmax(rel_logits, dim=-1)

    sub_scores, sub_labels = sub_probs.max(dim=-1)  # (N,)
    obj_scores, obj_labels = obj_probs.max(dim=-1)  # (N,)
    rel_scores, rel_labels = rel_probs.max(dim=-1)  # (N,)

    # 4) 길이 정합 (가장 작은 N에 맞추기)
    N = min(sub_labels.shape[0], obj_labels.shape[0], rel_labels.shape[0])
    sub_labels = sub_labels[:N]
    obj_labels = obj_labels[:N]
    rel_labels = rel_labels[:N]
    sub_scores = sub_scores[:N]
    obj_scores = obj_scores[:N]
    rel_scores = rel_scores[:N]

    # 5) (옵션) 라벨 매핑 준비 — 지금은 정수, 3단계에서 문자열 매핑 붙임
    def idx2ent(i): return int(i)
    def idx2rel(i): return int(i)

    # 6) 임계치로 필터하고 triplet 생성
    for i in range(N):
        rscore = float(rel_scores[i].detach().cpu().item())
        if rscore < float(conf_th):
            continue
        one = {
            "subj": idx2ent(sub_labels[i].detach().cpu().item()),
            "pred": idx2rel(rel_labels[i].detach().cpu().item()),
            "obj":  idx2ent(obj_labels[i].detach().cpu().item()),
            "score": rscore,
        }
        if isinstance(sub_boxes, torch.Tensor) and sub_boxes.shape[0] > i:
            one["sub_box"] = sub_boxes[i].detach().cpu().tolist()
        if isinstance(obj_boxes, torch.Tensor) and obj_boxes.shape[0] > i:
            one["obj_box"] = obj_boxes[i].detach().cpu().tolist()
        triplets.append(one)

    return triplets