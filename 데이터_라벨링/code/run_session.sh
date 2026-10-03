#!/usr/bin/env bash
# 한 세션에 4·9·10·11 단계를 순서대로 돌린다.
#
#     bash run_session.sh 026 2026-10-04
#
# 5 단계(이미지 변환)는 ReBound 변환이 원본 .npy 를 직접 읽으므로 건너뛴다.
# 12 단계(툴 적재)는 사람이 확인하는 단계라 포함하지 않는다.
set -euo pipefail

SES=$1
DATE=${2:-$(date +%F)}
ROOT=/home/acelab/Documents/Lidar_Camera_Calibration
SCRATCH=/tmp/claude-1000/-home-acelab/ce6f8a3b-9b56-4565-96d2-09d74f939267/scratchpad
S=/home/acelab/ros2_ws/cam_lidar_recording/sessions/session_$SES
R=$ROOT/포인트보정/results/${DATE}_session_$SES
RB=/home/acelab/ros2_ws/cam_lidar_recording/rebound/session_$SES
CFG=$SCRATCH/det/mmdet3d/.mim/configs/pointpillars/pointpillars_hv_fpn_sbn-all_8xb4-2x_nus-3d.py
CKPT=$SCRATCH/ckpt/pp_nus.pth

cd "$ROOT"
mkdir -p "$R/autolabel"

echo "=== session_$SES : 4 단계 오도메트리 (deskew on) ==="
if [ -d "$R/deskew_on" ]; then echo "  이미 있음, 건너뜀"; else
PYTHONPATH=$SCRATCH/pylibs python3 포인트보정/code/run_kiss_icp.py "$S/lidar.pcap" \
    --voxel 0.2 --max-range 25 -o "$R/deskew_on"
fi

echo "=== session_$SES : 9 단계 키프레임 ==="
PYTHONPATH=$SCRATCH/pylibs python3 포인트보정/code/select_keyframes.py "$S" \
    --poses "$R/deskew_on" --min-dist 1.0 --max-gap 3.0 -o "$R/keyframes.json"

echo "=== session_$SES : 검출기 (점수 0.1) ==="
if [ -f "$R/autolabel/detections.json" ]; then echo "  이미 있음, 건너뜀"; else
PYTHONPATH=$SCRATCH/det python3 데이터_라벨링/code/auto_label.py "$S" \
    --poses "$R/deskew_on" --keyframes "$R/keyframes.json" \
    --config "$CFG" --checkpoint "$CKPT" --score 0.1 \
    -o "$R/autolabel"          # auto_label.py 는 폴더를 받아 detections.json 을 쓴다
fi

echo "=== session_$SES : 10 단계 초안 다듬기 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/refine_boxes.py "$S" \
    --poses "$R/deskew_on" --detections "$R/autolabel/detections.json" \
    -o "$R/autolabel/refined.json"

echo "=== session_$SES : 11 단계 ReBound 변환 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/to_rebound.py "$S" \
    --poses "$R/deskew_on" --keyframes "$R/keyframes.json" \
    --detections "$R/autolabel/refined.json" -o "$RB"

# bounding/ 은 사람이 작업할 사본이다. confidence 를 100 으로 올려 두지 않으면
# ReBound 기본 임계값에 걸려 화면에 안 보인다.
python3 - "$RB" <<'PY'
import json, sys, glob, os
rb = sys.argv[1]; n = 0
for d in sorted(glob.glob(rb + "/pred_bounding/*/boxes.json")):
    j = json.load(open(d))
    for b in j["boxes"]:
        b["confidence"] = 100
    for sub in ("bounding", "pred_bounding_original"):
        f = d.replace("pred_bounding", sub)
        os.makedirs(os.path.dirname(f), exist_ok=True)
        json.dump(j if sub == "bounding" else json.load(open(d)),
                  open(f, "w"), indent=1)
    n += len(j["boxes"])
print(f"  bounding/ 갱신, 박스 {n} 개")
PY

echo "=== session_$SES : 확인용 영상 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/render_boxes_video.py "$S" \
    --poses "$R/deskew_on" --detections "$R/autolabel/refined.json" \
    --score 0.1 -o "$S/boxes_refined.mp4"

echo "=== session_$SES 끝 ==="
