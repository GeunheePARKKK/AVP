#!/usr/bin/env bash
# 한 세션에 4·9·10·11 단계를 순서대로 돌린다.
#
#     bash run_session.sh 026 2026-10-04
#
# 라이다 단독 파이프라인이다. 카메라를 쓰는 단계는 없다.
# 세 번째 인자로 autolabel 폴더 이름을 바꿀 수 있다 (session_025 는 autolabel_s01).
set -euo pipefail

SES=$1
DATE=${2:-$(date +%F)}
ROOT=/home/acelab/Documents/Lidar_Camera_Calibration
SCRATCH=/tmp/claude-1000/-home-acelab/ce6f8a3b-9b56-4565-96d2-09d74f939267/scratchpad
S=/home/acelab/ros2_ws/cam_lidar_recording/sessions/session_$SES
R=$ROOT/포인트보정/results/${DATE}_session_$SES
SUB=${3:-autolabel}
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
if [ -f "$R/$SUB/detections.json" ]; then echo "  이미 있음, 건너뜀"; else
PYTHONPATH=$SCRATCH/det python3 데이터_라벨링/code/auto_label.py "$S" \
    --poses "$R/deskew_on" --keyframes "$R/keyframes.json" \
    --config "$CFG" --checkpoint "$CKPT" --score 0.1 \
    -o "$R/$SUB"          # auto_label.py 는 폴더를 받아 detections.json 을 쓴다
fi

echo "=== session_$SES : 10 단계 초안 다듬기 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/refine_boxes.py "$S" \
    --poses "$R/deskew_on" --detections "$R/$SUB/detections.json" \
    -o "$R/$SUB/refined.json"

echo "=== session_$SES : 꼬깔콘 검출 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/detect_cones.py "$S" \
    --poses "$R/deskew_on" --keyframes "$R/keyframes.json" \
    --vehicles "$R/$SUB/refined.json" --taper 1.05 \
    -o "$R/$SUB/cones.json"

echo "=== session_$SES : 11 단계 ReBound 변환 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/to_rebound.py "$S" \
    --poses "$R/deskew_on" --keyframes "$R/keyframes.json" \
    --detections "$R/$SUB/refined.json" --extra "$R/$SUB/cones.json" \
    --n-sweep 30 -o "$RB"

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

echo "=== session_$SES : 확인용 3D 영상 ==="
PYTHONPATH=$SCRATCH/pylibs python3 데이터_라벨링/code/render_lidar_3d.py "$S" \
    --poses "$R/deskew_on" --detections "$R/$SUB/refined.json" \
    --extra "$R/$SUB/cones.json" \
    --eye=-22,0,16 --target=4,0,-0.5 --n-sweep 30 --color height \
    --drop-ground 0.25 --max-height 2.2 -o "$S/lidar3d.mp4"

echo "=== session_$SES 끝 ==="
