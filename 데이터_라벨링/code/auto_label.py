"""nuScenes 사전학습 검출기로 3D 박스 초안을 만든다.

    PYTHONPATH=<mmdet3d 환경> python3 auto_label.py <세션> \
        --poses <deskew_on> --keyframes <keyframes.json> \
        --checkpoint pp_nus.pth --config <pointpillars nus 설정> -o <출력>

왜 되는가
---------
처음에는 16 빔이라 사전학습 검출기가 안 먹힐 것으로 봤다. 단일 스캔 기준으로는
맞다 - 20 m 거리의 차에 점이 55 개뿐이다. 그런데 **nuScenes 모델은 10 스윕을
누적한 입력으로 학습**됐고, 우리도 정지 장면이라 같은 방식으로 누적할 수 있다.

  단일 스캔      약 27,000 점
  10 스윕 누적   약 260,000 점   ← nuScenes 입력(약 30만)과 비슷

좌표계 관례도 같다 (x=앞, y=왼쪽, z=위). 지면 높이도 라이다 기준 −1.91 m 로
nuScenes LIDAR_TOP 의 −1.84 m 와 7 cm 차이뿐이다.

입력 형식
---------
nuScenes 는 점마다 5 채널이다: x, y, z, intensity, dt
dt 는 그 점이 속한 스윕이 키프레임보다 몇 초 전인지다 (0 또는 음수).
누적은 **키프레임 시점의 라이다 좌표계**로 모은다 - ego pose 로 변환한다.

한계
----
오탐이 있다. 차 여러 대가 뭉쳐 'bus' 로 잡히거나, 기둥·볼라드가 낮은 점수의
pedestrian·motorcycle 로 잡힌다. 사람 검수가 필요하다. 다만 빈 화면에서
처음부터 그리는 것보다 훨씬 빠르다.
"""

import argparse
import json
import os
import sys
import warnings

import numpy as np

warnings.filterwarnings("ignore")


def build_input(ds, poses, k, n_sweep, out_path):
    """키프레임 k 를 nuScenes 형식 (x,y,z,intensity,dt) 으로 저장."""
    Tk_inv = np.linalg.inv(poses[k])
    acc = []
    for j in range(max(0, k - n_sweep + 1), k + 1):
        a, b = ds._spans[j]
        p = ds._xyz[a:b]
        inten = ds.intensity[a:b]
        T = Tk_inv @ poses[j]
        q = p @ T[:3, :3].T + T[:3, 3]
        dt = np.full(len(q), ds.scan_times[j] - ds.scan_times[k], np.float32)
        acc.append(np.column_stack([q, inten, dt]).astype(np.float32))
    arr = np.vstack(acc)
    m = ((np.abs(arr[:, 0]) < 50) & (np.abs(arr[:, 1]) < 50)
         & (arr[:, 2] > -5) & (arr[:, 2] < 3))
    arr[m].tofile(out_path)
    return int(m.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--keyframes", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--n-sweep", type=int, default=10)
    ap.add_argument("--score", type=float, default=0.3)
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses
    from mmdet3d.apis import init_model, inference_detector

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    kf = json.load(open(args.keyframes))["keyframes"]
    os.makedirs(args.out, exist_ok=True)
    tmp = os.path.join(args.out, "_input")
    os.makedirs(tmp, exist_ok=True)

    model = init_model(args.config, args.checkpoint, device=args.device)
    classes = list(model.dataset_meta["classes"])
    print(f"키프레임 {len(kf)} · {args.n_sweep} 스윕 누적 · 점수 임계 {args.score}")

    out = {"classes": classes, "score_threshold": args.score,
           "n_sweep": args.n_sweep, "frames": []}
    for i, f in enumerate(kf):
        path = os.path.join(tmp, f"{i:03d}.bin")
        npts = build_input(ds, poses, f["scan"], args.n_sweep, path)
        res, _ = inference_detector(model, path)
        p = res.pred_instances_3d
        sc = p.scores_3d.numpy()
        lb = p.labels_3d.numpy()
        bb = p.bboxes_3d.tensor.numpy()      # x,y,z(바닥중심),dx,dy,dz,yaw
        keep = sc >= args.score
        boxes = [{"class": classes[int(lb[j])], "score": float(sc[j]),
                  "center": [float(x) for x in bb[j, :3]],
                  "size": [float(x) for x in bb[j, 3:6]],
                  "yaw": float(bb[j, 6])} for j in np.where(keep)[0]]
        out["frames"].append({"keyframe": i, "scan": f["scan"],
                              "n_points": npts, "boxes": boxes})
        print(f"  {i:3d}/{len(kf)}  점 {npts:,}  박스 {len(boxes)}", flush=True)

    dst = os.path.join(args.out, "detections.json")
    with open(dst, "w") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    tot = sum(len(f["boxes"]) for f in out["frames"])
    from collections import Counter
    cnt = Counter(b["class"] for f in out["frames"] for b in f["boxes"])
    print(f"\n총 박스 {tot}개  {dict(cnt)}")
    print(f"저장: {dst}")


if __name__ == "__main__":
    main()
