"""볼라드를 라이다 점에서 직접 찾는다.

    python3 detect_bollards.py <세션> --poses <deskew_on> --keyframes <keyframes.json> \
        [--vehicles <refined.json>] -o bollards.json

왜 따로 만드나
--------------
nuScenes 사전학습 검출기에는 **볼라드 클래스가 없다.** 가장 가까운 것이
traffic_cone·barrier 인데, session_031 에서 10 m 이내로 나온 것이 traffic_cone
4 개뿐이라 쓸 수 없다.

반면 볼라드는 모양이 단순해서 기하로 찾기 쉽다. 지면에 서 있고, 가늘고
(지름 약 0.1 m), 0.7 m 쯤 올라가며, 주변이 비어 있다. 30 스윕을 누적하면
10 m 에서 수십 점이 찍힌다.

한계
----
**20 m 를 넘기면 못 찾는다.** 2 도 간격이라 15 m 에서 링이 2 개, 20 m 에서
1 개뿐이다. `dataset_config.yaml` 이 볼라드 라벨 범위를 10 m 로 잡은 것과 같은
이유다.
"""

import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def ground_of(z):
    hist, edges = np.histogram(z[z < np.median(z)], bins=100)
    return float(edges[int(np.argmax(hist))])


def clusters(xy, eps):
    """cKDTree 로 연결 요소를 찾는다 (DBSCAN 의 eps 연결과 같다)."""
    tree = cKDTree(xy)
    seen = np.zeros(len(xy), bool)
    out = []
    for i in range(len(xy)):
        if seen[i]:
            continue
        grp = [i]
        seen[i] = True
        head = 0
        while head < len(grp):
            for j in tree.query_ball_point(xy[grp[head]], eps):
                if not seen[j]:
                    seen[j] = True
                    grp.append(j)
            head += 1
        out.append(grp)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--keyframes", required=True)
    ap.add_argument("--vehicles", default=None,
                    help="refined.json. 차량 박스 안의 점을 빼는 데 쓴다")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--n-sweep", type=int, default=30)
    ap.add_argument("--max-range", type=float, default=12.0,
                    help="이보다 멀면 링이 1~2 개라 못 찾는다")
    ap.add_argument("--z-lo", type=float, default=0.15, help="지면 위 몇 m 부터 볼 것인가")
    ap.add_argument("--z-hi", type=float, default=1.00)
    ap.add_argument("--eps", type=float, default=0.25, help="군집 연결 거리 (m)")
    ap.add_argument("--min-points", type=int, default=6)
    ap.add_argument("--max-width", type=float, default=0.45, help="볼라드 지름 상한 (m)")
    ap.add_argument("--min-height", type=float, default=0.30)
    ap.add_argument("--max-height", type=float, default=1.00)
    ap.add_argument("--clear-radius", type=float, default=0.8,
                    help="이 반경 안에 다른 점이 있으면 벽·차 일부로 본다")
    ap.add_argument("--size", default="0.12,0.12,0.75", help="고정 크기 (규격품)")
    ap.add_argument("--link-dist", type=float, default=0.6,
                    help="키프레임 사이에서 같은 볼라드로 묶을 거리 (m)")
    ap.add_argument("--min-views", type=int, default=2)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(here)),
                                    "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    kfs = json.load(open(args.keyframes))
    kf = kfs["keyframes"] if isinstance(kfs, dict) and "keyframes" in kfs else kfs
    scans = [k["scan"] if isinstance(k, dict) else int(k) for k in kf]

    veh = {}
    if args.vehicles:
        vj = json.load(open(args.vehicles))
        veh = {f["scan"]: f["boxes"] for f in vj["frames"]}

    L, W, H = (float(x) for x in args.size.split(","))
    found = []
    for idx, k in enumerate(scans):
        acc = []
        for j in range(max(0, k - args.n_sweep + 1), k + 1):
            a, b = ds._spans[j]
            T = np.linalg.inv(poses[k]) @ poses[j]
            acc.append(ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3])
        A = np.vstack(acc)
        gnd = ground_of(A[:, 2])
        zr = A[:, 2] - gnd
        rng = np.hypot(A[:, 0], A[:, 1])
        m = (zr > args.z_lo) & (zr < args.z_hi) & (rng < args.max_range) & (rng > 1.5)

        # 차량 박스 안은 뺀다
        for b in veh.get(k, []):
            c = np.array(b["center"])
            l, w, h = b["size"]
            y = b["yaw"]
            d = A[:, :2] - c[:2]
            ca, sa = np.cos(-y), np.sin(-y)
            lo = np.stack([d[:, 0] * ca - d[:, 1] * sa,
                           d[:, 0] * sa + d[:, 1] * ca], axis=1)
            m &= ~((np.abs(lo[:, 0]) < l / 2 + 0.4) & (np.abs(lo[:, 1]) < w / 2 + 0.4))

        P = A[m]
        if len(P) < args.min_points:
            continue
        # 주변이 비었는지 보려면 밴드 밖 점도 필요하다 (벽은 위아래로 이어진다)
        tall = A[(zr > args.z_hi) & (zr < 2.5) & (rng < args.max_range)]
        tall_tree = cKDTree(tall[:, :2]) if len(tall) else None

        boxes = []
        for grp in clusters(P[:, :2], args.eps):
            if len(grp) < args.min_points:
                continue
            q = P[grp]
            wx = q[:, 0].max() - q[:, 0].min()
            wy = q[:, 1].max() - q[:, 1].min()
            if max(wx, wy) > args.max_width:
                continue
            zz = q[:, 2] - gnd
            hgt = zz.max() - zz.min()
            if not (args.min_height <= hgt <= args.max_height):
                continue
            if zz.min() > 0.45:                 # 바닥에서 시작해야 한다
                continue
            cx, cy = q[:, 0].mean(), q[:, 1].mean()
            # 위로 이어지면 기둥·벽이다
            if tall_tree is not None and tall_tree.query_ball_point([cx, cy], 0.35):
                continue
            boxes.append({"class": "bollard", "score": 0.5,
                          "center": [float(cx), float(cy), float(gnd)],
                          "size": [L, W, H], "yaw": 0.0, "points": len(grp)})
        found.append({"keyframe": idx, "scan": k, "boxes": boxes})
        print(f"  {idx+1}/{len(scans)}  스캔 {k}  볼라드 후보 {len(boxes)}")

    # 키프레임 사이에서 묶어, 한 번만 보인 것은 버린다
    obs = []
    for f in found:
        T = poses[f["scan"]]
        for b in f["boxes"]:
            c = np.array(b["center"]) @ T[:3, :3].T + T[:3, 3]
            obs.append((f["keyframe"], c, b))
    keep = set()
    if obs:
        cen = np.array([o[1][:2] for o in obs])
        tree = cKDTree(cen)
        for i, o in enumerate(obs):
            nb = tree.query_ball_point(cen[i], args.link_dist)
            if len(set(obs[j][0] for j in nb)) >= args.min_views:
                keep.add(i)
    kept = 0
    out = {"source": "detect_bollards.py", "params": vars(args), "frames": []}
    gi = 0
    for f in found:
        bs = []
        for b in f["boxes"]:
            if gi in keep:
                bs.append(b)
            gi += 1
        kept += len(bs)
        out["frames"].append({"keyframe": f["keyframe"], "scan": f["scan"], "boxes": bs})
    with open(args.out, "w") as fp:
        json.dump(out, fp, indent=1, ensure_ascii=False)
    raw = sum(len(f["boxes"]) for f in found)
    print(f"\n후보 {raw} 개 → {args.min_views} 회 이상 관측된 {kept} 개")
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
