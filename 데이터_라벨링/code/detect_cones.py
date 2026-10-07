"""주행 유도 꼬깔콘(traffic cone)을 라이다 점에서 직접 찾는다.

    python3 detect_cones.py <세션> --poses <deskew_on> --keyframes <keyframes.json> \
        [--vehicles <refined.json>] -o cones.json

왜 검출기를 안 쓰나
-------------------
nuScenes 사전학습 모델에 `traffic_cone` 클래스가 있긴 하다. 그런데 **실제로는
못 잡는다.** session_028 에서 12 m 이내로 낸 traffic_cone 이 6 개뿐이고, 기하로
찾은 것들과 1 m 안에서 겹친 것은 1 개(1 %)였다.

이유는 점 개수다. 검출기는 10 스윕 입력을 받는데 그때 꼬깔콘은 10 m 에서 열 점
남짓이고, PointPillars 의 voxel 이 0.25 m 라 voxel 한두 개에 다 들어간다.
반면 30 스윕을 누적하면 같은 꼬깔콘이 수백 점이 된다 (실측 412~723 점).

session_031 실측 (전방 좌측, 스캔 32, 1.4 m 간격으로 세 개)
----------------------------------------------------------
    중심            점    바닥폭   꼭대기폭   꼭대기 높이
    (2.16, 3.66)   412   0.36     0.24      0.72 m
    (3.58, 3.63)   681   0.40     0.24      0.72 m
    (4.92, 3.69)   723   0.45     0.24      0.72 m

**위로 갈수록 좁아지는 것**이 꼬깔콘의 표식이다. 기둥·벽은 폭이 일정하다.

한계
----
**10 m 를 넘기면 못 찾는다.** 2 도 간격이라 거기서 링이 2 개뿐이다.
가까우면(3 m 이내) 아래쪽이 라이다 수직 시야(-15 도) 밖이라 윗부분만 보인다.
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
    """2D 점을 eps 거리로 이어 붙여 군집을 만든다.

    격자에 올려 연결 성분을 찾는다. 점마다 이웃을 조회하는 방식은 벽처럼
    점이 수만 개 뭉친 곳에서 감당이 안 된다 — session_037 에서 키프레임당
    18~24 초가 걸렸고 244 장을 돌리다 멈췄다. 격자는 점 수에 선형이다.

    격자 칸을 eps 로 잡고 8 방향으로 이으면, 거리 eps 이내를 잇는 것과
    사실상 같은 결과가 나온다.
    """
    from scipy import ndimage

    if len(xy) == 0:
        return []
    g = np.floor(xy / eps).astype(np.int64)
    g -= g.min(axis=0)
    h, w = int(g[:, 0].max()) + 3, int(g[:, 1].max()) + 3
    if h * w > 40_000_000:                 # 격자가 너무 크면 예전 방식으로
        return _clusters_slow(xy, eps)
    occ = np.zeros((h, w), bool)
    occ[g[:, 0] + 1, g[:, 1] + 1] = True
    lab, n = ndimage.label(occ, structure=np.ones((3, 3), bool))
    ids = lab[g[:, 0] + 1, g[:, 1] + 1]
    order = np.argsort(ids, kind="stable")
    ids_sorted = ids[order]
    bounds = np.searchsorted(ids_sorted, np.arange(1, n + 2))
    return [order[bounds[i]:bounds[i + 1]].tolist() for i in range(n)
            if bounds[i + 1] > bounds[i]]


def _clusters_slow(xy, eps):
    """격자가 감당 안 될 때의 대비책 (cKDTree 연쇄)."""
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
    ap.add_argument("--top-lo", type=float, default=0.45,
                    help="꼭대기가 지면 위 이 높이 이상이어야 한다")
    ap.add_argument("--top-hi", type=float, default=0.95)
    ap.add_argument("--taper", type=float, default=0.85,
                    help="윗부분 폭 / 아랫부분 폭 이 값 이하여야 한다 (원뿔은 좁아진다)")
    ap.add_argument("--z-lo", type=float, default=0.15, help="지면 위 몇 m 부터 볼 것인가")
    ap.add_argument("--z-hi", type=float, default=1.05)
    ap.add_argument("--eps", type=float, default=0.30, help="군집 연결 거리 (m)")
    ap.add_argument("--min-points", type=int, default=20)
    ap.add_argument("--max-width", type=float, default=0.60, help="바닥 지름 상한 (m)")
    ap.add_argument("--min-height", type=float, default=0.15)
    ap.add_argument("--max-height", type=float, default=0.95)
    ap.add_argument("--clear-radius", type=float, default=0.25,
                    help="꼬깔콘 바로 위 이 반경 안에 점이 이어지면 기둥·벽으로 본다")
    ap.add_argument("--size", default="0.45,0.45,0.75", help="고정 크기 (규격품)")
    ap.add_argument("--link-dist", type=float, default=0.6,
                    help="키프레임 사이에서 같은 꼬깔콘으로 묶을 거리 (m)")
    ap.add_argument("--min-views", type=int, default=2)
    ap.add_argument("--row-spacing", default="0.8,2.5",
                    help="꼬깔콘 열의 이웃 간격 범위 (m). 이 안에 짝이 있으면 믿는다")
    ap.add_argument("--bridge-gap", type=int, default=3,
                    help="트랙이 이 프레임 수 이하로 끊기면 앞뒤를 보간해 메운다")
    ap.add_argument("--no-fill", action="store_true",
                    help="빠진 키프레임을 채우지 않는다 (깜빡임을 그대로 둔다)")
    ap.add_argument("--fill-radius", type=float, default=0.35,
                    help="전파할 자리에 점이 있는지 볼 반경 (m)")
    ap.add_argument("--fill-points", type=int, default=5,
                    help="그 반경 안 점이 이보다 적으면 가려진 것으로 보고 내지 않는다")
    ap.add_argument("--no-row", action="store_true",
                    help="열 구조를 쓰지 않는다 (외톨이 꼬깔콘만 있는 데이터용)")
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
    band_cache = {}
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
        veh_free = np.ones(len(A), bool)
        for b in veh.get(k, []):
            c = np.array(b["center"])
            l, w, h = b["size"]
            y = b["yaw"]
            d = A[:, :2] - c[:2]
            ca, sa = np.cos(-y), np.sin(-y)
            lo = np.stack([d[:, 0] * ca - d[:, 1] * sa,
                           d[:, 0] * sa + d[:, 1] * ca], axis=1)
            veh_free &= ~((np.abs(lo[:, 0]) < l / 2 + 0.4) & (np.abs(lo[:, 1]) < w / 2 + 0.4))
        m &= veh_free

        P = A[m]
        if len(P) < args.min_points:
            continue
        # 기둥·벽은 위로 이어진다. 그걸 보려면 밴드 밖 점이 필요한데,
        # **차량 박스는 여기서도 빼야 한다.** 꼬깔콘 바로 뒤에 주차된 차가 있으면
        # 차체(1.4~1.8 m)가 걸려 멀쩡한 꼬깔콘이 전부 탈락한다.
        up_m = (zr > 1.0) & (zr < 2.6) & (rng < args.max_range) & veh_free
        tall = A[up_m]
        tall_z = zr[up_m]
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
            if not (args.top_lo <= zz.max() <= args.top_hi):
                continue                        # 꼬깔콘 키는 0.7 m 안팎이다
            # 위로 갈수록 좁아지는가. 기둥·벽은 폭이 일정하다.
            mid = zz.max() - 0.25
            up, dn = zz >= mid, zz < mid
            if up.sum() >= 4 and dn.sum() >= 4:
                wu = max(q[up, 0].max() - q[up, 0].min(), q[up, 1].max() - q[up, 1].min())
                wd = max(q[dn, 0].max() - q[dn, 0].min(), q[dn, 1].max() - q[dn, 1].min())
                if wd > 0.05 and wu / wd > args.taper:
                    continue
            cx, cy = q[:, 0].mean(), q[:, 1].mean()
            # 위로 이어지면 기둥·벽이다. 꼬깔콘 **바로 위**만 본다 —
            # 멀리 있는 천장 배관이나 옆 구조물에 걸리지 않도록.
            if tall_tree is not None:
                nb = tall_tree.query_ball_point([cx, cy], args.clear_radius)
                if nb and (tall_z[nb] < zz.max() + 0.9).any():
                    continue
            boxes.append({"class": "traffic_cone", "score": 0.5,
                          "center": [float(cx), float(cy), float(gnd)],
                          "size": [L, W, H], "yaw": 0.0, "points": len(grp)})
        band_cache[idx] = (P[:, :3].astype(np.float32), gnd)
        found.append({"keyframe": idx, "scan": k, "boxes": boxes})
        print(f"  {idx+1}/{len(scans)}  스캔 {k}  꼬깔콘 후보 {len(boxes)}")

    # 키프레임 사이에서 묶어, 한 번만 보인 것은 버린다
    obs = []
    for f in found:
        T = poses[f["scan"]]
        for b in f["boxes"]:
            c = np.array(b["center"]) @ T[:3, :3].T + T[:3, 3]
            obs.append((f["keyframe"], c, b))
    keep = set()
    tid_of = {}
    if obs:
        cen = np.array([o[1][:2] for o in obs])
        tree = cKDTree(cen)
        # 같은 꼬깔콘을 키프레임 사이에서 묶어 물체 단위로 본다
        obj = {}
        for i in range(len(obs)):
            nb = tree.query_ball_point(cen[i], args.link_dist)
            root = min(nb)
            obj.setdefault(root, []).append(i)
        # 물체 중심과 관측 횟수
        oc, ov = [], []
        for root, idxs in obj.items():
            oc.append(cen[idxs].mean(axis=0))
            ov.append((root, idxs, len(set(obs[j][0] for j in idxs))))
        oc = np.array(oc)

        # 꼬깔콘은 **줄지어 선다.** 1 m 안팎 간격으로 짝이 있으면 믿을 만하다.
        # 테이퍼(위로 좁아짐)보다 이쪽이 훨씬 잘 갈린다 — 바닥이 라이다 수직
        # 시야 밖이라 좁은 윗부분만 보이는 경우가 많아 테이퍼가 흐려진다.
        lo_s, hi_s = (float(x) for x in args.row_spacing.split(","))
        in_row = np.zeros(len(oc), bool)
        if len(oc) > 1 and not args.no_row:
            t2 = cKDTree(oc)
            for i in range(len(oc)):
                for j in t2.query_ball_point(oc[i], hi_s):
                    if j != i and lo_s <= np.linalg.norm(oc[j] - oc[i]) <= hi_s:
                        in_row[i] = True
                        break

        tid_of = {}
        nkeep = 0
        for i, (root, idxs, views) in enumerate(ov):
            # 줄지어 있으면 한 번만 보여도 받아들이고, 외톨이는 여러 번 봐야 한다
            need = 1 if in_row[i] else args.min_views
            if views >= need:
                keep.update(idxs)
                tid = f"C{nkeep:03d}"
                nkeep += 1
                for j in idxs:
                    tid_of[j] = (tid, views, bool(in_row[i]))
        print(f"  물체 {len(ov)} 개 · 그중 줄지어 선 것 {int(in_row.sum())} 개")
    kept = 0
    out = {"source": "detect_bollards.py", "params": vars(args), "frames": []}
    gi = 0
    per_frame = {}
    world = {}                                   # 트랙 → 월드 중심
    for f in found:
        bs = []
        for b in f["boxes"]:
            if gi in keep:
                tid, views, row = tid_of[gi]
                b["track_id"] = tid
                b["views"] = views
                b["in_row"] = row
                b["source"] = "원본"
                bs.append(b)
                T = poses[f["scan"]]
                c = np.array(b["center"]) @ T[:3, :3].T + T[:3, 3]
                world.setdefault(tid, []).append(c)
            gi += 1
        per_frame[f["keyframe"]] = bs
    world = {t: np.mean(v, axis=0) for t, v in world.items()}

    # ---------- 빠진 키프레임 채우기 ----------
    # 검출된 프레임에만 박스를 내면 꼬깔콘이 깜빡인다. 차량에 쓴 것과 같은
    # 원리로, 트랙을 **모든 키프레임에 다시 뿌린다.** 거리 안에 들어오고
    # 그 자리에 점이 실제로 있을 때만 낸다.
    filled = 0
    if not args.no_fill:
        for f in found:
            kfi = f["keyframe"]
            have = {b["track_id"] for b in per_frame[kfi]}
            if kfi not in band_cache:
                continue
            Pb, gnd = band_cache[kfi]
            Tin = np.linalg.inv(poses[f["scan"]])
            for tid, wc in world.items():
                if tid in have:
                    continue
                c = wc @ Tin[:3, :3].T + Tin[:3, 3]
                if np.hypot(c[0], c[1]) >= args.max_range:
                    continue
                d = np.hypot(Pb[:, 0] - c[0], Pb[:, 1] - c[1])
                n = int((d < args.fill_radius).sum())
                if n < args.fill_points:
                    continue                      # 점이 없으면 가려졌거나 없는 것
                q = Pb[d < args.fill_radius]
                per_frame[kfi].append(
                    {"class": "traffic_cone", "score": 0.4,
                     "center": [float(q[:, 0].mean()), float(q[:, 1].mean()), float(gnd)],
                     "size": [L, W, H], "yaw": 0.0, "points": n,
                     "track_id": tid, "source": "전파"})
                filled += 1

    # ---------- 짧은 공백 메우기 ----------
    # 가려짐이나 점 부족으로 한두 프레임만 빠지는 깜빡임을 없앤다.
    bridged = 0
    if args.bridge_gap > 0:
        scan_of = {f["keyframe"]: f["scan"] for f in found}
        have = {}
        for kfi, bs in per_frame.items():
            for b in bs:
                have.setdefault(b["track_id"], {})[kfi] = b
        for tid, by_kf in have.items():
            ks = sorted(by_kf)
            for a, c in zip(ks, ks[1:]):
                gap = c - a - 1
                if not (0 < gap <= args.bridge_gap):
                    continue
                Ta, Tc = poses[scan_of[a]], poses[scan_of[c]]
                wa = np.array(by_kf[a]["center"]) @ Ta[:3, :3].T + Ta[:3, 3]
                wc = np.array(by_kf[c]["center"]) @ Tc[:3, :3].T + Tc[:3, 3]
                for kk in range(a + 1, c):
                    if kk not in scan_of:
                        continue
                    Tin = np.linalg.inv(poses[scan_of[kk]])
                    u = (kk - a) / (c - a)
                    cc = (wa * (1 - u) + wc * u) @ Tin[:3, :3].T + Tin[:3, 3]
                    if np.hypot(cc[0], cc[1]) >= args.max_range:
                        continue
                    per_frame[kk].append({**by_kf[a],
                                          "center": [float(x) for x in cc],
                                          "source": "보간", "points": 0})
                    bridged += 1
        if bridged:
            print(f"  짧은 공백을 메운 박스 {bridged} 개")

    for f in found:
        bs = per_frame[f["keyframe"]]
        kept += len(bs)
        out["frames"].append({"keyframe": f["keyframe"], "scan": f["scan"], "boxes": bs})
    with open(args.out, "w") as fp:
        json.dump(out, fp, indent=1, ensure_ascii=False)
    raw = sum(len(f["boxes"]) for f in found)
    print(f"\n후보 {raw} 개 → 최종 {kept} 개 (원본 {kept - filled} · 전파 {filled})")
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
