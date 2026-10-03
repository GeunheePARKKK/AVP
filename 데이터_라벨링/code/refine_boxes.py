"""검출 초안을 시간 일관성으로 다듬는다.

    python3 refine_boxes.py <세션> --poses <deskew_on> --detections <detections.json> \
        -o <refined.json>

왜 필요한가
-----------
검출기는 키프레임마다 따로 돌아서, 같은 차가 프레임마다 다른 박스로 나오고
어떤 프레임에서는 아예 빠진다. 차는 월드 좌표에 고정인데도 그렇다 — 시점·가림·
거리가 바뀌면서 점 패턴이 달라지고, 점수가 임계값 근처를 들락거리기 때문이다.

session_031 실측: 박스 690 개가 실제로는 물체 71 개였다. 1 장에만 나타난 군집은
평균 점수 0.16, 10 장 이상 나타난 군집은 0.33 이었다.

무엇을 하는가
-------------
1. 모든 박스를 ego pose 로 **월드 좌표**에 모은다
2. 가까운 것끼리 묶는다 (한 군집 = 물체 하나)
3. **관측이 적은 군집을 버린다** — 깜빡이는 것은 대개 오탐이다
4. 군집의 크기·방향·위치를 **관측 전체로 평균**낸다. 흔들림이 사라진다
5. 박스 안의 점에 **다시 맞춰** 크기와 방향을 다듬는다 (선택)
6. 각 군집을 **모든 키프레임에 다시 뿌린다**. 검출이 빠졌던 프레임도 채워진다
7. 군집마다 **트랙 ID** 를 준다

한계
----
**정지 물체를 전제한다.** 움직이는 차가 있으면 궤적을 따라 여러 군집으로 쪼개지거나
엉뚱하게 묶인다. 지하 주차장에서는 거의 전부 정지 상태라 문제가 적지만, 주행 중인
차가 있는 데이터에는 그대로 쓰면 안 된다.

결과는 여전히 **초안**이다. 사람 검수를 대체하지 않는다.
"""

import argparse
import json
import os
import sys

import numpy as np

VEHICLE = {"car", "truck", "bus", "trailer", "construction_vehicle"}


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def mean_yaw_mod180(yaws):
    """차 박스는 180° 돌려도 같은 상자다. 2배각 평균으로 다룬다."""
    d = 2 * np.asarray(yaws)
    return 0.5 * np.arctan2(np.sin(d).mean(), np.cos(d).mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--score", type=float, default=0.1)
    ap.add_argument("--max-range", type=float, default=20.0)
    ap.add_argument("--link-iou", type=float, default=0.3,
                    help="BEV IoU 가 이보다 크면 같은 물체로 묶는다")
    ap.add_argument("--link-dist", type=float, default=2.5,
                    help="묶기 후보를 추릴 거리 (m). IoU 계산을 줄이기 위한 것")
    ap.add_argument("--min-views", type=int, default=3,
                    help="이 횟수 미만으로 관측된 군집은 버린다")
    ap.add_argument("--min-points", type=int, default=5,
                    help="박스 안 점이 이보다 적으면 그 프레임에서는 빼놓는다")
    ap.add_argument("--no-refit", action="store_true", help="점에 맞춰 다시 재는 단계 생략")
    ap.add_argument("--no-shape-check", action="store_true",
                    help="차 모양인지 검사하는 단계 생략")
    ap.add_argument("--max-vertical", type=float, default=1.0,
                    help="차 지붕 위 빈 구간의 점 / 차 높이대 점 비율이 이보다 크면 "
                         "수직 구조물(기둥·벽)로 본다. 실측상 차는 0.0~0.5, 기둥은 3 이상")
    ap.add_argument("--min-car-points", type=int, default=50,
                    help="차 높이대(지면+0.3~2.0 m) 점이 이보다 적으면 버린다")
    ap.add_argument("--continue-ratio", type=float, default=0.6,
                    help="박스 양 끝 너머로 점이 이만큼 이어지면 벽으로 본다")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(here)),
                                    "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    det = json.load(open(args.detections))
    frames = det["frames"]

    # ---------- 1. 월드 좌표로 모은다 ----------
    obs = []
    for f in frames:
        T = poses[f["scan"]]
        for b in f["boxes"]:
            if b["class"] not in VEHICLE or b["score"] < args.score:
                continue
            if np.hypot(b["center"][0], b["center"][1]) >= args.max_range:
                continue
            c = np.array(b["center"]) @ T[:3, :3].T + T[:3, 3]
            obs.append(dict(kf=f["keyframe"], scan=f["scan"], center=c,
                            size=np.array(b["size"]), yaw=b["yaw"] + yaw_of(T[:3, :3]),
                            score=b["score"], cls=b["class"]))
    print(f"검출 {len(obs)} 개를 월드 좌표로 모았다")

    # ---------- 2. 묶는다 ----------
    # 중심 거리만으로는 같은 차의 흔들림(최대 1 m)과 옆 차와의 간격(2.5 m)을
    # 깔끔하게 가르지 못한다. 박스가 실제로 겹치는지(BEV IoU)로 판단한다.
    import cv2 as _cv

    def _rect(o):
        return ((float(o["center"][0]), float(o["center"][1])),
                (float(o["size"][0]), float(o["size"][1])),
                float(np.degrees(o["yaw"])))

    def _iou(a, b):
        r = _cv.rotatedRectangleIntersection(_rect(a), _rect(b))
        if r[0] == 0 or r[1] is None:
            return 0.0
        inter = _cv.contourArea(_cv.convexHull(r[1]))
        aa = a["size"][0] * a["size"][1]
        bb = b["size"][0] * b["size"][1]
        return float(inter / max(aa + bb - inter, 1e-9))

    cent = np.array([o["center"][:2] for o in obs])
    used = np.zeros(len(obs), bool)
    clusters = []
    for i in range(len(obs)):
        if used[i]:
            continue
        grp = [i]
        used[i] = True
        head = 0
        while head < len(grp):                      # 연쇄로 이어 붙인다
            d = np.linalg.norm(cent - cent[grp[head]], axis=1)
            for j in np.where((d < args.link_dist) & (~used))[0]:
                if _iou(obs[grp[head]], obs[int(j)]) >= args.link_iou:
                    grp.append(int(j))
                    used[j] = True
            head += 1
        clusters.append(grp)
    print(f"→ 군집 {len(clusters)} 개")

    # ---------- 3. 관측이 적은 것 버리기 ----------
    kept = []
    for g in clusters:
        views = len(set(obs[i]["kf"] for i in g))
        if views >= args.min_views:
            kept.append((g, views))
    dropped = len(clusters) - len(kept)
    print(f"→ {args.min_views} 회 미만 관측 {dropped} 개 버림, {len(kept)} 개 남음")

    # ---------- 4. 평균내기 ----------
    tracks = []
    for n, (g, views) in enumerate(kept):
        w = np.array([obs[i]["score"] for i in g])          # 점수로 가중
        w = w / w.sum()
        center = sum(w[k] * obs[i]["center"] for k, i in enumerate(g))
        size = sum(w[k] * obs[i]["size"] for k, i in enumerate(g))
        yaw = mean_yaw_mod180([obs[i]["yaw"] for i in g])
        cls = max(set(obs[i]["cls"] for i in g),
                  key=lambda c: sum(obs[i]["score"] for i in g if obs[i]["cls"] == c))
        tracks.append(dict(id=f"T{n:03d}", center=center, size=size, yaw=yaw,
                           cls=cls, views=views,
                           score=float(np.mean([obs[i]["score"] for i in g])),
                           kfs=sorted(set(obs[i]["kf"] for i in g))))

    # ---------- 5. 점에 맞춰 다시 재기 ----------
    if not args.no_refit:
        import cv2
        world_pts = []
        for f in frames:
            k = f["scan"]
            a, b = ds._spans[k]
            T = poses[k]
            world_pts.append(ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3])
        P = np.vstack(world_pts)
        refit = 0
        for t in tracks:
            l, wd, h = t["size"]
            # 사각 영역이 아니라 **박스 안쪽**(조금 넉넉히) 점만 쓴다.
            # 사각 영역을 쓰면 옆 차와 벽이 섞여 들어와 크기가 터무니없이 커진다.
            d2 = P[:, :2] - t["center"][:2]
            ca, sa = np.cos(-t["yaw"]), np.sin(-t["yaw"])
            loc = np.stack([d2[:, 0] * ca - d2[:, 1] * sa,
                            d2[:, 0] * sa + d2[:, 1] * ca], axis=1)
            near = P[(np.abs(loc[:, 0]) < l / 2 + 0.3)
                     & (np.abs(loc[:, 1]) < wd / 2 + 0.3)
                     & (P[:, 2] > t["center"][2] - h / 2 + 0.25)
                     & (P[:, 2] < t["center"][2] + h / 2)]
            if len(near) < 80:
                continue
            rect = cv2.minAreaRect(near[:, :2].astype(np.float32))
            (cx, cy), (w1, h1), ang = rect
            L, W = (max(w1, h1), min(w1, h1))
            new_yaw = np.radians(ang + (0 if w1 >= h1 else 90))
            if abs(wrap(2 * (new_yaw - t["yaw"]))) / 2 > np.radians(35):
                continue                                      # 방향이 크게 다르면 신뢰 안 함
            if not (2.0 <= L <= 7.0):
                continue

            # 주차된 차는 한쪽 면만 보이는 경우가 많다. 그때 minAreaRect 는 폭이
            # 얇게(0.3~1.0 m) 나온다. 보이는 면은 그대로 두고 **안 보이는 쪽으로**
            # 차 폭만큼 늘린다. 길이도 같은 이유로 너무 짧으면 늘린다.
            ca, sa = np.cos(new_yaw), np.sin(new_yaw)
            axis_l = np.array([ca, sa])        # 길이 방향
            axis_w = np.array([-sa, ca])       # 폭 방향
            c2 = np.array([cx, cy])
            if W < 1.6:
                grow = 1.8 - W
                # 센서(원점 근처)에서 먼 쪽으로 늘린다
                side = np.sign(np.dot(c2 - t["center"][:2] * 0, axis_w)) or 1.0
                away = -np.sign(np.dot(-c2, axis_w)) or 1.0
                c2 = c2 + axis_w * (away * grow / 2)
                W = 1.8
            if L < 3.6:
                grow = 4.3 - L
                away = -np.sign(np.dot(-c2, axis_l)) or 1.0
                c2 = c2 + axis_l * (away * grow / 2)
                L = 4.3
            if not (3.0 <= L <= 7.0 and 1.4 <= W <= 2.6):
                continue
            t["center"][0], t["center"][1] = float(c2[0]), float(c2[1])
            t["size"] = np.array([L, W, t["size"][2]])
            t["yaw"] = new_yaw
            refit += 1
        print(f"→ 점에 맞춰 다시 잰 군집 {refit} / {len(tracks)} 개")

    # ---------- 5.5 차 모양인지 검사 ----------
    # 벽이나 기둥은 여러 프레임에서 일관되게 검출되어 관측 횟수로는 걸러지지 않는다.
    # 모양으로 가린다.
    #   (a) 지면 위 2.4 m 를 넘는 점이 많다      → 벽·기둥·천장
    #   (b) 박스 양 끝 너머로 점이 그대로 이어진다 → 연속 구조물(벽)
    if not args.no_shape_check:
        wp = []
        for f in frames:
            k = f["scan"]
            a, b = ds._spans[k]
            T = poses[k]
            wp.append(ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3])
        PW = np.vstack(wp)
        zs = PW[:, 2]
        hist, edges = np.histogram(zs[zs < np.median(zs)], bins=100)
        ground = float(edges[int(np.argmax(hist))])
        hi = zs[zs > ground + 2.0]
        if len(hi) > 1000:
            h2, e2 = np.histogram(hi, bins=60)
            ceiling = float(e2[int(np.argmax(h2))]) - ground
        else:
            ceiling = 3.0
        print(f"지면 높이(월드) {ground:+.2f} m · 천장 높이 {ceiling:.2f} m")

        def local(pts, t):
            d = pts[:, :2] - t["center"][:2]
            ca, sa = np.cos(-t["yaw"]), np.sin(-t["yaw"])
            return np.stack([d[:, 0] * ca - d[:, 1] * sa,
                             d[:, 0] * sa + d[:, 1] * ca], axis=1)

        good, bad = [], []
        for t in tracks:
            l, wd, h = t["size"]
            sel = PW[(np.abs(PW[:, 0] - t["center"][0]) < l + 3)
                     & (np.abs(PW[:, 1] - t["center"][1]) < l + 3)]
            if len(sel) < 50:
                bad.append((t, "점 부족"))
                continue
            loc = local(sel, t)
            foot = (np.abs(loc[:, 0]) < l / 2) & (np.abs(loc[:, 1]) < wd / 2)
            zrel = sel[:, 2] - ground
            inside = foot & (zrel > 0.3) & (zrel < 2.0)      # 차 높이대
            if inside.sum() < args.min_car_points:
                bad.append((t, "점부족"))
                continue
            # 차 지붕(약 2 m)과 천장(약 3 m) 사이는 비어 있어야 한다.
            # 기둥·벽은 그 구간이 차 있다. 실내라 천장은 모든 것 위에 있으므로
            # 단순히 '높은 점이 있는가' 로는 가릴 수 없다.
            gap = foot & (zrel > 2.0) & (zrel < ceiling - 0.35)
            tall = gap.sum() / max(inside.sum(), 1)
            if tall > args.max_vertical:
                bad.append((t, f"수직구조물 {tall:.1f}"))
                continue
            # 양 끝 너머 1.5 m 구간에 점이 이어지는가
            ext = ((np.abs(loc[:, 1]) < wd / 2)
                   & (np.abs(loc[:, 0]) > l / 2) & (np.abs(loc[:, 0]) < l / 2 + 1.5)
                   & (sel[:, 2] - ground > 0.3) & (sel[:, 2] - ground < 1.8))
            ratio = ext.sum() / max(inside.sum(), 1)
            if ratio > args.continue_ratio:
                bad.append((t, f"벽처럼 이어짐 {ratio:.1f}"))
                continue
            good.append(t)
        print(f"→ 모양 검사: {len(good)} 개 통과, {len(bad)} 개 탈락")
        from collections import Counter
        for r, n in Counter(b[1].split()[0] for b in bad).most_common():
            print(f"     {r}: {n} 개")
        tracks = good

    # ---------- 6. 모든 키프레임에 다시 뿌리기 ----------
    out = {"classes": det.get("classes"), "source": "refine_boxes.py",
           "params": vars(args), "n_tracks": len(tracks), "frames": []}
    filled = 0
    total = 0
    for f in frames:
        k = f["scan"]
        Tin = np.linalg.inv(poses[k])
        a, b = ds._spans[k]
        pts_local = ds._xyz[a:b]
        boxes = []
        for t in tracks:
            c = t["center"] @ Tin[:3, :3].T + Tin[:3, 3]
            if np.hypot(c[0], c[1]) >= args.max_range:
                continue
            yaw = t["yaw"] + yaw_of(Tin[:3, :3])
            l, wd, h = t["size"]
            # 그 프레임에서 실제로 점이 보이는지
            R2 = np.array([[np.cos(-yaw), -np.sin(-yaw)], [np.sin(-yaw), np.cos(-yaw)]])
            loc = (pts_local[:, :2] - c[:2]) @ R2.T
            n_in = int(((np.abs(loc[:, 0]) < l / 2) & (np.abs(loc[:, 1]) < wd / 2)
                        & (pts_local[:, 2] > c[2]) & (pts_local[:, 2] < c[2] + h)).sum())
            if n_in < args.min_points:
                continue
            if f["keyframe"] not in t["kfs"]:
                filled += 1
            boxes.append({"class": t["cls"], "score": round(t["score"], 3),
                          "center": [float(x) for x in c],
                          "size": [float(x) for x in t["size"]],
                          "yaw": float(yaw), "track_id": t["id"],
                          "views": t["views"], "points": n_in})
        total += len(boxes)
        out["frames"].append({"keyframe": f["keyframe"], "scan": k, "boxes": boxes})

    with open(args.out, "w") as fp:
        json.dump(out, fp, indent=1, ensure_ascii=False)
    before = len(obs)
    print(f"\n박스 {before} → {total} 개 (프레임당 {before/len(frames):.1f} → {total/len(frames):.1f})")
    print(f"  트랙 {len(tracks)} 개, 검출이 빠졌던 자리를 채운 것 {filled} 개")
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
