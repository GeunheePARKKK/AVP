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
    ap.add_argument("--no-keep-raw", action="store_true",
                    help="원본 검출 기하를 쓰지 않고 월드 평균 박스를 쓴다 (권장하지 않음)")
    ap.add_argument("--no-snap", action="store_true",
                    help="프레임마다 위치를 그 프레임 점에 맞추는 단계 생략")
    ap.add_argument("--no-shape-check", action="store_true",
                    help="차 모양인지 검사하는 단계 생략")
    ap.add_argument("--max-vertical", type=float, default=1.0,
                    help="차 지붕 위 빈 구간의 점 / 차 높이대 점 비율이 이보다 크면 "
                         "수직 구조물(기둥·벽)로 본다. 실측상 차는 0.0~0.5, 기둥은 3 이상")
    ap.add_argument("--min-car-points", type=int, default=50,
                    help="차 높이대(지면+0.3~2.0 m) 점이 이보다 적으면 버린다")
    ap.add_argument("--max-above", type=float, default=0.7,
                    help="차 지붕 위(지면+1.95~2.8 m) 점 / 몸통대(0.3~1.8 m) 점. "
                         "실측상 차는 0.00~0.57, 벽·난간에 걸린 박스는 0.76 이상")
    ap.add_argument("--min-roof-area", type=float, default=0.05,
                    help="지붕대(지면+1.15~1.8 m) 점이 박스 바닥면에서 차지하는 비율. "
                         "차는 0.08 이상, 지붕이 없는 구조물은 0.02 이하")
    ap.add_argument("--max-yaw-dev", type=float, default=30.0,
                    help="한 트랙 안에서 프레임별 방향이 트랙 합의에서 이만큼(도) 넘게 "
                         "벗어나면 합의 방향으로 돌린다. 주차된 차는 회전하지 않는다")
    ap.add_argument("--max-roof-std", type=float, default=0.42,
                    help="박스 윗면 높이의 표준편차(m). 차는 지붕이 평평해 0.20~0.35, "
                         "계단·경사로는 0.47 이상. 격자 셀 35 개 이상일 때만 적용")
    ap.add_argument("--max-row-angle", type=float, default=60.0,
                    help="주변 주차열 방향과 이만큼(도) 넘게 어긋나면 버린다. "
                         "실측: 정상 0~41 도, 90 도 뒤집힌 박스 87~88 도")
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
    kf_scan = {f["keyframe"]: f["scan"] for f in frames}

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
                            score=b["score"], cls=b["class"],
                            raw=dict(center=list(b["center"]), size=list(b["size"]),
                                     yaw=float(b["yaw"]))))
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
        raw_by_kf = {}
        for i in g:                                     # 한 프레임에 둘이면 점수 높은 쪽
            k = obs[i]["kf"]
            if k not in raw_by_kf or obs[i]["score"] > raw_by_kf[k][1]:
                raw_by_kf[k] = (obs[i]["raw"], obs[i]["score"])
        tracks.append(dict(id=f"T{n:03d}", center=center, size=size, yaw=yaw,
                           cls=cls, views=views, raw_by_kf=raw_by_kf,
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
            # center[2] 는 **바닥중심**이다 (mmdet3d 관례, to_rebound.py:176 참고).
            # 예전에는 중심으로 착각해 z 를 -h/2 부터 잡아서 **지면 점**이 섞여
            # 들어왔다. 지면이 섞이면 minAreaRect 가 차 아래 바닥까지 감싸면서
            # 사각형이 커지고 중심이 밀린다.
            near = P[(np.abs(loc[:, 0]) < l / 2 + 0.3)
                     & (np.abs(loc[:, 1]) < wd / 2 + 0.3)
                     & (P[:, 2] > t["center"][2] + 0.3)
                     & (P[:, 2] < t["center"][2] + h)]
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

            # 주차된 차는 한쪽 면만 보여서 minAreaRect 의 폭이 얇게 나온다.
            # 보이는 면을 그대로 두고 **센서 반대쪽**으로 늘린다.
            #
            # 센서 위치를 월드 원점으로 착각해 늘리는 방향을 틀리게 잡은 적이 있다.
            # 센서는 매 스캔 움직이므로, 이 트랙을 관측한 프레임들의 **센서 위치
            # 평균**을 기준으로 삼는다.
            ca, sa = np.cos(new_yaw), np.sin(new_yaw)
            axis_l = np.array([ca, sa])
            axis_w = np.array([-sa, ca])
            c2 = np.array([cx, cy])
            sensor = np.mean([poses[kf_scan[k]][:3, 3][:2] for k in t["kfs"] if k in kf_scan], axis=0)
            to_sensor = sensor - c2
            if W < 1.6:
                grow = 1.8 - W
                away = -np.sign(np.dot(to_sensor, axis_w))     # 센서 반대쪽
                if away == 0:
                    away = 1.0
                c2 = c2 + axis_w * (away * grow / 2)
                W = 1.8
            if L < 3.6:
                grow = 4.3 - L
                away = -np.sign(np.dot(to_sensor, axis_l))
                if away == 0:
                    away = 1.0
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
    snapped = 0
    kept_raw = 0
    for f in frames:
        k = f["scan"]
        Tin = np.linalg.inv(poses[k])
        a, b = ds._spans[k]
        pts_local = ds._xyz[a:b]
        boxes = []
        for t in tracks:
            # ── 기하는 원본 검출에서 가져온다 ───────────────────────────
            # 월드 평균은 ego pose(오도메트리) 오차를 박스에 섞어 넣는다. 원본
            # 검출은 그 프레임의 점으로 그 프레임 좌표에서 계산된 것이라 그런
            # 오차가 없다. 다듬기는 **무엇이 진짜 물체인지 고르고 트랙을 잇는
            # 데만** 쓰고, 박스 기하는 원본을 그대로 쓴다. 원본이 없는
            # 프레임만 평균 박스를 전파해 채운다.
            raw = t["raw_by_kf"].get(f["keyframe"])
            if raw and not args.no_keep_raw:
                r = raw[0]
                c = np.array(r["center"], float)
                l, wd, h = (float(x) for x in r["size"])
                yaw = float(r["yaw"])
                src = "원본"
            else:
                c = t["center"] @ Tin[:3, :3].T + Tin[:3, 3]
                yaw = t["yaw"] + yaw_of(Tin[:3, :3])
                l, wd, h = (float(x) for x in t["size"])
                src = "전파"
            if np.hypot(c[0], c[1]) >= args.max_range:
                continue

            def local_xy(center, ang):
                ca_, sa_ = np.cos(-ang), np.sin(-ang)
                d_ = pts_local[:, :2] - center[:2]
                return np.stack([d_[:, 0] * ca_ - d_[:, 1] * sa_,
                                 d_[:, 0] * sa_ + d_[:, 1] * ca_], axis=1)

            if src == "전파" and not args.no_snap:
                # 전파한 박스만, 그 프레임 점에 위치를 맞춘다.
                # 원본 박스는 이미 그 프레임 점으로 만든 것이라 손대지 않는다.
                loc = local_xy(c, yaw)
                sel = ((np.abs(loc[:, 0]) < l / 2 + 0.5)
                       & (np.abs(loc[:, 1]) < wd / 2 + 0.5)
                       & (pts_local[:, 2] > c[2] + 0.3)
                       & (pts_local[:, 2] < c[2] + h))
                if sel.sum() >= 20:
                    lp = loc[sel]
                    axis = ((np.cos(yaw), np.sin(yaw)), (-np.sin(yaw), np.cos(yaw)))
                    d = []
                    for ax in (0, 1):
                        dim = l if ax == 0 else wd
                        lo2, hi2 = np.percentile(lp[:, ax], (2, 98))
                        if hi2 - lo2 > 0.8 * dim:
                            d.append((lo2 + hi2) / 2)     # 양끝이 다 보인다
                        else:
                            sgn = np.sign(np.dot(-c[:2], axis[ax])) or 1.0
                            d.append((hi2 if sgn > 0 else lo2) - sgn * dim / 2)
                    dl = float(np.clip(d[0], -0.6, 0.6))
                    dw = float(np.clip(d[1], -0.6, 0.6))
                    ca2, sa2 = np.cos(yaw), np.sin(yaw)
                    c = c.copy()
                    c[0] += dl * ca2 - dw * sa2
                    c[1] += dl * sa2 + dw * ca2
                    snapped += 1

            # 그 프레임에서 실제로 점이 보이는지
            loc = local_xy(c, yaw)
            n_in = int(((np.abs(loc[:, 0]) < l / 2) & (np.abs(loc[:, 1]) < wd / 2)
                        & (pts_local[:, 2] > c[2]) & (pts_local[:, 2] < c[2] + h)).sum())
            if n_in < args.min_points:
                continue
            if src == "원본":
                kept_raw += 1
            else:
                filled += 1

            boxes.append({"class": t["cls"], "score": round(t["score"], 3),
                          "center": [float(x) for x in c],
                          "size": [l, wd, h],
                          "yaw": float(yaw), "track_id": t["id"],
                          "views": t["views"], "points": n_in, "source": src})
        total += len(boxes)
        out["frames"].append({"keyframe": f["keyframe"], "scan": k, "boxes": boxes})

    # ---------- 6.5 트랙 안에서 방향 맞추기 ----------
    # 박스 기하를 원본 검출에서 가져오므로, 검출기가 한두 프레임에서 방향을
    # 90 도 틀리면 그대로 나간다. session_031 T016 은 16.1~18.2 초 4 프레임에서
    # 80 도 돌아갔다 (크기는 그대로, 점은 오히려 그때 가장 많았다).
    #
    # **주차된 차는 회전하지 않는다.** 트랙 안에서 방향이 튀면 합의 방향으로
    # 되돌린다. 합의는 이상치에 끌려가지 않도록 한 번 걸러 다시 구한다.
    fixed_yaw = 0
    if args.max_yaw_dev > 0:
        per = {}
        for fi, f in enumerate(out["frames"]):
            yz = yaw_of(poses[f["scan"]][:3, :3])
            for bi, b in enumerate(f["boxes"]):
                per.setdefault(b["track_id"], []).append((fi, bi, b["yaw"] + yz))
        lim = np.radians(args.max_yaw_dev)
        for tid, v in per.items():
            if len(v) < 4:
                continue
            ys = np.array([x[2] for x in v])
            cons = mean_yaw_mod180(ys)
            dev = np.abs(wrap(2 * (ys - cons))) / 2
            good = dev < np.radians(45)
            if good.sum() >= 3 and good.sum() < len(ys):
                cons = mean_yaw_mod180(ys[good])      # 이상치를 빼고 다시
                dev = np.abs(wrap(2 * (ys - cons))) / 2
            # 180 도 뒤집힘은 같은 상자지만, 트랙 안에서 앞뒤가 뒤죽박죽이면
            # 보기에 거슬린다. 멀쩡한 프레임들의 앞 방향에 맞춘다.
            gy = ys[good] if good.any() else ys
            full = float(np.arctan2(np.sin(gy).mean(), np.cos(gy).mean()))
            cons_w = cons if abs(wrap(cons - full)) < np.pi / 2 else cons + np.pi
            for (fi, bi, y), d in zip(v, dev):
                if d <= lim:
                    continue
                yz = yaw_of(poses[out["frames"][fi]["scan"]][:3, :3])
                b = out["frames"][fi]["boxes"][bi]
                b["yaw"] = float(wrap(cons_w - yz))
                b["yaw_fixed"] = round(float(np.degrees(d)), 1)
                fixed_yaw += 1
        if fixed_yaw:
            print(f"→ 트랙 합의 방향으로 되돌린 박스 {fixed_yaw} 개")

    # ---------- 7. 내보낼 박스로 다시 모양 검사 ----------
    # 5.5 단계 검사는 **월드 평균 박스**를 본다. 그런데 실제로 나가는 것은
    # 6 단계에서 원본 검출 기하로 바뀐 박스다. 둘이 다르면 검사를 통과한
    # 박스가 엉뚱한 곳에 놓인 채 나간다 — session_031 에서 벽·난간에 걸친
    # 박스 3 개가 이렇게 빠져나갔다. 그래서 **내보낼 박스 자체**를 다시 잰다.
    if not args.no_shape_check:
        wp2 = []
        for f in frames:
            a, b = ds._spans[f["scan"]]
            T = poses[f["scan"]]
            wp2.append(ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3])
        PW2 = np.vstack(wp2)
        z2 = PW2[:, 2]
        hh2, ee2 = np.histogram(z2[z2 < np.median(z2)], bins=100)
        gnd = float(ee2[int(np.argmax(hh2))])

        per_track = {}
        for f in out["frames"]:
            T = poses[f["scan"]]
            for b in f["boxes"]:
                c = np.array(b["center"]) @ T[:3, :3].T + T[:3, 3]
                yw = b["yaw"] + yaw_of(T[:3, :3])
                per_track.setdefault(b["track_id"], []).append((c, b["size"], yw))

        drop = {}
        keep_pose = {}
        for tid, v in per_track.items():
            c, size, yw = v[len(v) // 2]              # 중간 프레임을 대표로
            l, wd, h = size
            d = PW2[:, :2] - c[:2]
            ca, sa = np.cos(-yw), np.sin(-yw)
            lo = np.stack([d[:, 0] * ca - d[:, 1] * sa,
                           d[:, 0] * sa + d[:, 1] * ca], axis=1)
            zr = z2 - gnd
            foot = (np.abs(lo[:, 0]) < l / 2) & (np.abs(lo[:, 1]) < wd / 2)
            body = foot & (zr > 0.3) & (zr < 1.8)
            nb = int(body.sum())
            if nb < 30:
                continue                              # 점이 없으면 판단 보류
            above = (foot & (zr > 1.95) & (zr < 2.8)).sum() / nb
            roof = foot & (zr > 1.15) & (zr < 1.8)
            area = (len(np.unique(np.round(lo[roof] / 0.2).astype(int), axis=0))
                    * 0.04 / (l * wd)) if roof.sum() > 20 else 0.0
            # 윗면 평탄도 — 차는 지붕이 거의 수평이고, 계단·경사로는 올라간다.
            # 격자 셀이 적으면 측정이 흔들리므로 35 개 이상일 때만 본다.
            # 몸통대(0.3~1.8 m)로 재면 계단이 올라가는 부분이 잘려 신호가 사라진다.
            # 평탄도만 2.0 m 까지 본다.
            flat = foot & (zr > 0.3) & (zr < 2.0)
            cell = np.round(lo[flat] / 0.35).astype(int)
            uk, inv = np.unique(cell, axis=0, return_inverse=True)
            roof_std = 0.0
            if len(uk) >= 35:
                tops = np.zeros(len(uk))
                zb = zr[flat]
                for ci in range(len(uk)):
                    tops[ci] = zb[inv == ci].max()
                roof_std = float(tops.std())

            if os.environ.get("REFINE_DEBUG"):
                print(f"     [디버그] {tid} above {above:.2f} area {area:.2f} "
                      f"roof_std {roof_std:.3f} cells {len(uk)}")
            if above > args.max_above:
                drop[tid] = f"지붕 위에 점이 많다 {above:.2f}"
            elif area < args.min_roof_area:
                drop[tid] = f"지붕이 없다 {area:.2f}"
            elif roof_std > args.max_roof_std:
                drop[tid] = f"윗면이 평평하지 않다 {roof_std:.2f} (계단·경사로)"
            else:
                keep_pose[tid] = (c[:2], yw)
        # 주변 주차열 방향과 90 도 어긋난 박스. 주차된 차는 이웃과 나란하다.
        # 방향은 180 도 주기로 본다 (앞뒤 뒤집힘은 같은 박스, 90 도는 다른 박스).
        if len(keep_pose) >= 5:
            ids2 = sorted(keep_pose)
            for tid in ids2:
                c2, y2 = keep_pose[tid]
                nb = [keep_pose[o][1] for o in ids2
                      if o != tid and np.linalg.norm(keep_pose[o][0] - c2) < 12.0]
                if len(nb) < 4:
                    continue                       # 이웃이 적으면 판단 보류
                dd = 2 * np.asarray(nb)
                med = 0.5 * np.arctan2(np.sin(dd).mean(), np.cos(dd).mean())
                diff = abs(np.degrees(wrap(2 * (y2 - med)))) / 2
                if diff > args.max_row_angle:
                    drop[tid] = f"주차열과 {diff:.0f} 도 어긋남"

        if drop:
            for f in out["frames"]:
                f["boxes"] = [b for b in f["boxes"] if b["track_id"] not in drop]
            total = sum(len(f["boxes"]) for f in out["frames"])
            out["n_tracks"] = len(per_track) - len(drop)
            print(f"→ 내보낼 박스 재검사: {len(drop)} 개 트랙 탈락")
            for tid, why in sorted(drop.items()):
                print(f"     {tid}: {why}")
        else:
            print("→ 내보낼 박스 재검사: 탈락 없음")

    with open(args.out, "w") as fp:
        json.dump(out, fp, indent=1, ensure_ascii=False)
    before = len(obs)
    print(f"\n박스 {before} → {total} 개 (프레임당 {before/len(frames):.1f} → {total/len(frames):.1f})")
    print(f"  트랙 {out['n_tracks']} 개, 검출이 빠졌던 자리를 채운 것 {filled} 개")
    print(f"  원본 기하 {kept_raw} 개 · 전파로 채운 것 {filled} 개 (점에 맞춘 것 {snapped} 개)")
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
