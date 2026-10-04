"""라이다 포인트클라우드에 3D 박스를 그려 영상으로 만든다.

    PYTHONPATH=<kiss-icp 환경> python3 render_lidar_3d.py <세션> \
        --poses <deskew_on> --detections <refined.json> -o out.mp4

카메라 투영(render_boxes_video.py)과 달리 **캘리브레이션을 쓰지 않는다.**
박스도 점도 같은 라이다 좌표계에 있으므로, 가상 카메라 하나만 놓고 그대로
그린다. 캘리브레이션 오차가 끼어들 여지가 없어서 박스 자체의 품질을 보는
데는 이쪽이 맞다.

점 색은 **거리**로 칠한다 (가까울수록 파랑). 링이 또렷해져서 바닥과 물체가
구분된다.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

COLOR = {"car": (80, 255, 80), "truck": (0, 200, 255), "bus": (255, 160, 0),
         "trailer": (255, 160, 0), "construction_vehicle": (200, 120, 255),
         "bollard": (255, 80, 255), "traffic_cone": (0, 140, 255),
         "pedestrian": (255, 255, 80), "cart": (180, 180, 255)}
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
NEAR = 0.5


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def corners_of(center, size, yaw):
    """mmdet3d 관례: center 는 바닥중심, size 는 (길이, 폭, 높이)."""
    l, w, h = size
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    base = np.array([[sx * l / 2, sy * w / 2, sz]
                     for sx, sy, sz in ((1, 1, 0), (1, -1, 0), (-1, -1, 0), (-1, 1, 0),
                                        (1, 1, h), (1, -1, h), (-1, -1, h), (-1, 1, h))])
    return base @ R.T + np.array(center)


class View:
    """ego 좌표계를 바라보는 가상 카메라."""

    def __init__(self, eye, target, w, h, fov_deg=55.0):
        eye = np.asarray(eye, float)
        f = np.asarray(target, float) - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        self.eye, self.R = eye, np.stack([r, -u, f])   # 화면 y 는 아래로
        self.f = (w / 2) / np.tan(np.radians(fov_deg) / 2)
        self.cx, self.cy, self.w, self.h = w / 2, h / 2, w, h

    def cam(self, P):
        return (np.asarray(P, float) - self.eye) @ self.R.T

    def project(self, P):
        c = self.cam(P)
        z = np.maximum(c[:, 2], 1e-6)
        return (np.stack([c[:, 0] / z * self.f + self.cx,
                          c[:, 1] / z * self.f + self.cy], 1), c[:, 2])

    def project_clipped(self, P):
        """박스 꼭짓점용. 카메라 뒤 점은 near 평면으로 당겨서 좌표 폭주를 막는다."""
        c = self.cam(P)
        out = []
        for q in c:
            if q[2] < NEAR:
                q = q * (NEAR / max(q[2], 1e-6)) if q[2] > 0 else None
            if q is None:
                out.append(None)
            else:
                out.append((int(q[0] / q[2] * self.f + self.cx),
                            int(q[1] / q[2] * self.f + self.cy)))
        return out, c[:, 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("--extra", default=None,
                    help="같은 키프레임 구조의 추가 검출 (예: bollards.json). 합쳐 그린다")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--n-sweep", type=int, default=5,
                    help="누적 스윕 수. 16 채널은 한 장이 성기므로 몇 장 겹쳐야 보인다")
    ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--eye", default="-13,0,11", help="가상 카메라 위치 (ego 좌표)")
    ap.add_argument("--target", default="7,0,-1")
    ap.add_argument("--point-size", type=int, default=2)
    ap.add_argument("--color", choices=("range", "height"), default="range",
                    help="점 색 기준. height 는 지면 기준 높이 — 차가 도드라진다")
    ap.add_argument("--drop-ground", type=float, default=None,
                    help="지면 위 이 높이(m) 아래 점을 뺀다. 0.25 쯤이면 바닥이 걷힌다")
    ap.add_argument("--max-height", type=float, default=None,
                    help="지면 위 이 높이(m) 위 점을 뺀다. 실내는 천장이 모든 것 위에 "
                         "있어서, 2.2 쯤으로 잘라야 차가 보인다")
    ap.add_argument("--frame", type=int, default=None,
                    help="이 키프레임 한 장만 그려 이미지로 저장")
    ap.add_argument("--spin", action="store_true",
                    help="한 장면을 한 바퀴 돌며 보여준다 (--frame 과 함께)")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(here)),
                                    "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    det = json.load(open(args.detections))
    if args.extra:
        ex = {f["keyframe"]: f["boxes"] for f in json.load(open(args.extra))["frames"]}
        for f in det["frames"]:
            f["boxes"] = f["boxes"] + ex.get(f["keyframe"], [])
    kf_scan = np.array([f["scan"] for f in det["frames"]])
    W, H = args.width, args.height

    # 지면 높이는 한 번만 구한다 (ego 좌표 z 의 최빈값)
    a0, b0 = ds._spans[min(20, len(ds._spans) - 1)]
    z0 = ds._xyz[a0:b0][:, 2]
    hist, edges = np.histogram(z0[z0 < np.median(z0)], bins=80)
    ground_z = float(edges[int(np.argmax(hist))])
    print(f"지면 높이(ego 좌표) {ground_z:+.2f} m")

    def sweep(k):
        """스캔 k 기준으로 앞 몇 장을 ego 보정해 누적한다."""
        acc = []
        for j in range(max(0, k - args.n_sweep + 1), k + 1):
            a, b = ds._spans[j]
            T = np.linalg.inv(poses[k]) @ poses[j]
            acc.append(ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3])
        return np.vstack(acc)

    def draw(k, view):
        img = np.zeros((H, W, 3), np.uint8)
        P = sweep(k)
        keep = np.linalg.norm(P[:, :2], axis=1) < args.max_range
        if args.drop_ground is not None:
            keep &= P[:, 2] > ground_z + args.drop_ground
        if args.max_height is not None:
            keep &= P[:, 2] < ground_z + args.max_height
        P = P[keep]
        rng = np.linalg.norm(P[:, :2], axis=1)
        uv, z = view.project(P)
        m = (z > NEAR) & (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
        if args.color == "height":
            key = np.clip((P[m][:, 2] - ground_z) / (args.max_height or 2.2), 0, 1) * 255
        else:
            key = np.clip(rng[m] / args.max_range * 255, 0, 255)
        uv, z = uv[m].astype(int), z[m]
        order = np.argsort(-z)                      # 먼 점부터 (화가 알고리즘)
        uv, key = uv[order], key[order]
        col = cv2.applyColorMap(key.astype(np.uint8).reshape(-1, 1),
                                cv2.COLORMAP_JET).reshape(-1, 3)
        # 점 찍기는 벡터화한다. 80 만 점을 파이썬 반복문으로 찍으면 한 프레임에
        # 수십 초가 걸린다. 먼 점부터 정렬돼 있으므로 뒤에 쓰인 값이 남아
        # 가까운 점이 위에 덮인다 (화가 알고리즘).
        s = args.point_size
        off = range(-(s // 2), s // 2 + 1) if s > 1 else (0,)
        for dy in off:
            for dx in off:
                u2 = uv[:, 0] + dx
                v2 = uv[:, 1] + dy
                ok = (u2 >= 0) & (u2 < W) & (v2 >= 0) & (v2 < H)
                img[v2[ok], u2[ok]] = col[ok]

        j = int(np.argmin(np.abs(kf_scan - k)))
        T_now = np.linalg.inv(poses[k]) @ poses[kf_scan[j]]
        boxes = det["frames"][j]["boxes"]
        # 먼 박스부터 그린다
        boxes = sorted(boxes, key=lambda b: -np.hypot(*b["center"][:2]))
        for b in boxes:
            cn = corners_of(b["center"], b["size"], b["yaw"])
            cn = cn @ T_now[:3, :3].T + T_now[:3, 3]
            pts, zc = view.project_clipped(cn)
            if (zc < NEAR).all():
                continue
            c = COLOR.get(b["class"], (255, 255, 255))
            th = 2 if max(b["size"][0], b["size"][1]) > 1.0 else 3   # 볼라드는 굵게
            for e in EDGES:
                p0, p1 = pts[e[0]], pts[e[1]]
                if p0 and p1:
                    cv2.line(img, p0, p1, c, th, cv2.LINE_AA)
            # 앞면을 채워 방향을 보이게
            front = [pts[i] for i in (0, 1, 5, 4)]
            if all(front):
                ov = img.copy()
                cv2.fillPoly(ov, [np.array(front, np.int32)], c)
                cv2.addWeighted(ov, 0.18, img, 0.82, 0, img)
        # 센서 위치
        o, _ = view.project_clipped(np.array([[0, 0, 0]]))
        if o[0]:
            cv2.circle(img, o[0], 6, (255, 255, 255), -1)
        cv2.putText(img, f"session {os.path.basename(args.session)}  scan {k}  "
                         f"keyframe {det['frames'][j]['keyframe']}  box {len(boxes)}",
                    (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    eye = np.array([float(x) for x in args.eye.split(",")])
    target = np.array([float(x) for x in args.target.split(",")])

    if args.frame is not None and not args.spin:
        k = int(kf_scan[args.frame])
        cv2.imwrite(args.out, draw(k, View(eye, target, W, H)))
        print(f"저장: {args.out}")
        return

    if args.spin:
        k = int(kf_scan[args.frame or 0])
        wr = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W, H))
        r = np.hypot(eye[0], eye[1])
        for i in range(120):
            a = 2 * np.pi * i / 120
            wr.write(draw(k, View([r * np.cos(a + np.pi), r * np.sin(a + np.pi), eye[2]],
                                  target, W, H)))
        wr.release()
        print(f"저장: {args.out}")
        return

    view = View(eye, target, W, H)
    wr = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W, H))
    n = len(ds._spans)
    for k in range(n):
        wr.write(draw(k, view))
        if (k + 1) % 50 == 0:
            print(f"  {k+1}/{n}")
    wr.release()
    print(f"{n} 프레임  →  {args.out} ({os.path.getsize(args.out)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
