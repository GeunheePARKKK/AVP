"""검출된 3D 박스를 카메라 4대 영상에 그려 20초 영상으로 만든다.

    PYTHONPATH=<mmdet3d 환경> python3 render_boxes_video.py <세션> \
        --poses <deskew_on> --detections <detections.json> -o out.mp4

검출은 키프레임(28장)에만 있지만 영상은 250프레임이다. 주차된 차는 월드 좌표에
고정이므로, 키프레임의 박스를 ego pose 로 각 프레임의 라이다 좌표계로 옮겨
그린다 (라벨링에서 쓸 '전파' 와 같은 원리).

박스는 mmdet3d 결과를 그대로 쓴다. ReBound 포맷 변환을 거치지 않으므로 그
단계의 관례 차이(size 순서 등)가 끼어들지 않는다.
"""

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np
import yaml

CALIB = "/home/acelab/Documents/Lidar_Camera_Calibration/calib_result/extrinsics_all.yaml"
AVP = "/home/acelab/Documents/TalkFile_AVP_calib"
USB_LATENCY_S = 0.0071
BAYER = {"BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
         "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR}
# 화면 배치: 좌상 전방, 우상 우측, 좌하 좌측, 우하 후방
LAYOUT = [("front", "전방 CAM3"), ("right", "우측 CAM1"),
          ("left", "좌측 CAM2"), ("rear", "후방 CAM4")]
COLOR = {"car": (0, 255, 0), "truck": (0, 200, 255), "bus": (255, 160, 0),
         "trailer": (255, 160, 0), "construction_vehicle": (200, 120, 255)}
NEAR = 0.3   # 카메라 앞 평면 (m)
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def corners_of(center, size, yaw):
    """mmdet3d 관례: center 는 바닥중심, size 는 (길이, 폭, 높이), yaw 는 z 축."""
    l, w, h = size
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    base = np.array([[sx * l / 2, sy * w / 2, sz * h / 2]
                     for sx, sy, sz in ((1, 1, 0), (1, -1, 0), (-1, -1, 0), (-1, 1, 0),
                                        (1, 1, 1), (1, -1, 1), (-1, -1, 1), (-1, 1, 1))])
    base[:, 2] = np.where(base[:, 2] > 0, h, 0.0)
    return base @ R.T + np.array(center)


def enhance(raw, code):
    lo, hi = np.percentile(raw, (0.1, 99.9))
    y = np.clip((raw.astype(np.float32) - lo) / max(hi - lo, 1e-6), 0, 1)
    bgr = cv2.cvtColor((np.power(y, 0.35) * 255).astype(np.uint8), code)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--detections", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--score", type=float, default=0.3)
    ap.add_argument("--max-range", type=float, default=20.0)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--classes", default="car,truck,bus,trailer,construction_vehicle",
                    help="그릴 클래스. 기본은 차량만 — 비차량은 전부 오탐이다 "
                         "(8세션 표본에 보행자 0명, 이륜차 0대)")
    ap.add_argument("--tile-width", type=int, default=960)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(here)),
                                    "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    det = json.load(open(args.detections))
    meta = json.load(open(os.path.join(args.session, "meta.json")))
    cal = yaml.safe_load(open(CALIB))["cameras"]

    # 키프레임별 박스 (라이다 좌표계) 와 그 스캔 번호
    kf_scan = np.array([f["scan"] for f in det["frames"]])
    keep_cls = set(x.strip() for x in args.classes.split(",") if x.strip())
    kf_boxes = [[b for b in f["boxes"]
                 if b["score"] >= args.score
                 and b["class"] in keep_cls
                 and np.hypot(b["center"][0], b["center"][1]) < args.max_range]
                for f in det["frames"]]

    cams = []
    for pos, label in LAYOUT:
        c = cal[pos]
        info = [x for x in meta["cameras"] if x["sn"] == c["serial"]][0]
        folder = f"cam{meta['camera_slots'][meta['cameras'].index(info)]}"
        rows = list(csv.DictReader(open(os.path.join(args.session, folder, "frames.csv"))))
        fs = cv2.FileStorage(os.path.join(AVP, c["intrinsics"]), cv2.FILE_STORAGE_READ)
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat().ravel()
        fs.release()
        cams.append(dict(label=label, folder=folder, info=info, rows=rows,
                         T=np.array(c["T_cam_lidar"]["data"]).reshape(4, 4), K=K, D=D,
                         shutter=np.array([float(r["host_recv_unix"]) for r in rows])
                         - (info["exposure_us"] / 1e6 + USB_LATENCY_S)))

    n = min(len(c["rows"]) for c in cams)
    tw = args.tile_width
    th = int(tw * 1200 / 2048)
    writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"),
                             args.fps, (tw * 2, th * 2))
    print(f"{n} 프레임 · 점수 {args.score} 이상 · {args.max_range:.0f} m 이내 · "
          f"클래스 {sorted(keep_cls)}")

    total = 0
    for i in range(n):
        tiles = []
        for cam in cams:
            t_s = float(cam["shutter"][i]) - ds.host_offset
            k = int(np.argmin(np.abs(ds.scan_times - t_s)))
            j = int(np.argmin(np.abs(kf_scan - k)))          # 가장 가까운 키프레임
            T_now = np.linalg.inv(poses[k]) @ poses[kf_scan[j]]   # 그 키프레임 → 현재

            raw = np.load(os.path.join(args.session, cam["folder"],
                                       f"{int(cam['rows'][i]['frame_id']):08d}.npy"))
            img = enhance(raw, BAYER[cam["info"]["pixel_format"]])
            # 왜곡을 먼저 펴면 이후 투영이 선형이라, 화면 밖으로 나가는 꼭짓점도
            # 좌표가 폭주하지 않는다. 가까운 차의 박스를 그리려면 이게 필요하다.
            img = cv2.undistort(img, cam["K"], cam["D"])
            K = cam["K"]
            h, w = img.shape[:2]
            drawn = 0
            for b in kf_boxes[j]:
                ctr = np.array(b["center"]) @ T_now[:3, :3].T + T_now[:3, 3]
                yaw = b["yaw"] + yaw_of(T_now[:3, :3])
                pts = corners_of(ctr, b["size"], yaw)
                pc = pts @ cam["T"][:3, :3].T + cam["T"][:3, 3]
                if (pc[:, 2] > NEAR).sum() == 0:            # 전부 카메라 뒤면 건너뛴다
                    continue
                if np.linalg.norm(pc.mean(0)) > args.max_range:
                    continue

                def proj(p):
                    u = K[0, 0] * p[0] / p[2] + K[0, 2]
                    v = K[1, 1] * p[1] / p[2] + K[1, 2]
                    return (int(round(u)), int(round(v)))

                col = COLOR.get(b["class"], (255, 255, 255))
                any_on_screen = False
                segs = []
                for a, bb in EDGES:
                    p0, p1 = pc[a], pc[bb]
                    z0, z1 = p0[2], p1[2]
                    if z0 <= NEAR and z1 <= NEAR:
                        continue                              # 양 끝이 카메라 뒤
                    if z0 <= NEAR or z1 <= NEAR:              # 한쪽만 뒤 → 앞 평면에서 자른다
                        t = (NEAR - z0) / (z1 - z0)
                        cut = p0 + t * (p1 - p0)
                        p0, p1 = (cut, p1) if z0 <= NEAR else (p0, cut)
                    q0, q1 = proj(p0), proj(p1)
                    segs.append((q0, q1))
                    if (0 <= q0[0] < w and 0 <= q0[1] < h) or (0 <= q1[0] < w and 0 <= q1[1] < h):
                        any_on_screen = True
                if not any_on_screen:
                    continue
                for q0, q1 in segs:
                    cv2.line(img, q0, q1, col, 3)             # cv2.line 이 화면 경계로 잘라 준다
                drawn += 1
            total += drawn
            tile = cv2.resize(img, (tw, th))
            cv2.rectangle(tile, (0, 0), (tw, 34), (0, 0, 0), -1)
            cv2.putText(tile, f"{cam['label']}  box {drawn}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            tiles.append(tile)
        frame = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        cv2.putText(frame, f"{i/args.fps:5.1f}s", (tw * 2 - 150, th * 2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        writer.write(frame)
        if (i + 1) % 25 == 0 or i + 1 == n:
            print(f"  {i+1}/{n}", flush=True)
    writer.release()
    print(f"박스 연 {total}개  →  {args.out} ({os.path.getsize(args.out)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
