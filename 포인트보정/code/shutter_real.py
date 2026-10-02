"""실측 셔터 시각으로 포인트를 보정한다 (Step 3·4·5) — 가정 없이.

    python3 shutter_real.py <세션폴더> --poses ../results/2026-10-02_session_025/deskew_on \
                            -o ../results/2026-10-02_session_025/shutter

deskew_to_shutter.py 와 다른 점
-------------------------------
  deskew_to_shutter.py : 카메라 로그가 없는 session_013 용. 셔터를 0/25/50/75 ms 로 가정.
  이 스크립트          : 세션의 frames.csv 에서 셔터 시각을 읽는다. 카메라가 서로 동기돼
                         있든 아니든 상관없다 — 필요한 건 '언제 찍혔나' 뿐이다.

셔터 시각
    frames.csv 의 host_recv_unix 는 호스트가 이미지를 다 받은 시각이다.
        셔터 ≈ host_recv_unix - (노출시간 + 7.1 ms)      (이 리그 실측)
    상대 시각(프레임끼리, 카메라끼리)은 ±0.5 ms 로 안정적이고, 절대 시각에는
    공통 바이어스가 남는다. 그 바이어스는 4대에 똑같이 걸리므로 카메라 사이
    기하는 흐트러지지 않는다.

시야 방향
    extrinsics_all.yaml 의 T_cam_lidar 에서 계산한다. 카메라 광축은 카메라 좌표계의
    +z 이므로, 라이다 좌표계에서는 R^T·[0,0,1] 이다.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import load_poses, yaw_of
from deskew_to_shutter import pose_at, deskew_to, gather, fov_mask

CALIB = "/home/acelab/Documents/Lidar_Camera_Calibration/calib_result/extrinsics_all.yaml"
USB_LATENCY_S = 0.0071
FOV_HALF = 45.0


def camera_table(session):
    """세션의 카메라별 (이름, 폴더, 시리얼, 시야중심 방위각, 셔터시각 배열)."""
    with open(os.path.join(session, "meta.json")) as f:
        meta = json.load(f)
    with open(CALIB) as f:
        cal = yaml.safe_load(f)["cameras"]
    by_serial = {}
    for pos, c in cal.items():
        R = np.array(c["T_cam_lidar"]["data"]).reshape(4, 4)[:3, :3]
        axis = R.T @ np.array([0.0, 0.0, 1.0])          # 광축을 라이다 좌표계로
        by_serial[c["serial"]] = (pos, float(np.degrees(np.arctan2(axis[1], axis[0])) % 360))

    out = []
    for i, c in enumerate(meta["cameras"]):
        folder = f"cam{meta['camera_slots'][i]}"
        path = os.path.join(session, folder, "frames.csv")
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        t = np.array([float(r["host_recv_unix"]) for r in rows])
        shutter = t - (c["exposure_us"] / 1e6 + USB_LATENCY_S)   # 유닉스
        pos, az = by_serial.get(c["sn"], ("?", 0.0))
        out.append(dict(name=c["name"], folder=folder, serial=c["sn"],
                        position=pos, az=az, shutter=shutter,
                        frame_ids=[int(r["frame_id"]) for r in rows]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--n-eval", type=int, default=12)
    ap.add_argument("--save-frame", type=int, default=None,
                    help="이 카메라 프레임 번호의 클라우드를 npy 로 저장")
    args = ap.parse_args()

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    tend = ds.scan_end_times[:len(poses)]
    n = min(len(ds), len(poses))
    os.makedirs(args.out, exist_ok=True)

    off = ds.host_offset          # 유닉스 → 라이다 시계
    print(f"라이다 시계 ↔ 호스트 오프셋 {off:.6f} s "
          f"(잔차 표준편차 {np.std(ds.scan_host_times - ds.scan_times - off)*1000:.2f} ms)")
    cams = camera_table(args.session)
    print(f"세션 {os.path.basename(args.session)} · 스캔 {n} · 카메라 {len(cams)}대\n")
    base = cams[0]["shutter"]
    for c in cams:
        m = min(len(base), len(c["shutter"]))
        d = (c["shutter"][:m] - base[:m]) * 1000
        print(f"  {c['name']} ({c['position']:5s}, 시야중심 {c['az']:5.1f}°) "
              f"프레임 {len(c['shutter']):3d} · {cams[0]['name']} 대비 {d.mean():+6.1f} ms (±{d.std():.1f})")

    # 평가 스캔 고르기 (직선 / 회전)
    yaws = np.zeros(n)
    for k in range(1, n):
        yaws[k] = yaw_of((np.linalg.inv(poses[k-1]) @ poses[k])[:3, :3])
    turn = [int(k) for k in np.argsort(-np.abs(yaws))[:args.n_eval] if 1 <= k < n-1]
    straight = [int(k) for k in np.argsort(np.abs(yaws))[:args.n_eval] if 2 < k < n-1]

    print(f"\n{'구간':6s} {'카메라':8s} {'시야 안 점':>9s} {'보정량 중앙값':>12s} {'최대':>8s}")
    summary = {}
    for tag, ks in (("직선", straight), ("회전", turn)):
        for c in cams:
            med, mx, npts = [], [], []
            for k in ks:
                # 이 스캔 시각에 가장 가까운 그 카메라의 셔터
                j = int(np.argmin(np.abs(c["shutter"] - off - ds.scan_times[k])))
                t_s = float(c["shutter"][j]) - off
                pts, ta = gather(ds, k, t_s)
                m = fov_mask(pts, c["az"], FOV_HALF) & (np.linalg.norm(pts, axis=1) < 30.0)
                if m.sum() < 100:
                    continue
                p, t = pts[m], ta[m]
                fixed = deskew_to(p, t, poses, tend, t_s)
                d = np.linalg.norm(fixed - p, axis=1)
                med.append(np.median(d)); mx.append(d.max()); npts.append(m.sum())
            if med:
                summary[(tag, c["name"])] = (np.median(med), np.max(mx))
                print(f"{tag:6s} {c['name']:8s} {int(np.mean(npts)):9d} "
                      f"{np.median(med)*100:10.2f} cm {np.max(mx)*100:6.1f} cm")

    # 한 시점의 카메라별 클라우드 저장
    if args.save_frame is not None:
        f = args.save_frame
        for c in cams:
            if f >= len(c["shutter"]):
                continue
            t_s = float(c["shutter"][f]) - off
            k = int(np.argmin(np.abs(ds.scan_times - t_s)))
            pts, ta = gather(ds, k, t_s)
            fixed = deskew_to(pts, ta, poses, tend, t_s)
            np.save(os.path.join(args.out, f"frame{f:05d}_{c['name']}.npy"),
                    fixed.astype(np.float32))
            np.save(os.path.join(args.out, f"frame{f:05d}_{c['name']}_raw.npy"),
                    pts.astype(np.float32))
        print(f"\n프레임 {f} 의 카메라별 클라우드 저장: {args.out}")


if __name__ == "__main__":
    main()
