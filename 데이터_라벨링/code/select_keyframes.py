"""라벨링할 키프레임을 고른다 — 시간이 아니라 이동 거리 기준.

    python3 select_keyframes.py <세션폴더> --poses <deskew_on> -o <출력.json>

왜 거리 기준인가
----------------
nuScenes 는 2 Hz 로 뽑지만 그쪽은 시속 25~50 km 라 키프레임 간격이 약 5 m 다.
우리는 시속 4 km 여서 2 Hz 로 뽑으면 0.55 m 간격이 되고, 정지 구간에서는
같은 장면이 그대로 중복된다 (session_025 의 경우 51 장 중 12 장).

규칙
    직전 키프레임에서 min_dist 이상 움직였거나 max_gap 초가 지나면 선택.
    거리 조건이 중복을 막고, 시간 조건이 정지 구간도 최소한 담기게 한다.

출력
    키프레임마다 스캔 번호, 시각, ego pose, 그리고 카메라 4 대의 대응 프레임
    (셔터 시각이 가장 가까운 것). 라벨링 툴에 넣을 목록이 된다.
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import load_poses

USB_LATENCY_S = 0.0071


def camera_shutters(session):
    """카메라별 (이름, 폴더, 시리얼, 프레임ID 배열, 셔터 유닉스시각 배열)."""
    with open(os.path.join(session, "meta.json")) as f:
        meta = json.load(f)
    out = []
    for i, c in enumerate(meta["cameras"]):
        folder = f"cam{meta['camera_slots'][i]}"
        path = os.path.join(session, folder, "frames.csv")
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        t = np.array([float(r["host_recv_unix"]) for r in rows])
        out.append(dict(name=c["name"], folder=folder, serial=c["sn"],
                        frame_ids=[int(r["frame_id"]) for r in rows],
                        shutter=t - (c["exposure_us"] / 1e6 + USB_LATENCY_S)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--min-dist", type=float, default=1.0, help="직전 키프레임과의 최소 이동(m)")
    ap.add_argument("--max-gap", type=float, default=3.0, help="안 움직여도 이 초가 지나면 선택")
    args = ap.parse_args()

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    n = min(len(ds), len(poses))
    pos = poses[:n, :3, 3]
    t = ds.scan_times[:n]
    off = ds.host_offset
    cams = camera_shutters(args.session)

    sel, last = [0], 0
    for k in range(1, n):
        if (np.linalg.norm(pos[k] - pos[last]) >= args.min_dist
                or t[k] - t[last] >= args.max_gap):
            sel.append(k)
            last = k

    frames = []
    for k in sel:
        t_lidar_unix = float(t[k] + off)
        entry = {"scan": int(k), "time_unix": t_lidar_unix,
                 "time_rel_s": float(t[k] - t[0]),
                 "ego_pose": poses[k].tolist(), "cameras": {}}
        for c in cams:
            j = int(np.argmin(np.abs(c["shutter"] - t_lidar_unix)))
            entry["cameras"][c["name"]] = {
                "folder": c["folder"], "serial": c["serial"],
                "frame_id": int(c["frame_ids"][j]),
                "shutter_unix": float(c["shutter"][j]),
                "dt_ms": float((c["shutter"][j] - t_lidar_unix) * 1000)}
        frames.append(entry)

    d = np.linalg.norm(np.diff(pos[sel], axis=0), axis=1)
    meta = {"session": os.path.basename(args.session.rstrip("/")),
            "rule": {"min_dist_m": args.min_dist, "max_gap_s": args.max_gap},
            "n_scans": int(n), "n_keyframes": len(sel),
            "duration_s": float(t[n-1] - t[0]),
            "travel_m": float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
            "spacing_m": {"mean": float(d.mean()), "min": float(d.min()),
                          "max": float(d.max())}}
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"meta": meta, "keyframes": frames}, f, indent=1, ensure_ascii=False)

    print(f"{meta['session']}: 스캔 {n} → 키프레임 {len(sel)}")
    print(f"  주행 {meta['travel_m']:.1f} m / {meta['duration_s']:.1f} s")
    print(f"  간격 평균 {d.mean():.2f} m (최소 {d.min():.2f}, 최대 {d.max():.2f})")
    dt = [abs(f["cameras"][c]["dt_ms"]) for f in frames for c in f["cameras"]]
    print(f"  라이다↔카메라 시각차 중앙값 {np.median(dt):.1f} ms, 최대 {max(dt):.1f} ms")
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
