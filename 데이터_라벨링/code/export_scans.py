"""pcap → 스캔별 파일. KISS-ICP 외의 도구로 넘길 때 쓴다.

    python3 export_scans.py ../data/session_013/lidar.pcap -o /tmp/scans
    python3 export_scans.py ../data/session_013/lidar.pcap -o /tmp/scans --fmt bin

    npy  (x, y, z, t) float32 — t 는 스캔 시작 기준 상대 초. 점별 시각이 남는다
    bin  (x, y, z, i) float32 — KITTI 관례. 점별 시각이 사라진다

deskew 를 할 거라면 npy 를 쓴다. bin 은 점별 시각이 없어서 보정이 불가능하고,
KISS-ICP 의 generic/kitti 로더도 이 형식에서는 시각을 복원하지 못한다.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vlp16


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--fmt", choices=("npy", "bin"), default="npy")
    ap.add_argument("--min-points", type=int, default=5000)
    args = ap.parse_args()

    d = vlp16.decode(args.pcap)
    os.makedirs(args.out, exist_ok=True)

    n = 0
    for a, b in d.revolutions():
        if b - a < args.min_points:
            continue
        xyz = d.xyz[a:b]
        if args.fmt == "npy":
            t = d.time[a:b] - d.time[a]
            arr = np.column_stack([xyz, t]).astype(np.float32)
            np.save(os.path.join(args.out, f"{n:05d}.npy"), arr)
        else:
            arr = np.column_stack([xyz, d.intensity[a:b]]).astype(np.float32)
            arr.tofile(os.path.join(args.out, f"{n:05d}.bin"))
        n += 1

    print(f"{n} 개 스캔을 {args.fmt} 로 저장: {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
