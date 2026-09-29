"""session_013 pcap 에 KISS-ICP 를 돌린다. deskew 켠 상태가 기본.

    python3 run_kiss_icp.py ../data/session_013/lidar.pcap
    python3 run_kiss_icp.py ../data/session_013/lidar.pcap --no-deskew   # 비교용
    python3 run_kiss_icp.py ../data/session_013/lidar.pcap --max-range 80

결과는 --out 폴더(기본 ../results/session_013)에 남는다.

    poses_kitti.txt   스캔당 3x4 행렬을 한 줄로 편 것 (KITTI 관례)
    poses_tum.txt     timestamp tx ty tz qx qy qz qw (TUM 관례, evo 로 바로 읽힘)
    summary.txt       스캔 수, 이동 거리, 설정값

deskew 효과를 보려면 두 번 돌려서 궤적을 비교하면 된다. 차량이 정지해
있으면 차이가 거의 없고, 회전할 때 가장 크게 벌어진다.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset


def rot_to_quat(R):
    """3x3 회전행렬 → (qx, qy, qz, qw). Shepperd 방식으로 부호 안전하게."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def build_odometry(deskew, max_range):
    """KISS-ICP 코어를 만든다. 버전에 따라 설정 방식이 갈려서 나눠 잡는다."""
    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP

    try:                                  # 1.2 계열
        cfg = load_config(None, deskew=deskew, max_range=max_range)
    except TypeError:                     # 1.0 ~ 1.1 계열
        cfg = load_config(None)
        cfg.data.deskew = deskew
        if max_range is not None:
            cfg.data.max_range = max_range
    return KissICP(cfg), cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("-o", "--out", default="../results/session_013")
    ap.add_argument("--no-deskew", dest="deskew", action="store_false")
    ap.add_argument("--max-range", type=float, default=100.0)
    ap.add_argument("--n-scans", type=int, default=-1)
    ap.add_argument("--save-deskewed", action="store_true",
                    help="보정된 스캔을 deskewed/00000.npy 형태로 저장 (스캔당 약 300 KB)")
    args = ap.parse_args()

    ds = VLP16PcapDataset(args.pcap)
    n = len(ds) if args.n_scans < 0 else min(args.n_scans, len(ds))
    print(f"{ds.model} / {ds.return_mode} / 스캔 {len(ds)} 개 중 {n} 개 처리, "
          f"deskew={args.deskew}, max_range={args.max_range}")

    odom, cfg = build_odometry(args.deskew, args.max_range)

    if args.save_deskewed:
        os.makedirs(os.path.join(args.out, "deskewed"), exist_ok=True)

    pose_list = []
    for i in range(n):
        points, timestamps = ds[i]
        returned = odom.register_frame(points, timestamps)

        # pose 를 어디서 읽는지가 버전마다 다르다.
        #   1.3 계열 : register_frame 이 (보정된 스캔, 정합에 쓴 점) 을 주고
        #              누적 pose 는 odom.last_pose 에만 있다
        #   1.0~1.2  : odom.poses 에 전부 쌓인다
        if hasattr(odom, "last_pose"):
            pose_list.append(np.array(odom.last_pose, dtype=np.float64, copy=True))

        if args.save_deskewed:
            frame = returned[0] if isinstance(returned, tuple) else returned
            np.save(os.path.join(args.out, "deskewed", f"{i:05d}.npy"),
                    np.asarray(frame, dtype=np.float32))

        if (i + 1) % 20 == 0 or i + 1 == n:
            print(f"  {i + 1}/{n}")

    if not pose_list:
        pose_list = list(getattr(odom, "poses", []))
    poses = np.asarray(pose_list)           # (n,4,4)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise SystemExit(f"예상과 다른 pose 모양: {poses.shape}. "
                         "KISS-ICP 버전을 확인하세요.")

    os.makedirs(args.out, exist_ok=True)
    kitti = os.path.join(args.out, "poses_kitti.txt")
    tum = os.path.join(args.out, "poses_tum.txt")
    np.savetxt(kitti, poses[:, :3, :].reshape(len(poses), 12), fmt="%.9e")

    times = ds.get_frames_timestamps()[:len(poses)]
    with open(tum, "w") as f:
        for t, T in zip(times, poses):
            qx, qy, qz, qw = rot_to_quat(T[:3, :3])
            x, y, z = T[:3, 3]
            f.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} "
                    f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")

    step = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    dist = float(step.sum())
    summary = (f"pcap        {os.path.abspath(args.pcap)}\n"
               f"scans       {len(poses)}\n"
               f"deskew      {args.deskew}\n"
               f"max_range   {args.max_range}\n"
               f"이동 거리    {dist:.3f} m\n"
               f"스캔당 이동  평균 {step.mean():.4f} m, 최대 {step.max():.4f} m\n"
               f"시작→끝 직선 {np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3]):.3f} m\n")
    with open(os.path.join(args.out, "summary.txt"), "w") as f:
        f.write(summary)
    print(summary, end="")
    print(f"저장: {kitti}\n      {tum}")


if __name__ == "__main__":
    main()
