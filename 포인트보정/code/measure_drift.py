"""KISS-ICP 궤적의 드리프트를 잰다 — 정지 물체가 시간이 지나며 얼마나 어긋나는가.

    python3 measure_drift.py <세션폴더> --poses ../results/.../deskew_on

왜 재는가
---------
주차된 차는 월드 좌표에 고정이므로 박스를 한 번 그려 ego pose 로 전 프레임에
전파할 수 있다 — 오도메트리가 정확하다면. nuScenes 지침은 "정지 객체의 점이
localization 오차로 흔들리면 프레임마다 별도 cuboid 를 만들라" 고 한다.
nuScenes 는 Monte Carlo Localization 으로 10 cm 이하를 확보했다. 우리는 GPS 도
HD 지도도 없는 순수 라이다 오도메트리라 직접 재야 한다.

재는 법
-------
시간 간격 dt 만큼 떨어진 두 스캔을 pose 로 월드 좌표에 올린다. 장면이 정지해
있으므로 완벽한 pose 라면 두 점구름이 포개져야 한다. 포개지지 않는 만큼이
그 구간의 드리프트다. ICP 로 남은 변환을 구해 크기를 잰다.
"""
import argparse, os, sys
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import load_poses


def voxel(P, s=0.15):
    k = np.floor(P / s).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[idx]


def icp(src, dst, iters=30, max_d=0.5):
    """점-점 ICP. 남은 변환(R,t)과 정합 후 중앙값 거리를 돌려준다."""
    tree = cKDTree(dst)
    R = np.eye(3); t = np.zeros(3); cur = src.copy()
    for _ in range(iters):
        d, j = tree.query(cur, distance_upper_bound=max_d)
        m = np.isfinite(d)
        if m.sum() < 50:
            break
        a = cur[m]; b = dst[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        S = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
        dR = Vt.T @ S @ U.T
        dt = cb - dR @ ca
        cur = cur @ dR.T + dt
        R = dR @ R; t = dR @ t + dt
    d, j = cKDTree(dst).query(cur, distance_upper_bound=max_d)
    m = np.isfinite(d)
    return R, t, (float(np.median(d[m])) if m.any() else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--pairs", type=int, default=8)
    args = ap.parse_args()

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    n = min(len(ds), len(poses))
    hz = 1.0 / np.median(np.diff(ds.scan_times[:n]))
    print(f"스캔 {n} 개, {hz:.1f} Hz\n")

    def world(k):
        p, _ = ds[k]
        p = p[np.linalg.norm(p, axis=1) < args.max_range]
        T = poses[k]
        return voxel(p @ T[:3, :3].T + T[:3, 3])

    def overlap(A, B, pA, pB, R):
        """두 센서 위치 모두에서 R 안에 들어오는 점만 남긴다.

        이렇게 하지 않으면 간격이 벌어질수록 서로 다른 면을 보게 되고, ICP 가
        다른 물체끼리 맞추려 해서 드리프트가 과장된다."""
        fa = (np.linalg.norm(A - pA, axis=1) < R) & (np.linalg.norm(A - pB, axis=1) < R)
        fb = (np.linalg.norm(B - pA, axis=1) < R) & (np.linalg.norm(B - pB, axis=1) < R)
        return A[fa], B[fb]

    print(f"{'간격':>6s} {'쌍':>4s} {'위치 어긋남':>12s} {'회전':>9s} {'정합 후 거리':>12s}")
    rows = []
    for dt in (1.0, 2.0, 5.0, 10.0, 15.0):
        step = int(round(dt * hz))
        ks = np.linspace(1, n - step - 1, args.pairs).astype(int)
        ks = [k for k in ks if 0 <= k and k + step < n]
        if not ks:
            continue
        tr, ro, rs = [], [], []
        npts = []
        for k in ks:
            A, B = world(k), world(k + step)
            A, B = overlap(A, B, poses[k][:3, 3], poses[k + step][:3, 3], args.max_range)
            if len(A) < 300 or len(B) < 300:
                continue
            R, t, med = icp(A, B)
            tr.append(np.linalg.norm(t))
            ro.append(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
            rs.append(med)
            npts.append(min(len(A), len(B)))
        if not tr:
            print(f"{dt:5.0f}s    - 겹치는 영역 부족")
            continue
        rows.append((dt, np.median(tr), np.median(ro), np.median(rs)))
        print(f"{dt:5.0f}s {len(tr):4d} {np.median(tr)*100:10.1f} cm "
              f"{np.median(ro):7.2f}° {np.median(rs)*100:10.1f} cm   겹침 점 {int(np.median(npts))}")

    if rows:
        d15 = [r for r in rows if r[0] == 15.0]
        print("\n판정")
        for dt, tr, ro, rs in rows:
            print(f"  {dt:4.0f}초 간격에서 정지 물체가 {tr*100:.1f} cm, {ro:.2f}° 어긋난다")
        if d15:
            v = d15[0][1] * 100
            print(f"\n  nuScenes 기준(10 cm 이하)과 비교: 15초 구간 {v:.1f} cm "
                  f"→ {'전파 가능' if v <= 10 else '프레임별 수정 필요'}")


if __name__ == "__main__":
    main()
