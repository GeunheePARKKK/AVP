"""KISS-ICP 궤적으로 스캔 안 움직임을 재고, 모션 왜곡이 실제로 얼마나 되는지 측정한다.

    python3 analyze_deskew.py ../data/session_013/lidar.pcap --poses ../results/session_013

재는 것
-------
1. 스캔 사이 움직임 — 자세 차이에서 속도와 회전 속도.
2. 이음매(seam) — 한 회전의 처음 10 %와 마지막 10 %는 같은 방향을 0.1초 간격으로 본다.
   차가 움직였으면 같은 벽이 두 자리에 찍힌다. 면 대 면 최근접 거리로 그 어긋남을 잰다.
   (방위각으로 점을 짝지으면 보정 후 면을 따라 미끄러진 것까지 오차로 잡혀 못 쓴다.)
3. 지도 선명도 — 연속 스캔을 자세로 쌓아 격자에 넣고 채워진 칸 수. 번질수록 늘어난다.
4. 검산 — 우리 보정이 KISS-ICP 내부 보정과 같은 값을 내는지.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset


# ---------------------------------------------------------------- SE(3)
def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def se3_log(T):
    """4x4 → (회전벡터, 이동). 스캔 한 칸(0.1초) 움직임처럼 작은 값에만 쓴다."""
    R, t = T[:3, :3], T[:3, 3]
    th = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    if th < 1e-9:
        return np.zeros(3), t.copy()
    w = th / (2 * np.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    W = skew(w)
    V_inv = (np.eye(3) - 0.5 * W
             + (1 - th * np.sin(th) / (2 * (1 - np.cos(th)))) / th**2 * (W @ W))
    return w, V_inv @ t


def se3_exp(w, u):
    th = np.linalg.norm(w)
    W = skew(w)
    if th < 1e-9:
        R, V = np.eye(3) + W, np.eye(3) + 0.5 * W
    else:
        R = np.eye(3) + np.sin(th) / th * W + (1 - np.cos(th)) / th**2 * (W @ W)
        V = np.eye(3) + (1 - np.cos(th)) / th**2 * W + (th - np.sin(th)) / th**3 * (W @ W)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, V @ u
    return T


def yaw_of(R):
    return np.degrees(np.arctan2(R[1, 0], R[0, 0]))


# ---------------------------------------------------------------- 스캔 안 보정
def deskew_scan(points, t_norm, delta, ref=1.0, nbin=512):
    """delta = T(k-1 ← k), 스캔 한 칸 동안의 움직임. 점을 ref 시각의 센서 자세로 옮긴다.

    ref: 1.0 스캔 끝(KISS-ICP 규약) · 0.5 중간 · 0.25 셔터가 스캔 25 % 지점일 때.
    등속 가정으로 exp((t-ref)·log(delta)) 를 점마다 곱한다."""
    w, u = se3_log(delta)
    out = np.empty_like(points)
    bins = np.clip((t_norm * nbin).astype(int), 0, nbin - 1)
    for b in np.unique(bins):
        m = bins == b
        s = t_norm[m].mean() - ref
        T = se3_exp(w * s, u * s)
        out[m] = points[m] @ T[:3, :3].T + T[:3, 3]
    return out


# ---------------------------------------------------------------- 이음매 (평면 맞춤)
def seam_sets(points, t_norm, deg=6.0, rng=25.0):
    """이음매 양옆의 얇은 쐐기. 회전 시작 방위각에서 +deg (머리, t≈0), -deg (꼬리, t≈1).
    둘은 같은 벽을 0.1초 간격으로 본다. 쐐기를 넓게 잡으면 서로 다른 벽을 보게 된다."""
    az = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360.0
    a0 = az[np.argmin(t_norm)]
    off = (az - a0 + 180) % 360 - 180
    near = np.linalg.norm(points, axis=1) < rng
    # VLP-16 방위각은 시계 방향이라 ROS 좌표에서 az 는 360° → 0° 로 줄어든다.
    head = np.where((t_norm < 0.1) & (off <= 0) & (off > -deg) & near)[0]
    tail = np.where((t_norm > 0.9) & (off >= 0) & (off < deg) & near)[0]
    return head, tail


def fit_plane(P):
    """최소제곱 평면 → (법선, 평면 위 한 점, 잔차 RMS)."""
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c)
    nrm = Vt[-1]
    return nrm, c, float(np.sqrt(np.mean(((P - c) @ nrm) ** 2)))


def seam_offset(points, head, tail, rms_max=0.06):
    """꼬리 점으로 평면을 맞추고 머리 점이 그 평면에서 얼마나 떨어졌는지 (m).

    면을 따라 미끄러진 성분은 빠지고 수직 어긋남만 남는다 — 그게 모션 왜곡이다.
    꼬리 점 자체가 평면이 아니면(rms 큼) 판단할 수 없으므로 NaN."""
    if len(head) < 30 or len(tail) < 30:
        return np.nan, np.nan
    nrm, c, rms = fit_plane(points[tail])
    if rms > rms_max:
        return np.nan, rms
    return float(np.median(np.abs((points[head] - c) @ nrm))), rms


# ---------------------------------------------------------------- 이웃 스캔 지도와의 정합 오차
def local_map(ds, poses, k, back=5, voxel=0.05, rng=40.0):
    """스캔 k 직전 back 개를 보정해서 월드 좌표로 쌓은 작은 지도."""
    pts = []
    for j in range(max(1, k - back), k):
        p, t = ds[j]
        m = np.linalg.norm(p, axis=1) < rng
        d = orthonormalize(np.linalg.inv(poses[j - 1]) @ poses[j])
        w = deskew_scan(p[m], t[m], d, ref=1.0)
        pts.append(w @ poses[j][:3, :3].T + poses[j][:3, 3])
    P = np.concatenate(pts)
    key = np.floor(P / voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[idx]


def map_residual(points, pose, tree, rng=40.0, max_d=1.0):
    """점을 월드로 옮겨 지도까지의 최근접 거리 중앙값 (m). 멀리 튄 점(새로 보인 곳)은 제외."""
    m = np.linalg.norm(points, axis=1) < rng
    w = points[m] @ pose[:3, :3].T + pose[:3, 3]
    d, _ = tree.query(w)
    d = d[d < max_d]
    return float(np.median(d)), len(d)


# ---------------------------------------------------------------- 지도 선명도
def occupied_voxels(clouds, poses, voxel=0.10):
    keys = [np.floor((P @ T[:3, :3].T + T[:3, 3]) / voxel).astype(np.int32)
            for P, T in zip(clouds, poses)]
    k = np.concatenate(keys)
    return len(np.unique(k, axis=0)), len(k)


def orthonormalize(T):
    """텍스트로 저장된 자세는 회전부가 1e-10 쯤 어긋나 Sophus 가 거부한다. SVD 로 되돌린다."""
    U, _, Vt = np.linalg.svd(T[:3, :3])
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    out = T.copy()
    out[:3, :3] = R
    return out


def load_poses(path):
    P = np.loadtxt(os.path.join(path, "poses_kitti.txt")).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(P), 1, 1))
    T[:, :3, :] = P
    return np.array([orthonormalize(t) for t in T])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--poses", default="../results/session_013")
    ap.add_argument("-o", "--out", default="../results/session_013")
    ap.add_argument("--map-scans", type=int, default=30)
    ap.add_argument("--voxel", type=float, default=0.10)
    args = ap.parse_args()

    ds = VLP16PcapDataset(args.pcap)
    poses = load_poses(args.poses)
    n = min(len(ds), len(poses))
    os.makedirs(args.out, exist_ok=True)
    print(f"스캔 {n} 개")

    # ---- 1. 스캔 사이 움직임
    steps, yaws = np.zeros(n), np.zeros(n)
    for k in range(1, n):
        d = np.linalg.inv(poses[k - 1]) @ poses[k]
        steps[k], yaws[k] = np.linalg.norm(d[:3, 3]), yaw_of(d[:3, :3])
    dt = float(np.median(np.diff(ds.scan_times)))
    print(f"스캔 간격  {dt*1000:.1f} ms")
    print(f"속도      평균 {steps[1:].mean()/dt*3.6:.1f} km/h, 최대 {steps.max()/dt*3.6:.1f} km/h")
    print(f"회전      |yaw| 평균 {np.abs(yaws[1:]).mean():.2f}°/스캔, 최대 {np.abs(yaws).max():.2f}°/스캔"
          f" ({np.abs(yaws).max()/dt:.0f}°/s)")
    print(f"20 m 앞 점이 한 스캔 동안 끌리는 양(회전분): 최대 {np.radians(np.abs(yaws).max())*20:.2f} m")
    np.save(os.path.join(args.out, "per_scan_motion.npy"), np.c_[np.arange(n), steps, yaws])

    # ---- 2. 이음매: 회전 구간 / 직선 구간
    turn = np.argsort(-np.abs(yaws))[:6]
    straight = np.argsort(np.abs(yaws) + (steps < 0.1) * 10)[:6]
    rows = []
    for tag, ks in (("회전", turn), ("직선", straight)):
        for k in sorted(int(x) for x in ks):
            if k < 1:
                continue
            from scipy.spatial import cKDTree
            pts, tt = ds[k]
            delta = orthonormalize(np.linalg.inv(poses[k - 1]) @ poses[k])
            fixed = deskew_scan(pts, tt, delta, ref=1.0)
            tree = cKDTree(local_map(ds, poses, k))
            r0, m0 = map_residual(pts, poses[k], tree)
            r1, _ = map_residual(fixed, poses[k], tree)
            moved = np.linalg.norm(fixed - pts, axis=1)
            rows.append((tag, k, steps[k], yaws[k], r0, r1, float(moved.max()), m0))
    print("\n구간  스캔   이동(m)  yaw(°)   지도오차 보정전(cm) 보정후(cm)  개선   최대보정(m)")
    for tag, k, s, y, r0, r1, mv, m0 in rows:
        print(f"{tag}  {k:4d}  {s:7.3f} {y:7.2f}   {r0*100:12.1f} {r1*100:10.1f}  "
              f"{(r1-r0)/r0*100:+6.1f}%   {mv:8.3f}")
    for tag in ("회전", "직선"):
        v = np.array([[r[4], r[5]] for r in rows if r[0] == tag], dtype=float)
        v = v[~np.isnan(v).any(axis=1)]
        if len(v):
            print(f"{tag} 중앙값: 지도오차 보정 전 {np.median(v[:,0])*100:.1f} cm → "
                  f"보정 후 {np.median(v[:,1])*100:.1f} cm")

    # ---- 3. 검산: 우리 보정 vs KISS-ICP 내부 보정
    from kiss_icp.config import load_config
    from kiss_icp.preprocess import get_preprocessor
    cfg = load_config(None)
    cfg.data.deskew, cfg.data.max_range, cfg.data.min_range = True, 1e6, 0.0
    pre = get_preprocessor(cfg)
    k = int(turn[0])
    pts, tt = ds[k]
    delta = orthonormalize(np.linalg.inv(poses[k - 1]) @ poses[k])
    mine = deskew_scan(pts, tt, delta, ref=1.0, nbin=4096)
    theirs = np.asarray(pre.preprocess(pts, tt, delta))
    if len(mine) == len(theirs):
        d = np.linalg.norm(mine - theirs, axis=1)
        print(f"\n검산 — 우리 보정 vs KISS-ICP 내부 보정 (스캔 {k}): "
              f"최대 {d.max()*1000:.2f} mm, 중앙값 {np.median(d)*1000:.3f} mm")
    else:
        print(f"\n검산 — 점 수가 달라 직접 비교 못 함: {len(mine)} vs {len(theirs)}")

    # ---- 4. 지도 선명도 (회전 구간 연속 스캔)
    lo = max(1, int(turn[0]) - args.map_scans // 2)
    hi = min(n, lo + args.map_scans)
    raw, fix = [], []
    for k in range(lo, hi):
        pts, tt = ds[k]
        m = np.linalg.norm(pts, axis=1) < 40.0
        delta = orthonormalize(np.linalg.inv(poses[k - 1]) @ poses[k])
        raw.append(pts[m])
        fix.append(deskew_scan(pts[m], tt[m], delta, ref=1.0))
    Ts = poses[lo:hi]
    v0, ntot = occupied_voxels(raw, Ts, args.voxel)
    v1, _ = occupied_voxels(fix, Ts, args.voxel)
    print(f"\n지도 선명도 (스캔 {lo}~{hi-1}, 점 {ntot:,}, {args.voxel*100:.0f} cm 격자)")
    print(f"  보정 전 채워진 칸 {v0:,} → 보정 후 {v1:,}  ({(v1-v0)/v0*100:+.1f} %)")


if __name__ == "__main__":
    main()
