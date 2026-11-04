"""B안 완성형 — 카메라 셔터 시각 기준으로 점을 되돌린다 (Step 3 · 4 · 5).

    python3 deskew_to_shutter.py ../data/session_013/lidar.pcap --poses ../results/deskew_on

session_013 은 자유구동이라 실제 셔터 시각이 없다. 그래서 A안 배치를 가정한다.
라이다는 스캔 시작(0 ms)에 앞을 보고 시계 방향으로 돌아 25 ms 우 · 50 ms 뒤 · 75 ms 좌를 지난다
(이 pcap 에서 실측 확인). 카메라 4대가 그 시각에 각각 찍는다고 두고, 카메라마다 한 장씩
'그 셔터 순간의 포인트클라우드' 를 만든다.

    Step 3  셔터 시각의 라이다 자세를 앞뒤 스캔 자세에서 보간
    Step 4  점마다 p' = T_W(t_ref)^-1 · T_W(t_i) · p_i
    Step 5  카메라 수만큼 반복

같이 재는 것 — 보정을 안 하면 얼마나 어긋나는가
    (a) 4대 동시 촬영 (모두 0 ms) + 보정 없음 : 우 25 ms · 뒤 50 ms · 좌 75 ms 만큼 시간이 어긋난다
    (b) A안 (0 / 25 / 50 / 75 ms 에 각각 촬영) + 보정 없음 : 시야 중심은 맞고 가장자리만 ±12.5 ms
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import load_poses, orthonormalize, se3_exp, se3_log, yaw_of

# (이름, 셔터 시각 ms, 시야 중심 방위각°) — ROS 좌표 0°=앞, 90°=좌, 180°=뒤, 270°=우
CAMERAS = [("CAM1 앞", 0.0, 0.0), ("CAM2 우", 25.0, 270.0),
           ("CAM3 뒤", 50.0, 180.0), ("CAM4 좌", 75.0, 90.0)]
FOV = 45.0        # 각 카메라가 담당하는 시야 반각


def pose_at(poses, tend, t):
    """절대 시각 t 의 라이다 자세. 스캔 자세 사이를 등속(SE3 보간)으로 채운다 — Step 3."""
    k = int(np.clip(np.searchsorted(tend, t), 1, len(poses) - 1))
    s = (t - tend[k - 1]) / (tend[k] - tend[k - 1])
    w, u = se3_log(orthonormalize(np.linalg.inv(poses[k - 1]) @ poses[k]))
    return poses[k - 1] @ se3_exp(w * s, u * s)


def deskew_to(points, t_abs, poses, tend, t_ref, nbin=256):
    """점을 기준 시각 t_ref 의 라이다 좌표로 — Step 4. p' = T(t_ref)^-1 · T(t_i) · p"""
    A = np.linalg.inv(pose_at(poses, tend, t_ref))
    out = np.empty_like(points)
    lo, hi = t_abs.min(), t_abs.max()
    bins = np.clip(((t_abs - lo) / max(hi - lo, 1e-9) * nbin).astype(int), 0, nbin - 1)
    for b in np.unique(bins):
        m = bins == b
        T = A @ pose_at(poses, tend, float(t_abs[m].mean()))
        out[m] = points[m] @ T[:3, :3].T + T[:3, 3]
    return out


def fov_mask(points, center, half=FOV):
    az = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360.0
    return np.abs((az - center + 180) % 360 - 180) < half


def scan_abs_times(ds, k):
    """스캔 k 의 점별 절대 시각 (초)."""
    a, b = ds._spans[k]
    tau = ds._time[a:b] - ds._time[a]
    return ds.scan_times[k] + tau


def gather(ds, k, t_s, window=0.05):
    """셔터 시각 t_s 에서 ±window 안에 찍힌 점만 모은다 (앞뒤 스캔까지 훑는다).

    라이다는 방향마다 찍는 시각이 다르고 100 ms 마다 같은 방향으로 돌아온다. 한 스캔만
    쓰면 셔터가 스캔 시작에 있을 때 그 방향 시야의 절반이 거의 한 바퀴(100 ms) 어긋난다.
    ±50 ms 창으로 고르면 방위각마다 셔터에 가장 가까운 관측 하나씩이 들어온다."""
    P, T = [], []
    for j in (k - 1, k, k + 1):
        if not 0 <= j < len(ds):
            continue
        pts, _ = ds[j]
        ta = scan_abs_times(ds, j)
        m = np.abs(ta - t_s) <= window
        if m.any():
            P.append(pts[m])
            T.append(ta[m])
    return np.concatenate(P), np.concatenate(T)


def figures(ds, poses, tend, k, stats, out):
    """막대그림(카메라별 오차)과 방위각별 시간차 그림."""
    from plot_results import _mpl, save
    plt = _mpl()
    BLUE, RED, GRAY = "#0070C0", "#C00000", "#8C8C8C"

    fig, axs = plt.subplots(1, 2, figsize=(19 / 2.54, 8 / 2.54), sharey=True)
    names = [c[0] for c in CAMERAS]
    x = np.arange(len(names))
    for ax, (tag, acc) in zip(axs, stats.items()):
        sim = [np.median(np.concatenate(acc[n]["sim"])) * 100 for n in names]
        stag = [np.median(np.concatenate(acc[n]["stag"])) * 100 for n in names]
        ax.bar(x - 0.2, sim, 0.4, color=RED, label="4대 동시 촬영")
        ax.bar(x + 0.2, stag, 0.4, color=BLUE, label="A안 (0 / 25 / 50 / 75 ms)")
        ax.set_xticks(x)
        ax.set_xticklabels(names, fontsize=9)
        ax.set_title(tag, fontsize=10.5)
        ax.grid(axis="y", alpha=0.25)
    axs[0].set_ylabel("셔터 시각과의 점 위치 차이 (cm, 중앙값)")
    axs[0].legend(fontsize=9, frameon=False)
    fig.text(0.5, -0.04, "보정하지 않았을 때 카메라 시야(±45°) 안의 라이다 점이 셔터 순간 위치와 얼마나 다른가 "
                         "· session_013 실측 (평균 6 km/h)", ha="center", fontsize=8.5, color=GRAY)
    save(fig, out, "그림_셔터별_오차")
    plt.close(fig)

    # 방위각별 시간차: 4대가 스캔 시작에 동시 촬영했다고 볼 때
    t_s = ds.scan_times[k]
    pts, ta = gather(ds, k, t_s)
    dt = (ta - t_s) * 1000.0
    m = (np.abs(pts[:, 2]) < 2.0) & (np.linalg.norm(pts, axis=1) < 25)
    fig, ax = plt.subplots(figsize=(11 / 2.54, 9.5 / 2.54))
    sc = ax.scatter(pts[m, 0], pts[m, 1], c=dt[m], cmap="coolwarm", s=2, lw=0, vmin=-50, vmax=50)
    ax.plot(0, 0, "o", color="black", ms=5)
    for name, ms, center in CAMERAS:
        r = 21
        a = np.radians(center)
        ax.annotate(name.split()[1], (r * np.cos(a), r * np.sin(a)), fontsize=9.5,
                    ha="center", va="center", color="#404040")
    ax.set_aspect("equal")
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("4대가 같은 순간에 찍었다면\n방향마다 라이다 점의 시각이 이만큼 다르다", fontsize=10.5)
    cb = fig.colorbar(sc, ax=ax, fraction=0.045, pad=0.02)
    cb.set_label("셔터 대비 점의 시각 (ms)")
    save(fig, out, "그림_방향별_시간차")
    plt.close(fig)


def fig_concept(ds, poses, tend, k, out):
    """카메라 1대 = 클라우드 1장. 4대면 같은 스캔에서 4장을 만든다는 걸 보여준다."""
    from plot_results import _mpl, save
    plt = _mpl()
    BLUE, RED, GRAY, DARK = "#0070C0", "#C00000", "#8C8C8C", "#262626"

    fig = plt.figure(figsize=(21 / 2.54, 13.5 / 2.54))
    gs = fig.add_gridspec(2, 4, height_ratios=[0.85, 2.6], hspace=0.05, wspace=0.12)

    # ---- 위: 시간축
    ax = fig.add_subplot(gs[0, :])
    ax.plot([0, 100], [0.55, 0.55], color=DARK, lw=1.4)
    for ms, lab in ((0, "앞"), (25, "우"), (50, "뒤"), (75, "좌"), (100, "앞")):
        ax.plot([ms, ms], [0.5, 0.6], color=DARK, lw=1.2)
        ax.text(ms, 0.66, f"{ms} ms", ha="center", fontsize=8.5, color=DARK)
        ax.text(ms, 0.38, lab, ha="center", fontsize=9, color=GRAY)
    ax.text(50, 0.86, "라이다 한 바퀴 — 방향마다 찍는 시각이 다르다", ha="center", fontsize=10, color=DARK)
    for ms, name, col in ((0, "CAM1", BLUE), (25, "CAM2", RED), (50, "CAM3", "#2E8B57"), (75, "CAM4", "#7030A0")):
        ax.annotate("", xy=(ms, 0.45), xytext=(ms, 0.12),
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=1.6))
        ax.text(ms, 0.03, name, ha="center", fontsize=9, color=col, fontweight="bold")
    ax.text(50, -0.12, "카메라 셔터 — 25 ms 씩 어긋난다", ha="center", fontsize=10, color=DARK)
    ax.set_xlim(-8, 108)
    ax.set_ylim(-0.3, 1.0)
    ax.axis("off")

    # ---- 아래: 카메라별 클라우드에서 점이 옮겨간 거리
    for i, (name, ms, center) in enumerate(CAMERAS):
        ax = fig.add_subplot(gs[1, i])
        t_s = ds.scan_times[k] + ms / 1000.0
        pts, ta = gather(ds, k, t_s)
        fixed = deskew_to(pts, ta, poses, tend, t_s)
        moved = np.linalg.norm(fixed - pts, axis=1) * 100
        m = (np.abs(pts[:, 2]) < 2.0) & (np.linalg.norm(pts, axis=1) < 22)
        sc = ax.scatter(fixed[m, 0], fixed[m, 1], c=moved[m], cmap="magma_r", s=1.6, lw=0, vmin=0, vmax=40)
        a = np.radians(center)
        ax.annotate("", xy=(16 * np.cos(a), 16 * np.sin(a)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.6))
        ax.set_title(f"{name} · 셔터 {ms:.0f} ms", fontsize=9.5)
        ax.set_aspect("equal")
        ax.set_xlim(-22, 22)
        ax.set_ylim(-22, 22)
        ax.set_xticks([])
        ax.set_yticks([])
    cb = fig.colorbar(sc, ax=fig.axes[1:], fraction=0.02, pad=0.01)
    cb.set_label("점이 옮겨간 거리 (cm)")
    fig.text(0.5, 0.005, "카메라 한 대마다 스캔 전체를 그 셔터 순간의 차량 위치로 옮겨 클라우드 한 장씩 "
                         "(파란 화살표 = 그 카메라가 보는 쪽 — 그쪽 점은 거의 안 움직인다)",
             ha="center", fontsize=8.5, color=GRAY)
    save(fig, out, "그림_카메라별_보정_개념")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--poses", default="../results/deskew_on")
    ap.add_argument("-o", "--out", default="../results/shutter")
    ap.add_argument("--scan", type=int, default=-1, help="npy 로 저장할 스캔 (기본: 회전이 가장 큰 스캔)")
    ap.add_argument("--n-eval", type=int, default=12, help="통계에 쓸 스캔 수 (직선/회전 각각)")
    ap.add_argument("--save", action="store_true", help="카메라별 포인트클라우드 npy 저장")
    args = ap.parse_args()

    ds = VLP16PcapDataset(args.pcap)
    poses = load_poses(args.poses)
    tend = ds.scan_end_times[:len(poses)]
    n = min(len(ds), len(poses))
    os.makedirs(args.out, exist_ok=True)

    yaws = np.zeros(n)
    for k in range(1, n):
        yaws[k] = yaw_of((np.linalg.inv(poses[k - 1]) @ poses[k])[:3, :3])
    turn = [int(k) for k in np.argsort(-np.abs(yaws))[:args.n_eval]]
    straight = [int(k) for k in np.argsort(np.abs(yaws))[:args.n_eval] if k > 2]

    # ---- 보정을 안 했을 때 카메라별로 얼마나 어긋나는가
    print(f"카메라 시야 ±{FOV:.0f}° · 점 위치 오차 = 그 점을 셔터 시각으로 되돌릴 때 움직인 거리\n")
    stats = {}
    for tag, ks in (("회전 구간", turn), ("직선 구간", straight)):
        acc = {c[0]: {"sim": [], "stag": []} for c in CAMERAS}
        stats[tag] = acc
        for k in ks:
            if not 1 <= k < n - 1:
                continue
            t0 = ds.scan_times[k]
            for name, ms, center in CAMERAS:
                for key, t_s in (("sim", t0), ("stag", t0 + ms / 1000.0)):
                    pts, ta = gather(ds, k, t_s)          # 셔터에 가장 가까운 관측만
                    m = fov_mask(pts, center) & (np.linalg.norm(pts, axis=1) < 30.0)
                    if m.sum() < 100:
                        continue
                    p, t = pts[m], ta[m]
                    fixed = deskew_to(p, t, poses, tend, t_s)
                    acc[name][key].append(np.linalg.norm(fixed - p, axis=1))
        print(f"[{tag}]  스캔 {len(ks)}개 평균")
        print("  카메라     셔터    동시촬영 무보정 (중앙/최대)   A안 무보정 (중앙/최대)")
        for name, ms, _ in CAMERAS:
            s, g = acc[name]["sim"], acc[name]["stag"]
            if not s:
                continue
            s, g = np.concatenate(s), np.concatenate(g)
            print(f"  {name}  {ms:5.0f} ms   {np.median(s)*100:7.1f} / {s.max()*100:6.1f} cm"
                  f"      {np.median(g)*100:7.1f} / {g.max()*100:6.1f} cm")
        print()

    # ---- 카메라별 포인트클라우드 만들기 (B안 결과물)
    k = int(np.argmax(np.abs(yaws))) if args.scan < 0 else args.scan
    figures(ds, poses, tend, k, stats, os.path.join(args.out, "figures"))
    fig_concept(ds, poses, tend, k, os.path.join(args.out, "figures"))
    t0 = ds.scan_times[k]
    print(f"스캔 {k} (회전 {abs(yaws[k]):.2f}°/스캔) → 카메라별 '셔터 순간' 포인트클라우드")
    for name, ms, center in CAMERAS:
        t_s = t0 + ms / 1000.0
        pts, ta = gather(ds, k, t_s)
        fixed = deskew_to(pts, ta, poses, tend, t_s)
        moved = np.linalg.norm(fixed - pts, axis=1)
        own = fov_mask(pts, center)
        print(f"  {name}: 점 {len(pts):,} · 전체 이동 중앙 {np.median(moved)*100:5.1f} cm "
              f"/ 최대 {moved.max()*100:6.1f} cm · 자기 시야만 중앙 {np.median(moved[own])*100:5.1f} cm")
        if args.save:
            np.save(os.path.join(args.out, f"scan{k:05d}_{name.split()[0]}.npy"),
                    fixed.astype(np.float32))
    if args.save:
        pts, _ = ds[k]
        np.save(os.path.join(args.out, f"scan{k:05d}_raw.npy"), pts.astype(np.float32))
        print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
