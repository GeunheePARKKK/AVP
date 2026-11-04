"""KISS-ICP 결과 그림. SVG(글자 살아 있음) + PNG 로 저장한다.

    python3 plot_results.py ../data/session_013/lidar.pcap \
        --on ../results/deskew_on --off ../results/deskew_off -o ../results/figures

그림
----
1. 궤적 — deskew 켬 / 끔, 회전 구간 표시
2. 스캔별 움직임과 지도 정합 오차 — 보정 전 / 후, 회전량과 같이
3. 스캔 하나 보정 전 / 후 (위에서 본 그림, 색 = 스캔 안 시각)
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import (deskew_scan, load_poses, local_map, map_residual,
                            orthonormalize, yaw_of)

BLUE, RED, ORANGE, GRAY, DARK = "#0070C0", "#C00000", "#ED7D31", "#8C8C8C", "#262626"


def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "Malgun Gothic", "axes.unicode_minus": False,
                         "svg.fonttype": "none", "font.size": 10})
    return plt


def save(fig, out, name):
    os.makedirs(out, exist_ok=True)
    stem = os.path.join(out, name)
    fig.savefig(stem + ".svg", bbox_inches="tight")
    fig.savefig(stem + ".png", dpi=250, bbox_inches="tight")
    print("saved", name)


def fig_trajectory(plt, on, off, yaws, out):
    fig, ax = plt.subplots(figsize=(13 / 2.54, 11 / 2.54))
    a, b = on[:, :3, 3], off[:, :3, 3]
    ax.plot(b[:, 0], b[:, 1], color=GRAY, lw=1.4, ls="--", label="deskew 끔")
    ax.plot(a[:, 0], a[:, 1], color=BLUE, lw=1.8, label="deskew 켬")
    hot = np.abs(yaws) > 1.0
    ax.scatter(a[hot, 0], a[hot, 1], s=14, color=RED, zorder=3, label="회전 구간 (|yaw| > 1°/스캔)")
    ax.plot(*a[0, :2], "o", color="#2E8B57", ms=8, zorder=4)
    ax.plot(*a[-1, :2], "s", color=DARK, ms=7, zorder=4)
    ax.annotate("시작", a[0, :2], textcoords="offset points", xytext=(8, -4), fontsize=9, color="#2E8B57")
    ax.annotate("끝", a[-1, :2], textcoords="offset points", xytext=(8, 0), fontsize=9, color=DARK)
    d = np.linalg.norm(a - b, axis=1)
    ax.set_title(f"session_013 · 20초 · 이동 33.3 m\n두 궤적 차이 최대 {d.max():.2f} m", fontsize=10.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8.5, loc="upper left", bbox_to_anchor=(0.0, 1.0), frameon=False)
    save(fig, out, "그림_궤적")
    plt.close(fig)


def fig_per_scan(plt, steps, yaws, res, out, dt=0.0999):
    k, r0, r1 = res[:, 0], res[:, 1] * 100, res[:, 2] * 100
    fig, axs = plt.subplots(2, 1, figsize=(16 / 2.54, 10 / 2.54), sharex=True,
                           gridspec_kw={"height_ratios": [1, 1.3]})
    axs[0].plot(np.arange(len(steps)) * dt, np.abs(yaws) / dt, color=RED, lw=1.2)
    axs[0].set_ylabel("회전 속도 (°/s)")
    axs[0].grid(alpha=0.25)
    axs[0].set_title("session_013 · 스캔마다: 회전 속도와 지도 정합 오차", fontsize=10.5)
    axs[1].plot(k * dt, r0, color=GRAY, lw=1.2, label="보정 전")
    axs[1].plot(k * dt, r1, color=BLUE, lw=1.4, label="보정 후 (deskew)")
    axs[1].set_ylabel("지도까지 거리 중앙값 (cm)")
    axs[1].set_xlabel("시간 (s)")
    axs[1].grid(alpha=0.25)
    axs[1].legend(fontsize=9, frameon=False)
    fig.text(0.5, -0.03, "지도 = 직전 5 스캔을 보정해 쌓은 점구름 · 값이 작을수록 스캔이 지도와 잘 맞는다",
             ha="center", fontsize=8.5, color=GRAY)
    save(fig, out, "그림_스캔별_오차")
    plt.close(fig)


def fig_before_after(plt, ds, poses, k, out, half=12.0):
    pts, tt = ds[k]
    delta = orthonormalize(np.linalg.inv(poses[k - 1]) @ poses[k])
    fixed = deskew_scan(pts, tt, delta, ref=1.0)
    m = (np.abs(pts[:, 2]) < 1.5) & (np.linalg.norm(pts, axis=1) < half * 2)
    fig, axs = plt.subplots(1, 2, figsize=(20 / 2.54, 9.5 / 2.54))
    for ax, P, ttl in ((axs[0], pts, "보정 전 — 한 바퀴(0.1초)를 한 순간처럼"),
                       (axs[1], fixed, "보정 후 — 점마다 그 순간의 자세로 되돌림")):
        sc = ax.scatter(P[m, 0], P[m, 1], c=tt[m] * 100, cmap="viridis", s=1.6, lw=0)
        ax.plot(0, 0, "o", color=RED, ms=5)
        ax.set_title(ttl, fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlim(-half * 2, half * 2)
        ax.set_ylim(-half, half)
        ax.set_xlabel("x (m)")
    axs[0].set_ylabel("y (m)")
    cb = fig.colorbar(sc, ax=axs, fraction=0.02, pad=0.01)
    cb.set_label("스캔 안 시각 (ms)")
    moved = np.linalg.norm(fixed - pts, axis=1)
    fig.text(0.5, -0.02, f"스캔 {k} (회전 {abs(yaw_of(delta[:3,:3])):.2f}°/스캔, 이동 "
                         f"{np.linalg.norm(delta[:3,3]):.2f} m) · 점이 옮겨간 거리 최대 {moved.max():.2f} m",
             ha="center", fontsize=8.5, color=GRAY)
    save(fig, out, "그림_스캔보정_전후")
    plt.close(fig)


def fig_map_zoom(plt, ds, poses, k, out, span=5, win=5.0):
    """회전 구간에서 연속 스캔을 월드 좌표로 겹쳐 벽 한 곳을 확대.
    보정 전에는 스캔마다 벽이 조금씩 어긋나 두꺼워지고, 보정 후에는 얇아진다."""
    ks = range(max(1, k - span // 2), max(1, k - span // 2) + span)
    raw, fix, idx = [], [], []
    for j in ks:
        p, t = ds[j]
        m = (np.abs(p[:, 2]) < 1.2) & (np.linalg.norm(p, axis=1) < 30)
        d = orthonormalize(np.linalg.inv(poses[j - 1]) @ poses[j])
        w = lambda P: P @ poses[j][:3, :3].T + poses[j][:3, 3]
        raw.append(w(p[m]))
        fix.append(w(deskew_scan(p[m], t[m], d, ref=1.0)))
        idx.append(np.full(m.sum(), j))
    raw, fix, idx = np.concatenate(raw), np.concatenate(fix), np.concatenate(idx)
    # 센서 앞쪽 8 m 지점을 중심으로 확대
    c = poses[k][:3, 3] + poses[k][:3, :3] @ np.array([8.0, 0.0, 0.0])
    fig, axs = plt.subplots(1, 2, figsize=(19 / 2.54, 9 / 2.54), sharex=True, sharey=True)
    for ax, P, ttl in ((axs[0], raw, "보정 전 — 스캔마다 벽이 어긋난다"),
                       (axs[1], fix, "보정 후 (deskew) — 한 겹으로 모인다")):
        sc = ax.scatter(P[:, 0], P[:, 1], c=idx, cmap="viridis", s=4, lw=0)
        ax.set_title(ttl, fontsize=10.5)
        ax.set_aspect("equal")
        ax.set_xlim(c[0] - win, c[0] + win)
        ax.set_ylim(c[1] - win, c[1] + win)
        ax.set_xlabel("x (m)")
        ax.grid(alpha=0.2)
    axs[0].set_ylabel("y (m)")
    cb = fig.colorbar(sc, ax=axs, fraction=0.02, pad=0.01)
    cb.set_label("스캔 번호")
    fig.text(0.5, -0.02, f"회전 구간 스캔 {min(ks)}~{max(ks)} 를 자세로 겹쳐 놓고 벽 한 곳을 확대 "
                         f"(회전 약 {abs(yaw_of(orthonormalize(np.linalg.inv(poses[k-1]) @ poses[k])[:3,:3])):.1f}°/스캔)",
             ha="center", fontsize=8.5, color=GRAY)
    save(fig, out, "그림_지도확대_전후")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--on", default="../results/deskew_on")
    ap.add_argument("--off", default="../results/deskew_off")
    ap.add_argument("-o", "--out", default="../results/figures")
    ap.add_argument("--stride", type=int, default=2, help="지도 오차를 몇 스캔마다 계산할지")
    args = ap.parse_args()

    plt = _mpl()
    ds = VLP16PcapDataset(args.pcap)
    on, off = load_poses(args.on), load_poses(args.off)
    n = min(len(ds), len(on), len(off))

    steps, yaws = np.zeros(n), np.zeros(n)
    for k in range(1, n):
        d = np.linalg.inv(on[k - 1]) @ on[k]
        steps[k], yaws[k] = np.linalg.norm(d[:3, 3]), yaw_of(d[:3, :3])

    from scipy.spatial import cKDTree
    res = []
    for k in range(6, n, args.stride):
        pts, tt = ds[k]
        delta = orthonormalize(np.linalg.inv(on[k - 1]) @ on[k])
        fixed = deskew_scan(pts, tt, delta, ref=1.0)
        tree = cKDTree(local_map(ds, on, k))
        r0, _ = map_residual(pts, on[k], tree)
        r1, _ = map_residual(fixed, on[k], tree)
        res.append((k, r0, r1))
        if len(res) % 20 == 0:
            print(f"  {k}/{n}")
    res = np.array(res)
    np.save(os.path.join(args.on, "map_residual.npy"), res)

    fig_trajectory(plt, on[:n], off[:n], yaws, args.out)
    fig_per_scan(plt, steps, yaws, res, args.out)
    kturn = int(np.argmax(np.abs(yaws)))
    fig_before_after(plt, ds, on, kturn, args.out)
    fig_map_zoom(plt, ds, on, kturn, args.out)

    turn = np.abs(yaws[res[:, 0].astype(int)]) > 1.0
    print(f"\n지도 오차 중앙값 (cm)  보정 전 {np.median(res[:,1])*100:.1f} → 보정 후 {np.median(res[:,2])*100:.1f}")
    print(f"  회전 구간만      보정 전 {np.median(res[turn,1])*100:.1f} → 보정 후 {np.median(res[turn,2])*100:.1f}")
    print(f"  직선 구간만      보정 전 {np.median(res[~turn,1])*100:.1f} → 보정 후 {np.median(res[~turn,2])*100:.1f}")


if __name__ == "__main__":
    main()
