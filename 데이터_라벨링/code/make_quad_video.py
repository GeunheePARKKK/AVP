"""세션의 카메라 4대를 2x2 로 합쳐 mp4 로 만든다. 눈으로 보기용.

    python3 make_quad_video.py sessions/session_035
    python3 make_quad_video.py sessions/session_035 --scale 0.5 --fps 10

export_images.enhance_v2 와 같은 보정을 쓰되, 디베이어 직후 축소해서
톤매핑·노이즈 제거를 작은 해상도에서 돌린다 (장당 2 초 → 0.2 초).
"""
import argparse, csv, json, os, sys
from multiprocessing import Pool

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_images import BAYER, BAYER_EA, _white, MAX_TONE_GAIN


def one_cam(args):
    path, fmt, gamma_applied, wb_applied, size = args
    raw = np.load(path)
    white = _white(fmt, raw)
    x = raw.astype(np.float32) * (255.0 / white)
    q = 0.1 if gamma_applied else 0.5
    for dy in (0, 1):
        for dx in (0, 1):
            sub = x[dy::2, dx::2]
            x[dy::2, dx::2] = np.clip(sub - np.percentile(sub, q), 0, None)
    x16 = np.clip(x * 256.0, 0, 65535).astype(np.uint16)
    bgr = cv2.cvtColor(x16, BAYER_EA.get(fmt, BAYER[fmt])).astype(np.float32) / 256.0
    # 여기서 축소. 이후 단계가 전부 1/9 비용이 된다.
    bgr = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
    if not (gamma_applied or wb_applied):
        lum = bgr.sum(2)
        m = lum >= max(np.percentile(lum, 99.0), 3.0 * 255.0 / MAX_TONE_GAIN)
        if m.sum() > 200:
            g = np.maximum(bgr[m].mean(0), 1e-3)
            bgr *= np.clip(g.mean() / g, 0.5, 2.0)
    w = max(np.percentile(bgr, 99.7), 255.0 / MAX_TONE_GAIN)
    y = np.clip(bgr / w, 0, 1)
    out = (np.power(y, 1.0 / (1.15 if gamma_applied else 2.6)) * 255).astype(np.uint8)
    # 영상은 장수가 많아 NLM 이 너무 느리다. 색만 누르는 빠른 대안을 쓴다.
    lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    a = cv2.bilateralFilter(a, 5, 40, 9)
    b = cv2.bilateralFilter(b, 5, 40, 9)
    l = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8)).apply(l)
    l = cv2.addWeighted(l, 1.5, cv2.GaussianBlur(l, (0, 0), 1.0), -0.5, 0)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def frame_job(job):
    fid, items, size, labels = job
    tiles = []
    for (a, lab) in zip(items, labels):
        im = one_cam(a) if a else np.zeros((size[1], size[0], 3), np.uint8)
        cv2.rectangle(im, (0, 0), (size[0], 26), (0, 0, 0), -1)
        cv2.putText(im, lab, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 220, 255), 1, cv2.LINE_AA)
        tiles.append(im)
    quad = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    cv2.putText(quad, f"{fid}", (10, quad.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
    return fid, quad


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("-o", "--out", default=None, help="기본: <세션>/quad.mp4")
    ap.add_argument("--scale", type=float, default=0.32, help="카메라 1대당 축소 비율")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--jobs", type=int, default=max(1, os.cpu_count() - 2))
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.session, "meta.json")))
    out = args.out or os.path.join(args.session, "quad.mp4")

    # 슬롯 → (폴더, 픽셀포맷, 플래그). 방향 이름은 extrinsics 기준.
    DIR = {1: "RIGHT", 2: "LEFT", 3: "FRONT", 4: "BACK"}
    per_slot = {}
    for i, c in enumerate(meta["cameras"]):
        slot = meta["camera_slots"][i]
        folder = os.path.join(args.session, f"cam{slot}")
        rows = {}
        p = os.path.join(folder, "frames.csv")
        if os.path.exists(p):
            for r in csv.DictReader(open(p)):
                rows[int(r["frame_id"])] = os.path.join(folder, f"{int(r['frame_id']):08d}.npy")
        per_slot[slot] = (rows, c["pixel_format"],
                          bool(c.get("gamma_enable", False)),
                          str(c.get("balance_white_auto", "Off")) != "Off",
                          c["name"])

    w = int(meta["cameras"][0]["width"] * args.scale) // 2 * 2
    h = int(meta["cameras"][0]["height"] * args.scale) // 2 * 2
    size = (w, h)

    order = [3, 2, 1, 4]            # FRONT LEFT / RIGHT BACK
    labels = [f"{DIR[s]}  cam{s}" for s in order]
    all_ids = sorted(set().union(*[set(per_slot[s][0]) for s in order if s in per_slot]))
    if args.limit:
        all_ids = all_ids[:args.limit]

    jobs = []
    for fid in all_ids:
        items = []
        for s in order:
            rows, fmt, ga, wa, _ = per_slot.get(s, ({}, "BayerGB8", False, False, ""))
            p = rows.get(fid)
            items.append((p, fmt, ga, wa, size) if p else None)
        jobs.append((fid, items, size, labels))

    print(f"{len(jobs)} 프레임 · {w*2}x{h*2} · {args.fps} fps · 프로세스 {args.jobs}")
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w * 2, h * 2))
    if not vw.isOpened():
        raise SystemExit(f"VideoWriter 를 열 수 없습니다: {out}")
    done = 0
    with Pool(args.jobs) as pool:
        for fid, quad in pool.imap(frame_job, jobs, chunksize=2):
            vw.write(quad)
            done += 1
            if done % 25 == 0 or done == len(jobs):
                print(f"\r  {done}/{len(jobs)}", end="", flush=True)
    vw.release()
    print(f"\n완료: {out}  ({os.path.getsize(out)/1e6:.0f} MB)")


if __name__ == "__main__":
    main()
