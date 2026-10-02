"""라이다 점을 카메라 이미지에 투영해서 겹쳐 그린다.

    python3 project_overlay.py sessions/session_030 --cam front --frame 20
    python3 project_overlay.py sessions/session_030 --cam front --frame 20 --mp4 --max-frames 200

캘리브레이션이 아직 유효한지 눈으로 판정하는 용도다. 라이다 점이 이미지의
물체 윤곽(주차된 차, 기둥, 연석)에 얹히면 유효하고, 전체가 한쪽으로 밀려
있으면 리그가 움직인 것이다.

--------------------------------------------------------------------------
셔터 시각

  free-run 에서 frames.csv 의 host_recv_unix 는 셔터가 아니라 호스트가
  이미지를 다 받은 시각이다. 이 리그 실측값으로 되돌린다.

      셔터 ≈ host_recv_unix - (노출시간 + 7.1 ms)

  7.1 ms 는 센서 읽기 + USB3 전송(2.46 MB). 노출시간은 meta.json 에 있다.
  남는 스케줄링 지터는 상수로 못 지운다.

  라이다는 그 셔터 시각에 가장 가까운 회전을 쓴다. 정지 중에는 이것만으로
  충분하고, 주행 중에는 점을 셔터 순간으로 되돌리는 보정이 따로 필요하다
  (--compensate, 오도메트리 pose 가 있어야 동작).
"""

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import vlp16  # noqa: E402

CALIB_DIR = "/home/acelab/Documents/Lidar_Camera_Calibration/calib_result"
AVP_DIR = "/home/acelab/Documents/TalkFile_AVP_calib"
USB_LATENCY_S = 0.0071          # 센서 읽기 + USB3 전송, 이 리그 실측

BAYER_CV = {
    "BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
    "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR,
}


def load_calib(cam):
    with open(os.path.join(CALIB_DIR, "extrinsics_all.yaml")) as f:
        y = yaml.safe_load(f)
    c = y["cameras"][cam]
    T = np.array(c["T_cam_lidar"]["data"], dtype=np.float64).reshape(4, 4)
    path = os.path.join(AVP_DIR, c["intrinsics"])
    if not os.path.exists(path):
        raise SystemExit(f"intrinsics 파일이 없습니다: {path}")
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("camera_matrix").mat()
    D = fs.getNode("distortion_coefficients").mat()
    fs.release()
    return c["serial"], T, K, D.ravel()


def boost(img):
    """raw Bayer 는 어두우므로 퍼센타일 정규화 + 감마로 보이게 만든다."""
    lo, hi = np.percentile(img, (1, 99.5))
    if hi <= lo:
        return img
    x = np.clip((img.astype(np.float32) - lo) / (hi - lo), 0, 1)
    return (np.power(x, 0.5) * 255).astype(np.uint8)


def cam_folder_for(serial, meta):
    for i, c in enumerate(meta["cameras"]):
        if c["sn"] == serial:
            return f"cam{meta['camera_slots'][i]}", c
    raise SystemExit(f"세션에 시리얼 {serial} 카메라가 없습니다")


def render(session, cam, frame_idx, decoded, meta, calib, point_size, max_range):
    serial, T, K, D = calib
    folder, caminfo = cam_folder_for(serial, meta)
    with open(os.path.join(session, folder, "frames.csv")) as f:
        rows = list(csv.DictReader(f))
    if frame_idx >= len(rows):
        raise SystemExit(f"프레임 {frame_idx} 없음 (총 {len(rows)})")
    row = rows[frame_idx]

    exposure_s = float(caminfo.get("exposure_us", 8000.0)) / 1e6
    t_shutter = float(row["host_recv_unix"]) - (exposure_s + USB_LATENCY_S)

    # --- 이미지 ---
    npy = os.path.join(session, folder, f"{int(row['frame_id']):08d}.npy")
    raw = np.load(npy)
    bgr = cv2.cvtColor(boost(raw), BAYER_CV[caminfo["pixel_format"]])

    # --- 셔터에 가장 가까운 회전 ---
    spans = list(decoded.revolutions())
    starts = np.array([decoded.host_ts[a] for a, _ in spans])
    k = int(np.argmin(np.abs(starts - t_shutter)))
    a, b = spans[k]
    dt = starts[k] - t_shutter
    pts = decoded.xyz[a:b].astype(np.float64)

    # --- 카메라 좌표계로 ---
    R, t = T[:3, :3], T[:3, 3]
    pc = pts @ R.T + t
    keep = (pc[:, 2] > 0.5) & (np.linalg.norm(pts, axis=1) < max_range)
    pc = pc[keep]
    dist = np.linalg.norm(pts[keep], axis=1)
    if len(pc) == 0:
        return bgr, 0, dt, len(rows)

    uv, _ = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, D)
    uv = uv.reshape(-1, 2)
    h, w = bgr.shape[:2]
    inside = ((uv[:, 0] >= 0) & (uv[:, 0] < w) &
              (uv[:, 1] >= 0) & (uv[:, 1] < h))
    uv, dist = uv[inside], dist[inside]

    # --- 거리 색 (가까움 빨강 → 멀음 파랑) ---
    norm = np.clip(dist / 30.0, 0, 1)
    colors = cv2.applyColorMap((norm * 255).astype(np.uint8),
                               cv2.COLORMAP_JET).reshape(-1, 3)
    for (u, v), col in zip(uv.astype(np.int32), colors):
        cv2.circle(bgr, (u, v), point_size, col.tolist(), -1)
    return bgr, len(uv), dt, len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--cam", default="front",
                    choices=["front", "left", "right", "rear"])
    ap.add_argument("--frame", type=int, default=20)
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--max-frames", type=int, default=200)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--point-size", type=int, default=2)
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--scale", type=float, default=0.5)
    args = ap.parse_args()

    with open(os.path.join(args.session, "meta.json")) as f:
        meta = json.load(f)
    calib = load_calib(args.cam)
    print(f"카메라 {args.cam} (시리얼 {calib[0]})")
    print("라이다 디코딩 중...")
    decoded = vlp16.decode(os.path.join(args.session, "lidar.pcap"))

    if not args.mp4:
        img, n, dt, total = render(args.session, args.cam, args.frame,
                                   decoded, meta, calib, args.point_size,
                                   args.max_range)
        out = args.out or os.path.join(args.session,
                                       f"overlay_{args.cam}_{args.frame:04d}.png")
        if args.scale != 1.0:
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale)
        cv2.imwrite(out, img)
        print(f"프레임 {args.frame}/{total}  투영된 점 {n}  "
              f"회전-셔터 시간차 {dt*1000:+.1f} ms")
        print(f"저장: {out}  ({os.path.getsize(out)/1e6:.1f} MB)")
        return

    n_frames = min(args.max_frames, len(os.listdir(
        os.path.join(args.session, cam_folder_for(calib[0], meta)[0]))) - 1)
    out = args.out or os.path.join(args.session, f"overlay_{args.cam}.mp4")
    writer = None
    for i in range(n_frames):
        img, n, dt, total = render(args.session, args.cam, i, decoded, meta,
                                   calib, args.point_size, args.max_range)
        if args.scale != 1.0:
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale)
        if writer is None:
            h, w = img.shape[:2]
            writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"),
                                     args.fps, (w, h))
        writer.write(img)
        if (i + 1) % 20 == 0 or i + 1 == n_frames:
            print(f"  {i+1}/{n_frames}")
    writer.release()
    print(f"저장: {out}  ({os.path.getsize(out)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
