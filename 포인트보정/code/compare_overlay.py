"""셔터 보정 전/후를 카메라 이미지 위에서 비교한다.

    python3 compare_overlay.py <세션> --cam CAM1 --frame 148 \
        --poses ../results/2026-10-02_session_025/deskew_on -o /tmp/out.png

빨강 = 보정 전 (스캔에 찍힌 그대로)
초록 = 보정 후 (그 카메라 셔터 순간의 위치로 되돌린 것)

둘이 겹치면 그 방향은 셔터와 라이다 시각이 거의 같다는 뜻이고,
벌어지면 그만큼 자차가 움직인 것이다. 화소 이동량은 거리에 반비례한다
(초점거리 2056 px 기준, 5 m 거리에서 6 cm 는 약 25 px).
"""
import argparse, csv, json, os, sys
import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlp16_dataset import VLP16PcapDataset
from analyze_deskew import load_poses
from deskew_to_shutter import deskew_to, gather
from shutter_real import camera_table, CALIB, FOV_HALF

AVP = "/home/acelab/Documents/TalkFile_AVP_calib"
BAYER = {"BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
         "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR}


def calib_for(serial):
    cal = yaml.safe_load(open(CALIB))["cameras"]
    for pos, c in cal.items():
        if c["serial"] == serial:
            T = np.array(c["T_cam_lidar"]["data"]).reshape(4, 4)
            fs = cv2.FileStorage(os.path.join(AVP, c["intrinsics"]), cv2.FILE_STORAGE_READ)
            K, D = fs.getNode("camera_matrix").mat(), fs.getNode("distortion_coefficients").mat()
            fs.release()
            return pos, T, K, D.ravel()
    raise SystemExit(f"{serial} 없음")


def boost(img, lo_p=0.5, hi_p=99.8, gamma=0.45):
    lo, hi = np.percentile(img, (lo_p, hi_p))
    x = np.clip((img.astype(np.float32) - lo) / max(hi - lo, 1e-6), 0, 1)
    return (np.power(x, gamma) * 255).astype(np.uint8)


def proj(pts, T, K, D, shape):
    pc = pts @ T[:3, :3].T + T[:3, 3]
    keep = pc[:, 2] > 0.5
    if keep.sum() == 0:
        return np.zeros((0, 2)), keep
    uv = cv2.projectPoints(pc[keep], np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    h, w = shape[:2]
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    idx = np.flatnonzero(keep)[ok]
    return uv[ok], idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session"); ap.add_argument("--cam", default="CAM1")
    ap.add_argument("--frame", type=int, default=148)
    ap.add_argument("--poses", required=True); ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--crop", default=None, help="x,y,w,h 확대 영역")
    ap.add_argument("--max-range", type=float, default=25.0)
    args = ap.parse_args()

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    tend = ds.scan_end_times[:len(poses)]
    off = ds.host_offset
    cam = [c for c in camera_table(args.session) if c["name"] == args.cam][0]
    pos, T, K, D = calib_for(cam["serial"])

    meta = json.load(open(os.path.join(args.session, "meta.json")))
    cinfo = [c for c in meta["cameras"] if c["name"] == args.cam][0]
    t_s = float(cam["shutter"][args.frame]) - off
    k = int(np.argmin(np.abs(ds.scan_times - t_s)))
    raw, ta = gather(ds, k, t_s)
    fixed = deskew_to(raw, ta, poses, tend, t_s)
    moved = np.linalg.norm(fixed - raw, axis=1)

    img = np.load(os.path.join(args.session, cam["folder"],
                               f"{cam['frame_ids'][args.frame]:08d}.npy"))
    bgr = cv2.cvtColor(boost(img), BAYER[cinfo["pixel_format"]])
    base = bgr.copy()

    near = np.linalg.norm(raw, axis=1) < args.max_range
    uv_r, ir = proj(raw[near], T, K, D, bgr.shape)
    uv_f, if_ = proj(fixed[near], T, K, D, bgr.shape)
    for (u, v) in uv_r.astype(int):
        cv2.circle(bgr, (u, v), 2, (0, 0, 255), -1)       # 빨강 = 보정 전
    for (u, v) in uv_f.astype(int):
        cv2.circle(bgr, (u, v), 2, (0, 255, 0), -1)       # 초록 = 보정 후

    # 화소 이동량 통계 (같은 점끼리)
    common = np.intersect1d(ir, if_)
    if len(common) > 10:
        pr = {i: uv for i, uv in zip(ir, uv_r)}
        pf = {i: uv for i, uv in zip(if_, uv_f)}
        dpx = np.array([np.linalg.norm(pf[i] - pr[i]) for i in common])
        print(f"{args.cam} ({pos}) 프레임 {args.frame} · 스캔 {k}")
        print(f"  시야 안 점 {len(common)}개")
        print(f"  3D 보정량  중앙값 {np.median(moved[near])*100:.2f} cm, 최대 {moved[near].max()*100:.1f} cm")
        print(f"  화소 이동  중앙값 {np.median(dpx):.1f} px, 최대 {dpx.max():.1f} px")
    cv2.imwrite(args.out, bgr)
    print(f"저장: {args.out}")

    if args.crop:
        x, y, w, h = [int(v) for v in args.crop.split(",")]
        c1, c2 = base[y:y+h, x:x+w], bgr[y:y+h, x:x+w]
        both = np.hstack([cv2.resize(c1, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST),
                          cv2.resize(c2, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)])
        p = args.out.replace('.png', '_crop.png')
        cv2.imwrite(p, both)
        print(f"저장(확대): {p}")


if __name__ == "__main__":
    main()
