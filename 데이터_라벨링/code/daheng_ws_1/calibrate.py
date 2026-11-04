"""Intrinsic calibration from the captured checkerboard views.

    python calibrate.py                 # standard 5-coefficient model
    python calibrate.py --prune 1.0     # drop views over 1.0 px and re-solve
    python calibrate.py --rational      # 8 coefficients, for strong wide-angle

Writes results/calibration.yaml (OpenCV FileStorage), results/calibration.json,
a per-view error report and undistorted samples.
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

from board import find_corners, load_board

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--images", default=os.path.join(HERE, "captures"))
    p.add_argument("--out", default=os.path.join(HERE, "results"))
    p.add_argument("--rational", action="store_true",
                   help="8-coefficient rational model (k1..k6)")
    p.add_argument("--fix-k3", action="store_true", help="hold k3 at zero")
    p.add_argument("--zero-tangential", action="store_true", help="hold p1,p2 at zero")
    p.add_argument("--prune", type=float, default=None, metavar="PX",
                   help="drop views whose RMS exceeds PX and recalibrate")
    p.add_argument("--pixel-size-um", type=float, default=3.45,
                   help="sensor pixel pitch, for the focal-length-in-mm readout "
                        "(MER2-240-159U3C / IMX392 = 3.45)")
    p.add_argument("--undistort-samples", type=int, default=3)
    return p.parse_args()


def detect_all(paths, board):
    obj_tpl = board.object_points()
    objpoints, imgpoints, used, failed = [], [], [], []
    shape = None
    for i, path in enumerate(paths, 1):
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            failed.append((path, "unreadable"))
            continue
        if shape is None:
            shape = img.shape[1], img.shape[0]
        elif (img.shape[1], img.shape[0]) != shape:
            failed.append((path, f"size {img.shape[1]}x{img.shape[0]} != {shape[0]}x{shape[1]}"))
            continue
        corners = find_corners(img, board)
        status = "ok" if corners is not None else "no board"
        print(f"  [{i:3d}/{len(paths)}] {os.path.basename(path):<20} {status}")
        if corners is None:
            failed.append((path, "board not found"))
            continue
        objpoints.append(obj_tpl.copy())
        imgpoints.append(corners.astype(np.float32))
        used.append(path)
    return objpoints, imgpoints, used, failed, shape


def per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist):
    errs = []
    for objp, imgp, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        proj, _ = cv2.projectPoints(objp, rv, tv, K, dist)
        d = (proj.reshape(-1, 2) - imgp.reshape(-1, 2))
        errs.append(float(np.sqrt((d ** 2).sum() / len(d))))
    return np.array(errs)


def solve(objpoints, imgpoints, shape, flags):
    return cv2.calibrateCamera(objpoints, imgpoints, shape, None, None, flags=flags)


def main():
    args = parse_args()
    board = load_board()
    os.makedirs(args.out, exist_ok=True)

    paths = sorted(glob.glob(os.path.join(args.images, "*.png")) +
                   glob.glob(os.path.join(args.images, "*.bmp")) +
                   glob.glob(os.path.join(args.images, "*.jpg")))
    if not paths:
        print(f"No images in {args.images}. Run capture_board.py first.")
        return 1

    print(f"{board}\nDetecting corners in {len(paths)} image(s):")
    objpoints, imgpoints, used, failed, shape = detect_all(paths, board)
    print(f"\nusable views: {len(used)}/{len(paths)}")
    if len(used) < 5:
        print("Need at least 5 usable views (10-20 recommended).")
        return 1
    if len(used) < 10:
        print("[!] Fewer than 10 views -- the distortion terms will be poorly constrained.")

    flags = 0
    if args.rational:
        flags |= cv2.CALIB_RATIONAL_MODEL
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    if args.zero_tangential:
        flags |= cv2.CALIB_ZERO_TANGENT_DIST

    rms, K, dist, rvecs, tvecs = solve(objpoints, imgpoints, shape, flags)
    errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist)

    dropped = []
    if args.prune is not None:
        keep = errs <= args.prune
        if keep.sum() < 5:
            print(f"[!] --prune {args.prune} would leave {int(keep.sum())} views; skipping.")
        elif keep.all():
            print(f"No view exceeds {args.prune} px; nothing pruned.")
        else:
            dropped = [os.path.basename(p) for p, k in zip(used, keep) if not k]
            print(f"\nPruning {len(dropped)} view(s) over {args.prune} px: "
                  f"{', '.join(dropped)}")
            objpoints = [o for o, k in zip(objpoints, keep) if k]
            imgpoints = [i for i, k in zip(imgpoints, keep) if k]
            used = [p for p, k in zip(used, keep) if k]
            rms, K, dist, rvecs, tvecs = solve(objpoints, imgpoints, shape, flags)
            errs = per_view_errors(objpoints, imgpoints, rvecs, tvecs, K, dist)

    w, h = shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    fov_x = np.degrees(2 * np.arctan(w / (2 * fx)))
    fov_y = np.degrees(2 * np.arctan(h / (2 * fy)))
    f_mm = fx * args.pixel_size_um / 1000.0

    print("\n" + "=" * 62)
    print(f"  RMS reprojection error : {rms:.4f} px   ({len(used)} views, {w}x{h})")
    print(f"  fx, fy                 : {fx:10.3f}  {fy:10.3f}  px")
    print(f"  cx, cy                 : {cx:10.3f}  {cy:10.3f}  px   "
          f"(frame centre {w / 2:.1f}, {h / 2:.1f})")
    print(f"  focal length           : {f_mm:.2f} mm  @ {args.pixel_size_um} um pixels")
    print(f"  field of view          : {fov_x:.1f} deg H x {fov_y:.1f} deg V")
    print("  distortion             : " +
          "  ".join(f"{v:+.5f}" for v in dist.ravel()))
    print("=" * 62)

    order = np.argsort(-errs)
    print("\nper-view error (worst first):")
    for i in order:
        flag = "  <-- check this view" if errs[i] > max(1.0, 2 * float(errs.mean())) else ""
        print(f"  {os.path.basename(used[i]):<20} {errs[i]:.4f} px{flag}")

    # A single mis-localised corner can drag one view to several px while the rest
    # sit near 0.1 px -- that outlier biases K, so surface it rather than bury it.
    outliers = [i for i in order if errs[i] > max(3 * float(np.median(errs)), 0.4)]
    if outliers and args.prune is None:
        cut = max(0.4, round(float(np.median(errs)) * 3, 2))
        print(f"\n[!] {len(outliers)} view(s) sit far above the median ({np.median(errs):.3f} px). "
              f"Re-run with  --prune {cut}  to drop them and re-solve.")

    if rms < 0.5:
        verdict = "good"
    elif rms < 1.0:
        verdict = "acceptable; re-check the worst views"
    else:
        verdict = "poor -- likely a wrong square size, blurred views, or a non-flat board"
    print(f"\nverdict: {verdict}")

    # ---- save -------------------------------------------------------------
    yaml_path = os.path.join(args.out, "calibration.yaml")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", w)
    fs.write("image_height", h)
    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", dist)
    fs.write("rms_reprojection_error", float(rms))
    fs.write("board_cols", board.cols)
    fs.write("board_rows", board.rows)
    fs.write("square_size_mm", board.square)
    fs.release()

    payload = {
        "model": "MER2-240-159U3C",
        "image_size": [w, h],
        "camera_matrix": K.tolist(),
        "distortion_coefficients": dist.ravel().tolist(),
        "distortion_model": "rational (k1,k2,p1,p2,k3,k4,k5,k6)" if args.rational
                            else "plumb_bob (k1,k2,p1,p2,k3)",
        "rms_reprojection_error_px": float(rms),
        "per_view_error_px": {os.path.basename(p): float(e) for p, e in zip(used, errs)},
        "views_used": [os.path.basename(p) for p in used],
        "views_dropped": dropped,
        "views_failed": [os.path.basename(p) for p, _ in failed],
        "board": {"cols": board.cols, "rows": board.rows, "square_size_mm": board.square},
        "derived": {"fx_mm": f_mm, "fov_x_deg": float(fov_x), "fov_y_deg": float(fov_y),
                    "pixel_size_um": args.pixel_size_um},
    }
    json_path = os.path.join(args.out, "calibration.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    # ---- undistorted samples ---------------------------------------------
    if args.undistort_samples > 0:
        sample_dir = os.path.join(args.out, "undistorted")
        os.makedirs(sample_dir, exist_ok=True)
        newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
        map1, map2 = cv2.initUndistortRectifyMap(K, dist, None, newK, (w, h), cv2.CV_16SC2)
        picks = [used[i] for i in np.linspace(0, len(used) - 1,
                                              min(args.undistort_samples, len(used))).astype(int)]
        for p in picks:
            img = cv2.imread(p)
            und = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
            pair = np.hstack([img, und])
            cv2.imwrite(os.path.join(sample_dir, os.path.basename(p)),
                        cv2.resize(pair, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        print(f"\nbefore/after samples: {sample_dir}  (left original, right undistorted)")

    print(f"saved: {yaml_path}\n       {json_path}")
    if failed:
        print(f"\n{len(failed)} image(s) had no detectable board:")
        for p, why in failed:
            print(f"  {os.path.basename(p)}: {why}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
