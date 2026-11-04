"""Live checkerboard capture for the Daheng MER2-240-159U3C.

    python capture_board.py            # auto-capture on, target 20 views
    python capture_board.py --manual   # SPACE to grab each view

Keys:  SPACE grab   A toggle auto   U undo last   Q/ESC finish

The overlay tracks where in the frame the board has already been seen -- keep
shooting until all nine coverage cells are green and you have both flat-on and
strongly tilted views. Lens distortion is estimated from the frame corners, so
views that only sit in the middle produce a confident but wrong calibration.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

from board import find_corners, load_board, sharpness, to_gray
from dhcam import DahengCamera

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.path.join(HERE, "captures"))
    p.add_argument("--target", type=int, default=20, help="views to aim for (default 20)")
    p.add_argument("--manual", action="store_true", help="start with auto-capture off")
    p.add_argument("--novelty", type=float, default=0.055,
                   help="min normalised pose distance between kept views (default 0.055)")
    p.add_argument("--min-sharpness", type=float, default=60.0,
                   help="reject blurred views below this Laplacian variance")
    p.add_argument("--preview-width", type=int, default=800,
                   help="detection runs at this width; lower = faster preview")
    p.add_argument("--exposure", type=float, default=None, help="fixed exposure in us")
    p.add_argument("--gain", type=float, default=None, help="fixed gain in dB")
    p.add_argument("--auto-exposure-max", type=float, default=20000.0,
                   help="ceiling for auto-exposure in us (default 20000); raise in a "
                        "dim room at the cost of frame rate and motion blur")
    return p.parse_args()


def pose_vector(corners, w, h):
    """Corner layout normalised by image size -- encodes position, scale and tilt."""
    return (corners.reshape(-1, 2) / np.array([w, h], np.float32)).ravel()


def novelty(vec, kept):
    if not kept:
        return 1.0
    return float(min(np.linalg.norm(vec - k) / np.sqrt(len(vec)) for k in kept))


def coverage_cell(corners, w, h):
    cx, cy = corners.reshape(-1, 2).mean(axis=0)
    return min(2, int(cy / h * 3)) * 3 + min(2, int(cx / w * 3))


def tilt_score(corners, board):
    """0 = board fronto-parallel, higher = more perspective foreshortening."""
    pts = corners.reshape(board.rows, board.cols, 2)
    top = np.linalg.norm(pts[0, -1] - pts[0, 0])
    bottom = np.linalg.norm(pts[-1, -1] - pts[-1, 0])
    left = np.linalg.norm(pts[-1, 0] - pts[0, 0])
    right = np.linalg.norm(pts[-1, -1] - pts[0, -1])
    hz = abs(top - bottom) / max(top, bottom, 1e-6)
    vt = abs(left - right) / max(left, right, 1e-6)
    return float(max(hz, vt))


def draw_hud(view, board, corners, state):
    """`corners` are already in preview coordinates -- do not rescale them."""
    h, w = view.shape[:2]

    # coverage grid
    for i in range(1, 3):
        cv2.line(view, (w * i // 3, 0), (w * i // 3, h), (60, 60, 60), 1)
        cv2.line(view, (0, h * i // 3), (w, h * i // 3), (60, 60, 60), 1)
    for cell in range(9):
        r, c = divmod(cell, 3)
        p0 = (c * w // 3 + 4, r * h // 3 + 4)
        p1 = ((c + 1) * w // 3 - 4, (r + 1) * h // 3 - 4)
        if state["cells"][cell]:
            cv2.rectangle(view, p0, p1, (0, 170, 0), 2)

    if corners is not None:
        cv2.drawChessboardCorners(view, board.size, corners, True)

    bar = np.zeros((66, w, 3), np.uint8)
    bar[:] = (28, 28, 28)
    kept, target = state["kept"], state["target"]
    colour = (0, 220, 0) if kept >= target else (240, 240, 240)
    cv2.putText(bar, f"{kept}/{target} views", (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    cv2.putText(bar, f"auto {'ON' if state['auto'] else 'OFF'}", (200, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 220, 0) if state["auto"] else (120, 120, 120), 2)
    cv2.putText(bar, f"tilted {state['tilted']}   cells {sum(state['cells'])}/9"
                     f"   {state['fps']:.0f} fps",
                (330, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    cv2.putText(bar, state["msg"], (12, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, state["msg_colour"], 1)
    cv2.putText(bar, "SPACE grab   A auto   U undo   Q done", (w - 400, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1)
    return np.vstack([view, bar])


def main():
    args = parse_args()
    board = load_board()
    os.makedirs(args.out, exist_ok=True)

    existing = sorted(f for f in os.listdir(args.out) if f.endswith(".png"))
    if existing:
        print(f"[!] {args.out} already holds {len(existing)} png(s); new files continue "
              f"the numbering. Delete them first for a clean run.")

    print(f"{board}")
    state = {"kept": 0, "target": args.target, "auto": not args.manual,
             "cells": [False] * 9, "tilted": 0, "msg": "show the board",
             "msg_colour": (200, 200, 200), "fps": 0.0}
    kept_vecs, saved_paths = [], []
    idx = len(existing)
    cooldown = 0
    last_t = time.perf_counter()

    with DahengCamera(exposure_us=args.exposure, gain_db=args.gain,
                      auto_exposure_max_us=args.auto_exposure_max) as cam:
        print(f"{cam.info['model_name']} sn={cam.info['sn']} {cam.width}x{cam.height}")
        for _ in range(20):  # let auto exposure/gain settle before reporting
            cam.read()
        exp, gain = cam.exposure_gain()
        print(f"exposure {exp:.0f} us, gain {gain:.1f} dB")
        if gain is not None and gain > 12:
            print("[!] Gain is near maximum -- the scene is dim and the images will be "
                  "noisy. Add light, or raise --auto-exposure-max (costs frame rate).")
        print("opening preview window -- it may take a second to appear")
        win = "checkerboard capture"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

        while True:
            now = time.perf_counter()
            state["fps"] = 0.8 * state["fps"] + 0.2 / max(now - last_t, 1e-6)
            last_t = now
            frame = cam.read()
            if frame is None:
                continue
            h, w = frame.shape[:2]
            scale = args.preview_width / w
            small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

            # Detect on the preview-sized image; refine at full res only on capture.
            corners_small = find_corners(small, board, fast=True)
            found = corners_small is not None
            if found:
                corners_small = corners_small.reshape(-1, 2)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("a"):
                state["auto"] = not state["auto"]
            if key == ord("u") and saved_paths:
                os.remove(saved_paths.pop())
                kept_vecs.pop()
                state["kept"] -= 1
                idx -= 1
                state["msg"] = "removed last view"
                state["msg_colour"] = (0, 170, 220)

            grab = key == ord(" ")
            if cooldown > 0:
                cooldown -= 1

            if found and (grab or (state["auto"] and cooldown == 0)):
                full = find_corners(frame, board)  # sub-pixel, full resolution
                if full is None:
                    state["msg"] = "corners lost at full res -- hold steadier"
                    state["msg_colour"] = (0, 140, 255)
                else:
                    gray = to_gray(frame)
                    sharp = sharpness(gray, full)
                    vec = pose_vector(full, w, h)
                    nov = novelty(vec, kept_vecs)
                    if sharp < args.min_sharpness and not grab:
                        state["msg"] = f"too blurry ({sharp:.0f})"
                        state["msg_colour"] = (0, 140, 255)
                    elif nov < args.novelty and not grab:
                        state["msg"] = "too similar -- move or tilt the board"
                        state["msg_colour"] = (0, 140, 255)
                    else:
                        idx += 1
                        path = os.path.join(args.out, f"calib_{idx:03d}.png")
                        cv2.imwrite(path, frame)
                        saved_paths.append(path)
                        kept_vecs.append(vec)
                        state["kept"] += 1
                        state["cells"][coverage_cell(full, w, h)] = True
                        # ~0.06 corresponds to roughly 20 deg out of plane at a
                        # typical working distance for a 540 mm board.
                        if tilt_score(full, board) > 0.06:
                            state["tilted"] += 1
                        state["msg"] = f"saved {os.path.basename(path)} (sharp {sharp:.0f})"
                        state["msg_colour"] = (0, 220, 0)
                        cooldown = 12
            elif not found:
                state["msg"] = "board not detected"
                state["msg_colour"] = (120, 120, 120)
            elif state["msg"].startswith("board not"):
                state["msg"] = "board locked"
                state["msg_colour"] = (200, 200, 200)

            cv2.imshow(win, draw_hud(small, board,
                                     corners_small.reshape(-1, 1, 2) if found else None,
                                     state))

        cv2.destroyAllWindows()

    print(f"\n{state['kept']} views saved to {args.out}")
    print(f"coverage {sum(state['cells'])}/9 cells, {state['tilted']} strongly tilted")
    if sum(state["cells"]) < 7 or state["tilted"] < 4:
        print("[!] Coverage is thin. Distortion coefficients need the board near the frame "
              "edges and at steep angles; consider capturing more before calibrating.")
    print("next:  python calibrate.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
