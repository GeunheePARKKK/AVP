"""Work out the board's true inner-corner count.

    python detect_board_size.py            # live preview, SPACE to analyse
    python detect_board_size.py img.png    # analyse an existing image

Count **inner corners** (where four squares meet), not squares -- an 11 x 10
square board has 10 x 9 inner corners.

Getting this number wrong is quiet and expensive: OpenCV will not report "9x6 not
found" on a 10x9 board, it will lock onto a 9x6 *sub-grid*, land on a different
sub-grid in each view, and calibration then fits garbage with no symptom until the
reprojection error comes out enormous.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VERIFY = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
# CALIB_CB_LARGER reports *a* grid bigger than the seed, but which one depends on the
# seed and flags -- on one test frame it returned 5x9, 9x9, 9x5 and 10x9 for different
# combinations. So it is only used to propose candidates; every candidate is then
# confirmed with an ordinary fixed-size detection.
_SEEDS = [((5, 5), cv2.CALIB_CB_LARGER | VERIFY),
          ((6, 4), cv2.CALIB_CB_LARGER | VERIFY),
          ((9, 6), cv2.CALIB_CB_LARGER | VERIFY),
          ((4, 4), cv2.CALIB_CB_LARGER | cv2.CALIB_CB_NORMALIZE_IMAGE),
          ((5, 5), cv2.CALIB_CB_LARGER)]


# Searching at full 2048 px costs ~400 ms per size tried. At 1024 px it is ~170 ms
# and measured *more* reliable on real captures (5/8 vs 4/8 true-size hits), with
# wrong sizes rejected 8/8 at every scale -- no false positives to worry about.
SEARCH_WIDTH = 1024


def _say(msg, end="\n"):
    print(msg, end=end, flush=True)


def _search_gray(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    if gray.shape[1] > SEARCH_WIDTH:
        sc = SEARCH_WIDTH / gray.shape[1]
        gray = cv2.resize(gray, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
    return gray


def confirm(gray, size):
    """True if this exact grid detects. Tries the transpose too, since the meta
    output's axis order is not documented in a way worth trusting."""
    for s in (size, (size[1], size[0])):
        if cv2.findChessboardCornersSB(gray, s, VERIFY)[0]:
            return s
    return None


def analyse(gray):
    """Return (cols, rows) of the largest grid that actually detects, or None."""
    candidates = set()
    for seed, flags in _SEEDS:
        try:
            ok, _, meta = cv2.findChessboardCornersSBWithMeta(gray, seed, flags)
        except Exception:
            continue
        if ok and meta is not None:
            candidates.add(tuple(sorted(meta.shape, reverse=True)))
        _say(".", end="")
    _say("")

    if not candidates:
        return None

    # The seeds under-report as often as not, so probe a margin above the largest
    # candidate rather than trusting it outright.
    big = max(candidates, key=lambda s: s[0] * s[1])
    probe = {c for c in candidates}
    for dc in range(0, 3):
        for dr in range(0, 3):
            probe.add((big[0] + dc, big[1] + dr))

    best = None
    for size in sorted(probe, key=lambda s: -s[0] * s[1]):
        got = confirm(gray, size)
        if got is not None:
            best = got
            break
        _say(".", end="")
    _say("")
    return best


def preview_and_grab(n_frames):
    """Live window so the board can be framed before analysis. Returns frames."""
    from dhcam import DahengCamera

    grabbed = []
    with DahengCamera() as cam:
        _say(f"{cam.info['model_name']} {cam.width}x{cam.height}")
        win = "board size -- fill the frame with the WHOLE board, then SPACE"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        while True:
            frame = cam.read()
            if frame is None:
                continue
            h, w = frame.shape[:2]
            sc = 900 / w
            view = cv2.resize(frame, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
            vh, vw = view.shape[:2]
            cv2.rectangle(view, (8, 8), (vw - 8, vh - 8), (0, 200, 255), 2)
            for i, line in enumerate([
                    "the ENTIRE board incl. white border must sit inside this box",
                    "hold it flat and roughly square to the camera",
                    "SPACE analyse    Q cancel"]):
                cv2.putText(view, line, (18, 34 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 200, 255), 2)
            cv2.imshow(win, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                for _ in range(n_frames):
                    f = cam.read()
                    if f is not None:
                        grabbed.append(f)
                break
        cv2.destroyAllWindows()
    return grabbed


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("image", nargs="?", help="image to analyse; omit for live preview")
    p.add_argument("--frames", type=int, default=3, help="frames to agree on (default 3)")
    p.add_argument("--write", action="store_true", help="save the result into board.json")
    args = p.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            _say(f"cannot read {args.image}")
            return 1
        frames = [img]
    else:
        frames = preview_and_grab(args.frames)
        if not frames:
            _say("cancelled")
            return 1

    results = []
    for i, frame in enumerate(frames, 1):
        _say(f"analysing frame {i}/{len(frames)} ", end="")
        size = analyse(_search_gray(frame))
        _say(f"  -> {size[0]} x {size[1]}" if size else "  -> nothing found")
        if size:
            results.append(size)

    if not results:
        _say("\nNo checkerboard detected. Check focus, lighting, and that the whole "
             "board is in frame.")
        return 1

    best = max(set(results), key=results.count)
    agree = results.count(best)
    _say(f"\n==> board is {best[0]} x {best[1]} inner corners "
         f"({best[0] + 1} x {best[1] + 1} squares), {agree}/{len(results)} frames agree")
    if agree < len(results):
        _say("[!] Frames disagree -- some probably had the board cut off. Re-run with "
             "the whole board inside the frame.")

    if not args.image:
        vis = frames[-1].copy()
        ok, corners = cv2.findChessboardCornersSB(
            cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY), best, VERIFY)
        if ok:
            cv2.drawChessboardCorners(vis, best, corners, True)
        sc = 900 / vis.shape[1]
        cv2.imshow("detected grid -- any key to close",
                   cv2.resize(vis, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if args.write:
        path = os.path.join(HERE, "board.json")
        with open(path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        cfg["cols"], cfg["rows"] = int(best[0]), int(best[1])
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        _say(f"wrote cols={best[0]} rows={best[1]} to {path}")
        _say(f"    square_size_mm is still {cfg['square_size_mm']} -- measure it with a "
             f"ruler, this tool cannot infer physical size from an image")
    else:
        _say("Set these in board.json, or re-run with --write.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
