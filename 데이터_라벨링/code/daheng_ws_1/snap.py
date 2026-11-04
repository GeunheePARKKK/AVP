"""Grab one frame and report exactly why the board did or didn't detect.

    python snap.py            # preview, SPACE to snap
    python snap.py out.png    # save the snapshot somewhere specific

Use this when capture_board.py sits at "board not detected" and it isn't obvious
why. It saves the frame, then reports which grid sizes detect on it -- so you can
tell a cut-off board (a smaller grid detects) from an occluded one (nothing does).
"""

import os
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from board import load_board  # noqa: E402
from detect_board_size import VERIFY, _search_gray, analyse  # noqa: E402
from dhcam import DahengCamera  # noqa: E402


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "snapshot.png")
    b = load_board()
    frame = None

    with DahengCamera() as cam:
        print(f"{cam.info['model_name']} {cam.width}x{cam.height}")
        for _ in range(20):
            cam.read()
        exp, gain = cam.exposure_gain()
        print(f"exposure {exp:.0f} us, gain {gain:.1f} dB")
        win = "snap -- SPACE to grab, Q to cancel"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        while True:
            f = cam.read()
            if f is None:
                continue
            sc = 900 / f.shape[1]
            view = cv2.resize(f, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
            cv2.putText(view, "SPACE grab    Q cancel", (18, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.imshow(win, view)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                frame = f
                break
        cv2.destroyAllWindows()

    if frame is None:
        print("cancelled")
        return 1

    cv2.imwrite(out, frame)
    print(f"saved {out}")

    gray = _search_gray(frame)
    configured = cv2.findChessboardCornersSB(gray, b.size, VERIFY)[0]
    print(f"\nconfigured {b.cols} x {b.rows}: {'DETECTED' if configured else 'not detected'}")

    if not configured:
        print("searching for any grid that does detect ", end="", flush=True)
        found = analyse(gray)
        if found is None:
            print("\n  -> nothing detects at any size.")
            print("     Something is covering part of the pattern, or the board is out "
                  "of frame / out of focus / too dark.")
            print("     Tape, stickers or glare touching the checker pattern are the "
                  "usual cause: covering two inner corners failed in every test, and "
                  "even one is a coin flip. Keep tape on the white margin or the back.")
        else:
            print(f"\n  -> largest detectable grid is {found[0]} x {found[1]}")
            if found[0] * found[1] < b.cols * b.rows:
                print(f"     Smaller than the configured {b.cols} x {b.rows}: part of the "
                      f"board is cut off by the frame edge, or is obscured.")
            else:
                print(f"     Larger than configured -- update board.json "
                      f"(detect_board_size.py --write).")
    else:
        vis = frame.copy()
        ok, corners = cv2.findChessboardCornersSB(
            cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY), b.size, VERIFY)
        if ok:
            cv2.drawChessboardCorners(vis, b.size, corners, True)
        sc = 900 / vis.shape[1]
        cv2.imshow("detected -- any key to close",
                   cv2.resize(vis, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
