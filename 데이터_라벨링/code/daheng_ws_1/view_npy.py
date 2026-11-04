"""저장된 .npy 프레임을 눈으로 보기 위한 도구.

수집은 속도 때문에 raw Bayer 를 .npy 로 저장한다. 이미지 뷰어로는 열리지
않으므로 이 스크립트로 변환하거나 바로 띄워 본다.

--------------------------------------------------------------------------
  # 한 장만 창으로 띄우기
  python view_npy.py sessions/session_001/cam1/00000001.npy

  # 한 세션의 cam1 을 넘겨 보기 (←/→ 키, Q 로 종료)
  python view_npy.py sessions/session_001/cam1

  # 4대 같은 FrameID 를 2x2 로 나란히 (동시성 눈으로 확인)
  python view_npy.py sessions/session_001 --frame 50

  # png 로 변환해서 저장
  python view_npy.py sessions/session_001/cam1 --to-png
  python view_npy.py sessions/session_001 --to-png --every 10
--------------------------------------------------------------------------
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

# MER2-240-159U3C 는 Bayer GB8/GB10/GB12 (공식 매뉴얼 확인).
# meta.json 의 pixel_format 으로 자동 판별하되, 실패하면 이 값을 쓴다.
DEFAULT_BAYER = cv2.COLOR_BayerGR2BGR       # 카메라 BayerGB 용

# ※ OpenCV 의 Bayer 이름은 카메라(GenICam) 이름과 R/B 가 뒤바뀌어 있다.
#   카메라 BayerGB(0,0 이 G, 옆이 B) 에는 cv2.COLOR_BayerGR2BGR 가 맞다.
#   이름 그대로 BayerGB2BGR 를 쓰면 빨강이 파랑으로 나온다 (2026-09-22 실험 확인).
_BAYER_MAP = {
    "BayerRG": cv2.COLOR_BayerBG2BGR, "BayerGB": cv2.COLOR_BayerGR2BGR,
    "BayerGR": cv2.COLOR_BayerGB2BGR, "BayerBG": cv2.COLOR_BayerRG2BGR,
}


def bayer_code_from_meta(path):
    """세션 폴더를 거슬러 올라가며 meta.json 의 pixel_format 을 찾는다."""
    d = os.path.abspath(path if os.path.isdir(path) else os.path.dirname(path))
    for _ in range(4):
        meta = os.path.join(d, "meta.json")
        if os.path.exists(meta):
            try:
                import json
                with open(meta, encoding="utf-8") as f:
                    m = json.load(f)
                pf = (m.get("cameras") or [{}])[0].get("pixel_format") or ""
                for key, code in _BAYER_MAP.items():
                    if key in pf:
                        return code
            except Exception:
                pass
            break
        d = os.path.dirname(d)
    return DEFAULT_BAYER


def to_bgr(arr, code):
    """raw 배열을 화면에 띄울 수 있는 BGR 로."""
    if arr.ndim == 3:
        return arr
    if arr.dtype != np.uint8:
        arr = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    try:
        return cv2.cvtColor(arr, code)
    except cv2.error:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)


def fit(img, width):
    if img.shape[1] <= width:
        return img
    h = int(img.shape[0] * width / img.shape[1])
    return cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA)


def label(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(img, text, (10, 21), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 220, 120), 2)
    return img


# ===========================================================================
#  모드 1 — 한 장
# ===========================================================================
def show_one(path, width):
    code = bayer_code_from_meta(path)
    img = fit(to_bgr(np.load(path), code), width)
    label(img, os.path.basename(path))
    cv2.imshow("view_npy  (아무 키나 눌러 닫기)", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ===========================================================================
#  모드 2 — 한 카메라 폴더를 넘겨 보기
# ===========================================================================
def browse(cam_dir, width):
    files = sorted(glob.glob(os.path.join(cam_dir, "*.npy")))
    if not files:
        print(f"{cam_dir} 에 .npy 가 없습니다.")
        return 1

    code = bayer_code_from_meta(cam_dir)
    win = "view_npy  ←/→ 이동   Q 종료"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    i = 0
    print(f"{len(files)} 장.  ←/→ 로 이동, Q 로 종료")

    while True:
        img = fit(to_bgr(np.load(files[i]), code), width)
        label(img, f"{os.path.basename(files[i])}   [{i + 1}/{len(files)}]")
        cv2.imshow(win, img)

        k = cv2.waitKey(0) & 0xFF
        if k in (ord("q"), 27):
            break
        if k in (81, ord("a")):          # ← 또는 A
            i = max(0, i - 1)
        elif k in (83, ord("d"), 32):    # → 또는 D 또는 SPACE
            i = min(len(files) - 1, i + 1)

    cv2.destroyAllWindows()
    return 0


# ===========================================================================
#  모드 3 — 4대 같은 FrameID 를 2x2 로
# ===========================================================================
def show_quad(session_dir, frame_id, width):
    """같은 FrameID 라도 offset 이 다르면 다른 순간이다. 여기서는 파일명
    기준으로 나란히 놓을 뿐이며, 정확한 동시성 확인은 offset 을 반영한
    결합 단계에서 한다."""
    code = bayer_code_from_meta(session_dir)
    tile_w = width // 2
    tiles = []

    for i in range(1, 5):
        p = os.path.join(session_dir, f"cam{i}", f"{frame_id:08d}.npy")
        if os.path.exists(p):
            t = fit(to_bgr(np.load(p), code), tile_w)
            label(t, f"CAM{i}  id={frame_id}")
        else:
            h = int(tile_w * 1200 / 2048)
            t = np.zeros((h, tile_w, 3), np.uint8)
            label(t, f"CAM{i}  없음")
        tiles.append(t)

    h = min(t.shape[0] for t in tiles)
    tiles = [t[:h] for t in tiles]
    grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    cv2.imshow(f"view_npy  frame {frame_id}  (아무 키나 눌러 닫기)", grid)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return 0


# ===========================================================================
#  모드 4 — png 변환
# ===========================================================================
def convert(path, every, quality_png=3):
    if os.path.isdir(path):
        cams = ([path] if glob.glob(os.path.join(path, "*.npy"))
                else sorted(glob.glob(os.path.join(path, "cam*"))))
    else:
        cams = [os.path.dirname(path)]

    code = bayer_code_from_meta(path)
    total = 0

    for cam_dir in cams:
        files = sorted(glob.glob(os.path.join(cam_dir, "*.npy")))
        if not files:
            continue
        out_dir = os.path.join(cam_dir, "png")
        os.makedirs(out_dir, exist_ok=True)

        picked = files[::every] if every > 1 else files
        for n, f in enumerate(picked, 1):
            img = to_bgr(np.load(f), code)
            name = os.path.splitext(os.path.basename(f))[0] + ".png"
            cv2.imwrite(os.path.join(out_dir, name), img,
                        [cv2.IMWRITE_PNG_COMPRESSION, quality_png])
            if n % 20 == 0 or n == len(picked):
                print(f"\r  {os.path.basename(cam_dir)}: {n}/{len(picked)}",
                      end="", flush=True)
        print(f"\r  {os.path.basename(cam_dir)}: {len(picked)}장 → {out_dir}")
        total += len(picked)

    print(f"\n{total} 장 변환 완료.")
    return 0


# ===========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help=".npy 파일 / camN 폴더 / 세션 폴더")
    ap.add_argument("--to-png", action="store_true", help="창 대신 png 로 변환")
    ap.add_argument("--every", type=int, default=1,
                    help="N장마다 하나만 변환 (--to-png 와 함께)")
    ap.add_argument("--frame", type=int, default=None,
                    help="세션 폴더에서 이 FrameID 를 4대 나란히")
    ap.add_argument("--width", type=int, default=1200, help="표시 가로 폭")
    args = ap.parse_args()

    p = os.path.abspath(args.path)
    if not os.path.exists(p):
        print(f"경로가 없습니다: {p}")
        return 1

    if args.to_png:
        return convert(p, max(1, args.every))
    if args.frame is not None:
        return show_quad(p, args.frame, args.width)
    if os.path.isfile(p):
        show_one(p, args.width)
        return 0
    if glob.glob(os.path.join(p, "*.npy")):
        return browse(p, args.width)

    # 세션 폴더인데 --frame 이 없으면 cam1 을 넘겨 본다
    cam1 = os.path.join(p, "cam1")
    if os.path.isdir(cam1):
        print("cam1 을 엽니다. 4대를 나란히 보려면 --frame 50 처럼 지정하세요.")
        return browse(cam1, args.width)

    print("표시할 .npy 를 찾지 못했습니다.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
