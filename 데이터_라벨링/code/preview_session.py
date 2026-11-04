"""세션 미리보기 — 카메라 프레임과 라이다 회전을 눈으로 확인한다.

    python3 preview_session.py sessions/session_003              스틸 1장
    python3 preview_session.py sessions/session_003 --gif        움직이는 GIF
    python3 preview_session.py sessions/session_003 --frame 30

검산은 숫자로 하고(check_session.py), 이건 "정말 찍혔나" 를 눈으로 보는 용도다.
숫자가 다 통과해도 렌즈 캡이 씌워져 있거나 라이다가 벽만 보고 있을 수 있다.

--------------------------------------------------------------------------
카메라 밝기

  raw Bayer 를 그대로 저장하므로 화면용으로는 어둡다 (이 리그는 화소 평균
  10/255). 퍼센타일로 정규화하고 감마를 먹여서 보이게만 만든다.
  --no-boost 로 끄면 저장된 값 그대로 나온다.
"""

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import vlp16  # noqa: E402

BAYER_CV = {
    "BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
    "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR,
}
# 10·12·16 비트도 같은 Bayer 배열이다. 비트수만 다르다.
for _b in ("RG", "GB", "GR", "BG"):
    for _d in ("10", "12", "16"):
        BAYER_CV[f"Bayer{_b}{_d}"] = BAYER_CV[f"Bayer{_b}8"]


def _to8(arr, pixfmt):
    """12 비트(uint16) 로 찍힌 것을 8 비트로 내린다.

    session_037 부터 BayerGB12 로 찍는데, 변환표에 8 비트만 있어서
    디모자이킹을 건너뛰고 흑백으로 나왔다. 색은 원본에 그대로 있다.
    """
    if arr.dtype != np.uint16:
        return arr
    bits = 16
    for b in ("10", "12", "16"):
        if pixfmt and pixfmt.endswith(b):
            bits = int(b)
            break
    return (arr >> (bits - 8)).astype(np.uint8)
# Bayer 2x2 배열에서 각 색이 어느 자리인지. OpenCV 이름 기준.
BAYER_POS = {
    "GB": {"G": [(0, 0), (1, 1)], "B": [(0, 1)], "R": [(1, 0)]},
    "BG": {"B": [(0, 0)], "G": [(0, 1), (1, 0)], "R": [(1, 1)]},
    "RG": {"R": [(0, 0)], "G": [(0, 1), (1, 0)], "B": [(1, 1)]},
    "GR": {"G": [(0, 0), (1, 1)], "R": [(0, 1)], "B": [(1, 0)]},
}


def _wb_bayer(arr, pixfmt):
    """디모자이킹 **전에** Bayer 평면에서 화이트밸런스를 건다.

    Bayer 는 녹색 화소가 적·청의 두 배라 그냥 변환하면 전체가 초록으로 뜬다.
    디모자이킹 후에 맞추면 이미 섞인 색을 되돌리는 셈이라 덜 정확하다.

    게인은 **채널 평균**으로 구한다. 퍼센타일(99.5%)로 구하면 밝은 조명이
    있는 장면에서 세 채널이 다 포화해 게인이 1.0 으로 나오고 보정이 통째로
    사라진다 (session_039 cam3 이 그랬다).
    """
    if arr.ndim != 2 or not pixfmt or not pixfmt.startswith("Bayer"):
        return arr
    pos = BAYER_POS.get(pixfmt[5:7])
    if pos is None:
        return arr
    f = arr.astype(np.float32)
    mean = {}
    for ch, cells in pos.items():
        mean[ch] = float(np.mean([f[dy::2, dx::2].mean() for dy, dx in cells]))
    if min(mean.values()) < 1.0:
        return arr
    for ch, cells in pos.items():
        g = float(np.clip(mean["G"] / mean[ch], 0.25, 4.0))
        if abs(g - 1.0) < 1e-3:
            continue
        for dy, dx in cells:
            f[dy::2, dx::2] *= g
    top = 4095.0 if (pixfmt.endswith("12") or arr.dtype == np.uint16) else 255.0
    if pixfmt.endswith("10"):
        top = 1023.0
    elif pixfmt.endswith("16"):
        top = 65535.0
    return np.clip(f, 0, top).astype(arr.dtype)

FONT = cv2.FONT_HERSHEY_SIMPLEX

# cv2.putText 는 한글을 못 그린다 (전부 '?' 로 나온다). 캡션에만 PIL 을 쓴다.
_FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]
_font_cache = {}


def _pil_font(size):
    if size in _font_cache:
        return _font_cache[size]
    from PIL import ImageFont
    f = None
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                f = ImageFont.truetype(path, size)
                break
            except OSError:
                continue
    _font_cache[size] = f or ImageFont.load_default()
    return _font_cache[size]


def draw_text(img, text, xy, size=15, color=(220, 220, 220)):
    """BGR numpy 이미지에 한글이 되는 글자를 얹는다."""
    from PIL import Image, ImageDraw
    pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(pil).text(xy, text, font=_pil_font(size),
                             fill=(color[2], color[1], color[0]))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


# ===========================================================================
#  카메라
# ===========================================================================
def load_frames_csv(cam_dir):
    rows = []
    path = os.path.join(cam_dir, "frames.csv")
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((int(r["frame_id"]), float(r["host_recv_unix"])))
            except (ValueError, KeyError):
                continue
    rows.sort()
    return rows


def load_cams(session_dir, meta):
    """카메라 폴더별로 frames.csv 와 PixelFormat 을 모은다.

    PixelFormat 은 meta.json 의 cameras 배열에 런타임 순서로 들어 있고,
    폴더 이름은 물리 슬롯 번호다. 둘을 camera_slots 로 이어 준다.
    (지금 이 리그는 4대 다 BayerGB8 이라 결과는 같지만, 한 대만 다른
    포맷으로 열렸을 때 조용히 색이 틀어지는 것을 막는다.)
    """
    slots = meta.get("camera_slots") or []
    pixfmt_by_slot = {}
    for i, c in enumerate(meta.get("cameras", [])):
        slot = slots[i] if i < len(slots) else i + 1
        pixfmt_by_slot[slot] = c.get("pixel_format")

    out = []
    for d in sorted(x for x in os.listdir(session_dir)
                    if x.startswith("cam") and x[3:].isdigit() and
                    os.path.isdir(os.path.join(session_dir, x))):
        cdir = os.path.join(session_dir, d)
        rows = load_frames_csv(cdir)
        if not rows:
            continue
        out.append({"name": d, "dir": cdir, "rows": rows,
                    "pixfmt": pixfmt_by_slot.get(int(d[3:])) or "BayerGB8"})
    return out


def render_camera_grid(cams, t, tile_w, boost=True, ncols=2):
    """카메라 여러 대를 격자로 깐다. 각 타일에 라이다와의 시차를 적는다.

    프리런(MCU 없음)으로 찍으면 4대가 각자 자기 클럭으로 셔터를 열기
    때문에, 같은 회전에 가장 가까운 프레임이 카메라마다 다른 시각이다.
    그 차이를 타일마다 보여 줘야 "네 대가 같은 순간을 본 것"으로
    잘못 읽지 않는다.
    """
    tiles = []
    for c in cams:
        fid, ct = min(c["rows"], key=lambda r: abs(r[1] - t))
        img = render_camera(c["dir"], fid, c["pixfmt"], tile_w, boost)
        if img is None:
            img = np.full((int(round(tile_w * 1200 / 2048)), tile_w, 3),
                          24, np.uint8)
            img = draw_text(img, f"{c['name']}  (파일 없음)", (8, 6), size=14,
                            color=(120, 120, 200))
        else:
            img = draw_text(img, f"{c['name']}  #{fid}  "
                                 f"{(ct - t) * 1000:+.0f} ms", (8, 6), size=14)
        tiles.append(img)

    if len(tiles) == 1:
        return tiles[0]

    gap = 4
    h = max(x.shape[0] for x in tiles)
    tiles = [_pad_to(x, h) for x in tiles]
    rows_img = []
    for i in range(0, len(tiles), ncols):
        row = tiles[i:i + ncols]
        while len(row) < ncols:      # 3대 같은 홀수면 빈 칸으로 채운다
            row.append(np.full((h, tile_w, 3), 24, np.uint8))
        strip = [row[0]]
        for x in row[1:]:
            strip += [np.full((h, gap, 3), 24, np.uint8), x]
        rows_img.append(np.hstack(strip))
    w = rows_img[0].shape[1]
    out = [rows_img[0]]
    for r in rows_img[1:]:
        out += [np.full((gap, w, 3), 24, np.uint8), r]
    return np.vstack(out)


def render_camera(cam_dir, fid, pixfmt, width, boost=True):
    npy = os.path.join(cam_dir, f"{fid:08d}.npy")
    png = os.path.join(cam_dir, f"{fid:08d}.png")
    if os.path.exists(npy):
        arr = np.load(npy)
    elif os.path.exists(png):
        arr = cv2.imread(png, cv2.IMREAD_UNCHANGED)
    else:
        return None

    if arr.ndim == 2:
        arr = _wb_bayer(arr, pixfmt)
        arr = _to8(arr, pixfmt)
        code = BAYER_CV.get(pixfmt)
        arr = (cv2.cvtColor(arr, code) if code
               else cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR))

    if boost:
        # 화이트밸런스는 _wb_bayer 에서 이미 걸었다. 여기서는 밝기만 올린다.
        # 상위 0.5% 를 흰색으로 잡고 감마로 어두운 쪽을 끌어올린다.
        f = arr.astype(np.float32)
        hi = max(float(np.percentile(f, 99.5)), 1.0)
        f = np.clip(f / hi, 0, 1)
        arr = (np.power(f, 0.55) * 255).astype(np.uint8)

    h = int(round(width * arr.shape[0] / arr.shape[1]))
    return cv2.resize(arr, (width, h), interpolation=cv2.INTER_AREA)


# ===========================================================================
#  라이다 — 위에서 내려다본 그림
# ===========================================================================
def render_bev(xyz, intensity, size=700, r_max=12.0, by="height"):
    img = np.full((size, size, 3), 18, np.uint8)
    c = size // 2
    scale = (size / 2 - 10) / r_max

    # 거리 링
    for r in range(2, int(r_max) + 1, 2):
        cv2.circle(img, (c, c), int(r * scale), (52, 52, 52), 1, cv2.LINE_AA)
        cv2.putText(img, f"{r}m", (c + 4, c - int(r * scale) + 14),
                    FONT, 0.38, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(img, (c, 0), (c, size), (44, 44, 44), 1)
    cv2.line(img, (0, c), (size, c), (44, 44, 44), 1)

    if len(xyz):
        # ROS 관례: x 전방(그림 위), y 좌측(그림 왼쪽)
        col = np.round(c - xyz[:, 1] * scale).astype(np.int32)
        row = np.round(c - xyz[:, 0] * scale).astype(np.int32)
        m = (col >= 0) & (col < size) & (row >= 0) & (row < size)

        if by == "intensity":
            v = np.clip(intensity[m] / 80.0, 0, 1)
        else:
            v = np.clip((xyz[m][:, 2] + 1.5) / 3.5, 0, 1)
        lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8),
                                cv2.COLORMAP_TURBO).reshape(256, 3)
        img[row[m], col[m]] = lut[(v * 255).astype(np.uint8)]

    # 센서 위치
    cv2.circle(img, (c, c), 4, (255, 255, 255), -1, cv2.LINE_AA)
    return draw_text(img, "전방", (c + 8, 6), size=14, color=(170, 170, 170))


def render_side(xyz, size_w=700, size_h=260, r_max=12.0):
    """옆에서 본 그림. 지면과 천장이 제대로 잡혔는지 보기 위한 것."""
    img = np.full((size_h, size_w, 3), 18, np.uint8)
    scale = (size_w / 2 - 10) / r_max
    cx, cz = size_w // 2, size_h // 2
    for r in range(2, int(r_max) + 1, 2):
        for s in (-1, 1):
            cv2.line(img, (int(cx + s * r * scale), 0),
                     (int(cx + s * r * scale), size_h), (44, 44, 44), 1)
    cv2.line(img, (0, cz), (size_w, cz), (52, 52, 52), 1)

    if len(xyz):
        col = np.round(cx - xyz[:, 1] * scale).astype(np.int32)
        row = np.round(cz - xyz[:, 2] * scale * 2.2).astype(np.int32)
        m = (col >= 0) & (col < size_w) & (row >= 0) & (row < size_h)
        v = np.clip((xyz[m][:, 2] + 1.5) / 3.5, 0, 1)
        lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8),
                                cv2.COLORMAP_TURBO).reshape(256, 3)
        img[row[m], col[m]] = lut[(v * 255).astype(np.uint8)]
    return draw_text(img, "옆에서 본 모습 (높이 2.2배 강조)", (8, 6),
                     size=13, color=(150, 150, 150))


# ===========================================================================
#  합성
# ===========================================================================
def label(img, text):
    bar = np.full((30, img.shape[1], 3), 30, np.uint8)
    bar = draw_text(bar, text, (10, 6), size=15)
    return np.vstack([bar, img])


def _pad_to(img, h):
    if img.shape[0] >= h:
        return img
    return np.vstack([img,
                      np.full((h - img.shape[0], img.shape[1], 3), 24, np.uint8)])


def compose(cam_img, bev, side, caption):
    """왼쪽에 카메라와 측면도를 쌓고, 오른쪽에 평면도를 둔다.

    카메라(2048x1200)는 가로가 길어 평면도(정사각형) 옆에 그냥 붙이면
    아래쪽이 통째로 비어 버린다. 측면도를 그 아래에 채워 높이를 맞춘다.
    """
    gap = 6
    if cam_img is not None:
        left = np.vstack([cam_img,
                          np.full((gap, cam_img.shape[1], 3), 24, np.uint8),
                          side])
    else:
        left = side
    h = max(left.shape[0], bev.shape[0])
    left, right = _pad_to(left, h), _pad_to(bev, h)
    grid = np.hstack([left, np.full((h, gap, 3), 24, np.uint8), right])
    return label(grid, caption)


def write_mp4(frames_bgr, out, fps, crf=22):
    """프레임을 ffmpeg 표준입력으로 흘려보내 mp4 로 만든다.

    GIF 는 프레임당 256색이라 포인트클라우드 색이 뭉치고 파일도 크다.
    브라우저에서 볼 것이라면 h264 쪽이 작고 깨끗하다.
    """
    import subprocess
    h, w = frames_bgr[0].shape[:2]
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
        # yuv420p 는 가로세로가 짝수여야 한다. 홀수면 1px 잘라 맞춘다.
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v", "libx264", "-preset", "slow", "-crf", str(crf),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        out,
    ]
    pr = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames_bgr:
        pr.stdin.write(np.ascontiguousarray(f).tobytes())
    pr.stdin.close()
    if pr.wait() != 0:
        raise RuntimeError("ffmpeg 인코딩 실패")


def build(session_dir, out, frame_idx=None, gif=False, fps=10,
          boost=True, cam_width=None, color_by="height", max_frames=60,
          scale=1.0, mp4=False):
    session_dir = os.path.abspath(session_dir)
    meta = {}
    mp = os.path.join(session_dir, "meta.json")
    if os.path.exists(mp):
        meta = json.load(open(mp, encoding="utf-8"))

    cams = load_cams(session_dir, meta)

    # 카메라가 여러 대면 2열 격자로 깐다. cam_width 는 격자 전체의 폭이므로
    # 타일 하나는 그 절반이 된다. 4대를 700 폭에 욱여넣으면 타일이 350 이라
    # 렌즈 캡이 씌워졌는지 정도밖에 안 보인다. 그래서 기본값을 늘린다.
    ncols = 2 if len(cams) > 1 else 1
    if cam_width is None:
        cam_width = 1200 if ncols == 2 else 700
    tile_w = cam_width // ncols

    pcap = os.path.join(session_dir, "lidar.pcap")
    print("라이다 디코딩 중...")
    d = vlp16.decode(pcap)
    revs = [(a, b) for a, b in d.revolutions()]
    if len(revs) >= 3:
        revs = revs[1:-1]          # 앞뒤 잘린 회전 제외
    print(f"  회전 {len(revs)} 개, 카메라 "
          + (", ".join(f"{c['name']} {len(c['rows'])}장" for c in cams)
             or "없음"))

    name = os.path.basename(session_dir)

    def one(rev_i):
        a, b = revs[rev_i]
        t = float(d.host_ts[a])
        cam_img = (render_camera_grid(cams, t, tile_w, boost, ncols)
                   if cams else None)
        bev = render_bev(d.xyz[a:b], d.intensity[a:b], by=color_by)
        side_w = cam_img.shape[1] if cam_img is not None else cam_width
        side = render_side(d.xyz[a:b], size_w=side_w,
                           size_h=max(160, 700 - (cam_img.shape[0] if
                                                  cam_img is not None else 0) - 6))
        cap = (f"{name}   t={t - float(d.host_ts[revs[0][0]]):5.2f}s   "
               f"rev {rev_i + 1}/{len(revs)}   점 {b - a:,}   "
               f"카메라 {len(cams)}대 (숫자는 라이다 회전과의 시차)")
        return compose(cam_img, bev, side, cap)

    if gif or mp4:
        n = min(len(revs), max_frames)
        idxs = np.linspace(0, len(revs) - 1, n).astype(int)
        frames = []
        for k, i in enumerate(idxs):
            f = one(i)
            if scale != 1.0:
                f = cv2.resize(f, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
            frames.append(f)
            print(f"\r  프레임 {k + 1}/{n}", end="", flush=True)
        print()
        if mp4:
            write_mp4(frames, out, fps)
            print(f"저장: {out}  "
                  f"({os.path.getsize(out) / 1e6:.1f} MB)")
            return out
        frames = [cv2.cvtColor(f, cv2.COLOR_BGR2RGB) for f in frames]
        from PIL import Image
        # GIF 는 프레임당 256색이다. 첫 프레임에서 만든 팔레트를 전 프레임에
        # 공용으로 쓰면 색이 프레임마다 튀지 않고 파일도 작아진다.
        imgs = [Image.fromarray(f) for f in frames]
        pal = imgs[0].convert("P", palette=Image.ADAPTIVE, colors=128)
        imgs = [im.quantize(palette=pal, dither=Image.NONE) for im in imgs]
        imgs[0].save(out, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=True)
    else:
        i = frame_idx if frame_idx is not None else len(revs) // 2
        i = max(0, min(i, len(revs) - 1))
        cv2.imwrite(out, one(i))
    print(f"저장: {out}  ({os.path.getsize(out) / 1e6:.1f} MB)")
    return out


def main():
    ap = argparse.ArgumentParser(description="세션 미리보기 렌더링")
    ap.add_argument("session")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--gif", action="store_true", help="움직이는 GIF 로 저장")
    ap.add_argument("--mp4", action="store_true",
                    help="mp4 로 저장 (GIF 보다 작고 색이 깨끗하다)")
    ap.add_argument("--frame", type=int, default=None, help="스틸로 쓸 회전 번호")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--max-frames", type=int, default=60)
    ap.add_argument("--cam-width", type=int, default=None,
                    help="카메라 영역 전체 폭 (기본: 1대 700, 여러 대 1200)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="GIF 출력 배율 (파일 크기 줄일 때)")
    ap.add_argument("--color", choices=["height", "intensity"],
                    default="height", help="점 색을 무엇으로 칠할지")
    ap.add_argument("--no-boost", action="store_true",
                    help="카메라 밝기 보정을 끈다")
    args = ap.parse_args()

    ext = "preview.mp4" if args.mp4 else (
        "preview.gif" if args.gif else "preview.png")
    out = args.out or os.path.join(os.path.abspath(args.session), ext)
    build(args.session, out, frame_idx=args.frame, gif=args.gif,
          fps=args.fps, boost=not args.no_boost, cam_width=args.cam_width,
          color_by=args.color, max_frames=args.max_frames, scale=args.scale,
          mp4=args.mp4)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
