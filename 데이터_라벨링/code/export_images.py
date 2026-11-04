"""raw Bayer npy → 보기 좋은 JPEG. 라벨링 툴에 넣을 이미지를 만든다.

    python3 export_images.py sessions/session_025 -o sessions/session_025/images
    python3 export_images.py sessions/session_025 -o /tmp/img --method 3      # 빠른 버전
    python3 export_images.py sessions/session_025 -o /tmp/img --jobs 4

--------------------------------------------------------------------------
왜 보정이 필요한가

  실내 주차장을 노출 8 ms · 게인 8 dB 로 찍어서 raw 평균 밝기가 4.7/255 다.
  디베이어만 하면 거의 검정이라 사람이 물체를 못 알아본다. 후처리는 없는
  정보를 만드는 게 아니라 있는 신호를 펴는 것이므로 노이즈도 같이 커진다.

  방식 3 : 퍼센타일 0.1~99.9 로 늘리고 감마 0.35.      빠르고 노이즈 적다
  방식 4 : 3 에 CLAHE(국소 히스토그램 평활화) 추가.     어두운 데가 더 보이나 노이즈 증폭
  방식 5 : 4 에 비국소 평균 노이즈 제거.                가장 깨끗하나 장당 2 초
  방식 7 : 아래 v2 파이프라인 (기본값).

  방식 3~5 의 문제는 순서다. 디베이어 '전에' 감마를 걸면 베이어 격자의
  각 색 채널에 서로 다른 기울기가 적용된 뒤 보간되므로, 가장자리에 색
  프린지와 지퍼 무늬가 남는다. 화이트밸런스도 없어서 형광등 조명의
  청록 캐스트가 그대로 간다.

  방식 7 은 순서를 바로잡는다.
      블랙레벨 차감 → 선형 상태로 디베이어(EA) → 밝은 영역에서 광원 추정해
      화이트밸런스 → 톤매핑 → 색노이즈 위주 제거 → 휘도만 CLAHE·언샤프

  어느 방식이든 없는 정보를 만들지는 못한다. 근본 해결은 촬영 설정이고,
  sync_cam.py 의 GAMMA_ENABLE / LIGHT_SOURCE_PRESET 주석에 실측값이 있다.
"""

import argparse
import csv
import json
import os
from multiprocessing import Pool

import cv2
import numpy as np

#  10/12 bit 도 디베이어 코드는 8 bit 와 같다. 다른 건 입력 dtype 과
#  최대값(1023 / 4095)뿐이라 _white() 로 따로 다룬다.
BAYER = {"BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
         "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR}
for _b in ("RG", "GB", "GR", "BG"):
    for _d in ("10", "12", "16"):
        BAYER[f"Bayer{_b}{_d}"] = BAYER[f"Bayer{_b}8"]


def _white(fmt, arr):
    """해당 픽셀포맷의 포화값. npy 에는 비트깊이가 안 남으므로 이름으로 센다."""
    for d, w in (("16", 65535.0), ("12", 4095.0), ("10", 1023.0)):
        if fmt.endswith(d):
            return w
    return 255.0


BAYER_EA = {"BayerRG8": cv2.COLOR_BayerRG2BGR_EA, "BayerGB8": cv2.COLOR_BayerGB2BGR_EA,
            "BayerGR8": cv2.COLOR_BayerGR2BGR_EA, "BayerBG8": cv2.COLOR_BayerBG2BGR_EA}
for _b in ("RG", "GB", "GR", "BG"):
    for _d in ("10", "12", "16"):
        BAYER_EA[f"Bayer{_b}{_d}"] = BAYER_EA[f"Bayer{_b}8"]


#  톤매핑이 걸 수 있는 최대 증폭. 화이트포인트로 나누는 식이라 장면이
#  어두울수록 배율이 올라가는데, 신호가 없는 카메라에서는 그게 전부
#  노이즈 증폭이 된다. 실측 (야간 노상, 12 bit):
#
#      CAM3 (조명 보임)   p99.7=255.0 → x 1.0   정상
#      CAM1              p99.7= 15.7 → x16.2   노이즈만 증폭
#      CAM4              p99.7=  9.6 → x26.6   마젠타로 끓음
#
#  마젠타가 되는 이유는 WB 게인이 raw 에 이미 박혀 있어서다. 형광등
#  프리셋의 B 게인(~2.7x)이 신호뿐 아니라 B 노이즈도 같이 키워서,
#  CAM4 암부 노이즈 std 가 R1.4 / G1.0 / B2.2 로 기울어 있다. 거기에
#  x26 을 걸면 색이 보라로 뜬다.
#
#  주차장 실측은 x1.0~2.2 라 원래 문제가 안 되는 구간이다. 상한은
#  '안 보이는 카메라를 억지로 들어올리지 않는다'는 안전장치다.
MAX_TONE_GAIN = 8.0
WB_GAIN_LIMIT = 2.0          # 채널 게인 상한. 노이즈에서 추정한 값 방어


def enhance_v2(raw, code, *, white=255.0, gamma_applied=False,
               wb_applied=False, denoise=True, clahe_clip=1.5, sharpen=0.6):
    """raw Bayer → 표시용 BGR. 디베이어를 톤매핑보다 먼저 한다.

    gamma_applied 는 카메라가 이미 sRGB 감마를 걸었는지 (meta 의
    cameras[].gamma_enable). 이걸 틀리면 감마가 두 번 들어가서 어두운
    쪽이 과하게 들리고 색노이즈가 폭발한다 — 실제로 그렇게 깨진다.

    wb_applied 는 카메라가 AWB 를 돌렸는지 (balance_white_auto != "Off").
    켜져 있으면 여기서 또 WB 를 걸지 않는다. 12 bit 는 감마가 꺼진 채로
    AWB 는 돌아가므로, 두 플래그를 따로 봐야 한다."""
    # 입력이 8 bit 든 12 bit 든 0~255 스케일로 맞춰 놓고 시작한다.
    x = raw.astype(np.float32) * (255.0 / white)
    # 1) 블랙레벨을 '베이어 채널별로' 뺀다.
    #    R/G/B 는 암전류와 오프셋이 서로 달라서, 전체에 스칼라 하나만
    #    빼면 그 차이가 톤매핑에서 그대로 증폭된다. 거의 새까만 화면이
    #    마젠타로 끓던 진짜 원인이 이것이다 (CAM4 실측: R=16 G=8 B=37
    #    /4095 로, 신호가 0 인데 B 바닥만 5배 높았다).
    q = 0.1 if gamma_applied else 0.5
    for dy in (0, 1):
        for dx in (0, 1):
            sub = x[dy::2, dx::2]
            x[dy::2, dx::2] = np.clip(sub - np.percentile(sub, q), 0, None)
    # 2) 디베이어. 16 bit 로 올려 보간 정밀도를 남긴다. 12 bit 입력은
    #    여기서 원래 계조가 그대로 살아 있다 (8 bit 는 이미 뭉개진 뒤다).
    x16 = np.clip(x * 256.0, 0, 65535).astype(np.uint16)
    bgr = cv2.cvtColor(x16, code).astype(np.float32) / 256.0
    # 3) 화이트밸런스. 그레이월드는 화면 대부분이 검정이라 못 쓴다.
    #    상위 1% 밝은 화소(조명과 그 반사)로 광원 색을 추정한다.
    #    카메라가 AWB 를 이미 돌렸으면 건드리지 않는다.
    if not (gamma_applied or wb_applied):
        lum = bgr.sum(2)
        # 밝기 하한을 같이 건다. 노이즈만 있는 화면에서 '상위 1%' 는
        # 적색 노이즈 스파이크라, 그걸로 광원을 추정하면 색이 뒤집힌다.
        m = lum >= max(np.percentile(lum, 99.0), 3.0 * 255.0 / MAX_TONE_GAIN)
        if m.sum() > 500:
            g = np.maximum(bgr[m].mean(0), 1e-3)
            gain = np.clip(g.mean() / g, 1.0 / WB_GAIN_LIMIT, WB_GAIN_LIMIT)
            bgr *= gain
    # 4) 톤매핑. 선형 입력은 2.6, 감마 입력은 1.15 만.
    #    화이트포인트에 하한을 둬서 빈 화면을 억지로 들어올리지 않는다.
    w = max(np.percentile(bgr, 99.7), 255.0 / MAX_TONE_GAIN)
    y = np.clip(bgr / w, 0, 1)
    out = (np.power(y, 1.0 / (1.15 if gamma_applied else 2.6)) * 255).astype(np.uint8)
    # 5) 게인 노이즈는 색 쪽이 크다. 색은 세게, 휘도는 약하게.
    #    실제로 건 배율에 비례해 색 쪽을 더 눌러 준다.
    if denoise:
        gain = 255.0 / w
        c = int(np.clip(8 + 4 * gain, 10, 45))
        out = cv2.fastNlMeansDenoisingColored(out, None, 3, c, 7, 21)
    # 6)(7) 휘도만 손본다. 색에 걸면 노이즈가 같이 커진다.
    lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    if clahe_clip > 0:
        l = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8)).apply(l)
    if sharpen > 0:
        l = cv2.addWeighted(l, 1 + sharpen, cv2.GaussianBlur(l, (0, 0), 1.2),
                            -sharpen, 0)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def enhance(raw, code, method):
    if raw.dtype != np.uint8:           # 12 bit 입력은 구버전 경로에 못 넣는다
        raw = (raw.astype(np.uint32) >> 4).astype(np.uint8)
    lo, hi = np.percentile(raw, (0.1, 99.9))
    y = np.clip((raw.astype(np.float32) - lo) / max(hi - lo, 1e-6), 0, 1)
    bgr = cv2.cvtColor((np.power(y, 0.35) * 255).astype(np.uint8), code)
    if method == 3:
        return bgr
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
    bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    if method == 4:
        return bgr
    if method == 5:
        return cv2.fastNlMeansDenoisingColored(bgr, None, 5, 5, 7, 21)
    if method == 6:                      # 빠른 대안
        return cv2.bilateralFilter(bgr, 5, 40, 40)
    raise SystemExit(f"모르는 방식: {method}")


def work(job):
    src, dst, fmt, method, quality, gamma_applied, wb_applied = job
    raw = np.load(src)
    if method == 7:
        img = enhance_v2(raw, BAYER_EA.get(fmt, BAYER[fmt]),
                         white=_white(fmt, raw), gamma_applied=gamma_applied,
                         wb_applied=wb_applied)
    else:
        img = enhance(raw, BAYER[fmt], method)
    cv2.imwrite(dst, img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--method", type=int, default=7, choices=[3, 4, 5, 6, 7],
                    help="7 = v2 파이프라인(기본). 3~6 은 구버전")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--quality", type=int, default=95)
    ap.add_argument("--limit", type=int, default=None, help="앞 N장만 (시험용)")
    ap.add_argument("--skip-existing", action="store_true",
                    help="이미 만들어진 jpg 는 건너뛴다 (중단 후 이어서 돌릴 때)")
    args = ap.parse_args()

    with open(os.path.join(args.session, "meta.json")) as f:
        meta = json.load(f)

    jobs = []
    for i, c in enumerate(meta["cameras"]):
        folder = f"cam{meta['camera_slots'][i]}"
        path = os.path.join(args.session, folder, "frames.csv")
        if not os.path.exists(path):
            continue
        outdir = os.path.join(args.out, c["name"])
        os.makedirs(outdir, exist_ok=True)
        rows = list(csv.DictReader(open(path)))
        if args.limit:
            rows = rows[:args.limit]
        for r in rows:
            fid = int(r["frame_id"])
            jobs.append((os.path.join(args.session, folder, f"{fid:08d}.npy"),
                         os.path.join(outdir, f"{fid:08d}.jpg"),
                         c["pixel_format"], args.method, args.quality,
                         bool(c.get("gamma_enable", False)),
                         str(c.get("balance_white_auto", "Off")) != "Off"))

    if args.skip_existing:
        before = len(jobs)
        jobs = [j for j in jobs if not os.path.exists(j[1])]
        print(f"이미 있는 {before - len(jobs)} 장 건너뜀")

    print(f"{len(jobs)} 장 · 방식 {args.method} · 프로세스 {args.jobs}")
    if not jobs:
        print("할 일 없음")
        return
    done = 0
    with Pool(args.jobs) as p:
        for _ in p.imap_unordered(work, jobs, chunksize=4):
            done += 1
            if done % 100 == 0 or done == len(jobs):
                print(f"  {done}/{len(jobs)}", flush=True)

    size = sum(os.path.getsize(os.path.join(dp, f))
               for dp, _, fs in os.walk(args.out) for f in fs)
    print(f"완료: {args.out}  ({size/1e6:.0f} MB)")


if __name__ == "__main__":
    main()
