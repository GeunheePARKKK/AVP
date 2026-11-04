"""카메라와 라이다가 실제로 얼마나 맞물려 있는지 측정한다.

    python3 sync_report.py sessions/session_003

--------------------------------------------------------------------------
"10 Hz 로 맞춘다" 는 두 가지를 뜻하고, 둘은 따로 논다

  주파수(frequency)  둘 다 초당 10번인가.
  위상(phase)        그 10번이 서로 같은 순간에 일어나는가.

  주파수가 맞아도 위상은 아무 값이나 될 수 있다. 카메라가 매 초 0.000,
  0.100, ... 에 찍고 라이다가 0.072, 0.172, ... 에 회전을 시작하면
  둘 다 정확히 10 Hz 이지만 모든 프레임이 72 ms 어긋난 상태다.

  게다가 두 시계가 아주 조금만 달라도 그 어긋남은 계속 움직인다.
  이 스크립트는 어긋난 양과 움직이는 속도를 둘 다 잰다.

--------------------------------------------------------------------------
진짜로 맞추는 방법

  라이다를 카메라 쪽 시계에 물려야 한다. VLP-16 은 그 수단을 갖고 있다.

    1. ESP32 의 1PPS 를 라이다 I/F Box 의 GPS PULSE 로 넣는다
    2. GPRMC 를 GPS RECEIVE 로 넣는다 (시각 문자열)
    3. 라이다에서 Phase Lock 을 켜고 각도를 정한다
       → PPS 가 들어오는 순간 라이다가 그 각도를 보고 있게 된다

  ESP32 는 FSYNC(카메라)와 PPS(라이다)를 같은 타이머에서 만들므로,
  3번까지 되면 두 센서가 한 시계 위에 놓인다. 그때 이 스크립트의
  '표류' 항목이 0 에 수렴한다.
"""

import argparse
import csv
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import vlp16  # noqa: E402
from timebase import Timebase  # noqa: E402

PERIOD_MS = 100.0          # 10 Hz
PERIOD_US = PERIOD_MS * 1000.0


def load_cam(session_dir):
    """카메라 폴더별 호스트 수신 시각."""
    out = {}
    for d in sorted(os.listdir(session_dir)):
        if not (d.startswith("cam") and d[3:].isdigit()):
            continue
        p = os.path.join(session_dir, d, "frames.csv")
        if not os.path.exists(p):
            continue
        ts = []
        with open(p, encoding="utf-8") as f:
            for r in csv.DictReader(f):
                try:
                    ts.append(float(r["host_recv_unix"]))
                except (ValueError, KeyError):
                    pass
        if ts:
            out[d] = np.array(sorted(ts))
    return out


def describe_rate(name, t):
    dt = np.diff(t)
    if not len(dt):
        return None
    per = dt.mean() * 1000
    hz = 1000.0 / per
    print(f"  {name:8s} {len(t):4d} 개   주기 {per:8.4f} ms "
          f"(± {dt.std() * 1000:.3f})   {hz:.6f} Hz")
    return hz


def lidar_phase_lock(d, revs, offset_deg=None):
    """카메라 없이, 라이다 타임스탬프만으로 Phase Lock 을 판정한다.

    PPS 가 잡혀 있으면 라이다의 us 카운터는 초 경계가 ESP32 PPS 에 맞춰진다.
    Phase Lock 이 걸려 있으면 그 경계에서 라이다가 offset 방위각을 본다.
    600 rpm = 10 Hz 이므로 0도 통과는 매번 100 ms 주기 안의 같은 위치에 온다.

        offset 0도  → 0도 통과가 0 ms 위치
        offset θ도  → (360-θ)/360 × 100 ms 위치

    펌웨어에서 FSYNC 와 PPS 는 같은 타이머 틱의 같은 레지스터 쓰기에서
    나가므로(onTimer), 카메라 셔터는 PPS 기준 0, 100, 200 ms 에 열린다.
    그래서 여기가 맞으면 카메라와의 정렬도 따라온다.
    """
    if len(revs) < 5:
        print("  회전이 너무 적어 판정할 수 없습니다.")
        return None
    # 0도 통과 시각: 회전 첫 점의 시각에서 이미 지나친 각도만큼 되돌린다
    t0 = np.array([d.packet_ts_us[a] - d.azimuth[a] / 360.0 * PERIOD_US
                   for a, b in revs])
    ph = np.mod(t0, PERIOD_US)
    ang = ph / PERIOD_US * 2 * np.pi          # 0/100 ms 경계를 넘나드므로 원형 평균
    pos = (np.angle(np.mean(np.exp(1j * ang))) % (2 * np.pi)) / (2 * np.pi) * PERIOD_US
    dev = (ph - pos + PERIOD_US / 2) % PERIOD_US - PERIOD_US / 2
    x = np.unwrap(t0, period=3600e6)
    x = (x - x[0]) / 1e6
    slope = np.polyfit(x, dev, 1)[0]          # us per second

    print(f"  0도 통과 위치  {pos / 1000:6.2f} ms   (100 ms 주기 안에서)")
    print(f"  흔들림        ±{dev.std():.0f} us   "
          f"(= ±{dev.std() / PERIOD_US * 360:.2f}도, 모터 서보 지터)")
    print(f"  이동          {slope:+.2f} us/s")

    ok_pos = None
    if offset_deg is not None:
        expect = ((360.0 - offset_deg) % 360.0) / 360.0 * PERIOD_US
        err = (pos - expect + PERIOD_US / 2) % PERIOD_US - PERIOD_US / 2
        ok_pos = abs(err) < 1000.0
        print(f"  기대 위치     {expect / 1000:6.2f} ms   "
              f"(Phase Lock offset {offset_deg:g}도)   차이 {err / 1000:+.2f} ms")
    ok_drift = abs(slope) < 3.0
    if ok_drift and ok_pos in (None, True):
        print("  → 라이다 위상이 PPS 에 고정되어 있습니다.")
    elif not ok_drift and ok_pos:
        # 평균 위치는 맞는데 흔들린다 = 모터가 위상을 끌어오는 중.
        # 2026-09-14 실측: Phase Lock 을 켠 직후 ±2 ms, 1분 뒤 ±0.1 ms.
        print("  → 위치는 맞는데 아직 흔들립니다. Phase Lock 을 막 켰거나 PPS 가")
        print("    방금 잡힌 경우 모터가 수렴하는 중입니다. 1분쯤 뒤 다시 찍어 보세요.")
    elif not ok_drift:
        print("  → 위상이 움직입니다. PPS 가 없거나 Phase Lock 이 꺼져 있습니다.")
    else:
        print("  → 고정되어는 있으나 명령한 각도와 다릅니다. offset 설정을 확인하세요.")
    return {"ok": ok_drift and ok_pos in (None, True),
            "pos_us": float(pos), "jitter_us": float(dev.std())}


def gprmc_check(pos):
    """라이다가 받은 GPRMC 시각과 라이다 자신의 타임스탬프를 대조한다.

    GPRMC 가 제대로 들어가면 라이다는 그 문장의 분:초로 '정시 이후 us'
    카운터의 초 단위를 맞춘다. 그러면 타임스탬프의 초와 문장의 mm*60+ss 가
    같아야 한다.

    단, 매초 PPS 직후에는 새 문장이 아직 다 도착하지 않았다. 9600 bps 로
    66글자면 약 69 ms 가 걸리고, 그동안 위치 패킷에는 직전 초의 문장이
    남아 있어 차이가 +1 로 보인다. 2026-09-15 실측(session_017):
        차이 0   → 초 경계 이후 70.9 ~ 999.8 ms
        차이 +1  → 초 경계 이후  0.0 ~  70.0 ms
    그래서 +1 은 '초 경계 직후 짧은 구간' 안에 있을 때만 정상으로 본다.
    """
    import re
    same, lag, bad = [], [], []
    for p in pos:
        m = re.match(r"\$GPRMC,(\d{2})(\d{2})(\d{2})", p["nmea"])
        if not m:
            continue
        _hh, mm, ss = map(int, m.groups())
        lid = int(p["lidar_us"] // 1_000_000)
        sub_ms = (p["lidar_us"] % 1_000_000) / 1000.0
        diff = (lid - (mm * 60 + ss) + 1800) % 3600 - 1800
        if diff == 0:
            same.append(sub_ms)
        elif diff == 1 and sub_ms < 200.0:
            lag.append(sub_ms)
        else:
            bad.append((diff, sub_ms))
    n = len(same) + len(lag) + len(bad)
    if not n:
        print("  GPRMC 문장이 라이다에 도달하지 않았습니다 (NMEA 필드 비어 있음).")
        print("  위상 동기에는 영향이 없습니다. 라이다 타임스탬프의 초·분이 ESP32 시간축에 안 묶일 뿐입니다.")
        return None
    print(f"  GPRMC 수신 {n}건 — 라이다 초와 문장 초 일치 {len(same)}건", end="")
    if lag:
        print(f", 새 문장 도착 전 구간 {len(lag)}건 (초 경계 후 {max(lag):.0f} ms 이내)")
    else:
        print()
    if bad:
        print(f"  [주의] 설명되지 않는 불일치 {len(bad)}건 — 예: 차이 {bad[0][0]:+d}초, "
              f"초 경계 후 {bad[0][1]:.0f} ms")
        print("  → 문장이 깨지거나 PPS 와 짝이 어긋납니다.")
        return False
    arrive = min(same) if same else None
    if arrive is not None:
        print(f"  새 문장 반영 시점: 매초 PPS 후 약 {arrive:.0f} ms "
              f"(9600 bps 로 66글자 전송 ≈ 69 ms)")
    print("  → 라이다 타임스탬프가 ESP32 의 GPRMC 시간축에 정확히 물려 있습니다.")
    return True


def camera_latency(session_dir):
    """카메라 트리거(FSYNC) → 실제 노출 시작까지의 '공통 지연' (카메라별로 본다).

    ESP32 는 트리거 시각(T 줄)과 카메라 Line1(ExposureActive)이 LOW 로 떨어진
    시각(EXP 줄)을 같은 us 시계로 기록한다.

        Line1 − 트리거 = 카메라 TriggerDelay 설정 + 공통 지연

    시차 발사(A)에서는 카메라마다 설정 지연이 0/25/50/75 ms 로 다르므로, 설정을
    빼고 남는 공통 지연(옵토 입력 + 노출 시작 + Line1 출력 + ESP32 인터럽트)을
    카메라별로 본다. 정상이면 네 대가 비슷하다.
      2026-09-15 실측 12.8 us (session_020, 1대)
      2026-09-21 실측 11~12 us (session_004, CAM2·CAM4)

    판정
      신호 지연은 카메라·배선마다 5~13 us 로 다르다. 빠르다고 잡음은 아니다
      −0.2~+2 ms 밖  설정과 전혀 안 맞는다. 잡음 엣지가 진짜 노출을 가린 것
                  (펌웨어가 첫 엣지를 세고 50 ms 락아웃을 걸기 때문)
    어느 쪽이든 시각 계산(timebase)은 설정값으로 대신하므로 데이터는 안전하다.
    다만 그 채널의 CNT 는 트리거 누락을 감지하지 못한다.

    반환: 정상 카메라들의 공통 지연 중앙값 (us). 없으면 None.
    """
    tb = Timebase(session_dir)
    if not tb.cameras or not tb.t_trig:
        return None
    print(f"  {'cam':<6}{'설정':>8}{'Line1 − 설정':>14}   판정")
    good = []
    any_exp = False
    for slot, c in sorted(tb.cameras.items()):
        if c.delay_measured_us is None:
            print(f"  CAM{slot:<3}{c.delay_set_us / 1000:>6.0f} ms"
                  f"{'—':>14}   Line1 기록 없음")
            continue
        any_exp = True
        over = c.delay_measured_us - c.delay_set_us
        if c.exp_suspect and c.exp_reason == "빠름":
            verdict = ("[오염] 너무 빠름 — 진짜 노출 알림은 10 us 이상 걸린다. "
                       "FSYNC 순간 잡음 엣지. 시각은 설정값으로 계산")
        elif c.exp_suspect:
            verdict = ("[오염] 설정과 전혀 안 맞음 — 잡음 엣지가 진짜 노출을 가림. "
                       "시각은 설정값으로 계산")
        else:
            verdict = "OK"
            good.append(over)
        print(f"  CAM{slot:<3}{c.delay_set_us / 1000:>6.0f} ms{over:>+11.1f} us   {verdict}")
    if not any_exp:
        return None
    if not good:
        print("  정상으로 판정된 카메라가 없어 공통 지연을 정하지 못했습니다.")
        print("  → Line1 배선(특히 FSYNC·PPS 선 가까이 지나는 채널)을 확인하세요.")
        return None
    lat = float(np.median(good))
    print(f"  공통 지연 {lat:.1f} us  (정상 카메라 {len(good)} 대 기준)")
    return lat


def trigger_azimuth(session_dir, d, latency_us=0.0):
    """카메라 셔터가 열린 순간마다 라이다가 가리키던 방위각.

    세 시계를 하나로 잇는다.
      ESP32   T,<N>,<us>          FSYNC(셔터) 상승 엣지 시각
      ESP32   NMEA,<PPS us>,...   그 초의 PPS 엣지 시각과 GPRMC 시각(hhmmss)
      라이다  '정시 이후 us'       GPRMC 가 들어가면 초 단위가 문장과 같아진다
    그래서 트리거 시각 t 는 라이다 시각 (mm*60+ss)*1e6 + (t - PPS) 로 바뀌고,
    그 순간 라이다 패킷의 방위각을 읽을 수 있다. PPS 와 GPRMC 가 둘 다
    들어와야만 가능한 계산이다.

    Phase Lock offset 이 0 이면 결과가 0 도 근처여야 한다.
    2026-09-15 session_017: 평균 +0.015 도, 표준편차 0.38 도 (= 106 us).
    """
    path = os.path.join(session_dir, "mcu_log.txt")
    if not os.path.exists(path):
        return None
    trig, pmap = {}, []
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            p = ln.strip().split(",")
            if p[0] == "T" and len(p) >= 3:
                try:
                    trig[int(p[1])] = int(p[2])
                except ValueError:
                    pass
            elif p[0] == "NMEA" and len(p) >= 8 and p[6] == "$GPRMC" and len(p[7]) >= 6:
                try:
                    mm, ss = int(p[7][2:4]), int(p[7][4:6])
                    pmap.append((int(p[1]), (mm * 60 + ss) * 1_000_000))
                except ValueError:
                    pass
    if not trig or not pmap:
        return None
    pmap.sort()
    pe = np.array([a for a, _ in pmap], float)
    pl = np.array([b for _, b in pmap], float)
    t_l = np.unwrap(d.packet_ts_us.astype(float), period=3600e6)
    az = np.unwrap(np.deg2rad(d.azimuth.astype(float)))
    order = np.argsort(t_l)
    t_l, az = t_l[order], az[order]
    mid = t_l[len(t_l) // 2]
    out = []
    for _n, tu in sorted(trig.items()):
        i = np.searchsorted(pe, tu, side="right") - 1
        if i < 0:
            continue
        lid = pl[i] + (tu - pe[i]) + latency_us
        lid += np.round((mid - lid) / 3600e6) * 3600e6
        if t_l[0] <= lid <= t_l[-1]:
            out.append(np.rad2deg(np.interp(lid, t_l, az)) % 360.0)
    if not out:
        return None
    a = np.array(out)
    dev = (a + 180.0) % 360.0 - 180.0
    us = dev / 360.0 * PERIOD_US
    print(f"  트리거 {len(a)}회 — 셔터 순간 라이다 방위각 평균 {dev.mean():+.3f}도, "
          f"표준편차 {dev.std():.3f}도, 범위 {dev.min():+.2f}~{dev.max():+.2f}도")
    print(f"  시간 오차 평균 {us.mean() / 1e6:+.6f} s, 표준편차 {us.std() / 1e6:.6f} s, "
          f"최대 {np.abs(us).max() / 1e6:.6f} s")
    return dev


def _azimuth_verdict(means, offset_deg):
    """카메라별 평균 오차 목록으로 판정한다 (두 검사가 같이 쓴다)."""
    m = float(np.mean(means))
    spread = float(np.ptp(means)) if len(means) > 1 else 0.0
    print(f"\n  카메라 평균 오차 {m:+.2f}°  (= {m / 360 * PERIOD_MS:+.2f} ms), "
          f"카메라 사이 차이 {spread:.1f}°")
    # 카메라마다 오차가 다르면 라이다 위상 문제가 아니다. 이때 offset 을
    # 바꾸라고 하면 한 대를 맞추고 나머지를 더 틀리게 만든다.
    if spread > 3.0:
        print("  [경고] 카메라마다 어긋난 정도가 다릅니다. Phase Lock offset 으로는")
        print("         고칠 수 없습니다. 원인 후보:")
        print("         - TriggerDelay 가 카메라에 적용되지 않았다 → merge.py 의")
        print("           '지연(ms)' 열이 설정값과 같은지 확인")
        print("         - sync_cam.CAMERA_AZIMUTH_DEG 가 실제 장착 순서와 다르다")
    elif abs(m) > 2.0:
        cur = offset_deg if offset_deg is not None else 0.0
        new = (cur - m) % 360.0
        print("  → 네 카메라가 같은 만큼 어긋나 있습니다. 라이다 위상 문제입니다.")
        print(f"    Phase Lock offset 을 {cur:.1f}° → {new:.1f}° 로 바꾸세요:")
        print(f"      python3 lidar_config.py --phase-lock on --offset "
              f"{new:.1f} --save --yes")
    else:
        print("  → 정상. 각 카메라의 노출 중앙에서 라이다가 그 카메라 쪽을 지나고 있습니다.")


def camera_azimuths_phase(session_dir, lidar_pos_us, lidar_jitter_us,
                          offset_deg=None):
    """GPRMC 없이 하는 A 방식 검증 — 회전 위상만으로 비교한다.

    PPS 가 잠겨 있으면 라이다 타임스탬프의 초 경계가 PPS 에 맞춰진다.
    GPRMC 는 거기에 '몇 분 몇 초' 이름표를 붙일 뿐이다. 회전 위상(100 ms
    주기 안의 위치)은 이름표 없이도 MCU 와 라이다가 같은 기준이다.

        라이다 0도 통과 위치  lidar_phase_lock() 이 라이다 타임스탬프로 잰 값
        카메라 노출 중앙 위치  PPS 엣지부터 (Line1 실측 지연 + 노출/2)
        방위각 = 360 × (카메라 위치 − 라이다 0도 위치) / 100 ms

    라이다 쪽은 회전마다의 흔들림 대신 평균 위치를 쓴다. 흔들림은 위
    '라이다 위상' 절에 따로 나온다.
    """
    tb = Timebase(session_dir)
    if not tb.cameras or not len(tb.pps_edges) or not tb.t_trig:
        return None
    print(f"  {'cam':<6}{'목표':>6}{'평균 오차':>10}{'표준편차':>10}"
          f"{'시간 환산':>11}{'지연(ms)':>10}  n")
    means = []
    for slot, c in sorted(tb.cameras.items()):
        mids = [c.shutter_us(fid, "mid") for fid, _ts, _r in c.rows]
        mids = [t for t in mids if t is not None]
        if not mids:
            print(f"  CAM{slot:<3}   트리거 기록과 맞는 프레임 없음")
            continue
        ph = tb.phase_in_rotation_us(np.array(mids))
        az = (360.0 * (ph - lidar_pos_us) / PERIOD_US) % 360.0
        dev = (az - c.azimuth_deg + 180.0) % 360.0 - 180.0
        means.append(dev.mean())
        us = dev / 360.0 * PERIOD_US
        print(f"  CAM{slot:<3}{c.azimuth_deg:>5.0f}°{dev.mean():>+9.2f}°"
              f"{dev.std():>9.2f}°{us.mean() / 1000:>+9.2f} ms"
              f"{c.delay_us / 1000:>10.2f}  {len(dev)}")
    if not means:
        return None
    print(f"  (카메라 표준편차는 카메라 쪽 흔들림만이다. 라이다 회전 흔들림 "
          f"±{lidar_jitter_us / PERIOD_US * 360:.2f}° 는 별도)")
    _azimuth_verdict(means, offset_deg)
    return means


def camera_azimuths(session_dir, d, offset_deg=None):
    """카메라마다 노출 중앙 순간에 라이다가 가리킨 방위각 — A 방식 검증.

    시차 발사가 맞게 동작하면 각 카메라의 노출 중앙에서 라이다는 그 카메라
    방위각(앞 0 · 오른쪽 90 · 뒤 180 · 왼쪽 270)을 지나고 있어야 한다.
    trigger_azimuth() 는 FSYNC 순간 하나만 보지만, 여기서는 카메라별 실제
    노출(Line1 실측 지연 + 노출 절반)을 쓴다. 계산은 timebase.py 와 같다.

    네 카메라가 모두 같은 방향으로 어긋나 있으면 라이다 Phase Lock offset 이
    어긋난 것이다. 권장값을 함께 출력한다.
    """
    tb = Timebase(session_dir)
    if not tb.cameras or not tb.has_pps_map or not tb.t_trig:
        return None
    t_l = np.unwrap(d.packet_ts_us.astype(float), period=3600e6)
    az = np.unwrap(np.deg2rad(d.azimuth.astype(float)))
    order = np.argsort(t_l)
    t_l, az = t_l[order], az[order]
    mid = t_l[len(t_l) // 2]

    print(f"  {'cam':<6}{'목표':>6}{'평균 오차':>10}{'표준편차':>10}"
          f"{'시간 환산':>11}{'지연(ms)':>10}  n")
    means = []
    for slot, c in sorted(tb.cameras.items()):
        vals = []
        for fid, _ts, _recv in c.rows:
            t = c.shutter_us(fid, "mid")
            if t is None:
                continue
            lid = float(tb.mcu_to_lidar_us(t))
            lid += np.round((mid - lid) / 3600e6) * 3600e6
            if t_l[0] <= lid <= t_l[-1]:
                vals.append(np.rad2deg(np.interp(lid, t_l, az)) % 360.0)
        if not vals:
            print(f"  CAM{slot:<3}   라이다와 겹치는 구간 없음")
            continue
        a = np.array(vals)
        dev = (a - c.azimuth_deg + 180.0) % 360.0 - 180.0
        means.append(dev.mean())
        us = dev / 360.0 * PERIOD_US
        print(f"  CAM{slot:<3}{c.azimuth_deg:>5.0f}°{dev.mean():>+9.2f}°"
              f"{dev.std():>9.2f}°{us.mean() / 1000:>+9.2f} ms"
              f"{c.delay_us / 1000:>10.2f}  {len(a)}")

    if not means:
        return None
    _azimuth_verdict(means, offset_deg)
    return means


def main():
    ap = argparse.ArgumentParser(description="카메라↔라이다 동기 상태 측정")
    ap.add_argument("session")
    args = ap.parse_args()
    sd = os.path.abspath(args.session)

    pcap = os.path.join(sd, "lidar.pcap")
    if not os.path.exists(pcap):
        print(f"lidar.pcap 이 없습니다: {sd}")
        return 1

    print(f"===== {os.path.basename(sd)} =====\n")
    d = vlp16.decode(pcap)
    revs = [(a, b) for a, b in d.revolutions()]
    if len(revs) >= 3:
        revs = revs[1:-1]
    rev_t = np.array([float(d.host_ts[a]) for a, b in revs])
    cams = load_cam(sd)

    offset_deg = None
    meta_path = os.path.join(sd, "meta.json")
    if os.path.exists(meta_path):
        import json
        try:
            m = json.load(open(meta_path, encoding="utf-8"))
            pl = (m.get("lidar", {}).get("settings") or {}).get("phaselock") or {}
            if str(pl.get("enabled", "")).lower() == "on":
                offset_deg = float(pl.get("offset", 0)) / 100.0
        except (ValueError, OSError):
            pass

    # ------------------------------------------------------------ 주파수
    print("주파수 — 초당 몇 번인가")
    lid_hz = describe_rate("라이다", rev_t)
    cam_hz = {}
    for name, t in cams.items():
        cam_hz[name] = describe_rate(name, t)
    if not cams:
        print("  (카메라 없음 — 라이다 전용 세션)")

    # ------------------------------------------------- 라이다 단독 위상 잠금
    print("\n라이다 위상 — 카메라 없이 라이다 타임스탬프만으로")
    ph_lock = lidar_phase_lock(d, revs, offset_deg)

    # ------------------------------------------------------------ 위상
    for name, t in cams.items():
        print(f"\n위상 — {name} 와 라이다 회전 시작의 간격")
        ph = []
        for x in t:
            prev = rev_t[rev_t <= x]
            if len(prev):
                ph.append((x - prev[-1]) * 1000.0)
        if len(ph) < 3:
            print("  겹치는 구간이 너무 짧습니다.")
            continue
        ph = np.array(ph)
        print(f"  간격  평균 {ph.mean():6.1f} ms   "
              f"범위 {ph.min():.1f} ~ {ph.max():.1f} ms   "
              f"(주기 {PERIOD_MS:.0f} ms 안에서의 위치)")

        # 표류 = 위상이 시간에 따라 움직이는 속도
        x = (t[:len(ph)] - t[0])
        slope, _ = np.polyfit(x, ph, 1)          # ms per second
        print(f"  표류  {slope:+.4f} ms/s", end="")
        if abs(slope) < 1e-4:
            print("   (움직이지 않음)")
        else:
            slip = PERIOD_MS / abs(slope)
            print(f"   → 1분 {slope * 60:+.1f} ms, "
                  f"{slip / 60:.0f} 분마다 한 프레임씩 밀림")

        if lid_hz and cam_hz.get(name):
            ppm = (cam_hz[name] / lid_hz - 1) * 1e6
            print(f"  클럭차 {ppm:+.0f} ppm "
                  f"(두 센서가 각자의 발진기로 도는 한 남는 값)")

        # 가장 가까운 회전까지의 거리 — 결합할 때 실제로 쓰는 값
        near = np.abs(t[:, None] - rev_t[None, :]).min(axis=1) * 1000
        print(f"  최근접 회전까지 평균 {near.mean():.1f} ms "
              f"(최대 {near.max():.1f} ms)")

    # ------------------------------------------------------------ PPS
    print("\n하드웨어 동기 상태")
    pos = vlp16.read_position_packets(pcap)
    if not pos:
        print("  위치 패킷이 없어 PPS 상태를 알 수 없습니다.")
        locked = False
    else:
        states = {}
        for p in pos:
            states[p["pps_state"]] = states.get(p["pps_state"], 0) + 1
        locked = states.get(2, 0) == len(pos)
        desc = ", ".join(f"{vlp16.PPS_STATES.get(k, k)} {v}"
                         for k, v in sorted(states.items()))
        print(f"  라이다 PPS  {desc}")
        nmea = next((p["nmea"] for p in pos if p["nmea"]), "")
        print(f"  NMEA        {nmea or '(없음)'}")
        print()
        if gprmc_check(pos):
            print("\n셔터 순간 라이다 방위각 — ESP32 트리거 시각을 PPS·GPRMC 로 라이다 시각에 옮김")
            if trigger_azimuth(sd, d) is None:
                print("  mcu_log.txt 에 트리거(T)나 GPRMC 기록이 없어 계산하지 못했습니다.")
            print("\n카메라 노출 지연 — ESP32 가 기록한 Line1 엣지 (카메라별 설정 지연을 뺀 값)")
            lat = camera_latency(sd)
            if lat is None:
                print("  노출 기록(EXP)이 없습니다. 카메라 Line1 이 ESP32 에 연결되어 있어야 합니다.")
            else:
                print(f"  → 실제 노출 시작 기준 (트리거 + {lat:.1f} us)")
                trigger_azimuth(sd, d, latency_us=lat)
            print("\n카메라별 노출 중앙의 라이다 방위각 — 시차 발사(A) 검증")
            print("  (시차 발사 + 권장 offset 이면 위 'FSYNC 순간 방위각' 은 0° 가 아니라")
            print("   offset 값 근처가 정상이다. 여기서 보는 값이 0 이어야 한다.)")
            if camera_azimuths(sd, d, offset_deg) is None:
                print("  카메라 프레임 또는 PPS·GPRMC 기록이 없어 계산하지 못했습니다.")
        elif locked and ph_lock:
            print("\n카메라별 노출 중앙의 라이다 방위각 — 시차 발사(A) 검증 (회전 위상 기준)")
            print("  GPRMC 가 없어 '몇 초인지' 는 모르지만, PPS 가 잠겨 있어 100 ms 주기")
            print("  안의 위치는 비교할 수 있습니다. A 방식 검증에는 이것으로 충분합니다.")
            if camera_azimuths_phase(sd, ph_lock["pos_us"], ph_lock["jitter_us"],
                                     offset_deg) is None:
                print("  카메라 프레임 또는 PPS 기록이 없어 계산하지 못했습니다.")

    print()
    if locked and offset_deg is None:
        print("  PPS 는 잡혀 있는데 세션 기록에 Phase Lock 설정이 없습니다. 켜려면:")
        print("      python3 lidar_config.py --phase-lock on --offset <각도> --save --yes")
    elif locked:
        pass
    else:
        print("  PPS 가 없어 두 센서는 각자의 발진기로 돌고 있습니다.")
        print("  위 '표류' 만큼 어긋남이 계속 움직이므로, 긴 녹화일수록")
        print("  프레임 짝짓기를 시각으로 다시 해야 합니다.")
        print()
        print("  하드웨어로 묶으려면 (lidar-pps-gprmc-wiring.md):")
        print("    1. ESP32 GPIO5  → 74AHCT125 → I/F Box GPS PULSE (노랑)")
        print("    2. ESP32 GPIO17 → MAX3232 → I/F Box GPS RECEIVE (흰색)")
        print("    3. 배선을 바꿨으면 라이다 전원을 껐다 켠다")
        print("       (켜진 채로 물리면 입력을 인식하지 못한 채 남는다)")
        print("    4. python3 lidar_config.py --phase-lock on --offset <각도> --yes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
