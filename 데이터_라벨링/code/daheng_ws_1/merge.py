"""세션 결합표 — 같은 트리거 펄스에 찍힌 프레임을 묶고 카메라별 셔터 시각을 붙인다.

    python3 merge.py sessions/session_006
    python3 merge.py sessions                    (전체 세션 일괄)

--------------------------------------------------------------------------
출력  <세션폴더>/matched.csv   — 트리거 펄스 N 한 줄에 카메라 네 대

    N, t_trig_us,
    cam1, cam1_mcu_us, cam1_lidar_us, cam1_unix,
    cam2, ...

    camK           파일명 (그 펄스에 이 카메라가 없으면 빈 칸)
    camK_mcu_us    노출 중앙의 MCU 시각 (마이크로초)
    camK_lidar_us  같은 순간의 라이다 '정시 이후 us'. VLP-16 패킷 타임스탬프와
                   바로 비교된다. 라이다가 PPS 에 잠기고 GPRMC 를 받아들인
                   세션에서만 채운다 (아니면 초 단위로 틀리므로 빈 칸)
    camK_unix      UNIX 초. 절대 시각은 ms 수준, 센서 간 상대는 us 수준

--------------------------------------------------------------------------
시차 발사(A 방식)에서는 한 줄의 네 장이 같은 순간이 아니다

    같은 N = 같은 FSYNC 펄스를 받은 네 장
    셔터 시각은 카메라마다 0 / 25 / 50 / 75 ms 뒤 — camK_mcu_us 가 각자 다르다

라이다와 맞출 때는 반드시 카메라별 시각(camK_lidar_us 또는 camK_unix)을 쓴다.
N 이나 t_trig_us 로 라이다를 찾으면 뒤 카메라가 50 ms 어긋난다.

시각 계산은 timebase.py 에 모여 있다 (to_rosbag.py · sync_report.py 와 같은 값).
"""

import argparse
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from timebase import Timebase, lidar_sync_ok, host_delay_estimates  # noqa: E402


def _lidar_state(session_dir):
    """pcap 이 있으면 라이다 동기 상태와 디코딩 결과. 없으면 (None, False, 사유)."""
    pcap = os.path.join(session_dir, "lidar.pcap")
    if not os.path.exists(pcap):
        return None, False, "lidar.pcap 없음"
    try:
        import vlp16
        d = vlp16.decode(pcap)
        ok, why = lidar_sync_ok(vlp16.read_position_packets(pcap))
        return d, ok, why
    except Exception as e:                       # 결합표는 라이다 없이도 만든다
        return None, False, f"디코딩 실패: {e}"


def merge(session_dir):
    name = os.path.basename(session_dir.rstrip("/\\"))
    print(f"\n===== {name} =====")

    tb = Timebase(session_dir)
    if not tb.t_trig:
        print("  [실패] mcu_log.txt 에 T 줄이 없음 — 촬영되지 않은 세션")
        return False
    if not tb.cameras:
        print("  [실패] camN 폴더 없음")
        return False

    d, lidar_ok, why = _lidar_state(session_dir)
    tb.set_anchor(decoded=d, lidar_ok=lidar_ok)
    print(f"  라이다      {why}")
    print(f"  UNIX 앵커   {tb.anchor_src or '없음 (UNIX 열 비움)'}")
    # 라이다가 GPRMC 를 받지 않았으면 라이다 시계의 '몇 분 몇 초' 가 MCU 와
    # 다르다. 그 상태로 camK_lidar_us 를 채우면 초 단위로 틀린 값이 되어
    # 라이다 패킷과 엉뚱하게 짝지어진다. 비워 두는 편이 안전하다.
    fill_lidar = tb.has_pps_map and lidar_ok
    if not tb.has_pps_map:
        print("  [주의] NMEA 줄이 없어 라이다 시각 열을 비웁니다.")
    elif d is not None and not lidar_ok:
        print("  [주의] 라이다가 GPRMC 를 받지 않아 라이다 시각 열을 비웁니다.")
        print("         (회전 위상은 맞으므로 sync_report.py 로 A 방식 검증은 됩니다)")

    # --- 카메라별 요약 ---
    # 지연을 세 가지로 본다.
    #   설정   sync_cam 이 카메라에 넣은 TriggerDelay
    #   Line1  ESP32 가 잰 실제 노출 시작 (EXP 줄). 가장 정밀하지만 신호가
    #          오염되면 틀린다
    #   도착   노트북 도착 시각 차이로 추정. 흔들리지만 Line1 과 무관하다
    #   사용   시각 계산에 실제로 쓴 값
    ok = True
    host_est = host_delay_estimates(tb)
    print(f"\n  {'cam':<6}{'frames':>7}{'offset':>7}{'설정':>8}{'Line1':>9}"
          f"{'도착':>8}{'사용':>9}{'방위':>7}{'매칭':>9}   (지연 ms)")
    print("  " + "-" * 74)
    cams = sorted(tb.cameras)
    by_n = {}                          # N -> {slot: frame_id}
    for slot in cams:
        c = tb.cameras[slot]
        hit = 0
        for fid, _ts, _recv in c.rows:
            n = c.pulse_of(fid)
            if n is not None and n in tb.t_trig:
                by_n.setdefault(n, {})[slot] = fid
                hit += 1
        if not c.rows or hit != len(c.rows) or c.offset_src != "EXP":
            ok = False
        note = "" if c.offset_src == "EXP" else f"  ({c.offset_src})"
        l1 = ("—" if c.delay_measured_us is None
              else f"{c.delay_measured_us / 1000:.3f}")
        he = host_est.get(slot)
        he_s = "—" if he is None else f"{he / 1000:.1f}"
        print(f"  CAM{slot:<3}{len(c.rows):>7}{str(c.offset):>7}"
              f"{c.delay_set_us / 1000:>8.0f}{l1:>9}{he_s:>8}"
              f"{c.delay_us / 1000:>9.3f}{c.azimuth_deg:>6.0f}°"
              f"{f'{hit}/{len(c.rows)}':>9}{note}")

    exps = {round(tb.cameras[s].exposure_us) for s in cams}
    print(f"  노출 {' / '.join(f'{e / 1000:.1f}' for e in sorted(exps))} ms")

    # --- 진단: 어느 쪽이 틀렸나 ---
    # 도착 시각 추정은 USB 흔들림이 있어 ±1.5 ms 까지는 같은 값으로 본다.
    HOST_TOL_US = 1500.0
    for slot in cams:
        c = tb.cameras[slot]
        he = host_est.get(slot)
        host_ok = he is None or abs(he - c.delay_set_us) <= HOST_TOL_US
        if not host_ok:
            print(f"  [경고] CAM{slot} 도착 시각으로 본 지연 {he / 1000:.1f} ms 가 설정 "
                  f"{c.delay_set_us / 1000:.0f} ms 와 다릅니다.")
            print(f"         TriggerDelay 가 카메라에 적용되지 않았을 수 있습니다.")
            ok = False
        elif c.exp_suspect:
            over = c.delay_measured_us - c.delay_set_us
            if c.exp_reason == "빠름":
                print(f"  [경고] CAM{slot} Line1 이 트리거 후 {over:.0f} us 에 찍혔습니다. "
                      f"진짜 노출 알림은 10 us 이상 걸립니다.")
            else:
                print(f"  [경고] CAM{slot} Line1 실측 {c.delay_measured_us / 1000:.3f} ms 가 "
                      f"설정 {c.delay_set_us / 1000:.0f} ms 와 다릅니다.")
                if he is not None:
                    print(f"         도착 시각으로는 {he / 1000:.1f} ms 라 TriggerDelay 는 정상입니다.")
            print(f"         Line1 입력(GPIO)에 FSYNC 순간 잡음이 들어와 진짜 노출 대신")
            print(f"         세어진 것으로 보입니다. 시각은 설정값으로 계산했습니다.")
            print(f"         → 이 채널의 CNT 는 믿을 수 없습니다 (트리거 누락 감지 불가).")
            ok = False

    # --- 결합표 ---
    out = os.path.join(session_dir, "matched.csv")
    full = 0
    with open(out, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        head = ["N", "t_trig_us"]
        for s in cams:
            head += [f"cam{s}", f"cam{s}_mcu_us", f"cam{s}_lidar_us", f"cam{s}_unix"]
        w.writerow(head)
        for n in sorted(by_n):
            row = [n, tb.t_trig[n]]
            for s in cams:
                fid = by_n[n].get(s)
                if fid is None:
                    row += ["", "", "", ""]
                    continue
                c = tb.cameras[s]
                t = c.shutter_us(fid, "mid")
                lid = (f"{float(tb.mcu_to_lidar_us(t)):.1f}"
                       if fill_lidar else "")
                unix = (f"{float(tb.to_unix(t)):.6f}"
                        if tb.anchor is not None else "")
                row += [f"{fid:08d}{_ext(tb, s, fid)}", f"{t:.1f}", lid, unix]
            if len(by_n[n]) == len(cams):
                full += 1
            w.writerow(row)

    print(f"\n  전 카메라가 모두 찍힌 펄스 : {full} / {len(tb.t_trig)}")
    print(f"  → {out}")
    if len(cams) > 1 and any(tb.cameras[s].delay_set_us for s in cams):
        print("  ※ 시차 발사 세션입니다. 한 줄의 카메라들은 같은 순간이 아닙니다.")
        print("    라이다와 맞출 때는 camK_lidar_us / camK_unix 를 쓰세요.")
    print(f"\n  → {'OK' if ok else '위 경고 확인'}")
    return ok


def _ext(tb, slot, fid):
    d = os.path.join(tb.dir, tb.cameras[slot].dirname)
    return ".png" if os.path.exists(os.path.join(d, f"{fid:08d}.png")) else ".npy"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="세션 폴더 또는 상위 폴더")
    args = ap.parse_args()

    p = os.path.abspath(args.path)
    if os.path.exists(os.path.join(p, "mcu_log.txt")):
        targets = [p]
    else:
        targets = sorted(os.path.join(p, d) for d in os.listdir(p)
                         if d.startswith("session_"))
    if not targets:
        print("세션을 찾지 못했습니다.")
        return 1
    results = [merge(t) for t in targets]
    if len(results) > 1:
        print(f"\n===== 전체 {len(results)} 세션 중 "
              f"{sum(1 for r in results if not r)} 개 경고 =====")
    return 0 if all(results) else 2


if __name__ == "__main__":
    sys.exit(main())
