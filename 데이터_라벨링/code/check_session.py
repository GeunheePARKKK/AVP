"""세션 검산 — 카메라 + 라이다.

    python3 check_session.py sessions/session_001
    python3 check_session.py sessions              전체 일괄

daheng_ws_1/check_session.py 의 확장판이다. MCU 로그와 frames.csv 파싱은
그쪽 함수를 그대로 쓰고, 두 가지를 더한다.

  1. 라이다 검사      패킷 유실, 회전 주기, 호스트/라이다 시계 드리프트, PPS
  2. 슬롯 인식        원본은 cam1 부터 순번으로 찾아서, 3번 카메라만 꽂혀
                      cam3/ 로 저장된 세션을 '프레임 없음' 으로 본다.
                      여기서는 폴더 이름을 그대로 읽는다.

세션이 20초짜리면 다시 찍는 비용도 20초다. 촬영 직후 반드시 돌릴 것.
"""

import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import vlp16  # noqa: E402

DEFAULT_DAHENG_WS = "/home/acelab/Documents/daheng_ws_1"
TRIGGER_PERIOD_US = 100000
GAP_FACTOR = 1.5
EXPECTED_PPS = 754.0          # 초당 데이터 패킷 (10 Hz 단일반사)
EXPECTED_HZ = 10.0
# 데이터 패킷 하나 = 블록 12개 x 110.592 us
PACKET_INTERVAL_US = 12 * 110.592


def load_parsers(ws=DEFAULT_DAHENG_WS):
    """MCU 로그 / frames.csv 파서를 daheng_ws_1 에서 가져온다."""
    ws = os.path.abspath(os.path.expanduser(ws))
    if ws not in sys.path:
        sys.path.insert(0, ws)
    try:
        import check_session as up
        return up.parse_mcu_log, up.parse_frames_csv
    except ImportError:
        return None, None


# ===========================================================================
#  라이다
# ===========================================================================
def check_lidar(session_dir):
    pcap = os.path.join(session_dir, "lidar.pcap")
    if not os.path.exists(pcap):
        print("  라이다: lidar.pcap 없음")
        return None

    size_mb = os.path.getsize(pcap) / 1e6
    try:
        d = vlp16.decode(pcap)
    except Exception as e:
        print(f"  [실패] pcap 디코딩 실패: {e}")
        return False

    if not d.n_packets:
        print(f"  [실패] 데이터 패킷이 하나도 없습니다 ({size_mb:.1f} MB)")
        return False

    ok = True
    print(f"  라이다  {d.model} / {d.return_mode}   {size_mb:.1f} MB")

    # --- 패킷 유실 ---
    # 패킷당 도착 시각을 알고 있으므로, 실제 걸린 시간에서 기대 개수를 낸다.
    uniq_pkt_ts = d.pkt_host_ts                    # 패킷당 정확히 하나
    span = float(uniq_pkt_ts[-1] - uniq_pkt_ts[0]) if len(uniq_pkt_ts) > 1 else 0
    expected = span * EXPECTED_PPS
    rate = 100.0 * d.n_packets / expected if expected else 0.0
    flag = ""
    if expected and rate < 98.0:
        flag, ok = "   [경고] 유실 2% 초과", False
    elif expected and rate < 99.5:
        flag = "   [주의]"
    print(f"          패킷 {d.n_packets} / 기대 {expected:.0f} "
          f"= {rate:.2f}%{flag}   길이 {span:.2f} s")

    # --- 유실 패킷 ---
    # 호스트 도착 시각으로 세면 안 된다. 리눅스가 수신 스레드를 늦게 깨우면
    # 패킷은 멀쩡히 다 왔는데도 도착 시각에 몇 ms 짜리 구멍이 생긴다.
    # 라이다가 패킷에 직접 찍은 시각은 그 영향을 받지 않으므로 이쪽으로 센다.
    lid_us = np.unwrap(d.pkt_ts_us.astype(np.float64), period=3600.0 * 1e6)
    steps = np.diff(lid_us) / PACKET_INTERVAL_US
    lost = int(np.round(steps - 1.0).clip(min=0).sum())
    if lost:
        worst = int(round(steps.max() - 1))
        print(f"          [주의] 유실 패킷 {lost} 개 "
              f"(가장 긴 구간에서 연속 {worst} 개)")
        if lost > d.n_packets * 0.02:
            ok = False
    else:
        print(f"          유실 패킷 없음")

    # 호스트 도착 지터는 참고용. 저장 스레드가 밀리는지 보는 값이지
    # 데이터 손상 여부가 아니다.
    dt = np.diff(uniq_pkt_ts)
    print(f"          호스트 도착 지터 최대 {dt.max() * 1000:.2f} ms "
          f"(정상 {PACKET_INTERVAL_US / 1000:.2f} ms 간격, 참고용)")

    # --- 회전 ---
    revs = list(d.revolutions())
    counts = np.array([b - a for a, b in revs])
    starts = np.array([d.packet_ts_us[a] for a, b in revs], dtype=np.float64)
    if len(starts) > 3:
        per = np.diff(starts[1:-1]) / 1000.0        # ms, 잘린 앞뒤 제외
        hz = 1000.0 / per.mean() if per.mean() else 0
        dev = ""
        if abs(hz - EXPECTED_HZ) > 0.15:
            dev, ok = f"   [주의] {EXPECTED_HZ} Hz 에서 벗어남", ok
        print(f"          회전 {len(revs)} 개  주기 {per.mean():.2f} ± "
              f"{per.std():.3f} ms → {hz:.3f} Hz{dev}")
        mid = counts[1:-1]
        print(f"          회전당 점 평균 {mid.mean():.0f}  "
              f"최소 {mid.min()}  최대 {mid.max()}  "
              f"(앞뒤 잘린 회전 2개 제외)")

    # --- 호스트 시계 대비 라이다 시계 드리프트 ---
    lid = d.pkt_ts_us.astype(np.float64) / 1e6
    lid = np.unwrap(lid, period=3600.0)             # 정시 되돌아감 처리
    if len(lid) > 10:
        drift_ms = ((lid[-1] - lid[0]) - (uniq_pkt_ts[-1] - uniq_pkt_ts[0])) * 1000
        ppm = drift_ms / (span * 1000) * 1e6 if span else 0
        note = "" if abs(ppm) < 200 else "   [주의] 드리프트가 큽니다"
        print(f"          시계 드리프트 {drift_ms:+.2f} ms / {span:.1f} s "
              f"({ppm:+.0f} ppm){note}")

    # --- PPS ---
    pos = vlp16.read_position_packets(pcap)
    if not pos:
        print("          [주의] 위치 패킷(8308) 없음 — PPS 상태를 알 수 없습니다.")
    else:
        states = {}
        for p in pos:
            states[p["pps_state"]] = states.get(p["pps_state"], 0) + 1
        desc = ", ".join(f"{vlp16.PPS_STATES.get(k, k)} {v}"
                         for k, v in sorted(states.items()))
        locked = states.get(2, 0)
        if locked == len(pos):
            print(f"          PPS Locked (전 구간)  — 하드웨어 시각 동기 정상")
        elif locked:
            print(f"          [주의] PPS 가 세션 중 흔들립니다: {desc}")
        else:
            print(f"          [주의] PPS {desc} — 라이다가 외부 시각을 "
                  f"못 받고 있습니다.")
            print( "                 카메라와의 결합은 호스트 수신 시각 "
                   "기준이 됩니다.")
        nmea = next((p["nmea"] for p in pos if p["nmea"]), "")
        if nmea:
            print(f"          NMEA {nmea}")

    return ok


# ===========================================================================
#  카메라 (슬롯 인식)
# ===========================================================================
def check_cameras(session_dir, log, parse_frames_csv):
    cam_dirs = sorted(d for d in os.listdir(session_dir)
                      if d.startswith("cam") and d[3:].isdigit() and
                      os.path.isdir(os.path.join(session_dir, d)))
    if not cam_dirs:
        print("  카메라: 폴더 없음 (라이다 전용 세션)")
        return None

    ok = True
    t_trig = log["t_trig"] if log else {}

    print(f"\n  {'cam':<7}{'frames':>8}{'files':>8}{'gapID':>8}"
          f"{'offset':>9}{'holes':>7}{'lookup':>9}")
    print("  " + "-" * 56)

    for cam in cam_dirs:
        slot = int(cam[3:])
        cdir = os.path.join(session_dir, cam)
        rows = parse_frames_csv(os.path.join(cdir, "frames.csv"))
        if not rows:
            print(f"  {cam:<7}{'—':>8}  (프레임 없음)")
            ok = False
            continue

        fids = [r[0] for r in rows]
        try:
            files = len([f for f in os.listdir(cdir)
                         if f.endswith((".npy", ".png"))])
        except OSError:
            files = 0
        if files != len(fids):
            ok = False

        # FrameID 가 건너뛰면 USB 전송 중 유실. 해당 프레임만 버리면 된다.
        id_gaps = sum(1 for a, b in zip(fids, fids[1:]) if b - a != 1)
        if id_gaps:
            ok = False

        first_n = log["first_exp"].get(slot) if log else None
        offset = (first_n - fids[0]) if first_n is not None else None

        # cam_timestamp 간격 구멍 = 트리거 씹음 교차검증
        ts = [r[1] for r in rows]
        holes = 0
        if len(ts) > 2:
            unit = 1000 if (ts[1] - ts[0]) > TRIGGER_PERIOD_US * 100 else 1
            for a, b in zip(ts, ts[1:]):
                if (b - a) / unit > TRIGGER_PERIOD_US * GAP_FACTOR:
                    holes += 1
        if holes:
            ok = False

        if offset is not None and t_trig:
            hit = sum(1 for f in fids if (f + offset) in t_trig)
            lookup = f"{hit}/{len(fids)}"
            if hit != len(fids):
                ok = False
        else:
            lookup = "N/A"

        print(f"  {cam:<7}{len(fids):>8}{files:>8}{id_gaps:>8}"
              f"{str(offset):>9}{holes:>7}{lookup:>9}")
    return ok


# ===========================================================================
#  MCU
# ===========================================================================
def check_mcu(session_dir, parse_mcu_log, slots):
    path = os.path.join(session_dir, "mcu_log.txt")
    if not os.path.exists(path):
        return None, None
    log = parse_mcu_log(path)
    t_trig = log["t_trig"]
    print(f"  MCU     트리거 {len(t_trig)} 줄, PPS {log['pps']}, "
          f"NMEA {log['nmea']}")
    ok = True
    if not t_trig:
        print("          [실패] T 줄이 없음 — GO 가 안 먹었거나 FSYNC 미송출")
        return log, False

    ns = sorted(t_trig)
    gaps = [t_trig[b] - t_trig[a] for a, b in zip(ns, ns[1:])]
    if gaps:
        bad = [g for g in gaps if abs(g - TRIGGER_PERIOD_US) > 5000]
        print(f"          트리거 간격 {min(gaps)} ~ {max(gaps)} us"
              + (f"   [주의] 이상 {len(bad)} 건" if bad else ""))
        if bad:
            ok = False

    if log["cnt"]:
        shot, *cams_cnt = log["cnt"]
        print(f"          CNT 쏜횟수 {shot}  채널별 노출 {cams_cnt}")
        # 펌웨어는 항상 4채널을 출력한다. 실제로 꽂혀 있는 슬롯만 본다.
        targets = slots or list(range(1, len(cams_cnt) + 1))
        for slot in targets:
            got = cams_cnt[slot - 1] if slot <= len(cams_cnt) else None
            if got == shot:
                print(f"            CAM{slot}: {got}/{shot}  OK")
                continue
            ok = False
            print(f"            CAM{slot}: {got}/{shot}  [경고] 불일치")
            # 0 인데 다른 채널이 세고 있으면 배선/슬롯표가 어긋난 것이다.
            # 트리거 씹음과 증상이 전혀 다르므로 구분해서 알려 준다.
            if not got:
                others = [i + 1 for i, c in enumerate(cams_cnt)
                          if c and (i + 1) != slot]
                if others:
                    print(f"              → 대신 채널 {others} 가 세고 있습니다. "
                          f"카메라의 Line1 배선이나")
                    print(f"                sync_cam.CAMERA_SNS 의 슬롯 순서가 "
                          f"어긋났을 수 있습니다.")
                else:
                    print(f"              → 어느 채널도 세지 않습니다. Line1 "
                          f"미배선 또는 LineSource 설정 확인.")
            elif got > shot:
                print(f"              → 쏜 횟수보다 많습니다. 입력 핀 채터링 "
                      f"(풀업/노이즈) 을 의심하세요.")
    else:
        print("          [주의] CNT 줄 없음 — Line1 미배선이거나 STOP 전 종료")
    return log, ok


# ===========================================================================
#  세션 하나
# ===========================================================================
def check(session_dir, ws=DEFAULT_DAHENG_WS):
    name = os.path.basename(session_dir.rstrip("/"))
    print(f"\n===== {name} =====")

    parse_mcu_log, parse_frames_csv = load_parsers(ws)
    cam_dirs = [d for d in os.listdir(session_dir)
                if d.startswith("cam") and d[3:].isdigit() and
                os.path.isdir(os.path.join(session_dir, d))]
    slots = sorted(int(d[3:]) for d in cam_dirs)

    results = []
    log = None
    if parse_mcu_log is not None:
        log, mcu_ok = check_mcu(session_dir, parse_mcu_log, slots)
        if mcu_ok is not None:
            results.append(mcu_ok)
    elif cam_dirs:
        print(f"  [주의] {ws} 에서 파서를 불러오지 못해 카메라 검사를 "
              f"건너뜁니다.")

    lidar_ok = check_lidar(session_dir)
    if lidar_ok is not None:
        results.append(lidar_ok)

    if cam_dirs and parse_frames_csv is not None:
        cam_ok = check_cameras(session_dir, log, parse_frames_csv)
        if cam_ok is not None:
            results.append(cam_ok)

    # --- 두 센서의 시간 범위가 겹치는지 ---
    _cross_check(session_dir, cam_dirs, parse_frames_csv)

    ok = all(results) if results else False
    print(f"\n  → {'OK' if ok else '문제 있음 — 위 경고 확인'}")
    return ok


def _cross_check(session_dir, cam_dirs, parse_frames_csv):
    """카메라와 라이다의 호스트 시각 구간이 실제로 겹치는지 본다."""
    if not cam_dirs or parse_frames_csv is None:
        return
    pcap = os.path.join(session_dir, "lidar.pcap")
    if not os.path.exists(pcap):
        return
    times, _buf, n = vlp16.read_pcap(pcap, vlp16.DATA_PORT)
    if not n:
        return
    l0, l1 = float(times[0]), float(times[-1])

    for cam in sorted(cam_dirs):
        rows = parse_frames_csv(os.path.join(session_dir, cam, "frames.csv"))
        if not rows:
            continue
        c0, c1 = rows[0][2], rows[-1][2]
        overlap = min(c1, l1) - max(c0, l0)
        total = max(c1, l1) - min(c0, l0)
        pct = 100.0 * overlap / total if total > 0 else 0
        mark = "" if overlap > 0 and pct > 80 else "   [주의] 구간이 어긋납니다"
        print(f"\n  겹침    {cam} {c0:.3f}~{c1:.3f}  "
              f"라이다 {l0:.3f}~{l1:.3f}")
        print(f"          공통 구간 {overlap:.2f} s / 전체 {total:.2f} s "
              f"= {pct:.1f}%{mark}")


def main():
    ap = argparse.ArgumentParser(description="세션 검산 (카메라 + 라이다)")
    ap.add_argument("path", help="세션 폴더 또는 상위 폴더")
    ap.add_argument("--daheng-ws", default=DEFAULT_DAHENG_WS)
    args = ap.parse_args()

    p = os.path.abspath(args.path)
    if not os.path.isdir(p):
        print(f"폴더가 없습니다: {p}")
        return 1
    if (os.path.exists(os.path.join(p, "meta.json")) or
            os.path.exists(os.path.join(p, "lidar.pcap")) or
            os.path.exists(os.path.join(p, "mcu_log.txt"))):
        targets = [p]
    else:
        targets = sorted(os.path.join(p, d) for d in os.listdir(p)
                         if d.startswith("session_"))
    if not targets:
        print("세션을 찾지 못했습니다.")
        return 1

    results = [check(t, args.daheng_ws) for t in targets]
    if len(results) > 1:
        bad = sum(1 for r in results if not r)
        print(f"\n===== 전체 {len(results)} 세션 중 {bad} 개 문제 =====")
    return 0 if all(results) else 2


if __name__ == "__main__":
    raise SystemExit(main())
