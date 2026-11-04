"""MCU 로그 / frames.csv 파서 — 여러 스크립트가 공유한다.

원래 check_session.py 안에 있던 함수다. 라이다 검산이 추가된 새
check_session.py 가 같은 이름을 쓰게 되면서 여기로 분리했다.
(같은 폴더에서 `import check_session` 을 하면 자기 자신을 불러오게 된다.)

    from mcu_parse import parse_mcu_log, parse_frames_csv

parse_mcu_log(path) → dict
    t_trig     {N: us}          T 줄. 트리거 펄스 번호 → MCU 시각
    first_exp  {cam: N}         카메라별 첫 EXP 줄의 펄스 번호 (offset 계산용)
    exp        [(cam, N, us)]   EXP 줄 전부
    cnt        [쏜횟수, c1, c2, c3, c4] 또는 None
    blank      [b1, b2, b3, b4] FSYNC 직후 걸러낸 잡음 엣지 수. 없으면 None
    pps, nmea  줄 수

parse_frames_csv(path) → [(frame_id, cam_timestamp, host_recv_unix)] (FrameID 순)
"""

import csv
import os


def parse_mcu_log(path):
    t_trig = {}          # N -> us
    first_exp = {}       # cam(1~4) -> N
    exp_lines = []
    cnt = None
    blank = None
    nmea = 0
    pps = 0

    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            tag = parts[0]
            try:
                if tag == "T" and len(parts) >= 3:
                    t_trig[int(parts[1])] = int(parts[2])
                elif tag == "EXP" and len(parts) >= 4:
                    cam, n = int(parts[1]), int(parts[2])
                    exp_lines.append((cam, n, int(parts[3])))
                    if cam not in first_exp:
                        first_exp[cam] = n
                elif tag == "CNT" and len(parts) >= 2:
                    cnt = [int(x) for x in parts[1:]]
                elif tag == "BLANK" and len(parts) >= 2:
                    # 펌웨어가 FSYNC 직후에 걸러낸 Line1 잡음 엣지 수
                    blank = [int(x) for x in parts[1:]]
                elif tag == "PPS":
                    pps += 1
                elif tag == "NMEA":
                    nmea += 1
            except ValueError:
                continue

    return {"t_trig": t_trig, "first_exp": first_exp, "exp": exp_lines,
            "cnt": cnt, "blank": blank, "pps": pps, "nmea": nmea}


def parse_frames_csv(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((int(r["frame_id"]), int(r["cam_timestamp"]),
                             float(r["host_recv_unix"])))
            except (ValueError, KeyError):
                continue
    rows.sort()
    return rows
