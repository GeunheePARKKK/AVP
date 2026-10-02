"""세션을 시간 구간으로 잘라 새 세션 폴더로 만든다.

    python3 trim_session.py sessions/session_025 --start 5 --duration 20
    python3 trim_session.py sessions/session_025 --start 5 --duration 20 --location 경북대학교_글로벌플라자_주차장

왜 필요한가

  녹화 시작 신호를 받고 차가 실제로 움직이기 시작할 때까지 몇 초가 걸린다.
  그 구간을 그대로 두면 정지 장면이 앞에 붙는다. 25초로 찍고 뒤 20초만
  남기면 운전 구간이 온전히 담긴다.

어떻게 자르는가

  기준 시각(anchor)은 '모든 센서가 데이터를 내놓기 시작한 순간' —
  카메라 4대의 첫 프레임 시각과 라이다 첫 패킷 시각 중 가장 늦은 것.
  그 시점부터 --start 초 뒤를 자르기 시작점으로 삼는다. 다섯 센서가
  모두 같은 절대 시각에서 잘리므로, 하드웨어 동기가 없어도 구간은 맞는다.

  카메라 : frames.csv 의 host_recv_unix 로 프레임 선별. npy 는 하드링크로
           연결하므로 디스크를 더 쓰지 않는다 (원본을 지워도 데이터는 남는다)
  라이다 : pcap 전역 헤더 24바이트를 복사하고, 패킷 레코드(16바이트 헤더
           + 데이터)를 타임스탬프로 걸러 그대로 옮긴다. 무손실이다
"""

import argparse
import csv
import json
import os
import shutil
import struct
import sys


def read_frames(cam_dir):
    with open(os.path.join(cam_dir, "frames.csv")) as f:
        return list(csv.DictReader(f))


def pcap_packets(path):
    """(timestamp, raw record bytes) 를 순서대로 내놓는다."""
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise SystemExit(f"pcap 헤더가 짧습니다: {path}")
        magic = gh[:4]
        if magic == b"\xd4\xc3\xb2\xa1":
            endian, nano = "<", False
        elif magic == b"\xa1\xb2\xc3\xd4":
            endian, nano = ">", False
        elif magic == b"\x4d\x3c\xb2\xa1":
            endian, nano = "<", True
        elif magic == b"\xa1\xb2\x3c\x4d":
            endian, nano = ">", True
        else:
            raise SystemExit(f"classic pcap 이 아닙니다 (magic {magic.hex()})")
        recs = []
        while True:
            ph = f.read(16)
            if len(ph) < 16:
                break
            ts_s, ts_frac, incl, orig = struct.unpack(endian + "IIII", ph)
            data = f.read(incl)
            if len(data) < incl:
                break                       # 녹화가 중간에 끊긴 꼬리
            ts = ts_s + ts_frac / (1e9 if nano else 1e6)
            recs.append((ts, ph + data))
    return gh, recs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--start", type=float, required=True,
                    help="기준 시각 기준 자르기 시작(초)")
    ap.add_argument("--duration", type=float, required=True, help="길이(초)")
    ap.add_argument("-o", "--out", default=None,
                    help="출력 폴더 (기본: <세션>_<duration>s)")
    ap.add_argument("--location", default=None,
                    help="meta.json 에 기록할 촬영 장소")
    ap.add_argument("--copy", action="store_true",
                    help="하드링크 대신 실제 복사")
    args = ap.parse_args()

    src = args.session.rstrip("/")
    dst = args.out or f"{src}_{int(args.duration)}s"
    if os.path.exists(dst):
        raise SystemExit(f"출력 폴더가 이미 있습니다: {dst}")

    with open(os.path.join(src, "meta.json")) as f:
        meta = json.load(f)

    cams = sorted(d for d in os.listdir(src)
                  if d.startswith("cam") and os.path.isdir(os.path.join(src, d)))
    if not cams:
        raise SystemExit("카메라 폴더가 없습니다")

    frames = {c: read_frames(os.path.join(src, c)) for c in cams}
    pcap_src = os.path.join(src, "lidar.pcap")
    gh, recs = pcap_packets(pcap_src)

    # --- 기준 시각: 모든 센서가 데이터를 내기 시작한 순간 ---
    firsts = [float(frames[c][0]["host_recv_unix"]) for c in cams]
    firsts.append(recs[0][0])
    anchor = max(firsts)
    t0 = anchor + args.start
    t1 = t0 + args.duration
    print(f"기준 시각 {anchor:.6f}  →  자르기 [{t0:.6f}, {t1:.6f})")
    print(f"  센서별 첫 데이터 시각 편차 {max(firsts) - min(firsts):.3f} s")

    os.makedirs(dst)

    # --- 카메라 ---
    kept = {}
    for c in cams:
        os.makedirs(os.path.join(dst, c))
        rows = [r for r in frames[c]
                if t0 <= float(r["host_recv_unix"]) < t1]
        for r in rows:
            name = f"{int(r['frame_id']):08d}.npy"
            s = os.path.join(src, c, name)
            d = os.path.join(dst, c, name)
            if not os.path.exists(s):
                raise SystemExit(f"프레임 파일이 없습니다: {s}")
            if args.copy:
                shutil.copy2(s, d)
            else:
                try:
                    os.link(s, d)
                except OSError:
                    shutil.copy2(s, d)
        with open(os.path.join(dst, c, "frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["frame_id", "cam_timestamp",
                                              "host_recv_unix"])
            w.writeheader()
            w.writerows(rows)
        kept[c] = len(rows)
        print(f"  {c}: {len(rows)} 장")

    # --- 라이다 ---
    sel = [r for ts, r in recs if t0 <= ts < t1]
    with open(os.path.join(dst, "lidar.pcap"), "wb") as f:
        f.write(gh)
        for r in sel:
            f.write(r)
    print(f"  라이다: {len(sel)} 패킷 "
          f"({os.path.getsize(os.path.join(dst, 'lidar.pcap'))/1e6:.1f} MB)")

    # --- meta.json ---
    meta["duration_s"] = args.duration
    meta["frames_written"] = kept
    meta["trimmed_from"] = {
        "session": os.path.basename(src),
        "anchor_unix": anchor,
        "start_offset_s": args.start,
        "cut_unix": [t0, t1],
    }
    if args.location:
        meta["location"] = args.location
    for extra in ("mcu_log.txt",):
        p = os.path.join(src, extra)
        if os.path.exists(p):
            shutil.copy2(p, os.path.join(dst, extra))
    with open(os.path.join(dst, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    print(f"완료: {dst}")


if __name__ == "__main__":
    main()
