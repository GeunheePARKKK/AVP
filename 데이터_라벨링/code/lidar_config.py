"""VLP-16 설정 조회 / 변경.

    python3 lidar_config.py                      현재 설정과 상태
    python3 lidar_config.py --backup s.json      설정을 파일로 저장
    python3 lidar_config.py --set-rpm 600 --yes
    python3 lidar_config.py --phase-lock on --offset 0 --yes
    python3 lidar_config.py --phase-lock off --yes
    python3 lidar_config.py --phase-lock on --offset 0 --save --yes   플래시 저장까지

--------------------------------------------------------------------------
Phase Lock 이 하는 일

  PPS 펄스가 들어오는 그 순간 라이다가 지정한 방위각을 보고 있도록
  회전 위상을 맞춘다. 600 rpm(정확히 10 Hz)에서 PPS 는 1초에 한 번이므로,
  한 번 맞으면 그 초 안의 회전 10번이 전부 결정된 시각에 일어난다.

  ESP32 는 카메라 FSYNC 와 PPS 를 같은 타이머에서 만든다. 따라서
  Phase Lock 이 걸리면 카메라 셔터와 라이다 방위각이 한 시계 위에 놓인다.

  offset 은 도(degree) 단위다. 카메라가 보는 쪽 방위각을 넣으면
  셔터가 열리는 순간 라이다가 같은 곳을 지나간다.

  ※ PPS 가 없으면 Phase Lock 은 아무 일도 하지 않는다. 먼저
    sync_report.py 나 --status 로 PPS 가 Locked 인지 확인할 것.

--------------------------------------------------------------------------
쓰기는 반드시 --yes 가 있어야 하고, 쓴 뒤에는 다시 읽어서 확인한다.

  이 장비의 웹 서버는 잘못된 요청에도 조용히 200 을 주는 일이 있어서,
  '보냈다'가 '바뀌었다'를 뜻하지 않는다. 그래서 항상 되읽어 대조한다.
"""

import argparse
import json
import sys
import urllib.error
import urllib.parse
import urllib.request

HOST = "192.168.1.201"
TIMEOUT = 8.0

# 이 필드들은 /cgi/setting 이 한 번에 받는다.
SETTING_FIELDS = ("rpm", "returns", "laser", "fov_start", "fov_end")
PHASELOCK_PATH = "/cgi/setting/phaselock"
SETTING_PATH = "/cgi/setting"
# /cgi/setting/* 로 바꾼 값은 RAM 에만 들어간다. 웹 UI 의 'Save Configuration'
# 버튼(tab/config.html 의 saveForm)이 이 주소로 POST 해야 플래시에 남는다.
# 저장하지 않으면 전원을 껐다 켤 때 조용히 이전 값으로 돌아간다 — Phase Lock 이
# 실제로 그렇게 사라졌다. 이 리그는 배선 뒤 전원 재투입이 필수라 저장이 필수다.
SAVE_PATH = "/cgi/save"


def get_json(host, path):
    url = f"http://{host}{path}"
    with urllib.request.urlopen(url, timeout=TIMEOUT) as r:
        return json.loads(r.read())


def post(host, path, fields):
    url = f"http://{host}{path}"
    data = urllib.parse.urlencode(fields).encode()
    req = urllib.request.Request(url, data=data, method="POST")
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
        return r.status, r.read().decode("utf-8", "replace")[:200]


def show(host):
    st = get_json(host, "/cgi/status.json")
    se = get_json(host, "/cgi/settings.json")

    motor = st.get("motor", {})
    gps = st.get("gps", {})
    pl = se.get("phaselock", {})

    print(f"라이다 {host}")
    print(f"  레이저        {se.get('laser')}")
    print(f"  반사 모드     {se.get('returns')}")
    print(f"  설정 rpm      {se.get('rpm')}   (= {se.get('rpm', 0) / 60:.3f} Hz)")
    print(f"  실측 rpm      {motor.get('rpm')}   모터 {motor.get('state')}")
    print(f"  시야각        {se.get('fov', {}).get('start')} ~ "
          f"{se.get('fov', {}).get('end')} 도")
    print()
    print(f"  PPS 상태      {gps.get('pps_state')}")
    print(f"  NMEA 위치     {gps.get('position') or '(없음)'}")
    try:
        off_deg = float(pl.get("offset", 0)) / 100.0
    except (TypeError, ValueError):
        off_deg = None
    print(f"  Phase Lock    {pl.get('enabled')}   "
          f"offset {off_deg if off_deg is not None else '?'} 도 "
          f"(원시값 {pl.get('offset')} = 0.01도 단위)")
    print(f"  모터 lock     {motor.get('lock')}   phase {motor.get('phase')}")

    ok = True
    if se.get("rpm") != 600:
        print(f"\n  [주의] rpm 이 600 이 아닙니다. 카메라 10 Hz 와 맞추려면 600.")
        ok = False
    if gps.get("pps_state") != "Locked":
        print(f"\n  [주의] PPS 가 '{gps.get('pps_state')}' 입니다.")
        print( "         Phase Lock 은 PPS 가 Locked 여야 동작합니다.")
        ok = False
    elif str(pl.get("enabled")).lower() in ("off", "false", "0"):
        print(f"\n  [주의] PPS 는 잡혔는데 Phase Lock 이 꺼져 있습니다.")
        print( "         --phase-lock on --offset <각도> 로 켜세요.")
        ok = False
    if ok:
        print("\n  카메라와 하드웨어로 물릴 조건을 모두 만족합니다.")
    return se


def verify(host, expect, label):
    """쓴 뒤 되읽어 실제로 바뀌었는지 확인한다."""
    import time
    time.sleep(1.5)
    se = get_json(host, "/cgi/settings.json")
    bad = []
    for path, want in expect.items():
        cur = se
        for k in path.split("."):
            cur = (cur or {}).get(k) if isinstance(cur, dict) else None
        if str(cur).lower() != str(want).lower():
            bad.append(f"{path}: 기대 {want}, 실제 {cur}")
    if bad:
        print(f"  [실패] {label} 이(가) 반영되지 않았습니다:")
        for b in bad:
            print(f"           {b}")
        print( "         이 펌웨어의 엔드포인트가 다를 수 있습니다.")
        print(f"         웹 UI(http://{host}) 에서 직접 바꾸세요.")
        return False
    print(f"  [OK]   {label} 반영 확인")
    return True


def watch(host, seconds):
    """PPS 상태와 NMEA 를 라이다의 위치 패킷에서 실시간으로 본다.

    배선을 만지면서 보라고 만든 것이다. 설정 API(status.json)를 반복해서
    긁으면 갱신이 느리고 웹 서버에도 부담이라, 라이다가 초당 150번씩
    쏘는 위치 패킷을 직접 받는다.

    상태값의 뜻이 진단에 그대로 쓰인다.
      Absent        PPS 입력에 펄스 자체가 없다      → 배선/전원 문제
      Synchronizing 펄스는 오는데 시각 문자열이 없다  → GPRMC 쪽 문제
      Locked        둘 다 정상
    """
    import socket
    import time
    STATES = {0: "Absent", 1: "Synchronizing", 2: "Locked", 3: "Error"}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", 8308))
    except OSError as e:
        print(f"[실패] UDP 8308 을 열 수 없습니다: {e}")
        return 1
    s.settimeout(3.0)

    print(f"위치 패킷 감시 {seconds}초 — 배선을 만지면서 보세요. Ctrl+C 로 중단.")
    print(f"  {'경과':>6}  {'PPS':<14} NMEA")
    t0 = time.time()
    last = None
    try:
        while time.time() - t0 < seconds:
            try:
                p = s.recv(1024)
            except socket.timeout:
                print("  위치 패킷이 오지 않습니다 (라이다 연결 확인)")
                continue
            if len(p) < 512:
                continue
            st = STATES.get(p[202], p[202])
            nmea = p[206:306].split(b"\x00")[0].decode("ascii", "replace").strip()
            cur = (st, nmea)
            if cur != last:
                print(f"  {time.time() - t0:5.1f}s  {st:<14} {nmea or '(없음)'}")
                last = cur
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n  중단")
    finally:
        s.close()
    print(f"\n  마지막 상태: {last[0] if last else '?'}")
    if last and last[0] == "Absent":
        print("  → PPS 입력에 펄스가 전혀 없습니다. 신호원(ESP32)이 켜져 있는지,")
        print("    GPIO5 가 I/F Box 의 GPS PULSE(노랑)에 닿아 있는지 보세요.")
    elif last and last[0] == "Synchronizing":
        print("  → 펄스는 도착합니다. 남은 것은 GPRMC 쪽입니다 (GPIO17/MAX3232).")
    return 0


def main():
    ap = argparse.ArgumentParser(description="VLP-16 설정 조회/변경")
    ap.add_argument("--watch", type=float, nargs="?", const=60.0, default=None,
                    metavar="초", help="PPS/NMEA 를 실시간으로 감시한다")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--backup", metavar="파일",
                    help="현재 설정을 JSON 으로 저장")
    ap.add_argument("--set-rpm", type=int, default=None,
                    help="회전 속도 (600 = 10 Hz)")
    ap.add_argument("--phase-lock", choices=["on", "off"], default=None)
    ap.add_argument("--offset", type=float, default=None,
                    help="Phase Lock 방위각 (도). PPS 순간 라이다가 볼 각도. "
                         "장비에는 0.01도 단위로 변환해 보낸다")
    ap.add_argument("--save", action="store_true",
                    help="현재 설정을 라이다 플래시에 저장한다 (전원을 꺼도 유지)")
    ap.add_argument("--yes", action="store_true",
                    help="실제로 쓴다. 없으면 무엇을 보낼지만 출력한다")
    args = ap.parse_args()

    if args.watch is not None:
        return watch(args.host, args.watch)

    try:
        se = show(args.host)
    except (urllib.error.URLError, OSError) as e:
        print(f"[실패] {args.host} 에 연결할 수 없습니다: {e}")
        print( "  - ping 192.168.1.201")
        print( "  - 이더넷 링크:  cat /sys/class/net/<iface>/carrier   (1 이어야 함)")
        print( "  - 라이다 전원과 랜선 확인")
        return 1

    if args.backup:
        with open(args.backup, "w", encoding="utf-8") as f:
            json.dump(se, f, indent=2, ensure_ascii=False)
        print(f"\n설정을 저장했습니다: {args.backup}")

    writes = []
    if args.set_rpm is not None:
        writes.append((SETTING_PATH, {"rpm": args.set_rpm},
                       {"rpm": args.set_rpm}, f"rpm={args.set_rpm}"))
    if args.phase_lock is not None:
        f = {"enabled": "on" if args.phase_lock == "on" else "off"}
        exp = {"phaselock.enabled": "On" if args.phase_lock == "on" else "Off"}
        if args.offset is not None:
            # 이 장비는 offset 을 0.01도 단위로 저장한다. 웹 UI 도 입력칸의
            # 값에 100 을 곱해서 보낸다 (tab/config.html 의 phaselock 폼):
            #     this.form.offset.value = 100 * this.form.offsetInput.value
            # 도 단위로 그냥 보내면 100배 작은 각도가 걸려서, 겉보기에는
            # 성공한 것처럼 보이는데 위상이 엉뚱한 곳에 맞는다.
            raw = int(round(args.offset * 100))
            f["offset"] = raw
            exp["phaselock.offset"] = raw
        writes.append((PHASELOCK_PATH, f, exp,
                       f"phase-lock={args.phase_lock}"
                       + (f" offset={args.offset}도(={f['offset']})"
                          if args.offset is not None else "")))
    if not writes and not args.save:
        return 0

    print("\n보낼 요청:")
    for path, fields, _exp, label in writes:
        print(f"  POST http://{args.host}{path}   {urllib.parse.urlencode(fields)}")
    if args.save:
        print(f"  POST http://{args.host}{SAVE_PATH}   (현재 설정 전체를 플래시에 저장)")
    if not args.yes:
        print("\n실제로 쓰려면 --yes 를 붙이세요. (지금은 아무것도 바꾸지 않았습니다)")
        return 0

    if args.phase_lock == "on":
        pps = get_json(args.host, "/cgi/status.json").get("gps", {}).get("pps_state")
        if pps != "Locked":
            print(f"\n[주의] PPS 가 '{pps}' 인 상태로 Phase Lock 을 켭니다.")
            print( "       PPS 가 들어오기 전까지는 아무 효과가 없습니다.")

    print()
    ok = True
    for path, fields, exp, label in writes:
        try:
            status, body = post(args.host, path, fields)
            print(f"  {label}: HTTP {status} {body.strip()[:60]}")
        except (urllib.error.URLError, OSError) as e:
            print(f"  {label}: [실패] {e}")
            ok = False
            continue
        ok &= verify(args.host, exp, label)

    if args.save and ok:
        try:
            status, _body = post(args.host, SAVE_PATH, {})
            print(f"  save: HTTP {status}")
            # 저장 직후 설정이 그대로인지만 확인할 수 있다. 플래시에 실제로
            # 남았는지는 전원을 껐다 켠 뒤 다시 조회해야 알 수 있다.
            after = get_json(args.host, "/cgi/settings.json")
            pl = after.get("phaselock", {})
            print(f"  [OK]   저장 요청 완료. 현재 Phase Lock {pl.get('enabled')} / "
                  f"offset {float(pl.get('offset', 0)) / 100:g}도")
            print( "         전원 재투입 후 이 스크립트로 다시 조회해 유지되는지 확인하세요.")
        except (urllib.error.URLError, OSError) as e:
            print(f"  save: [실패] {e}")
            ok = False
    elif writes and ok:
        print("\n  [주의] RAM 에만 반영되었습니다. 라이다 전원을 끄면 사라집니다.")
        print("         유지하려면 --save 를 붙여 다시 실행하세요.")

    print("\n" + ("완료. 다음 세션을 찍고 sync_report.py 로 확인하세요."
                 if ok else "일부가 반영되지 않았습니다. 위 내용을 확인하세요."))
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
