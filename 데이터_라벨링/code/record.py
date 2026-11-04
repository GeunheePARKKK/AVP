"""카메라 + 라이다 동시 수집 — collect.py 의 확장판.

daheng_ws_1/collect.py 는 카메라와 MCU 까지만 다루고, 세션 폴더에
lidar.pcap 자리만 비워 두었다. 이 스크립트가 그 자리를 채운다.

    python3 record.py                       20초 세션 1회
    python3 record.py --sessions 10         반복
    python3 record.py --lidar-only          MCU/카메라 없이 라이다만
    python3 record.py --duration 30 --no-preview

--------------------------------------------------------------------------
재사용하는 것 / 새로 쓴 것

  재사용  sync_cam.CameraArray     하드웨어 트리거 카메라 (그대로)
          collect.McuLink          ESP32 시리얼 + 로그 수신 (그대로)
          collect.find_port        포트 자동 판별 (그대로)
          collect.Preview          2x2 미리보기 (그대로)

  새로 씀 lidar_capture.VelodyneCapture   UDP → pcap
          SessionWriter                    아래 이유로 collect.Writer 를 대체

  collect.Writer 는 폴더 이름을 도착 순서 인덱스로 짓는다 (cam1, cam2...).
  4대 중 3번만 꽂혀 있으면 그 카메라가 cam1 폴더에 들어가서, 나중에
  ESP32 의 어느 GPIO 가 그 카메라를 쏘았는지 알 수 없게 된다.
  여기서는 CAMERA_SNS 안에서의 물리 슬롯 번호로 폴더를 짓는다.
  1대만 연결된 지금 FHH26070139 는 CAM3 이므로 cam3/ 에 저장된다.

--------------------------------------------------------------------------
세션 순서

  1. 라이다 수신 시작        ← 카메라보다 먼저. 소켓은 계속 열어 둔다.
  2. MCU READY 대기
  3. 카메라 열기 → 트리거 대기
  4. 세션마다: pcap 열기 → GO → N초 → STOP → pcap 닫기
  5. 요약을 meta.json 에 남긴다

  라이다 소켓은 세션 사이에도 닫지 않는다. 닫았다 열면 그 틈에 커널
  수신 버퍼가 넘쳐 다음 세션 첫 회전이 깨진 채로 시작한다.

--------------------------------------------------------------------------
저장 구조

  sessions/session_001/
      meta.json          설정 + 카메라 + 라이다 요약 (수집 후 갱신)
      mcu_log.txt        MCU 시리얼 원문 (T / EXP / CNT / PPS / NMEA)
      lidar.pcap         2368 + 8308 포트 원본
      cam3/
          frames.csv     frame_id, cam_timestamp, host_recv_unix
          00000001.npy   파일명 = FrameID (도착 순서가 아님)
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from datetime import datetime
from queue import Empty

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from lidar_capture import VelodyneCapture, DATA_PORT, POSITION_PORT  # noqa: E402

DEFAULT_DAHENG_WS = "/home/acelab/Documents/daheng_ws_1"
LIDAR_IP = "192.168.1.201"
EXPECTED_PPS = 754          # VLP-16 10 Hz 단일반사 초당 데이터 패킷
N_WRITERS = 4


class SerialDrain:
    """ESP32 시리얼을 읽기만 한다. 명령은 보내지 않는다.

    라이다 단독 수집에서도 반드시 필요하다. 펌웨어는 PPS 를 타이머
    인터럽트에서 올리지만 20 ms 뒤에 내리는 일은 loop() 가 한다. 아무도
    USB 시리얼을 읽지 않으면 출력 버퍼가 차서 Serial.print 가 막히고,
    loop() 가 멈춘 동안 PPS 는 HIGH 에서 내려오지 못한다.

    2026-09-14 실측: 같은 배선에서
        시리얼을 안 읽을 때  라이다 PPS Absent 1093/1093, GPIO5 3.3 V 고정
        계속 읽어 줄 때      2.3 초 만에 Locked

    네이티브 USB 포트는 열어도 ESP32 가 리셋되지 않는다 (PPS 번호가 이어짐).
    읽은 내용은 세션마다 mcu_log.txt 로 남긴다 — GPRMC 를 실제로 보냈는지
    나중에 대조할 수 있다.
    """

    def __init__(self, port, baud=921600):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._fp = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.lines = 0
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                buf += self.ser.read(8192)
            except Exception:
                break
            while b"\n" in buf:
                ln, buf = buf.split(b"\n", 1)
                self.lines += 1
                with self._lock:
                    if self._fp is not None:
                        self._fp.write(ln.decode("utf-8", "replace").rstrip("\r") + "\n")

    def set_logfile(self, path):
        with self._lock:
            if self._fp is not None:
                self._fp.close()
                self._fp = None
            if path:
                self._fp = open(path, "w", encoding="utf-8")

    def close(self):
        self._stop.set()
        self._t.join(timeout=1.0)
        self.set_logfile(None)
        try:
            self.ser.close()
        except Exception:
            pass


def find_mcu_port():
    """ESP32 의 시리얼 포트를 찾는다. 없으면 None.

    collect.find_port 는 후보가 없으면 목록의 첫 번째 포트를 그냥 쓴다.
    ESP32 가 빠져 있으면 메인보드의 /dev/ttyS31 같은 것을 잡고
    "Could not configure port" 로 죽는데, 그 메시지만 보면 권한 문제처럼
    보여서 엉뚱한 곳을 뒤지게 된다. 여기서는 실제 USB 장치만 받아들인다.
    """
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:                       # 1순위: Espressif VID
        if (p.vid or 0) == 0x303A:
            return p.device
    for p in ports:                       # 2순위: USB 로 붙은 CDC/브리지
        if p.vid and ("ttyACM" in p.device or "ttyUSB" in p.device):
            return p.device
    return None


def lidar_snapshot(ip=LIDAR_IP, timeout=3.0):
    """세션을 찍는 순간의 라이다 설정과 상태.

    Phase Lock offset 이나 rpm 을 나중에 바꾸면, 옛 세션이 어떤 설정으로
    찍혔는지 알 방법이 없어진다. meta.json 에 같이 박아 둔다.
    sync_report.py 는 여기서 offset 을 읽어 기대 위상과 대조한다.
    """
    import urllib.request
    out = {}
    for key, path in (("settings", "/cgi/settings.json"),
                      ("status", "/cgi/status.json")):
        try:
            with urllib.request.urlopen(f"http://{ip}{path}", timeout=timeout) as r:
                out[key] = json.loads(r.read())
        except Exception as e:                 # 수집을 막지 않는다
            out[key + "_error"] = str(e)
    return out


def wait_pps_lock(lidar, timeout=25.0):
    """PPS 가 Locked 로 돌아올 때까지 기다린다.

    MCU 시리얼 포트를 여는 순간 ESP32 가 리셋되고, 그 몇 초 동안 PPS 가
    끊긴다. 라이다는 그때 lock 을 놓았다가 신호가 돌아오면 다시 잡는데,
    그 사이에 세션을 시작하면 앞부분이 동기가 안 된 채로 기록된다.
    나중에 pcap 을 열어보기 전에는 드러나지 않는 유형이라 여기서 막는다.

    PPS 를 아예 안 쓰는 구성도 있으므로, 못 잡아도 수집은 막지 않는다.
    """
    from vlp16 import PPS_STATES
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        st = lidar.last_pps_state
        if st == 2:
            print(f"  PPS Locked ({time.time() - t0:.1f}초 만에 재잠금)")
            return True
        if st != last:
            last = st
            print(f"\r  PPS {PPS_STATES.get(st, st)} — 재잠금 대기 중...  ",
                  end="", flush=True)
        time.sleep(0.3)
    print(f"\n  [주의] {timeout:.0f}초 안에 PPS 가 Locked 이 되지 않았습니다 "
          f"(현재 {PPS_STATES.get(lidar.last_pps_state, '?')}).")
    print( "         이 세션은 하드웨어 동기 없이 기록됩니다.")
    return False


def load_daheng(ws):
    """daheng_ws_1 의 모듈을 가져온다. 그쪽 파일은 건드리지 않는다."""
    ws = os.path.abspath(os.path.expanduser(ws))
    if not os.path.isdir(ws):
        raise SystemExit(
            f"daheng 워크스페이스를 찾을 수 없습니다: {ws}\n"
            f"  --daheng-ws 로 경로를 지정하세요.")
    if ws not in sys.path:
        sys.path.insert(0, ws)
    try:
        import sync_cam
        import collect
    except ImportError as e:
        raise SystemExit(f"{ws} 에서 sync_cam/collect 를 불러오지 못했습니다: {e}")
    return sync_cam, collect


# ===========================================================================
#  저장 스레드 — 물리 슬롯 번호로 폴더를 짓는다
# ===========================================================================
class SessionWriter:
    """큐에서 프레임을 꺼내 세션 폴더에 쓴다.

    수신 스레드에서 직접 디스크에 쓰지 않는 이유는 sync_cam.py 와 같다.
    쓰기가 밀리면 수신이 막히고, 그 유실은 FrameID 로 감지되지 않는다.
    """

    def __init__(self, queue, slots, fmt="npy", n_threads=N_WRITERS):
        self.q = queue
        self.slots = list(slots)        # 런타임 인덱스 -> 물리 슬롯(1부터)
        self.fmt = fmt
        self._n = n_threads
        self._stop = threading.Event()
        self._threads = []
        self._dir_lock = threading.Lock()
        self._session_dir = None
        self._csv = {}
        self._csv_lock = threading.Lock()
        self.written = [0] * len(self.slots)
        self.late = 0                   # 세션 밖에 도착해 버린 프레임
        if fmt == "png":
            import cv2
            self._cv2 = cv2

    def dirname(self, idx):
        return f"cam{self.slots[idx]}"

    def start(self):
        for _ in range(self._n):
            t = threading.Thread(target=self._loop, daemon=True)
            t.start()
            self._threads.append(t)

    def open_session(self, session_dir):
        with self._dir_lock:
            self._session_dir = session_dir
        with self._csv_lock:
            self._close_csv()
            for i in range(len(self.slots)):
                d = os.path.join(session_dir, self.dirname(i))
                os.makedirs(d, exist_ok=True)
                fp = open(os.path.join(d, "frames.csv"), "w",
                          encoding="utf-8", buffering=1)
                fp.write("frame_id,cam_timestamp,host_recv_unix\n")
                self._csv[i] = fp
        self.written = [0] * len(self.slots)
        self.late = 0

    def close_session(self):
        with self._dir_lock:
            self._session_dir = None
        with self._csv_lock:
            self._close_csv()

    def _close_csv(self):
        for fp in self._csv.values():
            try:
                fp.flush()
                fp.close()
            except OSError:
                pass
        self._csv = {}

    def _loop(self):
        while not self._stop.is_set():
            try:
                idx, fid, cam_ts, recv, arr = self.q.get(timeout=0.2)
            except Empty:
                continue
            with self._dir_lock:
                base = self._session_dir
            if base is None:
                self.late += 1
                continue
            if idx >= len(self.slots):
                continue

            d = os.path.join(base, self.dirname(idx))
            # 파일명 = FrameID. 도착 순서로 이름을 붙이면 유실 시 전부 밀린다.
            if self.fmt == "png":
                path = os.path.join(d, f"{fid:08d}.png")
                self._cv2.imwrite(path, arr,
                                  [self._cv2.IMWRITE_PNG_COMPRESSION, 1])
            else:
                np.save(os.path.join(d, f"{fid:08d}.npy"), arr)

            with self._csv_lock:
                fp = self._csv.get(idx)
                if fp is not None:
                    fp.write(f"{fid},{cam_ts},{recv:.6f}\n")
            self.written[idx] += 1

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.close_session()


# ===========================================================================
#  라이다 사전 점검
# ===========================================================================
def lidar_preflight(timeout=3.0):
    """데이터가 실제로 흐르는지, PPS 가 잡혀 있는지 본다.

    PPS 가 Absent 면 라이다의 타임스탬프는 자기 내부 시계다. 카메라와
    묶으려면 결국 호스트 시각으로 맞춰야 하고 정밀도가 떨어진다.
    수집을 막지는 않되 반드시 알려 준다.
    """
    info = {"data": 0, "position": 0, "pps_state": None, "nmea": ""}
    socks = {}
    try:
        for port in (DATA_PORT, POSITION_PORT):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", port))
            s.settimeout(timeout)
            socks[port] = s

        try:
            d = socks[DATA_PORT].recv(2048)
            info["data"] = len(d)
        except socket.timeout:
            pass
        try:
            p = socks[POSITION_PORT].recv(1024)
            info["position"] = len(p)
            if len(p) >= 512:
                info["pps_state"] = p[202]
                info["nmea"] = p[206:306].split(b"\x00")[0] \
                    .decode("ascii", "replace").strip()
        except socket.timeout:
            pass
    except OSError as e:
        info["error"] = str(e)
    finally:
        for s in socks.values():
            s.close()
    return info


def print_lidar_preflight(info):
    from vlp16 import PPS_STATES
    print("라이다 점검")
    if info.get("error"):
        print(f"  [실패] 소켓을 열 수 없습니다: {info['error']}")
        return False
    if not info["data"]:
        print(f"  [실패] UDP {DATA_PORT} 에 데이터가 오지 않습니다.")
        print(f"         - 라이다 전원과 이더넷 연결 확인")
        print(f"         - ping {LIDAR_IP}")
        print(f"         - 다른 프로그램이 포트를 잡고 있는지: "
              f"ss -ulpn 'sport = :{DATA_PORT}'")
        return False
    print(f"  [OK]   데이터 패킷 {info['data']} B 수신")

    state = info["pps_state"]
    if state is None:
        print(f"  [주의] 위치 패킷(UDP {POSITION_PORT})이 오지 않습니다.")
    elif state == 2:
        print(f"  [OK]   PPS Locked — MCU 의 1PPS/GPRMC 가 물려 있습니다.")
    else:
        print(f"  [주의] PPS {PPS_STATES.get(state, state)} — "
              f"라이다가 외부 시각을 못 받고 있습니다.")
        print( "         타임스탬프가 라이다 내부 시계라 카메라와의 결합은")
        print( "         호스트 수신 시각 기준이 됩니다 (수 ms 수준 오차).")
        print( "         ESP32 의 GPIO5(1PPS) / GPIO17(GPRMC) 배선을 확인하세요.")
    if info["nmea"]:
        print(f"  NMEA   {info['nmea']}")
    return True


# ===========================================================================
#  세션 하나
# ===========================================================================
def run_session(session_dir, duration, lidar, mcu=None, cams=None,
                writer=None, fmt="npy", preview=None, preview_interval=0.5,
                drain=None):
    os.makedirs(session_dir, exist_ok=True)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": duration,
        "format": fmt,
        "trigger_hz": 10,
        "lidar": {"ip": LIDAR_IP, "model": "VLP-16",
                  "data_port": DATA_PORT, "position_port": POSITION_PORT,
                  **lidar_snapshot()},
    }
    if cams is not None:
        meta.update(cams.meta())
        meta["camera_slots"] = writer.slots
    meta_path = os.path.join(session_dir, "meta.json")
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    if mcu is not None:
        mcu.set_logfile(os.path.join(session_dir, "mcu_log.txt"))
    if drain is not None:
        drain.set_logfile(os.path.join(session_dir, "mcu_log.txt"))
    if writer is not None:
        writer.open_session(session_dir)

    time.sleep(0.2)
    pcap_path = os.path.join(session_dir, "lidar.pcap")
    lidar.start_session(pcap_path)
    # 화면에는 이번 세션 몫만 보여 준다. stats 는 프로그램 시작부터의 누적이라
    # 그대로 찍으면 두 번째 세션이 0 이 아닌 값에서 시작한 것처럼 보인다.
    lidar_base = lidar.stats["written"]

    t_start = time.time()
    if mcu is not None:
        print(f"  GO  → {duration}초 촬영")
        mcu.send("GO")
    else:
        print(f"  라이다만 {duration}초 수집")

    next_draw = 0.0
    while True:
        el = time.time() - t_start
        if el >= duration:
            break
        if el >= next_draw:
            next_draw = el + preview_interval
            if preview is not None:
                preview.render(el, writer.written, cams.queue.qsize())
            saved = writer.written if writer is not None else "-"
            print(f"\r    {el:5.1f}s  카메라 {saved}  "
                  f"라이다 {lidar.stats['written'] - lidar_base}  "
                  f"큐 {lidar.queue.qsize()}   ", end="", flush=True)
        time.sleep(0.02 if preview is not None else 0.2)
    print()

    if mcu is not None:
        mcu.send("STOP")
        print("  STOP → 큐 비우는 중...")

    # 카메라 큐가 비기를 기다린다. 라이다는 stop_session 이 알아서 비운다.
    if cams is not None:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if cams.queue.empty():
                time.sleep(0.7)
                if cams.queue.empty():
                    break
            time.sleep(0.1)
    time.sleep(0.3)

    lidar_summary = lidar.stop_session()
    if writer is not None:
        writer.close_session()
    if mcu is not None:
        mcu.set_logfile(None)
    if drain is not None:
        drain.set_logfile(None)

    # --- 요약을 meta.json 에 덧붙인다 ---
    meta["stopped"] = datetime.now().isoformat(timespec="seconds")
    meta["lidar"].update(lidar_summary or {})
    if writer is not None:
        meta["frames_written"] = {writer.dirname(i): n
                                  for i, n in enumerate(writer.written)}
        if writer.late:
            meta["frames_late"] = writer.late
    if mcu is not None:
        for line in reversed(mcu.last_lines):
            if line.startswith("CNT,"):
                meta["mcu_cnt"] = line
                break
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # --- 화면 요약 ---
    if writer is not None:
        print(f"  카메라 저장: {writer.written}")
    if lidar_summary:
        got = lidar_summary["data_packets"]
        exp = lidar_summary["expected_data_packets"]
        rate = f"{100.0 * got / exp:.2f}%" if exp else "-"
        print(f"  라이다 저장: {lidar_summary['packets_written']} 패킷 "
              f"({lidar_summary['bytes_written'] / 1e6:.1f} MB), "
              f"데이터 {got}/{exp} = {rate}")
        if lidar_summary["dropped_queue"]:
            print(f"  [주의] 라이다 큐 넘침 {lidar_summary['dropped_queue']} "
                  f"— 디스크가 못 따라오고 있습니다.")
        if exp and got < exp * 0.98:
            print(f"  [주의] 라이다 패킷 유실이 2% 를 넘습니다.")
    if mcu is not None:
        for line in mcu.last_lines[-30:]:
            if line.startswith("CNT,"):
                print(f"  MCU {line}")


# ===========================================================================
#  main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description="카메라 + 라이다 동시 수집",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--sessions", type=int, default=1, help="세션 반복 횟수")
    ap.add_argument("--duration", type=float, default=20.0, help="세션 길이(초)")
    ap.add_argument("--gap", type=float, default=3.0, help="세션 간 대기(초)")
    ap.add_argument("--out", default=os.path.join(_HERE, "sessions"),
                    help="저장 루트 폴더")
    ap.add_argument("--daheng-ws", default=DEFAULT_DAHENG_WS,
                    help="sync_cam.py / collect.py 가 있는 폴더")
    ap.add_argument("--port", default=None, help="MCU 시리얼 포트")
    ap.add_argument("--exposure", type=float, default=None)
    ap.add_argument("--gain", type=float, default=None)
    ap.add_argument("--format", choices=["npy", "png"], default="npy")
    ap.add_argument("--lidar-only", action="store_true",
                    help="MCU/카메라 없이 라이다만 수집")
    ap.add_argument("--lidar-ip", default=LIDAR_IP,
                    help="라이다 IP. 'any' 면 출처를 가리지 않는다")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--preview-fps", type=float, default=2.0)
    ap.add_argument("--preview-width", type=int, default=480)
    args = ap.parse_args()

    out_root = os.path.abspath(args.out)
    os.makedirs(out_root, exist_ok=True)

    # ---------------------------------------------------------- 라이다 점검
    info = lidar_preflight()
    if not print_lidar_preflight(info):
        return 1
    print()

    src = None if args.lidar_ip == "any" else args.lidar_ip
    lidar = VelodyneCapture(expect_src=src)
    lidar.open()
    if lidar.src_mac != b"\x00" * 6:
        print("라이다 MAC:", ":".join(f"{b:02x}" for b in lidar.src_mac), "\n")

    mcu = cams_ctx = writer = preview = None
    try:
        if args.lidar_only:
            # 카메라도 GO 명령도 없지만 ESP32 시리얼은 반드시 읽어 준다.
            # 읽지 않으면 ESP32 가 PPS 를 내리지 못해 라이다가 동기를 잃는다
            # (SerialDrain 설명 참고).
            drain = None
            port = args.port or find_mcu_port()
            if port:
                try:
                    drain = SerialDrain(port)
                    print(f"ESP32 {port} 로그 수신 중 (명령은 보내지 않음)")
                except Exception as e:
                    print(f"[주의] ESP32 포트 {port} 를 열 수 없습니다: {e}")
            if drain is None:
                print("[주의] ESP32 시리얼을 읽지 못합니다. 이대로면 PPS 가 멈춰")
                print("       라이다가 동기를 잃을 수 있습니다.")
            if info.get("pps_state") is not None:
                time.sleep(0.5)
                wait_pps_lock(lidar)
                print()
            try:
                _run_sessions(args, out_root, lidar, None, None, None, None,
                              drain=drain)
            finally:
                if drain is not None:
                    drain.close()
            return 0

        sync_cam, collect = load_daheng(args.daheng_ws)
        exposure = (args.exposure if args.exposure is not None
                    else sync_cam.DEFAULT_EXPOSURE_US)
        gain = args.gain if args.gain is not None else sync_cam.DEFAULT_GAIN_DB

        # ------------------------------------------------------ 연결된 카메라
        import gxipy as gx
        dm = gx.DeviceManager()
        _, dev_list = dm.update_device_list()
        found = {d.get("sn") for d in dev_list}
        del dm
        slots, sns = [], []
        for i, sn in enumerate(sync_cam.CAMERA_SNS):
            if sn in found:
                slots.append(i + 1)
                sns.append(sn)
        if not sns:
            print(f"[실패] CAMERA_SNS 중 연결된 카메라가 없습니다.")
            print(f"       연결된 sn: {sorted(found)}")
            print(f"       {args.daheng_ws}/list_cams.py 로 확인하세요.")
            return 1
        missing = [sn for sn in sync_cam.CAMERA_SNS if sn not in found]
        if missing:
            print(f"[주의] 연결 안 된 카메라 {len(missing)} 대를 건너뜁니다: "
                  f"{missing}")
        print(f"사용할 카메라: " +
              ", ".join(f"CAM{s}({sn})" for s, sn in zip(slots, sns)) + "\n")

        # ------------------------------------------------------------- MCU
        port = args.port or find_mcu_port()
        if port is None:
            print("[실패] ESP32 를 찾지 못했습니다.")
            print("  USB 열거 자체가 안 되고 있습니다:")
            print("    lsusb -d 303a:            (Espressif 장치가 보여야 함)")
            print("    ls /dev/ttyACM*")
            print("  - C타입 케이블이 데이터 지원 케이블인지 (충전 전용 아님)")
            print("  - 보드를 뽑았다 다시 꽂아 보세요")
            print("  - 라이다만 받으려면:  python3 record.py --lidar-only")
            print("  - 포트를 직접 알고 있다면:  --port /dev/ttyACM0")
            return 1
        print(f"MCU 포트: {port}")
        import serial
        try:
            mcu = collect.McuLink(port)
        except serial.SerialException as e:
            print(f"[실패] 포트를 열 수 없습니다: {e}")
            print("  1) sudo usermod -aG dialout $USER  (재로그인 필요)")
            print("  2) sudo systemctl stop ModemManager")
            print("  3) 다른 시리얼 모니터가 포트를 잡고 있는지 확인")
            return 1
        mcu.start_reader()
        print("MCU READY 대기 중...")
        ready = mcu.wait_ready(10.0)
        if not ready:
            # 포트를 여는 순간 호스트 tty 가 ESP32 출력을 되돌려 보내면(echo)
            # ESP32 명령 버퍼에 자기 로그 조각이 섞인다. 그 뒤에 붙은 PING 은
            # '알 수 없는 명령' 이 된다 (ESP32 통계에 '# ===== 방식' 이 명령으로
            # 찍힌 것이 그 흔적). 개행으로 버퍼를 비우고 한 번 더 부른다.
            mcu.send("")
            mcu.send("PING")
            ready = mcu._ready.wait(5.0)
        if not ready:
            print("[실패] READY 를 받지 못했습니다.")
            print("  - 펌웨어가 READY 를 출력하는 버전인지")
            print("  - Arduino IDE 의 USB CDC On Boot 설정")
            return 1
        print("MCU READY\n")

        # 포트를 여느라 ESP32 가 리셋되었다. PPS 가 돌아올 때까지 기다린다.
        if info.get("pps_state") is not None:
            wait_pps_lock(lidar)
            print()

        # --------------------------------------------------------- 카메라
        with sync_cam.CameraArray(sns=sns, exposure_us=exposure,
                                  gain_db=gain) as cams:
            writer = SessionWriter(cams.queue, slots, fmt=args.format)
            writer.start()
            if not args.no_preview:
                try:
                    preview = collect.Preview(cams, tile_w=args.preview_width)
                    print(f"프리뷰 켜짐. 창에서 Q 를 누르면 프리뷰만 꺼집니다.")
                except Exception as e:
                    print(f"[주의] 프리뷰를 열 수 없습니다: {e}")

            _run_sessions(args, out_root, lidar, mcu, cams, writer, preview)

            if preview is not None:
                preview.close()
            cams.print_stats()
            writer.stop()
        return 0

    finally:
        if mcu is not None:
            try:
                mcu.send("STOP")
            except Exception:
                pass
            mcu.close()
        lidar.close()
        print(f"\n저장 위치: {out_root}")
        print(f"검산:  python3 check_session.py {out_root}/session_XXX")


def _run_sessions(args, out_root, lidar, mcu, cams, writer, preview, drain=None):
    existing = [d for d in os.listdir(out_root) if d.startswith("session_")]
    start_n = 1 + max([int(d.split("_")[1]) for d in existing
                       if d.split("_")[1].isdigit()] or [0])
    pv_int = 1.0 / max(args.preview_fps, 0.2)

    try:
        for k in range(args.sessions):
            n = start_n + k
            sd = os.path.join(out_root, f"session_{n:03d}")
            print(f"\n===== session_{n:03d} ({k + 1}/{args.sessions}) =====")
            run_session(sd, args.duration, lidar, mcu, cams, writer,
                        args.format, preview, pv_int, drain=drain)
            if k < args.sessions - 1:
                print(f"  {args.gap}초 대기")
                time.sleep(args.gap)
    except KeyboardInterrupt:
        print("\n[중단] 정리 중...")
        if mcu is not None:
            mcu.send("STOP")
        lidar.stop_session()
        if writer is not None:
            writer.close_session()
        time.sleep(0.5)


if __name__ == "__main__":
    raise SystemExit(main())
