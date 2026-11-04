"""데이터 수집 — MCU 핸드셰이크 + 카메라 4대 동시 저장.

    python collect.py                       세션 1회 (20초)
    python collect.py --sessions 10         20초 세션 10회
    python collect.py --duration 30         길이 변경
    python collect.py --port COM5           포트 직접 지정
    python collect.py --exposure 5000 --gain 10
    python collect.py --format png          npy 대신 png 로 저장 (느림)

--------------------------------------------------------------------------
동작 순서

  1. MCU 시리얼 연결 → READY 대기
  2. 로그 수신 스레드 시작        ← GO 보다 먼저. EXP 줄을 놓치지 않으려고.
  3. 카메라 4대 열기 + 트리거 대기 상태
  4. "GO"   → MCU 가 10 Hz FSYNC 송출 시작 → 사진이 찍히기 시작
  5. N초 대기
  6. "STOP" → FSYNC 정지, MCU 가 CNT 요약 출력
  7. 큐 비우기, 다음 세션

  카메라는 세션마다 다시 열지 않는다. stream_on 을 유지해야 FrameID 가
  끊기지 않는다. GO/STOP 만 반복한다.

--------------------------------------------------------------------------
저장 구조

  sessions/session_001/
      meta.json          세션 설정, 카메라 sn, 노출값 등
      mcu_log.txt        MCU 시리얼 원문 통째로 (T / EXP / CNT / PPS / NMEA)
      cam1/
          frames.csv     frame_id, cam_timestamp_ns, host_recv_unix
          00000001.npy   파일명 = FrameID (도착 순서가 아님)
          00000002.npy
      cam2/ cam3/ cam4/
      lidar.pcap         ← 라이다 캡처는 별도로 여기에 넣을 것

  결합은 나중에 별도 스크립트가 이 폴더만 읽어서 수행한다.
  수집 중에는 파싱도 계산도 하지 않는다 — 예외가 나면 세션이 통째로
  날아가기 때문이다.
"""

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime
from queue import Empty

import numpy as np
import serial
import serial.tools.list_ports

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from sync_cam import CameraArray, DEFAULT_EXPOSURE_US, DEFAULT_GAIN_DB  # noqa: E402

SERIAL_BAUD = 921600
N_WRITERS = 4              # 저장 스레드 수


# ===========================================================================
#  MCU 시리얼
# ===========================================================================
def find_port(hint=None):
    if hint:
        return hint
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError(
            "시리얼 포트를 찾지 못했습니다.\n"
            "  리눅스: ls /dev/ttyACM* /dev/ttyUSB* 로 확인\n"
            "  ESP32-S3 네이티브 USB(C타입)는 보통 /dev/ttyACM0 입니다.")

    # ESP32-S3 (VID 0x303A = Espressif) 를 최우선으로 본다.
    for p in ports:
        if (p.vid or 0) == 0x303A:
            return p.device

    for p in ports:
        blob = f"{p.description} {p.manufacturer} {p.product}".lower()
        if any(k in blob for k in ("ch340", "cp210", "usb serial", "espressif",
                                   "usb-serial", "jtag", "usb single serial")):
            return p.device

    # 리눅스에서는 ttyACM(네이티브 USB CDC) 을 ttyUSB(브리지) 보다 우선.
    for pref in ("ttyACM", "ttyUSB"):
        for p in ports:
            if pref in p.device:
                return p.device

    print(f"자동 판별 실패. 첫 번째 포트를 씁니다: {ports[0].device}")
    print("  후보:", ", ".join(f"{p.device}({p.description})" for p in ports))
    return ports[0].device


class McuLink:
    """MCU 와의 시리얼 연결. 수신은 전용 스레드가 파일로 흘려보낸다."""

    def __init__(self, port, baud=SERIAL_BAUD):
        # 포트를 여는 순간 DTR 로 ESP32 가 리셋된다. READY 를 기다려야 한다.
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._fp = None
        self._fp_lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self.last_lines = []

    def start_reader(self):
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").rstrip("\r")
                if "READY" in text:
                    self._ready.set()
                self.last_lines.append(text)
                if len(self.last_lines) > 200:
                    self.last_lines.pop(0)
                with self._fp_lock:
                    if self._fp is not None:
                        self._fp.write(text + "\n")

    def wait_ready(self, timeout=10.0):
        """부팅 후 READY 를 기다린다. 안 오면 PING 을 한 번 던져 본다."""
        if self._ready.wait(timeout / 2):
            return True
        self.send("PING")
        return self._ready.wait(timeout / 2)

    def set_logfile(self, path_or_none):
        with self._fp_lock:
            if self._fp is not None:
                self._fp.flush()
                self._fp.close()
                self._fp = None
            if path_or_none:
                self._fp = open(path_or_none, "w", encoding="utf-8")

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        self.ser.flush()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.set_logfile(None)
        try:
            self.ser.close()
        except Exception:
            pass


def send_trigger_delays(mcu, cams):
    """카메라별 TriggerDelay 를 MCU 에 알려준다.

    MCU 는 Line1 엣지를 이 위상 근처(±1.5 ms)에서만 노출로 센다. FSYNC 가
    나가는 순간 Line1 에 유도되는 가짜 엣지를 버리기 위해서다. 알려주지
    않으면 MCU 는 전부 0 으로 보고, 늦게 찍는 카메라의 노출을 세지 못한다.
    """
    d = [0] * 4
    for c in cams.cams:
        if 1 <= c.slot <= 4:
            d[c.slot - 1] = int(round(getattr(c, "trigger_delay_us", 0) or 0))
    want = "DLY," + ",".join(str(x) for x in d)
    mcu.send(want)
    print("MCU 에 카메라 지연 통보: " +
          " · ".join(f"CAM{i + 1} {v / 1000:.0f}ms" for i, v in enumerate(d)))

    # MCU 가 받은 값을 그대로 되돌려 준다. 명령이 잘리거나 잘못 파싱되면
    # 뒤쪽 카메라의 기대 위상이 0 이 되어 잡음을 세게 되므로 반드시 확인한다.
    for _ in range(20):
        time.sleep(0.05)
        echo = next((l for l in reversed(mcu.last_lines)
                     if l.startswith("DLY,")), None)
        if echo is None:
            continue
        if echo.strip() == want:
            return True
        print(f"  [경고] MCU 가 받은 값이 다릅니다: {echo.strip()}")
        print(f"         보낸 값: {want}")
        print(f"         Line1 잡음이 걸러지지 않아 CNT 를 믿을 수 없습니다.")
        return False
    print("  [주의] MCU 응답을 받지 못했습니다. 펌웨어가 DLY 를 지원하는지 "
          "확인하세요 (부팅 메시지의 명령 목록).")
    return False


# ===========================================================================
#  저장 스레드
# ===========================================================================
class Writer:
    """큐에서 프레임을 꺼내 현재 세션 폴더에 쓴다.

    콜백에서 직접 디스크에 쓰지 않는 이유: 콜백이 밀리면 SDK 수신 스레드가
    막혀 프레임을 놓친다. 그 누락은 FrameID 로 감지되지 않는 유형이다.
    """

    def __init__(self, queue, fmt="npy", n_threads=N_WRITERS, n_cam=4,
                 slots=None):
        self.q = queue
        # 큐 인덱스 → 물리 슬롯 번호. 폴더 이름은 슬롯으로 짓는다.
        # 예전에는 도착 순서(cam1, cam2 ...)로 지어서, 3번 카메라만 꽂으면
        # cam1/ 에 들어가 어느 GPIO 가 쏜 것인지 알 수 없게 되었다.
        self.slots = list(slots) if slots else list(range(1, n_cam + 1))
        self.n_cam = len(self.slots)
        self.fmt = fmt
        self._stop = threading.Event()
        self._threads = []
        self._n = n_threads

        self._dir_lock = threading.Lock()
        self._session_dir = None
        self._csv = {}          # cam_idx -> file object
        self._csv_lock = threading.Lock()
        self.written = [0] * self.n_cam

        if fmt == "png":
            import cv2
            self._cv2 = cv2

    def start(self):
        for _ in range(self._n):
            t = threading.Thread(target=self._loop, daemon=True)
            t.start()
            self._threads.append(t)

    def open_session(self, session_dir, n_cam=None):
        n_cam = n_cam or self.n_cam
        with self._dir_lock:
            self._session_dir = session_dir
        with self._csv_lock:
            self._close_csv()
            for i in range(n_cam):
                d = os.path.join(session_dir, f"cam{self.slots[i]}")
                os.makedirs(d, exist_ok=True)
                fp = open(os.path.join(d, "frames.csv"), "w",
                          encoding="utf-8", buffering=1)
                fp.write("frame_id,cam_timestamp,host_recv_unix\n")
                self._csv[i] = fp
        self.written = [0] * n_cam

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
            except Exception:
                pass
        self._csv = {}

    def _loop(self):
        while not self._stop.is_set():
            try:
                cam_idx, fid, cam_ts, recv, arr = self.q.get(timeout=0.2)
            except Empty:
                continue

            with self._dir_lock:
                base = self._session_dir
            if base is None:
                continue     # 세션 밖에서 도착한 프레임은 버린다

            if cam_idx >= len(self.slots):
                continue
            d = os.path.join(base, f"cam{self.slots[cam_idx]}")
            # 파일명 = FrameID. 도착 순서로 이름을 붙이면 유실 시 밀린다.
            if self.fmt == "png":
                path = os.path.join(d, f"{fid:08d}.png")
                self._cv2.imwrite(path, arr,
                                  [self._cv2.IMWRITE_PNG_COMPRESSION, 1])
            else:
                path = os.path.join(d, f"{fid:08d}.npy")
                np.save(path, arr)

            with self._csv_lock:
                fp = self._csv.get(cam_idx)
                if fp is not None:
                    fp.write(f"{fid},{cam_ts},{recv:.6f}\n")
            self.written[cam_idx] += 1

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.close_session()


# ===========================================================================
#  프리뷰 — 2x2 격자
# ===========================================================================
#  저장은 전부 하고, 화면에는 초당 몇 장만 띄운다.
#  전 프레임을 화면에 그리면 4대 x 10 Hz = 초당 40번 디모자이크+축소가
#  필요해 CPU 가 계속 밀리고, 그러면 콜백이 밀려 프레임을 놓친다.
#  프리뷰의 목적은 "4대가 살아있나 / 화면이 까맣지 않나" 확인이므로
#  초당 2장이면 충분하다.
# ===========================================================================
# ※ OpenCV 의 Bayer 이름은 카메라(GenICam) 이름과 R/B 가 뒤바뀌어 있다.
#   카메라 BayerGB(0,0 이 G, 옆이 B) 에는 cv2.COLOR_BayerGR2BGR 가 맞다.
#   이름 그대로 BayerGB2BGR 를 쓰면 빨강이 파랑으로 나온다 (2026-09-22 실험 확인).
_BAYER_MAP = {
    "BayerRG": "COLOR_BayerBG2BGR", "BayerGB": "COLOR_BayerGR2BGR",
    "BayerGR": "COLOR_BayerGB2BGR", "BayerBG": "COLOR_BayerRG2BGR",
}


class Preview:
    WIN = "collect - 4 cams (Q: 프리뷰 끄기)"

    def __init__(self, cams, tile_w=480):
        import cv2
        self.cv2 = cv2
        self.cams = cams
        self.n = len(cams.cams)
        self.cols = 1 if self.n == 1 else 2
        self.rows = (self.n + self.cols - 1) // self.cols
        self.tile_w = tile_w
        self.enabled = True
        self._code = None

        pf = (cams.cams[0].pixel_format or "") if cams.cams else ""
        for key, name in _BAYER_MAP.items():
            if key in pf:
                self._code = getattr(cv2, name)
                break
        cv2.namedWindow(self.WIN, cv2.WINDOW_AUTOSIZE)

    def _slot(self, idx):
        """큐 인덱스 → 물리 슬롯. CameraArray 에 slots 가 없으면 idx+1."""
        slots = getattr(self.cams, "slots", None)
        return slots[idx] if slots and idx < len(slots) else idx + 1

    def _tile(self, idx, elapsed):
        cv2 = self.cv2
        item = self.cams.latest.get(idx)
        h = int(self.tile_w * 1200 / 2048)

        if item is None:
            img = np.zeros((h, self.tile_w, 3), np.uint8)
            label, color = f"CAM{self._slot(idx)}  (프레임 없음)", (60, 60, 220)
        else:
            fid, arr = item
            small = cv2.resize(arr, (self.tile_w, h),
                               interpolation=cv2.INTER_NEAREST)
            if small.ndim == 2:
                img = (cv2.cvtColor(small, self._code) if self._code
                       else cv2.cvtColor(small, cv2.COLOR_GRAY2BGR))
            else:
                img = small
            label, color = f"CAM{self._slot(idx)}  id={fid}", (0, 220, 120)

        cv2.rectangle(img, (0, 0), (self.tile_w - 1, h - 1), color, 2)
        cv2.rectangle(img, (0, 0), (self.tile_w, 28), (0, 0, 0), -1)
        cv2.putText(img, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2)
        return img

    def render(self, elapsed, written, qsize):
        if not self.enabled:
            return
        cv2 = self.cv2
        tiles = [self._tile(i, elapsed) for i in range(self.n)]
        blank = np.zeros_like(tiles[0])
        while len(tiles) < self.rows * self.cols:
            tiles.append(blank)
        grid = np.vstack([np.hstack(tiles[r * self.cols:(r + 1) * self.cols])
                          for r in range(self.rows)])

        bar = np.zeros((34, grid.shape[1], 3), np.uint8)
        cv2.putText(bar, f"{elapsed:5.1f}s   saved {written}   queue {qsize}",
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (230, 230, 230), 1)
        cv2.imshow(self.WIN, np.vstack([grid, bar]))

        if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q")):
            self.close()
            print("\n  [프리뷰 끔] 촬영은 계속됩니다.")

    def close(self):
        if self.enabled:
            self.enabled = False
            try:
                self.cv2.destroyWindow(self.WIN)
                self.cv2.waitKey(1)
            except Exception:
                pass


# ===========================================================================
#  세션 하나
# ===========================================================================
def run_session(mcu, cams, writer, session_dir, duration, fmt, preview=None,
                preview_interval=0.5):
    os.makedirs(session_dir, exist_ok=True)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": duration,
        "format": fmt,
        "trigger_hz": 10,
        **cams.meta(),
    }
    with open(os.path.join(session_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    mcu.set_logfile(os.path.join(session_dir, "mcu_log.txt"))
    writer.open_session(session_dir)

    time.sleep(0.2)                      # 로그 파일 준비
    print(f"  GO  → {duration}초 촬영")
    mcu.send("GO")

    t0 = time.time()
    next_draw = 0.0
    while True:
        el = time.time() - t0
        if el >= duration:
            break
        # 프리뷰는 메인 스레드에서 그린다. cv2 창은 스레드를 옮기면
        # 플랫폼에 따라 불안정하다.
        if el >= next_draw:
            next_draw = el + preview_interval
            if preview is not None:
                preview.render(el, writer.written, cams.queue.qsize())
            print(f"\r    {el:5.1f}s  저장 {writer.written}  "
                  f"큐 {cams.queue.qsize()}   ", end="", flush=True)
        time.sleep(0.02 if preview is not None else 0.2)
    print()

    mcu.send("STOP")
    print("  STOP → 큐 비우는 중...")

    # 마지막 프레임과 MCU 의 CNT 줄이 도착할 시간을 준다.
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if cams.queue.empty():
            time.sleep(0.7)
            if cams.queue.empty():
                break
        time.sleep(0.1)

    time.sleep(0.3)
    writer.close_session()
    mcu.set_logfile(None)

    print(f"  저장 완료: {writer.written}")
    for line in mcu.last_lines[-30:]:
        if line.startswith("CNT,"):
            print(f"  MCU {line}")


# ===========================================================================
#  main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sessions", type=int, default=1, help="세션 반복 횟수")
    ap.add_argument("--duration", type=float, default=20.0, help="세션 길이(초)")
    ap.add_argument("--gap", type=float, default=3.0, help="세션 간 대기(초)")
    ap.add_argument("--out", default="sessions", help="저장 루트 폴더")
    ap.add_argument("--port", default=None, help="MCU 시리얼 포트")
    ap.add_argument("--exposure", type=float, default=DEFAULT_EXPOSURE_US)
    ap.add_argument("--gain", type=float, default=DEFAULT_GAIN_DB)
    ap.add_argument("--format", choices=["npy", "png"], default="npy")
    ap.add_argument("--cams", type=int, default=0,
                    help="사용할 카메라 대수 (0 = CAMERA_SNS 전부). 예: --cams 1")
    ap.add_argument("--allow-partial", action="store_true",
                    help="연결 안 된 카메라는 건너뛰고 있는 것만 사용")
    ap.add_argument("--no-preview", action="store_true",
                    help="프리뷰 창 없이 실행 (CPU 여유 확보)")
    ap.add_argument("--preview-fps", type=float, default=2.0,
                    help="프리뷰 갱신 속도. 높이면 프레임 누락 위험이 는다")
    ap.add_argument("--preview-width", type=int, default=480,
                    help="타일 하나의 가로 픽셀")
    args = ap.parse_args()

    out_root = os.path.abspath(args.out)
    os.makedirs(out_root, exist_ok=True)

    port = find_port(args.port)
    print(f"MCU 포트: {port}")
    try:
        mcu = McuLink(port)
    except serial.SerialException as e:
        print(f"[실패] 포트를 열 수 없습니다: {e}")
        if sys.platform.startswith("linux"):
            print("\n우분투에서 흔한 원인:")
            print("  1) 권한 — dialout 그룹에 없음")
            print(f"       sudo usermod -aG dialout $USER")
            print( "       (로그아웃 후 다시 로그인해야 적용됩니다)")
            print( "  2) ModemManager 가 포트를 잡음")
            print( "       sudo systemctl stop ModemManager")
            print( "  3) 다른 프로그램(시리얼 모니터 등)이 포트 사용 중")
        return 1
    mcu.start_reader()

    print("MCU READY 대기 중...")
    if not mcu.wait_ready(10.0):
        print("[실패] READY 를 받지 못했습니다.")
        print("  - 펌웨어가 최신인지 (READY 를 출력하는 버전인지)")
        print("  - Arduino IDE 의 USB CDC On Boot 설정")
        print("  - 시리얼 모니터가 포트를 잡고 있지 않은지")
        mcu.close()
        return 1
    print("MCU READY\n")

    try:
        with CameraArray(exposure_us=args.exposure, gain_db=args.gain,
                         n_cams=args.cams,
                         allow_partial=args.allow_partial) as cams:
            send_trigger_delays(mcu, cams)
            writer = Writer(cams.queue, fmt=args.format, n_cam=len(cams.cams),
                            slots=cams.slots)
            writer.start()

            preview = None
            if not args.no_preview:
                try:
                    preview = Preview(cams, tile_w=args.preview_width)
                    print(f"프리뷰 켜짐 ({args.preview_fps:.0f} fps). "
                          f"창에서 Q 를 누르면 프리뷰만 꺼집니다.")
                except Exception as e:
                    print(f"[주의] 프리뷰를 열 수 없습니다: {e}")
            pv_int = 1.0 / max(args.preview_fps, 0.2)

            # 기존 세션 번호 다음부터
            existing = [d for d in os.listdir(out_root)
                        if d.startswith("session_")]
            start_n = 1 + max([int(d.split("_")[1]) for d in existing
                               if d.split("_")[1].isdigit()] or [0])

            try:
                for k in range(args.sessions):
                    n = start_n + k
                    sd = os.path.join(out_root, f"session_{n:03d}")
                    print(f"\n===== session_{n:03d} "
                          f"({k + 1}/{args.sessions}) =====")
                    run_session(mcu, cams, writer, sd, args.duration,
                                args.format, preview, pv_int)
                    if k < args.sessions - 1:
                        print(f"  {args.gap}초 대기")
                        time.sleep(args.gap)
            except KeyboardInterrupt:
                print("\n[중단] STOP 전송")
                mcu.send("STOP")
                time.sleep(0.5)

            if preview is not None:
                preview.close()
            cams.print_stats()
            writer.stop()

    finally:
        try:
            mcu.send("STOP")
        except Exception:
            pass
        mcu.close()

    print(f"\n저장 위치: {out_root}")
    print("세션마다 check_session.py 로 검산하세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
