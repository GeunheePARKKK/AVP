"""카메라 4대 + 라이다 동시 수집 — MCU 없이 프리런(free-run) 으로.

    python3 record_freerun.py                       20초 x 3 세션, 세션마다 rosbag2
    python3 record_freerun.py --sessions 1 --duration 5
    python3 record_freerun.py --no-rosbag           변환 없이 원본만
    python3 record_freerun.py --exposure 30000      어두우면 노출을 올린다

--------------------------------------------------------------------------
record.py 와 무엇이 다른가

  record.py 는 ESP32 가 만드는 FSYNC 를 카메라 Line0 으로 받아 4대를 한
  순간에 쏜다. 그 신호가 없으면 sync_cam.TriggerCamera 는 TriggerMode=On
  으로 대기만 하고 한 장도 찍지 않는다.

  이 스크립트는 MCU 가 없을 때 쓴다. 트리거를 끄고 카메라가 자기 클럭으로
  돌게 한 뒤, 지정한 프레임레이트로 제한해서 받는다.

      record.py                    record_freerun.py
    ─────────────────────────────────────────────────────────────
      TriggerMode = On             TriggerMode = Off
      Line0 (ESP32 FSYNC)          AcquisitionFrameRate 로 자체 구동
      4대가 같은 순간에 셔터        4대가 각자 자기 클럭으로 셔터
      MCU 로그로 사후 보정 가능      호스트 수신 시각이 전부

  ※ 이 데이터로는 카메라끼리도, 카메라와 라이다 사이도 하드웨어 동기가
    없다. 프레임 짝짓기는 frames.csv 의 host_recv_unix 로 사후에 해야 하고
    USB3 전송 지연(이 리그 실측 노출시간 + 7.1 ms)과 스케줄링 지터가
    그대로 남는다. 캘리브레이션이나 센서퓨전이 목적이면 MCU 를 물리고
    record.py 로 다시 찍을 것.

--------------------------------------------------------------------------
daheng_ws_1 은 고치지 않는다

  sync_cam.TriggerCamera 는 하드웨어 트리거 전용이라 그대로는 못 쓴다.
  대신 같은 파일의 검증된 조각들을 가져다 쓴다.

    _must_enum / _must_float / _try_enum / _read_*   feature 헬퍼
    _reader_loop                                     폴링 수신 루프
    CAMERA_SNS                                       SN ↔ 물리 슬롯 표

  라이다와 저장 스레드는 이 워크스페이스의 것을 그대로 쓴다.

    lidar_capture.VelodyneCapture    UDP → pcap
    record.SessionWriter             물리 슬롯 번호로 폴더를 짓는 저장 스레드
    to_rosbag.convert                세션 폴더 → rosbag2

--------------------------------------------------------------------------
저장 구조

  sessions/session_008/
      meta.json          설정 + 카메라/라이다 요약
      lidar.pcap         UDP 2368 + 8308 원본
      cam1/ .. cam4/
          frames.csv     frame_id, cam_timestamp, host_recv_unix
          00000000.npy   파일명 = FrameID (도착 순서가 아님)
      rosbag/            세션 하나당 rosbag2 한 덩어리
          metadata.yaml
          rosbag_0.db3
"""

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from lidar_capture import VelodyneCapture, DATA_PORT, POSITION_PORT  # noqa: E402
from record import (SessionWriter, lidar_preflight,  # noqa: E402
                    print_lidar_preflight, load_daheng, LIDAR_IP,
                    DEFAULT_DAHENG_WS)

DEFAULT_FPS = 10.0          # record.py 의 트리거 주파수와 같게 맞춘 기본값
POLL_TIMEOUT_MS = 200


# ===========================================================================
#  카메라 한 대 — 트리거 없이 자체 클럭으로 돈다
# ===========================================================================
class FreeRunCamera:
    """sync_cam.TriggerCamera 의 프리런 판.

    다른 점은 트리거 설정뿐이다. 노출/게인 고정, 수신 버퍼 확대, 폴링
    수신은 전부 같은 이유로 같게 둔다 (sync_cam.py 의 주석 참고).
    """

    def __init__(self, sc, dm, sn, cam_idx, slot, queue, latest,
                 exposure_us, gain_db, fps, pixel_format=None):
        self.sn = sn
        self.idx = cam_idx                  # 런타임 인덱스 (큐에 실려 나감)
        self.slot = slot                    # 물리 슬롯 (CAMERA_SNS 안에서의 위치)
        self.name = f"CAM{slot}"
        self.stats = {"queued": 0, "incomplete": 0, "null": 0,
                      "dropped_queue": 0, "error": 0}

        self._cam = dm.open_device_by_sn(sn)
        rc = self._cam.get_remote_device_feature_control()
        self._rc = rc

        # --- 트리거를 끈다 ---
        # TriggerMode=Off 가 조용히 실패하면 카메라는 Line0 을 계속 기다리고
        # 한 장도 안 찍힌다. 20초를 다 보내고 나서야 0장인 걸 알게 되므로
        # 여기서 반드시 예외를 낸다 (sync_cam 이 On 을 _must 로 거는 것과
        # 같은 이유, 방향만 반대다).
        sc._try_enum(rc, "AcquisitionMode", "Continuous")
        sc._try_enum(rc, "TriggerSelector", "FrameStart")
        sc._must_enum(rc, "TriggerMode", "Off")

        # --- 프레임레이트 제한 ---
        # 제한하지 않으면 이 모델은 159 fps 까지 올라간다. 4대면 초당
        # 1.5 GB 라 디스크도 USB3 도 못 받는다. 반드시 묶어 둔다.
        self.fps_mode = None
        for name in ("AcquisitionFrameRateMode", "AcquisitionFrameRateEnable"):
            if sc._try_enum(rc, name, "On"):
                self.fps_mode = name
                break
        self.fps_set = None
        try:
            self.fps_set = sc._must_float(rc, "AcquisitionFrameRate", fps)
        except Exception as e:
            raise RuntimeError(
                f"{self.name}: 프레임레이트를 제한할 수 없습니다 ({e}).\n"
                f"  이 상태로 찍으면 카메라가 최고 속도로 돌아 디스크가 "
                f"넘칩니다.") from e

        # --- 노출 / 게인 고정 (sync_cam 과 같은 이유로 전부 Off) ---
        period_us = 1e6 / fps
        if exposure_us > period_us:
            raise ValueError(
                f"노출 {exposure_us:.0f} us 가 프레임 주기 {period_us:.0f} us "
                f"보다 깁니다. 카메라가 {fps} fps 를 못 냅니다.")
        sc._must_enum(rc, "ExposureAuto", "Off")
        sc._must_float(rc, "ExposureTime", exposure_us)
        sc._must_enum(rc, "GainAuto", "Off")
        sc._must_float(rc, "Gain", gain_db)

        # --- 화질 (감마 / 화이트밸런스) ---
        # GammaEnable=False 면 12 bit ADC 가 선형으로 8 bit 에 잘려서 어두운
        # 쪽 4 bit 가 0 으로 뭉친다. 주차장처럼 어두운 장면에서는 이게
        # 노출·게인보다 영향이 크다. sync_cam._apply_quality 주석에 실측값.
        sc._apply_quality(rc, pixel_format)

        self.width = sc._read_int(rc, "Width")
        self.height = sc._read_int(rc, "Height")
        self.pixel_format = sc._read_enum(rc, "PixelFormat")
        self.exposure_us = sc._read_float(rc, "ExposureTime")
        self.gain_db = sc._read_float(rc, "Gain")
        # 실제로 걸린 값을 meta.json 에 남긴다. 나중에 세션끼리 밝기를
        # 비교할 때 설정을 되짚을 수 있어야 한다.
        self.gamma_enable = sc._read_bool(rc, "GammaEnable")
        self.gamma_mode = sc._read_enum(rc, "GammaMode")
        self.light_source = sc._read_enum(rc, "LightSourcePreset")
        self.balance_white_auto = sc._read_enum(rc, "BalanceWhiteAuto")
        # 카메라가 실제로 낼 수 있다고 보고하는 값. 노출이 길면 여기서 깎인다.
        self.resulting_fps = (sc._read_float(rc, "CurrentAcquisitionFrameRate")
                              or sc._read_float(rc, "ResultingFrameRate"))

        ds = self._cam.data_stream[0]
        try:
            ds.set_acquisition_buffer_number(sc.ACQ_BUFFER_NUM)
        except Exception:
            pass
        try:
            ds.StreamBufferHandlingMode.set("OldestFirst")
        except Exception:
            pass

        self._ds = ds
        self._sc = sc
        self._queue = queue
        self._latest = latest
        self._thread = None
        self._stop = threading.Event()

    def stream_on(self):
        """AcquisitionStart. 프리런이므로 이 순간부터 실제로 찍기 시작한다."""
        self._cam.stream_on()
        self._thread = threading.Thread(
            target=self._sc._reader_loop,
            args=(self, self._queue, self.stats, self._latest, self._stop),
            daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for fn in (self._cam.stream_off, self._cam.close_device):
            try:
                fn()
            except Exception:
                pass

    def stream_stats(self):
        out = {}
        for key, feat in (("delivered", "StreamDeliveredFrameCount"),
                          ("lost", "StreamLostFrameCount"),
                          ("incomplete", "StreamIncompleteFrameCount")):
            try:
                out[key] = int(getattr(self._ds, feat).get())
            except Exception:
                out[key] = None
        return out


# ===========================================================================
#  카메라 묶음
# ===========================================================================
class FreeRunArray:
    def __init__(self, sc, sns, slots, exposure_us, gain_db, fps, pixel_format=None,
                 queue_maxsize=None):
        from queue import Queue
        self.sc = sc
        self.sns = list(sns)
        self.slots = list(slots)
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self.pixel_format = pixel_format
        self.fps = fps
        self.queue = Queue(maxsize=queue_maxsize or sc.QUEUE_MAXSIZE)
        self.latest = {}
        self.cams = []
        self._dm = None

    def __enter__(self):
        import gxipy as gx
        self._dm = gx.DeviceManager()
        self._dm.update_device_list()
        print(f"카메라 {len(self.sns)} 대 여는 중 (프리런 {self.fps} fps)...")
        for i, (sn, slot) in enumerate(zip(self.sns, self.slots)):
            cam = FreeRunCamera(self.sc, self._dm, sn, i, slot, self.queue,
                                self.latest, self.exposure_us, self.gain_db,
                                self.fps, self.pixel_format)
            self.cams.append(cam)
            res = (f"  실제 {cam.resulting_fps:.2f} fps"
                   if cam.resulting_fps else "")
            print(f"  {cam.name}  sn={sn}  {cam.width}x{cam.height} "
                  f"{cam.pixel_format}  exp={cam.exposure_us:.0f}us "
                  f"gain={cam.gain_db:.1f}dB  gamma={cam.gamma_enable} "
                  f"wb={cam.light_source}/{cam.balance_white_auto}{res}")
        for cam in self.cams:
            cam.stream_on()
        # 프리런은 stream_on 순간부터 찍는다. 세션 밖 프레임은 SessionWriter
        # 가 late 로 세고 버린다.
        print("전 카메라 촬영 중. (세션이 열리기 전 프레임은 버려진다)\n")
        return self

    def __exit__(self, *exc):
        for cam in self.cams:
            cam.close()
        self.cams = []
        self._dm = None

    @property
    def n(self):
        return len(self.cams)

    def meta(self):
        return {
            "cameras": [
                {"name": c.name, "sn": c.sn, "index": c.idx,
                 "width": c.width, "height": c.height,
                 "pixel_format": c.pixel_format,
                 "exposure_us": c.exposure_us, "gain_db": c.gain_db,
                 "gamma_enable": c.gamma_enable, "gamma_mode": c.gamma_mode,
                 "light_source": c.light_source,
                 "balance_white_auto": c.balance_white_auto}
                for c in self.cams
            ],
            "trigger": {"mode": "Off", "source": "free-run",
                        "fps_requested": self.fps,
                        "fps_resulting": [c.resulting_fps for c in self.cams]},
        }

    def print_stats(self):
        print("\n--- 카메라별 수신 통계 ---")
        for c in self.cams:
            s, ss = c.stats, c.stream_stats()
            print(f"  {c.name}: 큐투입 {s['queued']}  불완전 {s['incomplete']}  "
                  f"큐넘침 {s['dropped_queue']}  오류 {s['error']}  "
                  f"| SDK delivered={ss['delivered']} lost={ss['lost']}")


# ===========================================================================
#  세션 하나
# ===========================================================================
def run_session(session_dir, duration, lidar, cams, writer, fps,
                lidar_only=False):
    """lidar_only=True 면 카메라 프레임을 저장하지 않는다 (매핑 주행용).

    카메라는 프리런으로 계속 돌지만 writer.open_session 을 부르지 않으므로
    프레임은 writer.late 로 세어지고 버려진다. 카메라를 끄지 않는 이유는,
    끄면 다음 세션에서 다시 켜는 데 시간이 걸려 매핑과 본 녹화 사이가
    벌어지기 때문이다. 그 사이에 주차된 차가 바뀌면 지도가 쓸모없어진다.
    """
    os.makedirs(session_dir, exist_ok=True)

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "duration_s": duration,
        "format": "npy",
        "mode": "free-run (MCU 없음, 하드웨어 동기 없음)"
                + ("  [매핑용 — 라이다만 저장]" if lidar_only else ""),
        "lidar_only": lidar_only,
        "trigger_hz": fps,
        "lidar": {"ip": LIDAR_IP, "model": "VLP-16",
                  "data_port": DATA_PORT, "position_port": POSITION_PORT},
    }
    meta.update(cams.meta())
    meta["camera_slots"] = writer.slots
    meta_path = os.path.join(session_dir, "meta.json")
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # 세션 경계를 열기 직전에 큐를 비운다. 프리런은 계속 찍고 있으므로
    # 여기 남아 있는 것은 전부 세션 시작 이전 프레임이다.
    drained = 0
    while not cams.queue.empty():
        try:
            cams.queue.get_nowait()
            drained += 1
        except Exception:
            break

    if not lidar_only:
        writer.open_session(session_dir)
    pcap_path = os.path.join(session_dir, "lidar.pcap")
    lidar.start_session(pcap_path)
    lidar_base = lidar.stats["written"]

    t_start = time.time()
    print(f"  녹화 {duration}초"
          + ("  [라이다만 — 카메라 미저장]" if lidar_only else "")
          + (f"  (시작 전 프레임 {drained} 장 버림)" if drained else ""))
    while True:
        el = time.time() - t_start
        if el >= duration:
            break
        cam_txt = "카메라 미저장" if lidar_only else f"카메라 {writer.written}"
        print(f"\r    {el:5.1f}s  {cam_txt}  "
              f"라이다 {lidar.stats['written'] - lidar_base}  "
              f"큐 카메라 {cams.queue.qsize()} / 라이다 {lidar.queue.qsize()}   ",
              end="", flush=True)
        time.sleep(0.2)
    print()

    # 이미 큐에 들어온 것(= 마감 전에 찍힌 것)만 흘려보내고 바로 닫는다.
    #
    # record.py 는 여기서 큐가 빈 것을 두 번 확인하며 기다린다. 그쪽은 MCU
    # 가 STOP 으로 셔터를 멈춰 주므로 기다려도 새 프레임이 안 들어온다.
    # 프리런은 아무도 멈추지 않아서, 같은 식으로 기다리면 그 1초 동안 찍힌
    # 프레임이 계속 세션에 들어간다 (3초 세션에 41장이 담겼다).
    # 그래서 '빌 때까지'만 기다리고 곧바로 경계를 닫는다. SDK 에서 아직
    # 꺼내지 않은 프레임은 버려지며, writer.late 로 세어 meta.json 에 남는다.
    print("  큐 비우는 중...")
    deadline = time.time() + 2.0
    while time.time() < deadline and not cams.queue.empty():
        time.sleep(0.02)

    # 카메라 경계를 먼저 닫는다. 라이다 쪽이 큐를 비우는 동안에도 카메라는
    # 계속 찍고 있으므로, 순서가 반대면 그만큼이 세션에 더 들어간다.
    if not lidar_only:
        writer.close_session()
    lidar_summary = lidar.stop_session()

    meta["stopped"] = datetime.now().isoformat(timespec="seconds")
    meta["lidar"].update(lidar_summary or {})
    if not lidar_only:
        meta["frames_written"] = {writer.dirname(i): n
                                  for i, n in enumerate(writer.written)}
        if writer.late:
            meta["frames_late"] = writer.late
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # --- 화면 요약 ---
    if lidar_only:
        print("  카메라: 저장하지 않음 (매핑용)")
    else:
        print(f"  카메라 저장: {writer.written}  "
              f"(기대 각 {int(duration * fps)} 장 내외)")
    if lidar_summary:
        got = lidar_summary["data_packets"]
        exp = lidar_summary["expected_data_packets"]
        rate = f"{100.0 * got / exp:.2f}%" if exp else "-"
        print(f"  라이다 저장: {lidar_summary['packets_written']} 패킷 "
              f"({lidar_summary['bytes_written'] / 1e6:.1f} MB), "
              f"데이터 {got}/{exp} = {rate}")
        if lidar_summary["dropped_queue"]:
            print(f"  [주의] 라이다 큐 넘침 "
                  f"{lidar_summary['dropped_queue']} — 디스크가 못 따라옵니다.")
    for c in cams.cams:
        if c.stats["dropped_queue"]:
            print(f"  [주의] {c.name} 큐 넘침 {c.stats['dropped_queue']} 장 "
                  f"— 디스크가 못 따라옵니다.")
    return meta


# ===========================================================================
#  rosbag2 변환
# ===========================================================================
def to_bag(session_dir, storage):
    """세션 하나를 rosbag2 한 덩어리로 바꾼다.

    수집이 다 끝난 뒤에 부른다. 변환에서 예외가 나도 원본 pcap/npy 는
    그대로 남으므로 to_rosbag.py 로 다시 돌리면 된다.
    """
    out = os.path.join(session_dir, "rosbag")
    if os.path.exists(out):
        print(f"  [건너뜀] {out} 이 이미 있습니다.")
        return None
    try:
        import to_rosbag
    except SystemExit as e:
        print(f"  [실패] rosbag2 변환 불가: {e}")
        print( "         source /opt/ros/humble/setup.bash 후 "
               "to_rosbag.py 로 따로 돌리세요.")
        return None
    t0 = time.time()
    r = to_rosbag.convert(session_dir, out, storage_id=storage, quiet=True)
    print(f"  rosbag2  PointCloud2 {r['clouds']} 개, Image {r['images']} 장 "
          f"→ {out}  ({time.time() - t0:.0f}초)")
    return r


# ===========================================================================
#  main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description="카메라 4대 + 라이다 프리런 수집 (MCU 없이)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--sessions", type=int, default=1,
                    help="세션 반복 횟수. --duration 에 값이 두 개면 무시된다")
    ap.add_argument("--duration", type=float, nargs="+", default=[20.0],
                    metavar="초",
                    help="세션 길이(초). 값을 두 개 주면 매핑 주행 모드가 된다 — "
                         "첫 번째는 **라이다만** 켜고 찍는 시간, 두 번째는 "
                         "카메라까지 켜고 찍는 시간. 예: --duration 300 20")
    ap.add_argument("--gap", type=float, default=3.0, help="세션 간 대기(초)")
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS,
                    help="카메라 프레임레이트 (프리런)")
    ap.add_argument("--out", default=os.path.join(_HERE, "sessions"),
                    help="저장 루트 폴더")
    ap.add_argument("--daheng-ws", default=DEFAULT_DAHENG_WS)
    ap.add_argument("--exposure", type=float, default=None,
                    help="노출(us). 생략하면 sync_cam 기본값")
    ap.add_argument("--gain", type=float, default=None, help="게인(dB)")
    ap.add_argument("--pixel-format", default=None,
                    choices=["BayerGB8", "BayerGB10", "BayerGB12"],
                    help="비트깊이. 생략하면 sync_cam.PIXEL_FORMAT "
                         "(12 bit 는 용량 2배, 감마는 자동으로 꺼진다)")
    ap.add_argument("--lidar-ip", default=LIDAR_IP,
                    help="라이다 IP. 'any' 면 출처를 가리지 않는다")
    ap.add_argument("--no-rosbag", action="store_true",
                    help="rosbag2 변환 없이 원본만 남긴다")
    ap.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"])
    args = ap.parse_args()

    out_root = os.path.abspath(args.out)
    os.makedirs(out_root, exist_ok=True)

    # ---------------------------------------------------------- 라이다 점검
    info = lidar_preflight()
    if not print_lidar_preflight(info):
        return 1
    print()

    # ---------------------------------------------------------- 카메라 확인
    sync_cam, _collect = load_daheng(args.daheng_ws)
    exposure = (args.exposure if args.exposure is not None
                else sync_cam.DEFAULT_EXPOSURE_US)
    gain = args.gain if args.gain is not None else sync_cam.DEFAULT_GAIN_DB

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
        print("[실패] CAMERA_SNS 중 연결된 카메라가 없습니다.")
        print(f"       연결된 sn: {sorted(found)}")
        return 1
    missing = [sn for sn in sync_cam.CAMERA_SNS if sn not in found]
    if missing:
        print(f"[주의] 연결 안 된 카메라 {len(missing)} 대를 건너뜁니다: "
              f"{missing}")
    print("사용할 카메라: " +
          ", ".join(f"CAM{s}({sn})" for s, sn in zip(slots, sns)))

    # 디스크가 버티는지 미리 계산해서 알려 준다. 20초 4대면 2 GB 다.
    mb_s = len(sns) * 2048 * 1200 * args.fps / 1e6
    LIDAR_MB_S = 1.03          # VLP-16 실측 (20 초에 20.6 MB)
    if len(args.duration) >= 2:
        total_gb = (mb_s * args.duration[1] + LIDAR_MB_S * args.duration[0]) / 1e3
    else:
        total_gb = mb_s * args.duration[0] * args.sessions / 1e3
    free_gb = os.statvfs(out_root).f_bavail * os.statvfs(out_root).f_frsize / 1e9
    if len(args.duration) >= 2:
        print(f"예상 기록량  매핑 {args.duration[0]:.0f}초 (라이다만, "
              f"{LIDAR_MB_S * args.duration[0]:.0f} MB)"
              f" + 본녹화 {args.duration[1]:.0f}초 ({mb_s:.0f} MB/s)"
              f" = 약 {total_gb:.1f} GB")
    else:
        print(f"예상 기록량  {mb_s:.0f} MB/s × {args.duration[0]:.0f}초 × "
              f"{args.sessions} 세션 = 약 {total_gb:.1f} GB"
              + ("" if args.no_rosbag else f" (+ rosbag2 사본 {total_gb:.1f} GB)"))
    print(f"디스크 여유  {free_gb:.0f} GB")
    need = total_gb * (1 if args.no_rosbag else 2)
    if free_gb < need * 1.2:
        print(f"[실패] 디스크가 부족합니다. 최소 {need * 1.2:.0f} GB 필요.")
        return 1

    if not args.no_rosbag:
        try:
            import rosbag2_py  # noqa: F401
        except ImportError:
            print("\n[주의] rosbag2_py 를 불러올 수 없습니다. 수집은 그대로")
            print("       진행하고, 변환만 건너뜁니다. 나중에")
            print("       source /opt/ros/humble/setup.bash 후")
            print("       python3 to_rosbag.py <세션> 로 돌리세요.")
    print()

    # ------------------------------------------------------------ 수집
    src = None if args.lidar_ip == "any" else args.lidar_ip
    lidar = VelodyneCapture(expect_src=src)
    lidar.open()
    if lidar.src_mac != b"\x00" * 6:
        print("라이다 MAC:", ":".join(f"{b:02x}" for b in lidar.src_mac), "\n")

    done = []
    try:
        with FreeRunArray(sync_cam, sns, slots, exposure, gain,
                          args.fps, args.pixel_format) as cams:
            writer = SessionWriter(cams.queue, slots, fmt="npy")
            writer.start()

            existing = [d for d in os.listdir(out_root)
                        if d.startswith("session_")]
            start_n = 1 + max([int(d.split("_")[1]) for d in existing
                               if d.split("_")[1].isdigit()] or [0])
            try:
                # --duration 에 값이 두 개면 매핑 주행 모드:
                #   세션 1 = 라이다만, 세션 2 = 카메라까지. --sessions 는 무시.
                if len(args.duration) >= 2:
                    plan = [(args.duration[0], True), (args.duration[1], False)]
                else:
                    plan = [(args.duration[0], False)] * args.sessions

                for k, (dur, lidar_only) in enumerate(plan):
                    n = start_n + k
                    sd = os.path.join(out_root, f"session_{n:03d}")
                    tag = " [매핑]" if lidar_only else ""
                    print(f"\n===== session_{n:03d} "
                          f"({k + 1}/{len(plan)}){tag} =====")
                    run_session(sd, dur, lidar, cams, writer, args.fps,
                                lidar_only=lidar_only)
                    done.append((sd, lidar_only))
                    if k < len(plan) - 1:
                        print(f"  {args.gap}초 대기")
                        time.sleep(args.gap)
            except KeyboardInterrupt:
                print("\n[중단] 정리 중...")
                lidar.stop_session()
                writer.close_session()
                time.sleep(0.5)

            cams.print_stats()
            writer.stop()
    finally:
        lidar.close()

    # --------------------------------------------------- 세션마다 rosbag2
    # 매핑 세션은 카메라가 없으므로 rosbag 으로 만들 것이 없다
    todo = [sd for sd, lo in done if not lo]
    if todo and not args.no_rosbag:
        print(f"\n===== rosbag2 변환 ({len(todo)} 세션) =====")
        for sd in todo:
            print(f"  {os.path.basename(sd)}")
            try:
                to_bag(sd, args.storage)
            except Exception as e:
                print(f"  [실패] {e}")
                print(f"         python3 to_rosbag.py {sd} 로 다시 시도하세요.")

    print(f"\n저장 위치: {out_root}")
    for sd in done:
        print(f"  {sd}")
    if done:
        print(f"\n검산:  python3 check_session.py {done[0]}")
        print(f"눈으로:  python3 preview_session.py {done[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
