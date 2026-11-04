"""하드웨어 트리거 모드 Daheng 카메라 래퍼 — 데이터 수집 전용.

dhcam.py 와의 차이 (둘은 요구사항이 정반대라 분리해 둔다)

              dhcam.py (캘리브레이션)      sync_cam.py (수집)
  트리거      Free Run                     Line0 하드웨어 트리거
  노출        Auto                         고정 (지연이 상수여야 함)
  프레임 획득  polling (get_image)          콜백 + 큐
  대수        1대                          4대 동시
  설정 실패    조용히 무시 (_try)            즉시 예외 (_must)

dhcam.py 는 그대로 두고 이 파일을 따로 쓴다. 캘리브레이션은 계속 필요하다.

--------------------------------------------------------------------------
설정 실패를 무시하면 안 되는 이유
  TriggerMode=On 이 조용히 실패하면 카메라는 Free Run 으로 계속 돌고,
  로그는 멀쩡해 보이는데 사진이 트리거와 무관하게 찍힌다. 데이터가
  조용히 망가지는 유형이라 반드시 예외를 내야 한다.

콜백 안에서 반드시 복사해야 하는 이유
  SDK 가 콜백에 넘겨주는 image_buf 는 SDK 소유 메모리이고 곧 재사용된다.
  copy() 없이 큐에 넣으면 다음 프레임이 덮어써서 이미지가 섞인다.
"""

import os
import sys
import threading
import time
from queue import Queue, Full

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_VENDOR = os.path.join(_HERE, "vendor")
if _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

import gxipy as gx  # noqa: E402


# ===========================================================================
#  설정 — list_cams.py 결과로 채울 것
# ===========================================================================
CAMERA_SNS = [
    "FHH26070137",    # CAM1  → ESP32 GPIO6
    "FHH26070138",  # CAM2  → ESP32 GPIO15
    "FHH26070139",  # CAM3  → ESP32 GPIO38
    "FHH26070140",  # CAM4  → ESP32 GPIO39
]

# 노출은 반드시 고정. 오토로 두면 트리거→셔터 지연이 프레임마다 달라져
# 상수 보정이 불가능해진다.
DEFAULT_EXPOSURE_US = 16000.0   # 16 ms
DEFAULT_GAIN_DB     = 16.0      # 이 모델의 상한

# --- 화질 ---------------------------------------------------------------
#  실내 주차장 촬영(session_008~034)은 노출 8 ms · 게인 8 dB · 감마 Off 로
#  찍혔고, raw 평균 밝기가 2~7/255, 전체 픽셀의 52~79% 가 0~1 에 깔렸다.
#  원인은 노출이 아니라 인코딩이었다. GammaEnable=False 면 12 bit ADC 가
#  선형으로 8 bit 에 잘리므로, 어두운 쪽 4 bit 가 통째로 0 으로 양자화된다.
#  실측(MER2-240-159U3C, 같은 장면):
#
#      감마 Off, 8 ms, 8 dB   평균  1.6/255   0~1 픽셀 92.3%
#      감마 On,  8 ms, 8 dB   평균  7.9/255   0~1 픽셀 42.7%   ← 공짜
#      + 형광등 WB            평균  8.9/255   0~1 픽셀 37.6%
#      + 게인 16 dB           평균 15.7/255   0~1 픽셀 28.2%
#      + 노출 16 ms           평균 20.7/255   0~1 픽셀 21.2%
#      + 노출 24 ms           평균 26.1/255   0~1 픽셀 15.3%
#
#  노출은 모션 블러와 맞바꾼다. session_031/034 의 실측 프레임간 변위는
#  p75 기준 13~65 px/frame(0.1 s) 이라, 노출 16 ms 면 블러 2~10 px 다.
#  측면 카메라가 가장 불리하므로 주행 속도가 빠른 구간은 16 ms 를 넘기지
#  않는 편이 낫다.
#  --- 비트깊이 ---
#  BayerGB8  : 1 바이트/화소. 4대 10 Hz 에 98 MB/s.
#  BayerGB12 : 2 바이트/화소. 4대 10 Hz 에 196 MB/s. ADC 원본 12 bit 를
#              그대로 받으므로 어두운 쪽이 잘리지 않는다. 용량 2배.
PIXEL_FORMAT        = "BayerGB12"

#  --- 감마 ---
#  8 bit 로 받을 때만 켠다. 12 bit 는 ADC 전체를 그대로 받으므로 잘릴 게
#  없고, 감마를 걸면 선형성만 잃는다 (투영·측광 후처리가 선형을 가정한다).
#  None = 자동: 8 bit 면 True, 10/12 bit 면 False.
GAMMA_ENABLE        = None
#  --- 화이트밸런스 ---
#  프리셋은 BalanceRatio 를 raw 에 '구워 넣는다'. 실측(MER2-240-159U3C):
#
#      CoolWhiteFluorescence   R=1.504 G=1.000 B=3.020
#      Daylight6500K           R=1.836 G=1.000 B=2.105
#      Off                     R=1.000 G=1.000 B=1.000
#
#  8 bit 는 후처리로 WB 를 돌릴 여유가 없으니 구워 넣는 게 낫다.
#  12 bit 는 반대다. B 를 3배 곱하면 신호뿐 아니라 B 노이즈도 3배가 되고
#  (CAM4 암부 std R1.4/G1.0/B2.2), 어두운 화면에서 마젠타로 뜬다.
#  게다가 B 가 먼저 포화해 다이내믹레인지를 버린다. 센서 원본 그대로
#  받고 WB 는 export_images 에서 건다.
#  None = 자동: 8 bit 면 형광등 프리셋, 10/12 bit 면 Off.
LIGHT_SOURCE_PRESET = None
#  AWB 는 끈다. 프레임마다 게인이 바뀌면 같은 물체 색이 세션 내내 흔들려
#  측광·학습에 불리하고, 어두운 카메라(측면/후방)에서는 노이즈를 보고
#  게인이 튄다. 프리셋만으로 광원은 충분히 맞는다.
BALANCE_WHITE_AUTO  = "Off"

# 트리거 주기(10 Hz = 100 ms)보다 노출이 길면 다음 트리거를 씹는다.
TRIGGER_PERIOD_US   = 100000.0
EXPOSURE_SAFETY     = 0.8       # 주기의 80% 를 넘으면 거부

# ===========================================================================
#  시차 발사 (A 방식) — 라이다가 지나갈 때 하나씩 찍기
# ===========================================================================
#  라이다는 100 ms 에 걸쳐 360° 를 훑으므로, 4대를 동시에 찍으면 앞 카메라만
#  라이다와 맞고 뒤 카메라는 50 ms 어긋난다. 그래서 카메라마다 TriggerDelay 를
#  줘서 라이다가 그 카메라 쪽을 지날 때 노출한다 (nuScenes 방식).
#
#      FSYNC ─┬─ CAM1   +0 ms   앞      0°
#             ├─ CAM2  +25 ms   오른쪽  90°
#             ├─ CAM3  +50 ms   뒤    180°
#             └─ CAM4  +75 ms   왼쪽  270°
#
#  대가: 카메라끼리는 동시가 아니다 (이웃한 두 대가 25 ms 차이).
#  배선은 그대로다. GPIO4 하나가 4대에 같은 펄스를 보내고 카메라가 각자
#  기다렸다 찍는다. MER2-240-159U3C TriggerDelay 최대 3,000,000 us (실측 확인).
#
#  방위각은 라이다 좌표 기준, 위에서 볼 때 시계 방향(VLP-16 회전 방향)이다.
#  공간 캘리브레이션 결과가 나오면 이 표만 고치면 된다.
#  STAGGERED_TRIGGER = False 로 두면 예전처럼 4대 동시 발사.
STAGGERED_TRIGGER   = True
CAMERA_AZIMUTH_DEG  = [90.0, 270.0, 0.0, 180.0]      # 슬롯 1~4

# 노출이 끝나고 센서를 읽어내는 시간. 다음 트리거 전에 끝나야 한다.
# MER2-240-159U3C 최대 159 fps → 한 장 6.3 ms. 여유를 두어 8 ms.
READOUT_MARGIN_US   = 8000.0


def trigger_delay_us(slot):
    """슬롯(1~4) 의 TriggerDelay. 라이다가 그 방위각에 오는 시각."""
    if not STAGGERED_TRIGGER:
        return 0.0
    i = slot - 1
    az = CAMERA_AZIMUTH_DEG[i] if 0 <= i < len(CAMERA_AZIMUTH_DEG) else 0.0
    return (az % 360.0) / 360.0 * TRIGGER_PERIOD_US


def camera_azimuth_deg(slot):
    i = slot - 1
    return CAMERA_AZIMUTH_DEG[i] if 0 <= i < len(CAMERA_AZIMUTH_DEG) else None


def max_exposure_us(slot):
    """그 슬롯이 쓸 수 있는 최대 노출. 지연 + 노출 + 읽기 < 트리거 주기."""
    return TRIGGER_PERIOD_US - trigger_delay_us(slot) - READOUT_MARGIN_US


def recommended_phaselock_offset(exposure_us):
    """라이다 통과 순간이 노출 '중앙' 에 오도록 하는 Phase Lock offset (도).

    지연을 방위각대로만 주면 라이다가 카메라 중심을 지나는 순간이 노출
    '시작' 이 된다. 노출 10 ms 동안 라이다는 36° 를 더 돌므로, 노출 중앙에서는
    이미 18° 지나 있다. 지연을 노출 절반만큼 당기면 앞 카메라가 음수가 되므로,
    대신 라이다 위상을 그만큼 앞으로 당긴다:

        offset = −(노출 / 2) / 주기 × 360°       노출 10 ms → 342°

    노출을 바꾸면 이 값도 바뀐다.
    """
    return (-(exposure_us / 2.0) / TRIGGER_PERIOD_US * 360.0) % 360.0

# 콜백 → 저장 스레드 사이 큐 깊이. 한 프레임 약 2.5 MB.
QUEUE_MAXSIZE       = 120

# SDK 내부 수신 버퍼 개수. 기본값은 작아서 순간적인 디스크 지연에 취약하다.
ACQ_BUFFER_NUM      = 40


# ===========================================================================
#  feature 헬퍼
# ===========================================================================
def _must_enum(rc, name, value):
    """실패하면 예외. 데이터 정합성에 직결되는 설정에만 쓴다."""
    try:
        rc.get_enum_feature(name).set(value)
    except Exception as e:
        raise RuntimeError(f"필수 설정 실패: {name} = {value}  ({e})") from e


def _must_float(rc, name, value, clamp=True):
    """실패하면 예외. clamp=True 면 카메라가 허용하는 범위로 잘라 넣는다.

    ExposureTime / Gain 은 모델마다 상한이 다르다 (MER2-240-159U3C 의 Gain
    상한은 16 dB). 사용자가 범위를 모른 채 큰 값을 줬다고 촬영을 통째로
    막을 이유는 없으므로, 범위를 알려주고 경계값으로 맞춘다."""
    f = rc.get_float_feature(name)
    v = float(value)
    if clamp:
        try:
            rng = f.get_range()
            lo, hi = float(rng["min"]), float(rng["max"])
            if v < lo or v > hi:
                v2 = min(max(v, lo), hi)
                print(f"  [조정] {name} {v:.0f} → {v2:.0f} "
                      f"(허용 범위 {lo:.0f} ~ {hi:.0f})")
                v = v2
        except Exception:
            pass
    try:
        f.set(v)
    except Exception as e:
        raise RuntimeError(f"필수 설정 실패: {name} = {v}  ({e})") from e
    return v


def _try_enum(rc, name, value):
    """실패해도 넘어감. 카메라 모델에 따라 없을 수 있는 항목."""
    try:
        rc.get_enum_feature(name).set(value)
        return True
    except Exception:
        return False


def _try_bool(rc, name, value):
    """실패해도 넘어감. GammaEnable 처럼 모델별로 없을 수 있는 항목."""
    try:
        rc.get_bool_feature(name).set(bool(value))
        return True
    except Exception:
        return False


def _read_bool(rc, name):
    try:
        return bool(rc.get_bool_feature(name).get())
    except Exception:
        return None


def _apply_quality(rc, pixel_format=None):
    """어둠/색 캐스트를 촬영 단계에서 잡는다. 전부 선택 항목이라 실패는 무시.

    순서가 중요하다. PixelFormat 을 먼저 정해야 감마를 켤지 말지 정해지고,
    LightSourcePreset 은 BalanceRatio 를 덮어쓰므로 프리셋을 먼저 걸고
    그 위에 AWB 를 올린다."""
    pf = pixel_format or PIXEL_FORMAT
    if pf:
        if not _try_enum(rc, "PixelFormat", pf):
            print(f"  [경고] PixelFormat={pf} 설정 실패. 현재 값을 유지합니다.")
            pf = _read_enum(rc, "PixelFormat")

    deep = any(b in str(pf) for b in ("10", "12", "16"))

    gamma = GAMMA_ENABLE
    if gamma is None:                   # 자동: 8 bit 만 켠다
        gamma = not deep
    _try_bool(rc, "GammaEnable", gamma)
    if gamma:
        _try_enum(rc, "GammaMode", "SRGB")

    preset = LIGHT_SOURCE_PRESET
    if preset is None:                  # 자동: 8 bit 만 구워 넣는다
        preset = "Off" if deep else "CoolWhiteFluorescence"
    _try_enum(rc, "BalanceWhiteAuto", "Off")    # 프리셋을 덮지 않게 먼저
    _try_enum(rc, "LightSourcePreset", preset)
    if BALANCE_WHITE_AUTO != "Off":
        _try_enum(rc, "BalanceWhiteAuto", BALANCE_WHITE_AUTO)


def _read_enum(rc, name):
    try:
        v = rc.get_enum_feature(name).get()
        return v[1] if isinstance(v, (tuple, list)) else v
    except Exception:
        return None


def _read_float(rc, name):
    try:
        return float(rc.get_float_feature(name).get())
    except Exception:
        return None


def _read_int(rc, name):
    try:
        return int(rc.get_int_feature(name).get())
    except Exception:
        return None


# ===========================================================================
#  수신 스레드 — 카메라 한 대당 하나
# ===========================================================================
#  이 SDK 버전의 register_capture_callback 은 등록은 성공하지만 실제로
#  호출되지 않는다 (StreamDeliveredFrameCount 는 올라가는데 파이썬 콜백은
#  한 번도 불리지 않음). 그래서 검증된 폴링 방식(get_image)을 쓴다.
#
#  카메라마다 전용 스레드를 두므로, 한 대가 타임아웃으로 대기하는 동안
#  다른 대의 프레임을 놓치는 문제는 생기지 않는다. dhcam.py 가 쓰는 것과
#  같은 경로이며 1대 기준 35 fps 가 확인되었다 (10 Hz 대비 충분).
#
#  get_image() 는 호출할 때마다 새 버퍼를 만들어 반환하므로 콜백 방식과
#  달리 복사가 필요 없다.
# ===========================================================================
POLL_TIMEOUT_MS = 200


def _reader_loop(cam, q, stats, latest, stop_evt):
    ds = cam._ds
    idx = cam.idx
    while not stop_evt.is_set():
        try:
            raw = ds.get_image(POLL_TIMEOUT_MS)
        except Exception:
            stats["error"] += 1
            time.sleep(0.01)
            continue

        if raw is None:                 # 타임아웃 = 트리거 대기 중. 정상.
            continue
        if raw.get_status() != 0:
            stats["incomplete"] += 1
            continue

        arr = raw.get_numpy_array()
        if arr is None:
            stats["null"] += 1
            continue

        recv = time.time()
        fid = int(raw.get_frame_id())
        latest[idx] = (fid, arr)

        try:
            q.put_nowait((idx, fid, int(raw.get_timestamp()), recv, arr))
            stats["queued"] += 1
        except Full:
            stats["dropped_queue"] += 1


# ===========================================================================
#  카메라 한 대
# ===========================================================================
class TriggerCamera:
    def __init__(self, dm, sn, cam_idx, queue, latest, exposure_us, gain_db,
                 slot=None):
        self.sn = sn
        self.idx = cam_idx                 # 큐/프리뷰 인덱스 (0 부터, 연결된 것만)
        # 물리 슬롯 = CAMERA_SNS 안에서의 위치 (1 부터). 이 번호가 곧 ESP32 의
        # Line1 채널 번호이고, 세션 폴더 이름(camN) 이며, 로그의 EXP,<N> 이다.
        # 일부 카메라만 연결되면 idx 와 slot 이 달라진다 (3번만 꽂으면 idx 0,
        # slot 3). 사람에게 보이는 이름과 폴더에는 반드시 slot 을 쓴다.
        self.slot = slot if slot is not None else cam_idx + 1
        self.name = f"CAM{self.slot}"
        self.stats = {"queued": 0, "incomplete": 0, "null": 0,
                      "dropped_queue": 0, "error": 0}

        self._cam = dm.open_device_by_sn(sn)
        rc = self._cam.get_remote_device_feature_control()
        self._rc = rc

        # --- 트리거 ---
        _try_enum(rc, "AcquisitionMode", "Continuous")
        _try_enum(rc, "TriggerSelector", "FrameStart")
        _must_enum(rc, "TriggerMode", "On")
        _must_enum(rc, "TriggerSource", "Line0")
        _must_enum(rc, "TriggerActivation", "RisingEdge")

        # --- Line1 = ExposureActive (MCU 가 노출을 세게 함) ---
        _must_enum(rc, "LineSelector", "Line1")
        # Line1 이 출력 전용인 모델에서는 LineMode 가 쓰기 불가일 수 있다.
        if not _try_enum(rc, "LineMode", "Output"):
            print(f"  [주의] {self.name}: LineMode 설정 불가 "
                  f"(출력 전용 핀이면 정상)")
        _must_enum(rc, "LineSource", "ExposureActive")

        # --- 시차 발사: 이 카메라는 FSYNC 뒤 delay 만큼 기다렸다 노출 ---
        self.trigger_delay_us = trigger_delay_us(self.slot)
        self.azimuth_deg = camera_azimuth_deg(self.slot)

        # --- 노출 / 게인 고정 ---
        # 지연 + 노출 + 센서 읽기가 트리거 주기 안에 끝나야 다음 펄스를
        # 받을 수 있다. 가장 늦게 찍는 카메라가 노출 상한을 정한다.
        limit = min(TRIGGER_PERIOD_US * EXPOSURE_SAFETY,
                    max_exposure_us(self.slot))
        if exposure_us > limit:
            raise ValueError(
                f"{self.name}: 노출 {exposure_us / 1000:.1f} ms 는 너무 깁니다. "
                f"이 카메라는 FSYNC 뒤 {self.trigger_delay_us / 1000:.0f} ms 에 "
                f"찍으므로 노출은 최대 {limit / 1000:.1f} ms 입니다 "
                f"(지연 + 노출 + 읽기 {READOUT_MARGIN_US / 1000:.0f} ms "
                f"< 주기 {TRIGGER_PERIOD_US / 1000:.0f} ms). "
                f"어두우면 게인이나 조리개로 보완하세요.")

        _must_enum(rc, "ExposureAuto", "Off")
        _must_float(rc, "ExposureTime", exposure_us)
        # 0 도 명시적으로 넣는다. 예전 세션의 값이 카메라에 남아 있을 수 있다.
        # 조용히 실패하면 시차 발사가 깨진 채로 찍히므로 반드시 _must.
        _must_float(rc, "TriggerDelay", self.trigger_delay_us)
        _must_enum(rc, "GainAuto", "Off")
        _must_float(rc, "Gain", gain_db)
        # 컬러 카메라의 자동 화이트밸런스는 프레임마다 디지털 게인을 바꾼다.
        # raw Bayer 를 저장하므로 꺼서 결정론적으로 만든다.
        _apply_quality(rc)

        self.width = _read_int(rc, "Width")
        self.height = _read_int(rc, "Height")
        self.pixel_format = _read_enum(rc, "PixelFormat")
        self.exposure_us = _read_float(rc, "ExposureTime")
        self.gain_db = _read_float(rc, "Gain")

        # --- 수신 버퍼 늘리기 ---
        ds = self._cam.data_stream[0]
        try:
            ds.set_acquisition_buffer_number(ACQ_BUFFER_NUM)
        except Exception:
            pass
        try:
            ds.StreamBufferHandlingMode.set("OldestFirst")
        except Exception:
            pass

        self._ds = ds
        self._queue = queue
        self._latest = latest
        self._thread = None
        self._stop = threading.Event()

    def stream_on(self):
        """AcquisitionStart. 트리거 모드이므로 '촬영 시작'이 아니라 '대기 시작'.
        Line0 에 신호가 오기 전까지 한 장도 찍지 않는다.
        스트림을 연 뒤에 수신 스레드를 띄운다."""
        self._cam.stream_on()
        self._thread = threading.Thread(
            target=_reader_loop,
            args=(self, self._queue, self.stats, self._latest, self._stop),
            daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            self._cam.stream_off()
        except Exception:
            pass
        try:
            self._cam.close_device()
        except Exception:
            pass

    def stream_stats(self):
        """SDK 가 세는 수신/유실 카운터."""
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
#  카메라 4대 묶음
# ===========================================================================
class CameraArray:
    def __init__(self, sns=None, exposure_us=DEFAULT_EXPOSURE_US,
                 gain_db=DEFAULT_GAIN_DB, queue_maxsize=QUEUE_MAXSIZE,
                 n_cams=0, allow_partial=False):
        self.sns = list(sns) if sns else list(CAMERA_SNS)
        if n_cams:
            self.sns = self.sns[:n_cams]
        self.allow_partial = allow_partial
        self.queue = Queue(maxsize=queue_maxsize)
        # cam_idx -> (frame_id, ndarray)  프리뷰가 읽어가는 최신 프레임
        self.latest = {}
        self.cams = []
        self.slots = []                    # cams[i] 의 물리 슬롯 번호 (1 부터)
        self._dm = None
        self._exposure_us = exposure_us
        self._gain_db = gain_db

    def __enter__(self):
        self._dm = gx.DeviceManager()
        count, dev_list = self._dm.update_device_list()
        found = {d.get("sn") for d in dev_list}

        missing = [sn for sn in self.sns if sn not in found]
        if missing and self.allow_partial:
            # 테스트 단계에서 일부 카메라만 연결된 경우. 있는 것만 쓴다.
            print(f"[주의] 연결되지 않은 카메라 {len(missing)} 대를 건너뜁니다: "
                  f"{missing}")
            self.sns = [sn for sn in self.sns if sn in found]
            missing = []
        if not self.sns:
            raise RuntimeError(
                f"사용할 카메라가 없습니다.\n"
                f"연결된 sn: {sorted(found)}\n"
                f"list_cams.py 로 확인하고 CAMERA_SNS 를 고치세요.")
        if missing:
            raise RuntimeError(
                f"다음 카메라를 찾지 못했습니다: {missing}\n"
                f"연결된 sn: {sorted(found)}\n"
                f"1대만 테스트하려면 --cams 1 옵션을 쓰거나\n"
                f"list_cams.py 로 확인하고 CAMERA_SNS 를 고치세요.")

        print(f"카메라 {len(self.sns)} 대 여는 중...")
        self.slots = []
        for i, sn in enumerate(self.sns):
            slot = (CAMERA_SNS.index(sn) + 1) if sn in CAMERA_SNS else (i + 1)
            cam = TriggerCamera(self._dm, sn, i, self.queue, self.latest,
                                self._exposure_us, self._gain_db, slot=slot)
            self.cams.append(cam)
            self.slots.append(slot)
            print(f"  {cam.name}  sn={sn}  {cam.width}x{cam.height} "
                  f"{cam.pixel_format}  exp={cam.exposure_us:.0f}us "
                  f"gain={cam.gain_db:.1f}dB")

        # --- 시차 발사 요약 + 라이다 Phase Lock 권장값 ---
        if STAGGERED_TRIGGER:
            print("시차 발사 (라이다가 지나갈 때 하나씩): " + " · ".join(
                f"{c.name} +{c.trigger_delay_us / 1000:.0f} ms"
                for c in self.cams))
        else:
            print("동시 발사 (STAGGERED_TRIGGER = False)")
        if self.cams:
            exp = self.cams[0].exposure_us
            off = recommended_phaselock_offset(exp)
            print(f"라이다 Phase Lock offset 권장 {off:.1f}° "
                  f"(노출 {exp / 1000:.1f} ms 의 중앙을 라이다 통과 순간에 맞춤)")
            print(f"  설정:  python3 lidar_config.py --phase-lock on "
                  f"--offset {off:.1f} --save --yes")

        # 전부 설정이 끝난 뒤에 한꺼번에 대기 상태로.
        for cam in self.cams:
            cam.stream_on()
        print("전 카메라 트리거 대기 상태. (신호가 오기 전까지 촬영하지 않음)\n")
        return self

    def __exit__(self, *exc):
        for cam in self.cams:
            cam.close()
        self.cams = []
        self._dm = None

    def meta(self):
        return {
            "cameras": [
                {"name": c.name, "sn": c.sn, "slot": c.slot, "index": c.idx,
                 "trigger_delay_us": c.trigger_delay_us,
                 "azimuth_deg": c.azimuth_deg,
                 "width": c.width, "height": c.height,
                 "pixel_format": c.pixel_format,
                 "exposure_us": c.exposure_us, "gain_db": c.gain_db}
                for c in self.cams
            ],
            "trigger": {"mode": "On", "source": "Line0",
                        "activation": "RisingEdge",
                        "line1_source": "ExposureActive",
                        "staggered": STAGGERED_TRIGGER,
                        "readout_margin_us": READOUT_MARGIN_US},
            "recommended_phaselock_offset_deg": (
                recommended_phaselock_offset(self.cams[0].exposure_us)
                if self.cams else None),
        }

    @property
    def n(self):
        return len(self.cams)

    def print_stats(self):
        print("\n--- 카메라별 수신 통계 ---")
        for c in self.cams:
            s = c.stats
            ss = c.stream_stats()
            print(f"  {c.name}: 큐투입 {s['queued']}  "
                  f"불완전 {s['incomplete']}  큐넘침 {s['dropped_queue']}  "
                  f"오류 {s['error']}  "
                  f"| SDK delivered={ss['delivered']} lost={ss['lost']}")
