"""세션 시간축 — 카메라 셔터와 라이다 점을 하나의 시계 위에 올린다.

merge.py / to_rosbag.py / sync_report.py 가 같은 계산을 쓰도록 모아 둔 곳이다.
세 곳이 제각각 계산하면 언젠가 값이 어긋난다.

--------------------------------------------------------------------------
세 개의 시계

  MCU      ESP32 esp_timer 마이크로초. mcu_log.txt 의 T / EXP / PPS / NMEA 숫자
  라이다    패킷에 찍힌 '정시 이후 마이크로초'. GPRMC 가 들어가면 분·초가
           MCU 가 보낸 GPRMC 문장과 같아진다 (MCU 가 12:00:00 부터 세는 자체 시계)
  호스트    노트북 UNIX 시각. frames.csv 의 host_recv_unix, pcap 수신 시각

기준은 MCU 시계다. 카메라 셔터와 라이다 방위각은 이미 이 시계로 맞물려 있다
(FSYNC 와 PPS 가 같은 타이머 인터럽트에서 나가고, Phase Lock 이 라이다 회전을
PPS 에 묶는다). 호스트 시각은 UNIX 절대 시각을 붙이는 데만 한 번 쓴다.

    카메라 셔터 (MCU) = T[N] + 카메라별 지연          N = FrameID + offset
    라이다 점   (MCU) = 그 초의 PPS 엣지 + (라이다 시각 − 그 초의 GPRMC 시각)
    UNIX             = MCU / 1e6 + 세션 앵커

상대 시각은 마이크로초, 절대 시각(UNIX)은 밀리초 수준이다. 두 센서가 같은
앵커를 쓰므로 앵커 오차는 센서 사이 정합에 영향을 주지 않는다.

--------------------------------------------------------------------------
카메라별 지연 (A 방식: 라이다가 지나갈 때 하나씩 찍기)

  카메라마다 TriggerDelay 를 줘서 라이다가 그 카메라 쪽을 지날 때 노출한다.
  실제 지연은 Line1(EXP 줄)이 카메라별로 실측해 준다 — 설정값을 믿지 않고
  실측 중앙값을 쓴다. EXP 가 없으면 meta.json 의 설정값 + 옵토 지연을 쓴다.

  '셔터 시각' 은 노출 중앙이다. 라이다 통과 순간을 노출 중앙에 맞췄기
  때문이다 (sync_cam.recommended_phaselock_offset 참고).
"""

import csv
import json
import os
import re

import numpy as np

HOUR_US = 3600 * 1_000_000
PERIOD_US = 100_000.0              # 10 Hz
LINE0_DELAY_US = 13.0              # 옵토 입력 → 노출 시작 (09-15 실측 12.8 us)
# Line1 실측으로 인정하는 범위 (Line1 − 트리거 − 설정 지연).
# 한때 '8 us 미만은 잡음' 으로 걸렀지만, CAM3(지연 0) 의 5 us 엣지가 진짜
# 노출이었다 (2026-09-22 session_008: 그 채널의 엣지가 주기당 하나뿐이었다).
# 신호 지연은 카메라·배선마다 5~13 us 로 다르다. 잡음은 이제 펌웨어가
# 기대 위상(±1.5 ms) 으로 걸러내므로, 여기서는 설정과 동떨어진 값만 막는다.
EXP_WINDOW_LO_US = -200.0
EXP_WINDOW_HI_US = 2000.0
CAM_TRANSFER_S = 0.0071            # 노출 끝 → 호스트 도착 (센서 읽기 + USB3, 실측)


# ===========================================================================
#  파싱
# ===========================================================================
def _read_meta(session_dir):
    p = os.path.join(session_dir, "meta.json")
    if not os.path.exists(p):
        return {}
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _camera_settings(meta):
    """meta.json 에서 슬롯별 {exposure_us, trigger_delay_us, azimuth_deg}."""
    out = {}
    cams = meta.get("cameras") or []
    slots_list = meta.get("camera_slots")
    for i, c in enumerate(cams):
        slot = c.get("slot")
        if slot is None:
            slot = slots_list[i] if slots_list and i < len(slots_list) else i + 1
        out[int(slot)] = {
            "exposure_us": float(c.get("exposure_us") or 0.0),
            "trigger_delay_us": float(c.get("trigger_delay_us") or 0.0),
            "azimuth_deg": c.get("azimuth_deg"),
        }
    return out


def _parse_log(path):
    t_trig, exp, pps, edges = {}, {}, [], []
    if not os.path.exists(path):
        return t_trig, exp, pps, edges
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            p = ln.strip().split(",")
            try:
                if p[0] == "T" and len(p) >= 3:
                    t_trig[int(p[1])] = int(p[2])
                elif p[0] == "EXP" and len(p) >= 4:
                    exp.setdefault(int(p[1]), []).append((int(p[2]), int(p[3])))
                elif p[0] == "PPS" and len(p) >= 3:
                    edges.append(int(p[2]))
                elif (p[0] == "NMEA" and len(p) >= 8 and p[6] == "$GPRMC"
                      and len(p[7]) >= 6):
                    mm, ss = int(p[7][2:4]), int(p[7][4:6])
                    pps.append((int(p[1]), (mm * 60 + ss) * 1_000_000))
            except (ValueError, IndexError):
                continue
    return t_trig, exp, pps, edges


def _read_frames(path):
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


def lidar_sync_ok(pos):
    """라이다가 PPS 에 잠겨 있고 GPRMC 의 분·초를 받아들였는가.

    sync_report.gprmc_check 와 같은 기준이다. 둘 중 하나라도 아니면 라이다
    타임스탬프를 MCU 시계로 옮길 수 없다 (옮겨도 초 단위로 틀린다).
    """
    if not pos:
        return False, "위치 패킷 없음"
    locked = sum(1 for p in pos if p["pps_state"] == 2)
    if locked < 0.95 * len(pos):
        return False, f"PPS Locked {locked}/{len(pos)}"
    ok = bad = 0
    for p in pos:
        m = re.match(r"\$GPRMC,(\d{2})(\d{2})(\d{2})", p["nmea"])
        if not m:
            continue
        _hh, mm, ss = map(int, m.groups())
        lid_s = int(p["lidar_us"] // 1_000_000)
        sub_ms = (p["lidar_us"] % 1_000_000) / 1000.0
        diff = (lid_s - (mm * 60 + ss) + 1800) % 3600 - 1800
        if diff == 0 or (diff == 1 and sub_ms < 200.0):
            ok += 1
        else:
            bad += 1
    if ok == 0:
        return False, "GPRMC 가 라이다에 들어가지 않음"
    if bad > 0.05 * (ok + bad):
        return False, f"GPRMC 초 불일치 {bad}/{ok + bad}"
    return True, f"PPS Locked, GPRMC 일치 {ok}/{ok + bad}"


# ===========================================================================
#  시간축
# ===========================================================================
class Timebase:
    """한 세션의 시간 변환기.

        tb = Timebase(session_dir)
        tb.cameras                      {slot: CameraTiming}
        tb.shutter_us(slot, frame_id)   노출 중앙의 MCU 시각 (없으면 None)
        tb.mcu_to_lidar_us(t)           MCU → 라이다 '정시 이후 us'
        tb.lidar_to_mcu_us(arr)         라이다 → MCU
        tb.set_anchor(...) / tb.to_unix(t)
    """

    def __init__(self, session_dir):
        self.dir = os.path.abspath(session_dir)
        self.meta = _read_meta(self.dir)
        self.t_trig, self._exp, pps, edges = _parse_log(
            os.path.join(self.dir, "mcu_log.txt"))
        # PPS 상승 엣지의 MCU 시각. GPRMC 가 라이다에 안 들어가도 이 값은 있다.
        e = sorted(edges) if edges else sorted(a for a, _ in pps)
        self.pps_edges = np.array(e, dtype=np.float64)
        settings = _camera_settings(self.meta)

        # --- PPS ↔ GPRMC 대응표 (라이다 시각 변환용) ---
        pps.sort()
        self._pe = np.array([a for a, _ in pps], dtype=np.float64)
        pl = np.array([b for _, b in pps], dtype=np.float64)
        # GPRMC 의 분·초는 한 시간마다 0 으로 돌아온다. 이어 붙인다.
        self._pl = (np.unwrap(pl, period=HOUR_US) if len(pl) > 1 else pl)

        # --- 카메라 ---
        self.cameras = {}
        for d in sorted(os.listdir(self.dir)):
            if not (d.startswith("cam") and d[3:].isdigit()):
                continue
            if not os.path.isdir(os.path.join(self.dir, d)):
                continue
            slot = int(d[3:])
            rows = _read_frames(os.path.join(self.dir, d, "frames.csv"))
            self.cameras[slot] = CameraTiming(
                slot, d, rows, self.t_trig, self._exp.get(slot, []),
                settings.get(slot, {}))

        self.anchor = None             # UNIX 초 = MCU us / 1e6 + anchor
        self.anchor_src = None

    # ---------------------------------------------------------------- 카메라
    def shutter_us(self, slot, frame_id, where="mid"):
        c = self.cameras.get(slot)
        return None if c is None else c.shutter_us(frame_id, where)

    # ---------------------------------------------------------------- 라이다
    @property
    def has_pps_map(self):
        return len(self._pe) >= 1

    def mcu_to_lidar_us(self, t_mcu):
        """MCU 시각 → 라이다 '정시 이후 us' (0 ~ 3600e6). 스칼라 또는 배열."""
        t = np.asarray(t_mcu, dtype=np.float64)
        i = np.clip(np.searchsorted(self._pe, t, side="right") - 1,
                    0, len(self._pe) - 1)
        return np.mod(self._pl[i] + (t - self._pe[i]), HOUR_US)

    def lidar_to_mcu_us(self, t_lidar):
        """라이다 '정시 이후 us' 배열 → MCU 시각 배열.

        세션이 정각을 넘기면 라이다 값이 0 으로 돌아가므로 먼저 이어 붙이고,
        GPRMC 대응표와 같은 시간대(hour)로 맞춘 뒤 해당 초의 PPS 로 옮긴다.
        대응표 앞뒤로 벗어난 값은 가장 가까운 초에서 외삽한다 (라이다가 PPS 에
        잠겨 있으므로 1초당 정확히 1초씩 간다).
        """
        t = np.asarray(t_lidar, dtype=np.float64)
        if t.size == 0:
            return t
        if t.size > 1:
            t = np.unwrap(t, period=HOUR_US)
        t = t + np.round((np.median(self._pl) - np.median(t)) / HOUR_US) * HOUR_US
        i = np.clip(np.searchsorted(self._pl, t, side="right") - 1,
                    0, len(self._pl) - 1)
        return self._pe[i] + (t - self._pl[i])

    def phase_in_rotation_us(self, t_mcu):
        """MCU 시각이 PPS 기준 100 ms 주기 안에서 몇 us 위치인가.

        PPS 가 잠기면 라이다 '정시 이후 us' 의 초 경계가 PPS 에 맞춰진다.
        GPRMC 는 '몇 분 몇 초' 이름표만 붙인다. 그래서 GPRMC 가 없어도
        100 ms 주기 안의 위치(회전 위상)는 MCU 와 라이다가 같은 기준이다.
        """
        t = np.asarray(t_mcu, dtype=np.float64)
        i = np.clip(np.searchsorted(self.pps_edges, t, side="right") - 1,
                    0, len(self.pps_edges) - 1)
        return np.mod(t - self.pps_edges[i], PERIOD_US)

    # ---------------------------------------------------------------- UNIX
    def anchor_from_lidar(self, pkt_ts_us, pkt_host_ts):
        """라이다 패킷으로 앵커를 잡는다 (가장 정확).

        패킷은 보낸 뒤에 도착하므로 '호스트 − MCU' 는 늘 참값보다 조금 크다.
        그래서 아래쪽 끝(0.5 백분위)을 쓴다. 이더넷 지연은 1 ms 미만이다.
        """
        mcu = self.lidar_to_mcu_us(pkt_ts_us)
        diff = np.asarray(pkt_host_ts, dtype=np.float64) - mcu / 1e6
        return float(np.percentile(diff, 0.5))

    def anchor_from_cameras(self):
        """카메라 프레임으로 앵커를 잡는다 (라이다가 없을 때).

        호스트 도착 = 노출 시작 + 노출 + 센서 읽기·USB 전송(실측 7.1 ms).
        USB 지터가 있어 중앙값을 쓴다. 절대 시각이 수 ms 틀릴 수 있다.
        """
        diffs = []
        for c in self.cameras.values():
            exp_s = c.exposure_us / 1e6
            for fid, _ts, recv in c.rows:
                t0 = c.shutter_us(fid, "start")
                if t0 is not None:
                    diffs.append(recv - t0 / 1e6 - exp_s - CAM_TRANSFER_S)
        if not diffs:
            return None
        return float(np.median(diffs))

    def set_anchor(self, decoded=None, lidar_ok=False):
        """앵커를 정한다. 라이다가 동기 상태면 라이다로, 아니면 카메라로."""
        cam_a = self.anchor_from_cameras()
        lid_a = None
        if decoded is not None and lidar_ok and self.has_pps_map \
                and len(decoded.pkt_ts_us):
            lid_a = self.anchor_from_lidar(decoded.pkt_ts_us, decoded.pkt_host_ts)

        if lid_a is not None:
            self.anchor, self.anchor_src = lid_a, "라이다 패킷"
            # 교차 검증 — 두 앵커가 크게 다르면 대응표가 초 단위로 틀린 것
            if cam_a is not None and abs(lid_a - cam_a) > 0.05:
                self.anchor_src += (f" [경고: 카메라 기준과 {1000*(lid_a-cam_a):+.0f} ms "
                                    f"차이. GPRMC 대응이 의심됨]")
        elif cam_a is not None:
            self.anchor, self.anchor_src = cam_a, "카메라 프레임"
        return self.anchor

    def to_unix(self, t_mcu):
        if self.anchor is None:
            raise RuntimeError("앵커가 없습니다. set_anchor() 를 먼저 부르세요.")
        return np.asarray(t_mcu, dtype=np.float64) / 1e6 + self.anchor


def host_delay_estimates(tb):
    """노트북 도착 시각으로 카메라별 지연을 따로 추정한다 (Line1 과 무관).

    같은 펄스 N 의 사진들이 노트북에 도착한 시각 차이의 중앙값이다. 카메라
    설정이 같으면 노출·전송 지연이 같으므로, 도착 시각 차이가 곧 셔터 시각
    차이다. USB 흔들림(±수 ms)이 있지만 수백 장의 중앙값이라 0.1 ms 수준으로
    맞는다. Line1 신호가 오염돼도 'TriggerDelay 가 정말 먹었나' 를 확인할 수
    있다.

    반환: {slot: 추정 지연 us} — 설정 지연이 가장 작은 카메라를 기준으로 한다.
    """
    cams = tb.cameras
    if len(cams) < 2:
        return {}
    ref = min(cams, key=lambda s: (cams[s].delay_set_us, s))
    ref_by_n = {cams[ref].pulse_of(fid): recv for fid, _t, recv in cams[ref].rows}
    out = {ref: cams[ref].delay_set_us}
    for s, c in cams.items():
        if s == ref:
            continue
        diffs = [recv - ref_by_n[c.pulse_of(fid)]
                 for fid, _t, recv in c.rows if c.pulse_of(fid) in ref_by_n]
        if diffs:
            out[s] = cams[ref].delay_set_us + float(np.median(diffs)) * 1e6
    return out


class CameraTiming:
    """카메라 한 대의 offset 과 지연."""

    def __init__(self, slot, dirname, rows, t_trig, exp, setting):
        self.slot = slot
        self.dirname = dirname
        self.rows = rows
        self.t_trig = t_trig
        self.exposure_us = float(setting.get("exposure_us") or 0.0)
        self.delay_set_us = float(setting.get("trigger_delay_us") or 0.0)
        az = setting.get("azimuth_deg")
        self.azimuth_deg = float(az) if az is not None else 90.0 * (slot - 1)

        # --- offset: 첫 EXP 의 펄스 번호 − 첫 FrameID ---
        fids = [r[0] for r in rows]
        if exp and fids:
            self.offset = exp[0][0] - fids[0]
            self.offset_src = "EXP"
        elif fids:
            self.offset = 1 - fids[0]           # 첫 프레임 = 1번 펄스로 가정
            self.offset_src = "가정"
        else:
            self.offset, self.offset_src = None, "없음"

        # --- 지연: EXP 실측 중앙값. 단, 설정값과 맞는 표본만 쓴다 ---
        # Line1 입력에 FSYNC 순간의 잡음 엣지가 들어오면, 펌웨어가 그걸 노출로
        # 세고 50 ms 락아웃을 거는 바람에 진짜 노출(예: 25 ms 뒤)이 가려진다.
        # 그러면 개수(CNT)는 200 으로 멀쩡한데 시각만 0 ms 근처로 찍힌다
        # (2026-09-21 session_004 CAM1 실측: 설정 25 ms, EXP 0.003 ms).
        # 이 값을 믿으면 그 카메라 사진 시각이 25 ms 통째로 틀린다.
        #
        # 카메라 TriggerDelay 자체는 믿을 만하므로(호스트 도착 시각으로 교차
        # 확인됨), 설정값 + 8 us ~ +2 ms 안의 표본만 실측으로 인정하고, 절반
        # 넘게 벗어나면 설정값 + 옵토 지연을 쓴다.
        d_all = [us - t_trig[n] for n, us in exp if n in t_trig]
        d_ok = [x for x in d_all
                if EXP_WINDOW_LO_US <= x - self.delay_set_us <= EXP_WINDOW_HI_US]
        self.delay_measured_us = float(np.median(d_all)) if d_all else None
        self.exp_suspect = False
        self.exp_reason = None           # "빠름" | "가림"
        if d_ok and len(d_ok) * 2 >= len(d_all):
            self.delay_us = float(np.median(d_ok))
            self.delay_src = f"실측 n={len(d_ok)}"
            if len(d_ok) < len(d_all):
                self.delay_src += f" (이상치 {len(d_all) - len(d_ok)} 제외)"
            self.delay_spread_us = float(np.max(d_ok) - np.min(d_ok))
        else:
            self.delay_us = self.delay_set_us + LINE0_DELAY_US
            self.delay_spread_us = None
            if d_all:
                self.exp_suspect = True
                over = self.delay_measured_us - self.delay_set_us
                self.exp_reason = "빠름" if 0 <= over < EXP_WINDOW_LO_US else "가림"
                self.delay_src = "설정값 (Line1 실측이 오염 — 잡음 엣지)"
            else:
                self.delay_src = "설정값"

    def pulse_of(self, frame_id):
        return None if self.offset is None else frame_id + self.offset

    def shutter_us(self, frame_id, where="mid"):
        n = self.pulse_of(frame_id)
        if n is None or n not in self.t_trig:
            return None
        t = self.t_trig[n] + self.delay_us
        if where == "mid":
            t += self.exposure_us / 2.0
        return t
