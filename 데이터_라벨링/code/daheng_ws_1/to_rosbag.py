"""세션 폴더 → rosbag2 (.db3).

    source /opt/ros/humble/setup.bash
    python3 to_rosbag.py sessions/session_001
    python3 to_rosbag.py sessions/session_001 -o ~/ros2_ws/cam_lidar_recording
    python3 to_rosbag.py sessions/session_001 --debayer      # 바로 보이는 BGR 로

--------------------------------------------------------------------------
왜 수집과 변환을 나누는가

  수집 중에는 파싱도 계산도 하지 않는다. 변환기에서 예외가 나도 원본
  pcap 과 npy 는 그대로 남아 다시 돌리면 된다. 수집 루프 안에서 메시지를
  만들었다면 그 예외 하나에 세션이 통째로 날아간다.

--------------------------------------------------------------------------
타임스탬프 — 기본은 MCU 시계 기준 (정밀)

  카메라와 라이다를 ESP32 시계 하나 위에 올린 뒤 UNIX 로 바꿔 넣는다.
  계산은 timebase.py 에 있고 merge.py · sync_report.py 와 같은 값이다.

  카메라 : 노출 중앙 = T[N] + 카메라별 지연(Line1 실측) + 노출/2
           N = FrameID + offset.  시차 발사면 카메라마다 0/25/50/75 ms 다르다
  라이다 : 회전 첫 점의 라이다 시각 → PPS·GPRMC 대응표로 MCU 시각
           점별 'time' 필드는 그 회전 시작 기준 상대 시각(초)

  상대 시각은 us, 절대 시각(UNIX)은 ms 수준이다. 두 센서가 같은 앵커를
  쓰므로 앵커 오차가 센서 사이 정합을 흔들지 않는다.

  아래 경우는 호스트 수신 시각으로 내려간다 (수 ms 오차, 경고 출력).
    - mcu_log.txt 에 T 줄이 없다 (프리런 · 라이다 전용 세션)
    - 라이다가 PPS 에 잠기지 않았거나 GPRMC 를 받아들이지 않았다 → 라이다만
    - --host-time 을 줬다 (예전 방식 그대로)

--------------------------------------------------------------------------
토픽

  /velodyne_points        sensor_msgs/PointCloud2   회전당 1개, 10 Hz
  /cam<N>/image_raw       sensor_msgs/Image         트리거당 1개, 10 Hz
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import vlp16  # noqa: E402
from timebase import Timebase, lidar_sync_ok  # noqa: E402

try:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField, Image
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time as TimeMsg
except ImportError as e:
    raise SystemExit(
        f"ROS2 모듈을 불러오지 못했습니다: {e}\n"
        f"  source /opt/ros/humble/setup.bash 를 먼저 실행하세요.")

LIDAR_TOPIC = "/velodyne_points"
LIDAR_FRAME = "velodyne"

# Daheng PixelFormat -> ROS image encoding.
# Bayer 이름은 두 규격이 같은 배열을 다르게 부른다. Daheng 의 BayerRG8 은
# 첫 픽셀이 R, 그 오른쪽이 G 라는 뜻이고 ROS 에서는 그것을 bayer_rggb8 이라 쓴다.
BAYER_TO_ROS = {
    "BayerRG8": "bayer_rggb8",
    "BayerGB8": "bayer_gbrg8",
    "BayerGR8": "bayer_grbg8",
    "BayerBG8": "bayer_bggr8",
    "Mono8": "mono8",
}
# OpenCV 디베이어 코드 (--debayer 용)
# ※ OpenCV 의 Bayer 이름은 카메라(GenICam) 이름과 R/B 가 뒤바뀌어 있다.
#   카메라 BayerGB(0,0 이 G, 옆이 B) 에는 cv2.COLOR_BayerGR2BGR 가 맞다.
#   이름 그대로 BayerGB2BGR 를 쓰면 빨강이 파랑으로 나온다 (2026-09-22 실험 확인).
BAYER_TO_CV = {
    "BayerRG8": "COLOR_BayerBG2BGR",
    "BayerGB8": "COLOR_BayerGR2BGR",
    "BayerGR8": "COLOR_BayerGB2BGR",
    "BayerBG8": "COLOR_BayerRG2BGR",
}

# PointCloud2 레이아웃. velodyne_pointcloud 의 PointXYZIRT 와 같은 구성이다.
_POINT_DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                      ("intensity", "<f4"), ("ring", "<u2"),
                      ("time", "<f4")])          # itemsize 22, 패딩 없음
_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
]


def _stamp(unix_s):
    sec = int(unix_s)
    return TimeMsg(sec=sec, nanosec=int(round((unix_s - sec) * 1e9)))


def _ns(unix_s):
    return int(round(unix_s * 1e9))


# ===========================================================================
#  메시지 만들기
# ===========================================================================
def make_cloud(d, a, b, frame_id=LIDAR_FRAME, stamp_s=None):
    # 회전 경계는 numpy int64 로 들어온다. 그대로 넘기면 메시지 필드가
    # 파이썬 int 가 아니라며 거절한다.
    a, b = int(a), int(b)
    n = b - a
    arr = np.empty(n, dtype=_POINT_DT)
    arr["x"] = d.xyz[a:b, 0]
    arr["y"] = d.xyz[a:b, 1]
    arr["z"] = d.xyz[a:b, 2]
    arr["intensity"] = d.intensity[a:b]
    arr["ring"] = d.ring[a:b]
    arr["time"] = d.time[a:b]

    stamp = _stamp(float(d.host_ts[a]) if stamp_s is None else stamp_s)
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = n
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_DT.itemsize
    msg.row_step = _POINT_DT.itemsize * n
    msg.data = arr.tobytes()
    msg.is_dense = True          # 거리 0 은 디코더가 이미 걸러냈다
    return msg


def make_image(arr, unix_ts, encoding, frame_id):
    msg = Image()
    msg.header = Header(stamp=_stamp(unix_ts), frame_id=frame_id)
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(arr.strides[0])
    msg.data = arr.tobytes()
    return msg


# ===========================================================================
#  변환
# ===========================================================================
def convert(session_dir, out_uri, debayer=False, storage_id="sqlite3",
            lidar_only=False, quiet=False, host_time=False, stamp="mid"):
    session_dir = os.path.abspath(session_dir)
    if not os.path.isdir(session_dir):
        raise SystemExit(f"세션 폴더가 없습니다: {session_dir}")

    meta = {}
    meta_path = os.path.join(session_dir, "meta.json")
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as f:
            meta = json.load(f)

    if os.path.exists(out_uri):
        raise SystemExit(
            f"출력 폴더가 이미 있습니다: {out_uri}\n"
            f"  rosbag2 는 기존 폴더에 덮어쓰지 않습니다. 지우거나 -o 로 다른\n"
            f"  이름을 주세요:  rm -rf {out_uri}")

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))

    def log(*a):
        if not quiet:
            print(*a)

    # ------------------------------------------------------------- 시간축
    tb = Timebase(session_dir)
    pcap = os.path.join(session_dir, "lidar.pcap")
    tb_mode = (not host_time) and bool(tb.t_trig)
    lidar_precise = False
    decoded_for_anchor = None
    if tb_mode:
        if os.path.exists(pcap):
            ok, why = lidar_sync_ok(vlp16.read_position_packets(pcap))
            lidar_precise = ok and tb.has_pps_map
            if lidar_precise:
                decoded_for_anchor = vlp16.decode(pcap)
            else:
                log(f"[주의] 라이다 동기 확인 실패 ({why}) — "
                    f"라이다만 호스트 수신 시각으로 넣습니다.")
        tb.set_anchor(decoded=decoded_for_anchor, lidar_ok=lidar_precise)
        if tb.anchor is None:
            log("[주의] UNIX 앵커를 잡지 못해 호스트 수신 시각으로 넣습니다.")
            tb_mode = False
    if tb_mode:
        log(f"스탬프  MCU 시계 기준 (카메라 노출 {'중앙' if stamp == 'mid' else '시작'}), "
            f"앵커: {tb.anchor_src}")
        if not lidar_precise and os.path.exists(pcap):
            log("        ※ 라이다는 호스트 시각 — 카메라와 수 ms 어긋날 수 있음")
    else:
        why = "--host-time" if host_time else "mcu_log.txt 에 T 줄 없음"
        log(f"스탬프  호스트 수신 시각 ({why}) — 수 ms 수준 오차")

    # ------------------------------------------------------------- 라이다
    n_clouds = 0
    if os.path.exists(pcap):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=LIDAR_TOPIC, type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr"))
        log(f"라이다  {os.path.basename(pcap)} 디코딩 중...")
        d = decoded_for_anchor if decoded_for_anchor is not None \
            else vlp16.decode(pcap)
        revs = list(d.revolutions())
        # 첫 회전과 마지막 회전은 대개 잘려 있다. 온전한 것만 넣는다.
        full = [(a, b) for a, b in revs if b - a > 0]
        if len(full) >= 3:
            full = full[1:-1]

        # 회전 시작 시각 — 정밀 모드면 라이다 시각을 MCU 로 옮겨 UNIX 로
        starts = np.array([float(d.packet_ts_us[a]) for a, _ in full])
        if tb_mode and lidar_precise and len(starts):
            rev_unix = tb.to_unix(tb.lidar_to_mcu_us(starts))
        else:
            rev_unix = np.array([float(d.host_ts[a]) for a, _ in full])
        for (a, b), ts in zip(full, rev_unix):
            writer.write(LIDAR_TOPIC,
                         serialize_message(make_cloud(d, a, b, stamp_s=float(ts))),
                         _ns(float(ts)))
            n_clouds += 1
        log(f"        {d.model}/{d.return_mode}  패킷 {d.n_packets}  "
            f"점 {len(d):,}  회전 {n_clouds} 개 기록 "
            f"(앞뒤 잘린 회전 {len(revs) - n_clouds} 개 제외)")

        pos = vlp16.read_position_packets(pcap)
        if pos:
            states = sorted({p["pps_state"] for p in pos})
            log("        PPS: " + ", ".join(
                vlp16.PPS_STATES.get(s, str(s)) for s in states))
    else:
        log(f"[주의] {pcap} 이 없습니다 — 라이다 없이 진행합니다.")

    # ------------------------------------------------------------- 카메라
    n_images = 0
    if not lidar_only:
        cv2 = None
        if debayer:
            import cv2 as _cv2
            cv2 = _cv2

        pixfmt_by_slot = {}
        for i, c in enumerate(meta.get("cameras", [])):
            slot = (meta.get("camera_slots") or [i + 1])[i] \
                if meta.get("camera_slots") else i + 1
            pixfmt_by_slot[slot] = c.get("pixel_format")

        cam_dirs = sorted(d for d in os.listdir(session_dir)
                          if d.startswith("cam") and
                          os.path.isdir(os.path.join(session_dir, d)))
        for cam in cam_dirs:
            cdir = os.path.join(session_dir, cam)
            csv_path = os.path.join(cdir, "frames.csv")
            if not os.path.exists(csv_path):
                log(f"[주의] {cam}/frames.csv 가 없습니다 — 건너뜁니다.")
                continue

            try:
                slot = int(cam[3:])
            except ValueError:
                slot = None
            pixfmt = pixfmt_by_slot.get(slot) or "BayerRG8"
            base_enc = BAYER_TO_ROS.get(pixfmt)
            if base_enc is None:
                log(f"[주의] {cam}: 모르는 PixelFormat '{pixfmt}' — "
                    f"bayer_rggb8 로 가정합니다.")
                base_enc = "bayer_rggb8"
            if debayer and pixfmt in BAYER_TO_CV:
                code = getattr(cv2, BAYER_TO_CV[pixfmt])
                encoding = "bgr8"
            else:
                code, encoding = None, base_enc

            topic = f"/{cam}/image_raw"
            frame_id = cam
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type="sensor_msgs/msg/Image",
                serialization_format="cdr"))

            with open(csv_path, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))

            written = missing = 0
            fallback = 0
            ct = tb.cameras.get(slot) if tb_mode else None
            for row in rows:
                fid = int(row["frame_id"])
                ts = float(row["host_recv_unix"])
                if ct is not None:
                    t = ct.shutter_us(fid, stamp)
                    if t is not None:
                        ts = float(tb.to_unix(t))
                    else:
                        fallback += 1       # 그 펄스의 T 줄이 없음
                npy = os.path.join(cdir, f"{fid:08d}.npy")
                png = os.path.join(cdir, f"{fid:08d}.png")
                if os.path.exists(npy):
                    arr = np.load(npy)
                elif os.path.exists(png):
                    arr = cv2.imread(png, cv2.IMREAD_UNCHANGED) if cv2 else None
                    if arr is None:
                        import cv2 as _c
                        arr = _c.imread(png, _c.IMREAD_UNCHANGED)
                else:
                    missing += 1
                    continue
                if code is not None and arr.ndim == 2:
                    arr = cv2.cvtColor(arr, code)
                arr = np.ascontiguousarray(arr)
                writer.write(topic, serialize_message(
                    make_image(arr, ts, encoding, frame_id)), _ns(ts))
                written += 1

            n_images += written
            extra = ""
            if ct is not None:
                extra = (f"  지연 {ct.delay_us / 1000:.3f} ms ({ct.delay_src}), "
                         f"offset {ct.offset}")
                if fallback:
                    extra += f"  [호스트 시각 대체 {fallback}]"
            log(f"카메라  {cam}  {written} 장  ({pixfmt} → {encoding}){extra}"
                + (f"  [파일 없음 {missing}]" if missing else ""))

    del writer          # 닫으면서 metadata.yaml 을 쓴다
    return {"clouds": n_clouds, "images": n_images, "uri": out_uri}


def main():
    ap = argparse.ArgumentParser(description="세션 폴더 → rosbag2")
    ap.add_argument("session", help="session_NNN 폴더")
    ap.add_argument("-o", "--out", default=None,
                    help="출력 rosbag 폴더 (기본: <session>/rosbag)")
    ap.add_argument("--debayer", action="store_true",
                    help="Bayer 를 BGR 로 변환해서 넣는다 (용량 3배, 바로 보임)")
    ap.add_argument("--lidar-only", action="store_true")
    ap.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"])
    ap.add_argument("--host-time", action="store_true",
                    help="예전 방식: 호스트 수신 시각을 스탬프로 (수 ms 오차)")
    ap.add_argument("--stamp", choices=["mid", "start"], default="mid",
                    help="카메라 스탬프를 노출 중앙(기본) 또는 시작에")
    args = ap.parse_args()

    out = args.out or os.path.join(os.path.abspath(args.session), "rosbag")
    r = convert(args.session, os.path.abspath(out), debayer=args.debayer,
                storage_id=args.storage, lidar_only=args.lidar_only,
                host_time=args.host_time, stamp=args.stamp)

    print(f"\n완료: {r['uri']}")
    print(f"  PointCloud2 {r['clouds']} 개, Image {r['images']} 장")
    print(f"\n확인:")
    print(f"  ros2 bag info {r['uri']}")
    print(f"  ros2 bag play {r['uri']}")
    print(f"  rviz2   # Fixed Frame 을 '{LIDAR_FRAME}' 로 두고 "
          f"{LIDAR_TOPIC} 추가")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
