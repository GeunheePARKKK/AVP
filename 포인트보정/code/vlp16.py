"""VLP-16 pcap → 포인트클라우드 디코더.

ros-humble-velodyne 을 설치하지 않고도 pcap 을 풀 수 있게 직접 구현했다.
apt 설치에는 sudo 가 필요한데, 이 장비에서 그것 하나 때문에 수집 파이프라인
전체를 관리자 권한에 묶고 싶지 않았다. 계산 자체는 VLP-16 매뉴얼의
공식 그대로다.

--------------------------------------------------------------------------
패킷 구조 (1206 B)

    데이터 블록 12개 × 100 B          = 1200
        flag      uint16   0xEEFF (리틀엔디언으로 읽으면 0xEEFF)
        azimuth   uint16   0.01도 단위, 블록의 첫 발사 시퀀스 기준
        채널 32개 × 3 B    거리 uint16 (2 mm 단위) + 반사강도 uint8
                           앞 16개 = 시퀀스 0, 뒤 16개 = 시퀀스 1
    timestamp     uint32   정시 이후 마이크로초       = 1200..1203
    return mode   uint8    0x37 Strongest / 0x38 Last / 0x39 Dual
    model         uint8    0x22 = VLP-16

--------------------------------------------------------------------------
시간과 방위각 보간

  발사 주기 55.296 us 안에서 채널이 2.304 us 간격으로 순서대로 발사된다.
  블록 하나는 시퀀스 2개이므로 블록과 블록 사이는 110.592 us.
  블록에 적힌 방위각은 그 블록 첫 발사의 값이라, 나머지 31개는
  다음 블록의 방위각까지를 시간 비율로 나눠 보간해야 한다.
  이 보간을 생략하면 회전 중에 찍힌 점들이 한 방위각에 뭉쳐서
  10 Hz 에서 최대 0.2도, 거리 10 m 에서 약 3.5 cm 어긋난다.

--------------------------------------------------------------------------
좌표계

  매뉴얼 좌표 (y 가 전방) 를 velodyne_pointcloud 와 같은 ROS 관례
  (x 전방, y 좌측, z 상방) 로 바꿔서 내보낸다.
      x =  R cos(w) cos(a)
      y = -R cos(w) sin(a)
      z =  R sin(w)
  raw=True 로 부르면 매뉴얼 좌표 그대로 준다.

--------------------------------------------------------------------------
단독 실행

    python3 vlp16.py lidar.pcap              요약
    python3 vlp16.py lidar.pcap --dump 3     앞쪽 회전 3개를 npy 로 저장
"""

import argparse
import os
import struct

import numpy as np

DATA_PORT = 2368
POSITION_PORT = 8308
PACKET_BYTES = 1206
BLOCKS_PER_PACKET = 12
SEQS_PER_BLOCK = 2
CHANNELS = 16

DISTANCE_RESOLUTION_M = 0.002      # 2 mm
AZIMUTH_SCALE = 0.01               # 0.01 도
FIRING_INTERVAL_US = 2.304         # 채널 사이
SEQUENCE_INTERVAL_US = 55.296      # 시퀀스 사이 (= 16*2.304 + 18.432 재충전)
BLOCK_INTERVAL_US = SEQUENCE_INTERVAL_US * SEQS_PER_BLOCK   # 110.592

# 발사 순서대로의 수직각. 인덱스가 곧 ring 번호이며 velodyne_pointcloud 와 같다.
VERTICAL_ANGLES_DEG = np.array(
    [-15., 1., -13., 3., -11., 5., -9., 7.,
     -7., 9., -5., 11., -3., 13., -1., 15.], dtype=np.float64)

RETURN_MODES = {0x37: "Strongest", 0x38: "Last", 0x39: "Dual"}
MODELS = {0x21: "HDL-32E", 0x22: "VLP-16"}

_CHAN_DT = np.dtype([("dist", "<u2"), ("refl", "u1")])                  # 3 B
_BLOCK_DT = np.dtype([("flag", "<u2"), ("azimuth", "<u2"),
                      ("chan", _CHAN_DT, (32,))])                       # 100 B
_PACKET_DT = np.dtype([("blocks", _BLOCK_DT, (BLOCKS_PER_PACKET,)),
                       ("ts", "<u4"), ("mode", "u1"), ("model", "u1")])  # 1206 B


# ===========================================================================
#  pcap 읽기
# ===========================================================================
def read_pcap(path, port=DATA_PORT):
    """pcap 에서 지정 포트의 UDP 페이로드를 뽑는다.

    반환: (host_ts float64 (N,), payload bytes 이어붙인 buffer, N)
    이더넷/IP/UDP 헤더는 여기서 벗겨낸다. IHL 이 5가 아닌 경우도 처리한다.
    """
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise ValueError(f"{path}: pcap 헤더가 너무 짧습니다.")
        magic = struct.unpack("<I", gh[:4])[0]
        if magic == 0xA1B2C3D4:
            endian, nano = "<", False
        elif magic == 0xD4C3B2A1:
            endian, nano = ">", False
        elif magic == 0xA1B23C4D:
            endian, nano = "<", True
        elif magic == 0x4D3CB2A1:
            endian, nano = ">", True
        else:
            raise ValueError(f"{path}: pcap 매직이 아닙니다 (0x{magic:08x}). "
                             f"pcapng 라면 'editcap -F pcap' 으로 변환하세요.")
        linktype = struct.unpack(endian + "I", gh[20:24])[0]
        if linktype != 1:
            raise ValueError(f"{path}: linktype {linktype} — 이더넷(1)이 아닙니다.")

        rec_fmt = endian + "IIII"
        times, chunks = [], []
        blob = f.read()

    off, n = 0, len(blob)
    div = 1e9 if nano else 1e6
    while off + 16 <= n:
        ts_s, ts_frac, incl, _orig = struct.unpack_from(rec_fmt, blob, off)
        off += 16
        if off + incl > n:
            break                       # 잘린 마지막 레코드
        frame = blob[off:off + incl]
        off += incl

        if len(frame) < 42 or frame[12:14] != b"\x08\x00":
            continue                    # IPv4 가 아님
        ihl = (frame[14] & 0x0F) * 4
        if frame[23] != 17:             # UDP 아님
            continue
        u = 14 + ihl
        if len(frame) < u + 8:
            continue
        dport = struct.unpack_from("!H", frame, u + 2)[0]
        if dport != port:
            continue
        payload = frame[u + 8:]
        if port == DATA_PORT and len(payload) != PACKET_BYTES:
            continue
        times.append(ts_s + ts_frac / div)
        chunks.append(payload)

    if not chunks:
        return np.zeros(0), b"", 0
    return np.asarray(times, dtype=np.float64), b"".join(chunks), len(chunks)


# ===========================================================================
#  디코딩
# ===========================================================================
class Decoded:
    """한 pcap 을 통째로 푼 결과. 모든 배열의 길이는 점 개수와 같다."""

    __slots__ = ("xyz", "intensity", "ring", "time", "azimuth",
                 "packet_ts_us", "host_ts", "n_packets", "return_mode",
                 "model", "rev_starts", "pkt_ts_us", "pkt_host_ts")

    def __len__(self):
        return 0 if self.xyz is None else len(self.xyz)

    def revolutions(self):
        """회전 단위로 (start, stop) 인덱스를 순서대로 내놓는다."""
        b = self.rev_starts
        for i in range(len(b) - 1):
            yield b[i], b[i + 1]


def decode(path_or_buffer, host_times=None, n_packets=None, raw=False,
           drop_zero=True):
    """pcap 경로 또는 (buffer, host_times) 를 받아 Decoded 를 돌려준다."""
    if isinstance(path_or_buffer, (str, os.PathLike)):
        host_times, buf, n_packets = read_pcap(path_or_buffer, DATA_PORT)
    else:
        buf = path_or_buffer

    out = Decoded()
    out.n_packets = n_packets or 0
    if not n_packets:
        out.xyz = np.zeros((0, 3), np.float32)
        out.intensity = np.zeros(0, np.float32)
        out.ring = np.zeros(0, np.uint16)
        out.time = np.zeros(0, np.float32)
        out.azimuth = np.zeros(0, np.float32)
        out.packet_ts_us = np.zeros(0, np.int64)
        out.host_ts = np.zeros(0, np.float64)
        out.return_mode = out.model = None
        out.rev_starts = np.zeros(1, np.int64)
        out.pkt_ts_us = np.zeros(0, np.int64)
        out.pkt_host_ts = np.zeros(0, np.float64)
        return out

    pk = np.frombuffer(buf, dtype=_PACKET_DT, count=n_packets)

    modes = np.unique(pk["mode"])
    out.return_mode = RETURN_MODES.get(int(modes[0]), f"0x{int(modes[0]):02x}")
    out.model = MODELS.get(int(pk["model"][0]), f"0x{int(pk['model'][0]):02x}")
    if out.return_mode == "Dual":
        raise NotImplementedError(
            "Dual return 모드 pcap 입니다. 이 디코더는 단일 반사만 다룹니다.")

    az = pk["blocks"]["azimuth"].astype(np.float64)         # (N,12) 0.01도
    dist = pk["blocks"]["chan"]["dist"].astype(np.float32)  # (N,12,32)
    refl = pk["blocks"]["chan"]["refl"].astype(np.float32)

    N = az.shape[0]
    # --- 블록 사이 방위각 증가분 (36000 에서 되돌아가는 것을 처리) ---
    gap = np.empty_like(az)
    gap[:, :-1] = (az[:, 1:] - az[:, :-1]) % 36000.0
    gap[:, -1] = gap[:, -2]      # 마지막 블록은 직전 증가분을 그대로 쓴다

    # --- 시퀀스/채널별 보간 비율과 시간 오프셋 ---
    s_idx = np.arange(SEQS_PER_BLOCK, dtype=np.float64).reshape(1, 1, 2, 1)
    c_idx = np.arange(CHANNELS, dtype=np.float64).reshape(1, 1, 1, 16)
    within_block_us = SEQUENCE_INTERVAL_US * s_idx + FIRING_INTERVAL_US * c_idx
    frac = within_block_us / BLOCK_INTERVAL_US                    # (1,1,2,16)

    az4 = az[:, :, None, None] + gap[:, :, None, None] * frac     # (N,12,2,16)
    az4 = np.mod(az4, 36000.0)
    az_rad = np.deg2rad(az4 * AZIMUTH_SCALE)

    b_idx = np.arange(BLOCKS_PER_PACKET, dtype=np.float64).reshape(1, 12, 1, 1)
    t_us = BLOCK_INTERVAL_US * b_idx + within_block_us            # (1,12,2,16)

    d4 = dist.reshape(N, BLOCKS_PER_PACKET, SEQS_PER_BLOCK, CHANNELS)
    r4 = refl.reshape(N, BLOCKS_PER_PACKET, SEQS_PER_BLOCK, CHANNELS)
    R = d4 * DISTANCE_RESOLUTION_M

    w = np.deg2rad(VERTICAL_ANGLES_DEG).reshape(1, 1, 1, 16)
    cos_w, sin_w = np.cos(w), np.sin(w)
    xy = R * cos_w
    if raw:
        x = xy * np.sin(az_rad)
        y = xy * np.cos(az_rad)
    else:
        x = xy * np.cos(az_rad)
        y = -xy * np.sin(az_rad)
    z = R * sin_w

    ring = np.broadcast_to(np.arange(CHANNELS, dtype=np.uint16)
                           .reshape(1, 1, 1, 16), d4.shape)

    # --- 평탄화. C 순서가 곧 발사 시간 순서다 ---
    flat = (N * BLOCKS_PER_PACKET * SEQS_PER_BLOCK * CHANNELS,)
    x = x.reshape(flat).astype(np.float32)
    y = y.reshape(flat).astype(np.float32)
    z = z.reshape(flat).astype(np.float32)
    inten = r4.reshape(flat)
    ring = ring.reshape(flat).copy()
    az_flat = (az4.reshape(flat) * AZIMUTH_SCALE).astype(np.float32)
    d_flat = d4.reshape(flat)

    # 패킷 타임스탬프(정시 이후 us) + 패킷 안에서의 오프셋
    ts_pk = pk["ts"].astype(np.int64)
    t_abs = (ts_pk[:, None, None, None].astype(np.float64)
             + t_us).reshape(flat)

    host = np.repeat(host_times, BLOCKS_PER_PACKET * SEQS_PER_BLOCK * CHANNELS)

    # 패킷 단위 시각은 따로 보관한다. 아래에서 거리 0 인 점을 걸러내고 나면
    # 점 배열의 길이가 패킷 수의 384 배가 아니게 되어, packet_ts_us[::384] 로는
    # 더 이상 '패킷마다 하나' 를 뽑을 수 없다. 무반사 점이 많은 장면일수록
    # 어긋남이 커져서 패킷 유실을 오판하게 된다.
    out.pkt_ts_us = ts_pk.copy()
    out.pkt_host_ts = np.asarray(host_times, dtype=np.float64)

    # --- 회전 경계: 방위각이 크게 되돌아가는 지점 ---
    daz = np.diff(az_flat)
    wraps = np.flatnonzero(daz < -180.0) + 1
    out.rev_starts = np.concatenate(([0], wraps, [len(az_flat)])).astype(np.int64)

    if drop_zero:
        # 거리 0 = 반사 없음. 원점에 점을 쌓아두면 이후 처리가 전부 오염된다.
        keep = d_flat > 0
        # 회전 경계를 남는 점 기준으로 다시 센다
        kept_before = np.cumsum(keep)
        out.rev_starts = np.concatenate(
            ([0], kept_before[np.clip(out.rev_starts[1:] - 1, 0, None)]))
        x, y, z = x[keep], y[keep], z[keep]
        inten = inten[keep]
        ring = ring[keep]
        az_flat = az_flat[keep]
        t_abs = t_abs[keep]
        host = host[keep]

    out.xyz = np.stack([x, y, z], axis=1)
    out.intensity = inten
    out.ring = ring
    out.azimuth = az_flat
    out.packet_ts_us = t_abs
    out.host_ts = host
    # 회전 시작을 0 으로 하는 상대 시각(초). PointCloud2 의 'time' 필드 관례.
    out.time = np.zeros(len(t_abs), np.float32)
    for a, b in out.revolutions():
        if b > a:
            out.time[a:b] = ((t_abs[a:b] - t_abs[a]) / 1e6).astype(np.float32)
    return out


# ===========================================================================
#  위치 패킷 (PPS / NMEA)
# ===========================================================================
def read_position_packets(path):
    """8308 포트 패킷에서 PPS 상태와 NMEA 문장을 뽑는다.

    PPS 상태  : 0=없음 1=동기화중 2=PPS잠김 3=오류   (오프셋 202)
    NMEA      : 오프셋 206 부터 ASCII
    """
    times, buf, n = read_pcap(path, POSITION_PORT)
    out = []
    for i in range(n):
        p = buf[i * 512:(i + 1) * 512]
        if len(p) < 512:
            break
        ts_us = struct.unpack_from("<I", p, 198)[0]
        pps = p[202]
        nmea = p[206:306].split(b"\x00")[0].decode("ascii", "replace").strip()
        out.append({"host_ts": float(times[i]), "lidar_us": int(ts_us),
                    "pps_state": int(pps), "nmea": nmea})
    return out


PPS_STATES = {0: "Absent", 1: "Synchronizing", 2: "Locked", 3: "Error"}


# ===========================================================================
#  단독 실행
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="VLP-16 pcap 디코더")
    ap.add_argument("pcap")
    ap.add_argument("--dump", type=int, default=0,
                    help="앞쪽 회전 N 개를 npy 로 저장")
    ap.add_argument("--outdir", default="revs")
    ap.add_argument("--raw-frame", action="store_true",
                    help="ROS 관례 대신 매뉴얼 좌표계로 출력")
    args = ap.parse_args()

    d = decode(args.pcap, raw=args.raw_frame)
    revs = list(d.revolutions())
    print(f"파일        : {args.pcap}")
    print(f"모델/모드   : {d.model} / {d.return_mode}")
    print(f"데이터 패킷 : {d.n_packets}")
    print(f"점 개수     : {len(d):,} (거리 0 제외)")
    print(f"회전 수     : {len(revs)}")
    if d.n_packets:
        span = d.host_ts[-1] - d.host_ts[0]
        print(f"길이        : {span:.3f} s  → {len(revs) / max(span, 1e-9):.2f} Hz")
        pts = np.array([b - a for a, b in revs])
        print(f"회전당 점   : 평균 {pts.mean():.0f}  최소 {pts.min()}  최대 {pts.max()}")
        print(f"거리 범위   : {np.linalg.norm(d.xyz, axis=1).min():.2f} ~ "
              f"{np.linalg.norm(d.xyz, axis=1).max():.2f} m")

    pos = read_position_packets(args.pcap)
    if pos:
        states = {}
        for p in pos:
            states[p["pps_state"]] = states.get(p["pps_state"], 0) + 1
        desc = ", ".join(f"{PPS_STATES.get(k, k)} {v}회"
                         for k, v in sorted(states.items()))
        print(f"PPS 상태    : {desc}")
        nmea = next((p["nmea"] for p in pos if p["nmea"]), "")
        print(f"NMEA 예시   : {nmea or '(없음)'}")

    if args.dump:
        os.makedirs(args.outdir, exist_ok=True)
        for i, (a, b) in enumerate(revs[:args.dump]):
            arr = np.column_stack([d.xyz[a:b], d.intensity[a:b],
                                   d.ring[a:b].astype(np.float32),
                                   d.time[a:b]])
            path = os.path.join(args.outdir, f"rev_{i:04d}.npy")
            np.save(path, arr)
            print(f"  저장 {path}  {arr.shape}  (x,y,z,intensity,ring,time)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
