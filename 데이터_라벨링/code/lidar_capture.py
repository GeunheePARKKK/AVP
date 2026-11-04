"""Velodyne VLP-16 UDP 캡처 → pcap.

collect.py 의 세션 폴더에 비어 있던 lidar.pcap 을 채우는 모듈이다.

--------------------------------------------------------------------------
왜 tcpdump 가 아니라 직접 받는가

  tcpdump / dumpcap 은 raw 소켓이라 root 나 CAP_NET_RAW 가 필요하다.
  수집 스크립트 전체를 sudo 로 돌리면 카메라 SDK 와 시리얼 권한까지
  root 문맥으로 끌려가고, 만들어진 파일 주인도 root 가 된다.
  라이다는 255.255.255.255 로 브로드캐스트하므로 평범한 UDP 소켓으로
  전부 받을 수 있다. 권한이 필요 없다.

  대신 UDP 소켓은 페이로드만 주고 이더넷/IP/UDP 헤더는 주지 않는다.
  VeloView 나 velodyne_driver 가 읽는 pcap 은 그 헤더를 기대하므로
  아래에서 다시 만들어 붙인다. 목적지 주소는 짐작하지 않고
  IP_PKTINFO 로 커널에게 실제 값을 물어본다.

--------------------------------------------------------------------------
왜 수신 스레드와 기록 스레드를 나누는가

  sync_cam.py 가 콜백에서 디스크를 건드리지 않는 것과 같은 이유다.
  recvfrom 루프가 디스크에 막히면 커널 수신 버퍼가 차고, 그때부터
  패킷은 조용히 버려진다. 유실이 로그에 남지 않는 유형이라 위험하다.
  수신은 오직 큐에 넣기만 하고, 파일 쓰기는 별도 스레드가 맡는다.

--------------------------------------------------------------------------
단독 실행

    python3 lidar_capture.py                    5초 받아보고 요약만 출력
    python3 lidar_capture.py --duration 20 --out lidar.pcap
"""

import argparse
import os
import socket
import struct
import threading
import time
from queue import Queue, Empty, Full

# 리눅스 전용 상수. 파이썬 socket 모듈이 노출하지 않는 버전이 있어 직접 둔다.
IP_PKTINFO = 8

DATA_PORT = 2368          # 포인트 데이터 (1206 B)
POSITION_PORT = 8308      # 위치/GPS 패킷 (512 B), NMEA 와 PPS 상태가 들어 있다

# 한 패킷 약 1.2 kB, 10 Hz 단일반사에서 754 pkt/s ≒ 0.9 MB/s.
# 큐 20000 개면 약 26 초치 — 디스크가 잠깐 멎어도 버틴다.
QUEUE_MAXSIZE = 20000
RCVBUF_BYTES = 8 << 20

LINKTYPE_ETHERNET = 1
BROADCAST_MAC = b"\xff\xff\xff\xff\xff\xff"


# ===========================================================================
#  pcap 쓰기
# ===========================================================================
class PcapWriter:
    """libpcap 클래식 포맷. VeloView / Wireshark / velodyne_driver 호환."""

    def __init__(self, path, snaplen=65535):
        self.path = path
        # 버퍼링을 크게 잡는다. 1 MB 씩 몰아서 쓰면 write 횟수가 줄어
        # 기록 스레드가 큐를 비우는 속도가 안정된다.
        self._fp = open(path, "wb", buffering=1 << 20)
        self._fp.write(struct.pack("<IHHiIII",
                                   0xA1B2C3D4,   # magic (마이크로초 해상도)
                                   2, 4,         # version 2.4
                                   0,            # thiszone: 타임스탬프는 UTC
                                   0,            # sigfigs
                                   snaplen,
                                   LINKTYPE_ETHERNET))
        self.packets = 0
        self.bytes = 0

    def write(self, ts, eth_frame):
        sec = int(ts)
        usec = int(round((ts - sec) * 1e6))
        if usec == 1000000:               # 반올림이 1초를 넘기는 경계
            sec += 1
            usec = 0
        n = len(eth_frame)
        self._fp.write(struct.pack("<IIII", sec, usec, n, n))
        self._fp.write(eth_frame)
        self.packets += 1
        self.bytes += n

    def close(self):
        if self._fp is not None:
            self._fp.flush()
            os.fsync(self._fp.fileno())
            self._fp.close()
            self._fp = None


def _ip_checksum(header):
    """IPv4 헤더 체크섬. 0 으로 두면 Wireshark 가 bad checksum 으로 표시한다."""
    if len(header) % 2:
        header += b"\x00"
    total = 0
    for i in range(0, len(header), 2):
        total += (header[i] << 8) | header[i + 1]
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def build_frame(payload, src_ip, dst_ip, sport, dport, src_mac, dst_mac,
                ip_id=0):
    """UDP 페이로드를 이더넷 프레임으로 되감는다.

    UDP 체크섬은 0 (IPv4 에서 허용되는 '계산 안 함')으로 둔다. 실제 값은
    커널이 이미 검증하고 버렸으므로 여기서 다시 만들어도 원본이 아니다.
    """
    udp = struct.pack("!HHHH", sport, dport, 8 + len(payload), 0) + payload

    total_len = 20 + len(udp)
    ip_hdr = struct.pack("!BBHHHBBH4s4s",
                         0x45,            # version 4, IHL 5
                         0x00,            # DSCP/ECN
                         total_len,
                         ip_id & 0xFFFF,
                         0x0000,          # flags / fragment offset
                         255,             # TTL
                         socket.IPPROTO_UDP,
                         0,               # 체크섬 자리
                         src_ip, dst_ip)
    ip_hdr = (ip_hdr[:10]
              + struct.pack("!H", _ip_checksum(ip_hdr))
              + ip_hdr[12:])

    return dst_mac + src_mac + b"\x08\x00" + ip_hdr + udp


def _lookup_mac(ip):
    """ARP 캐시에서 MAC 을 찾는다. 없으면 None — 캡처 자체는 계속한다."""
    try:
        with open("/proc/net/arp") as f:
            next(f)
            for line in f:
                parts = line.split()
                if len(parts) >= 4 and parts[0] == ip:
                    mac = parts[3]
                    if mac != "00:00:00:00:00:00":
                        return bytes.fromhex(mac.replace(":", ""))
    except (OSError, StopIteration, ValueError):
        pass
    return None


# ===========================================================================
#  캡처
# ===========================================================================
class VelodyneCapture:
    """UDP 2368/8308 을 계속 받아 두고, 세션이 열려 있는 동안만 파일에 쓴다.

    수신 스레드는 open() 부터 close() 까지 쉬지 않고 돈다. 세션 사이에
    소켓을 닫았다 열면 그 틈에 커널 버퍼가 넘쳐 다음 세션 첫 회전이
    깨진 채로 시작한다. 세션 밖 패킷은 받아서 그냥 버린다.
    """

    def __init__(self, data_port=DATA_PORT, pos_port=POSITION_PORT,
                 expect_src=None, queue_maxsize=QUEUE_MAXSIZE):
        self.ports = [p for p in (data_port, pos_port) if p]
        self.data_port = data_port
        self.expect_src = expect_src          # 예: "192.168.1.201" (None = 전부)
        self.queue = Queue(maxsize=queue_maxsize)

        self._socks = {}
        self._stop = threading.Event()
        self._threads = []

        self._writer = None
        self._writer_lock = threading.Lock()
        self._ip_id = 0

        self.src_mac = b"\x00\x00\x00\x00\x00\x00"
        # 위치 패킷에서 본 최신 PPS 상태 (0 Absent / 1 Sync / 2 Locked / 3 Error).
        # 수신 스레드가 어차피 8308 을 받고 있으므로 여기서 같이 들고 있는다.
        # 소켓을 하나 더 열면 같은 포트를 두 번 bind 하게 되어 지저분하다.
        self.pos_port = pos_port
        self.last_pps_state = None
        self.stats = {"recv": 0, "written": 0, "dropped_queue": 0,
                      "foreign": 0, "error": 0}
        # 세션 요약용
        self._session = None

    # ---------------------------------------------------------------- 열기
    def open(self):
        for port in self.ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF_BYTES)
            except OSError:
                pass
            s.setsockopt(socket.IPPROTO_IP, IP_PKTINFO, 1)
            try:
                s.bind(("", port))
            except OSError as e:
                self.close()
                raise RuntimeError(
                    f"UDP {port} 를 열 수 없습니다: {e}\n"
                    f"  다른 프로그램이 이미 쓰고 있을 수 있습니다 "
                    f"(VeloView, velodyne_driver, 이전 실행이 남은 경우).\n"
                    f"  확인:  ss -ulpn 'sport = :{port}'") from e
            s.settimeout(0.5)
            self._socks[port] = s

        if self.expect_src:
            mac = _lookup_mac(self.expect_src)
            if mac:
                self.src_mac = mac

        for port, s in self._socks.items():
            t = threading.Thread(target=self._recv_loop, args=(port, s),
                                 daemon=True)
            t.start()
            self._threads.append(t)

        t = threading.Thread(target=self._write_loop, daemon=True)
        t.start()
        self._threads.append(t)
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ 수신 루프
    def _recv_loop(self, port, sock):
        ancsize = socket.CMSG_SPACE(64)
        while not self._stop.is_set():
            try:
                payload, anc, _flags, src = sock.recvmsg(2048, ancsize)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                self.stats["error"] += 1
                time.sleep(0.01)
                continue

            recv = time.time()
            if self.expect_src and src[0] != self.expect_src:
                self.stats["foreign"] += 1
                continue

            dst = b"\xff\xff\xff\xff"
            for lvl, typ, cdata in anc:
                if lvl == socket.IPPROTO_IP and typ == IP_PKTINFO:
                    dst = struct.unpack("I4s4s", cdata[:12])[2]

            self.stats["recv"] += 1
            if port == self.pos_port and len(payload) >= 512:
                self.last_pps_state = payload[202]
            try:
                self.queue.put_nowait((recv, payload, src, dst, port))
            except Full:
                # 큐가 넘쳤다는 것은 디스크가 못 따라온다는 뜻이다.
                # 조용히 넘어가지 않도록 반드시 센다.
                self.stats["dropped_queue"] += 1

    # ------------------------------------------------------------ 기록 루프
    def _write_loop(self):
        while not self._stop.is_set():
            try:
                recv, payload, src, dst, port = self.queue.get(timeout=0.2)
            except Empty:
                continue

            with self._writer_lock:
                w = self._writer
                if w is None:
                    continue          # 세션 밖 — 버린다
                self._ip_id = (self._ip_id + 1) & 0xFFFF
                frame = build_frame(payload,
                                    socket.inet_aton(src[0]), dst,
                                    src[1], port,
                                    self.src_mac, BROADCAST_MAC,
                                    ip_id=self._ip_id)
                try:
                    w.write(recv, frame)
                except OSError:
                    self.stats["error"] += 1
                    continue
                self.stats["written"] += 1

            if port == self.data_port and self._session is not None:
                self._note_data_packet(recv, payload)

    def _note_data_packet(self, recv, payload):
        """세션 요약에 쓸 값만 기록한다. 파싱은 수집이 끝난 뒤에 한다."""
        s = self._session
        s["data_packets"] += 1
        if len(payload) >= 1206:
            ts_us = struct.unpack_from("<I", payload, 1200)[0]
            if s["first_lidar_us"] is None:
                s["first_lidar_us"] = ts_us
            s["last_lidar_us"] = ts_us
        if s["first_host"] is None:
            s["first_host"] = recv
        s["last_host"] = recv

    # ---------------------------------------------------------------- 세션
    def start_session(self, pcap_path):
        os.makedirs(os.path.dirname(os.path.abspath(pcap_path)), exist_ok=True)
        w = PcapWriter(pcap_path)
        self._session = {"path": pcap_path, "data_packets": 0,
                         "first_host": None, "last_host": None,
                         "first_lidar_us": None, "last_lidar_us": None,
                         "started": time.time()}
        base = dict(self.stats)
        self._session["_base"] = base
        with self._writer_lock:
            self._writer = w
        return pcap_path

    def stop_session(self, drain=1.0):
        """큐를 비우고 파일을 닫는다. 세션 요약 dict 를 돌려준다."""
        deadline = time.time() + drain
        while time.time() < deadline and not self.queue.empty():
            time.sleep(0.05)

        with self._writer_lock:
            w, self._writer = self._writer, None
        if w is not None:
            w.close()

        s, self._session = self._session, None
        if s is None:
            return None

        base = s.pop("_base")
        span = ((s["last_host"] - s["first_host"])
                if s["first_host"] is not None else 0.0)
        lidar_span = None
        if s["first_lidar_us"] is not None:
            d = s["last_lidar_us"] - s["first_lidar_us"]
            if d < 0:                 # 정시를 넘어가면 0 으로 되돌아간다
                d += 3600 * 1000000
            lidar_span = d / 1e6

        return {
            "path": s["path"],
            "packets_written": w.packets if w else 0,
            "bytes_written": w.bytes if w else 0,
            "data_packets": s["data_packets"],
            "host_span_s": round(span, 3),
            "lidar_span_s": None if lidar_span is None else round(lidar_span, 3),
            "first_host_unix": s["first_host"],
            "last_host_unix": s["last_host"],
            "first_lidar_us": s["first_lidar_us"],
            "last_lidar_us": s["last_lidar_us"],
            "recv": self.stats["recv"] - base["recv"],
            "dropped_queue": self.stats["dropped_queue"] - base["dropped_queue"],
            "expected_data_packets": round(span * 754) if span else 0,
        }

    def close(self):
        self.stop_session(drain=0.2)
        self._stop.set()
        for s in self._socks.values():
            try:
                s.close()
            except OSError:
                pass
        self._socks = {}
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []


# ===========================================================================
#  단독 실행
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Velodyne UDP → pcap 캡처")
    ap.add_argument("--duration", type=float, default=5.0, help="캡처 길이(초)")
    ap.add_argument("--out", default=None,
                    help="pcap 경로. 생략하면 파일을 쓰지 않고 수신만 확인한다")
    ap.add_argument("--src", default="192.168.1.201",
                    help="라이다 IP. 'any' 로 두면 출처를 가리지 않는다")
    ap.add_argument("--data-port", type=int, default=DATA_PORT)
    ap.add_argument("--pos-port", type=int, default=POSITION_PORT)
    args = ap.parse_args()

    src = None if args.src == "any" else args.src
    cap = VelodyneCapture(data_port=args.data_port, pos_port=args.pos_port,
                          expect_src=src)
    print(f"UDP {args.data_port}, {args.pos_port} 수신 대기...")
    with cap:
        if cap.src_mac != b"\x00" * 6:
            print("라이다 MAC:", ":".join(f"{b:02x}" for b in cap.src_mac))
        out = args.out or os.devnull
        cap.start_session(out)
        t0 = time.time()
        while time.time() - t0 < args.duration:
            el = time.time() - t0
            print(f"\r  {el:5.1f}s  수신 {cap.stats['recv']}  "
                  f"기록 {cap.stats['written']}  "
                  f"큐 {cap.queue.qsize()}   ", end="", flush=True)
            time.sleep(0.2)
        print()
        summary = cap.stop_session()

    print("\n--- 요약 ---")
    exp = summary["expected_data_packets"]
    got = summary["data_packets"]
    for k, v in summary.items():
        if not k.startswith("first") and not k.startswith("last"):
            print(f"  {k}: {v}")
    if exp:
        print(f"  수신율: {100.0 * got / exp:.2f} %  ({got}/{exp})")
    if summary["dropped_queue"]:
        print("  [주의] 큐 넘침이 있습니다 — 디스크가 못 따라오고 있습니다.")
    if args.out:
        print(f"\n저장: {args.out}  ({summary['bytes_written'] / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
