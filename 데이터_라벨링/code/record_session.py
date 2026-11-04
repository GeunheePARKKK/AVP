#!/usr/bin/env python3
"""
카메라(Daheng) + VLP-16 라이다 동시 녹화 (ROS 불필요)

- 카메라 N대를 각각 별도 스레드에서 MJPG avi로 녹화하고 프레임별 시각을 함께 저장
- 라이다는 원본 UDP 패킷을 시각과 함께 그대로 보관 (무손실, 나중에 보정값 바꿔 재변환 가능)
- 모든 스레드가 같은 시각에 시작하도록 배리어로 맞춘 뒤 지정 시간만큼 기록

저장 결과: <out>/
  cam1.avi, cam1_times.npy, ...        프레임별 unix 시각(초)
  velodyne_packets.npz                 packets(N,1206) uint8 + times(N,)
  meta.json                            해상도/fps/시리얼/시작시각 등

사용 예)
  python3 record_session.py --sec 10
  python3 record_session.py --sec 10 --fps 20 --out recording_10s
"""
import argparse
import json
import os
import socket
import threading
import time

import cv2
import numpy as np


def wait_barrier(barrier, who, timeout=20.0):
    """모든 스레드가 같은 시각에 출발하도록 대기. 한 쪽이 실패해도 영원히 멈추지 않게 상한을 둔다."""
    try:
        barrier.wait(timeout=timeout)
        return True
    except threading.BrokenBarrierError:
        print(f"[오류] {who}: 다른 스레드가 준비되지 않아 녹화를 중단합니다.", flush=True)
        return False


# ---------------- 라이다: 원본 패킷 수집 ----------------
class LidarRecorder(threading.Thread):
    def __init__(self, port, barrier, seconds):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
        self.sock.bind(("", port))
        self.sock.settimeout(0.5)
        self.barrier, self.seconds = barrier, seconds
        self.pkts, self.times = [], []
        self.t_start = None

    def run(self):
        # 소켓 버퍼에 쌓여 있던 오래된 패킷 비우기.
        # 라이다는 1.3ms마다 패킷을 보내므로 '타임아웃까지 읽기'로는 루프가 끝나지 않는다.
        # 논블로킹으로 바꿔 버퍼가 빌 때(BlockingIOError)까지만, 그것도 시간 상한을 두고 읽는다.
        self.sock.setblocking(False)
        deadline = time.time() + 0.2
        while time.time() < deadline:
            try:
                self.sock.recv(2048)
            except BlockingIOError:
                break
            except OSError:
                break
        self.sock.setblocking(True)
        self.sock.settimeout(0.5)
        if not wait_barrier(self.barrier, "lidar"):
            return
        self.t_start = time.time()
        while time.time() - self.t_start < self.seconds:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            if len(data) == 1206:                      # 데이터 패킷만 (위치 패킷 512B 제외)
                self.pkts.append(data)
                self.times.append(time.time())
        self.sock.close()

    def save(self, path):
        if not self.pkts:
            return 0
        arr = np.frombuffer(b"".join(self.pkts), dtype=np.uint8).reshape(-1, 1206)
        np.savez_compressed(path, packets=arr, times=np.array(self.times))
        return len(self.pkts)


# ---------------- 카메라: MJPG 녹화 ----------------
class CamRecorder(threading.Thread):
    def __init__(self, gx, cam, name, out_dir, fps, barrier, seconds):
        super().__init__(daemon=True)
        self.gx, self.cam, self.name = gx, cam, name
        self.out_dir, self.fps = out_dir, fps
        self.barrier, self.seconds = barrier, seconds
        self.times = []
        self.dropped = 0
        self.shape = None
        self.t_start = None

    def run(self):
        writer = None
        self.cam.stream_on()
        # 첫 프레임으로 해상도 확정
        first = None
        for _ in range(30):
            first = self._grab()
            if first is not None:
                break
        if first is None:
            print(f"[{self.name}] 프레임을 받지 못해 녹화를 건너뜁니다.", flush=True)
            wait_barrier(self.barrier, self.name)
            self.cam.stream_off()
            return
        h, w = first.shape[:2]
        self.shape = (w, h)
        writer = cv2.VideoWriter(os.path.join(self.out_dir, f"{self.name}.avi"),
                                 cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h))
        if not wait_barrier(self.barrier, self.name):
            writer.release()
            self.cam.stream_off()
            return
        self.t_start = time.time()
        while True:
            now = time.time()
            if now - self.t_start >= self.seconds:
                break
            frame = self._grab()
            if frame is None:
                self.dropped += 1
                continue
            writer.write(frame)
            self.times.append(time.time())
        writer.release()
        self.cam.stream_off()

    def _grab(self):
        try:
            raw = self.cam.data_stream[0].get_image(timeout=500)
        except Exception:
            return None
        if raw is None or raw.get_status() != self.gx.GxFrameStatusList.SUCCESS:
            return None
        rgb = raw.convert("RGB")
        if rgb is None:
            return None
        return cv2.cvtColor(rgb.get_numpy_array(), cv2.COLOR_RGB2BGR)


def setup_camera(cam, gx, fps, exposure, exposure_max):
    """노출/프레임률 설정. 자동노출이 켜져 있으면 ExposureTime이 읽기전용이라 먼저 꺼야 한다."""
    cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
    if exposure:
        try:
            cam.ExposureAuto.set(gx.GxAutoEntry.OFF)
        except Exception:
            pass
        try:
            cam.ExposureTime.set(float(exposure))
        except Exception as e:
            print(f"[경고] 노출 설정 실패({e})")
    else:
        # 자동노출을 쓰되, 노출이 길어져 프레임률이 떨어지지 않도록 상한을 건다
        try:
            cam.ExposureAuto.set(gx.GxAutoEntry.CONTINUOUS)
            cam.AutoExposureTimeMax.set(float(exposure_max))
        except Exception:
            pass
    try:
        cam.GainAuto.set(gx.GxAutoEntry.CONTINUOUS)
    except Exception:
        pass
    try:
        cam.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON)
        cam.AcquisitionFrameRate.set(float(fps))
    except Exception:
        pass


def main():
    p = argparse.ArgumentParser(description="카메라 + VLP-16 동시 녹화")
    p.add_argument("--sec", type=float, default=10.0, help="녹화 길이(초)")
    p.add_argument("--fps", type=float, default=20.0, help="카메라 프레임률")
    p.add_argument("--out", default="recording_10s", help="저장 폴더")
    p.add_argument("--lidar-port", type=int, default=2368)
    p.add_argument("--no-lidar", action="store_true", help="라이다 없이 카메라만")
    p.add_argument("--exposure", type=float, help="고정 노출(us). 미지정 시 자동노출")
    p.add_argument("--exposure-max", type=float, default=40000.0,
                   help="자동노출 상한(us). 프레임률 유지를 위해 사용")
    args = p.parse_args()

    import gxipy as gx
    dm = gx.DeviceManager()
    n, info = dm.update_device_list()
    if n == 0:
        raise SystemExit("[오류] Daheng 카메라를 찾지 못했습니다.")
    os.makedirs(args.out, exist_ok=True)

    cams, sns = [], []
    for i in range(1, n + 1):
        cam = dm.open_device_by_index(i)
        setup_camera(cam, gx, args.fps, args.exposure, args.exposure_max)
        cams.append(cam)
        sns.append(info[i - 1].get("sn"))
        print(f"  cam{i}: SN={sns[-1]} model={info[i - 1].get('model_name')}")

    n_threads = n + (0 if args.no_lidar else 1)
    barrier = threading.Barrier(n_threads)
    workers = [CamRecorder(gx, c, f"cam{i + 1}", args.out, args.fps, barrier, args.sec)
               for i, c in enumerate(cams)]
    lidar = None if args.no_lidar else LidarRecorder(args.lidar_port, barrier, args.sec)
    if lidar:
        workers_all = workers + [lidar]
    else:
        workers_all = list(workers)

    print(f"\n[녹화] {args.sec:.0f}초 시작 (카메라 {n}대{'' if args.no_lidar else ' + 라이다'})")
    for w in workers_all:
        w.start()
    for w in workers_all:
        w.join(timeout=args.sec + 30)

    # ---- 저장 및 요약 ----
    meta = {"seconds": args.sec, "fps_requested": args.fps, "cameras": [], "created": time.time()}
    t0s = [w.t_start for w in workers if w.t_start] + ([lidar.t_start] if lidar and lidar.t_start else [])
    sync_spread = (max(t0s) - min(t0s)) * 1000 if t0s else 0.0

    print(f"\n{'소스':<10}{'프레임/패킷':>12}{'실측 fps':>10}{'유실':>7}  비고")
    for i, w in enumerate(workers):
        cnt = len(w.times)
        dur = (w.times[-1] - w.times[0]) if cnt > 1 else 0
        fps = (cnt - 1) / dur if dur > 0 else 0
        np.save(os.path.join(args.out, f"{w.name}_times.npy"), np.array(w.times))
        size = os.path.getsize(os.path.join(args.out, f"{w.name}.avi")) / 1e6 if cnt else 0
        print(f"{w.name:<10}{cnt:>12}{fps:>10.1f}{w.dropped:>7}  {size:.1f} MB  SN={sns[i]}")
        meta["cameras"].append({"name": w.name, "sn": sns[i], "frames": cnt,
                                "fps_measured": round(fps, 2), "dropped": w.dropped,
                                "resolution": w.shape})
    if lidar:
        npk = lidar.save(os.path.join(args.out, "velodyne_packets.npz"))
        dur = (lidar.times[-1] - lidar.times[0]) if npk > 1 else 0
        pps = (npk - 1) / dur if dur > 0 else 0
        size = os.path.getsize(os.path.join(args.out, "velodyne_packets.npz")) / 1e6 if npk else 0
        print(f"{'velodyne':<10}{npk:>12}{pps:>10.1f}{'-':>7}  {size:.1f} MB  (정상 754 pkt/s)")
        meta["lidar"] = {"packets": npk, "pps_measured": round(pps, 1), "port": args.lidar_port}

    meta["sync_spread_ms"] = round(sync_spread, 2)
    with open(os.path.join(args.out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    for c in cams:
        c.close_device()
    print(f"\n[완료] 스레드 시작 시각 편차 {sync_spread:.1f} ms")
    print(f"       저장 위치: {os.path.abspath(args.out)}")


if __name__ == "__main__":
    main()
