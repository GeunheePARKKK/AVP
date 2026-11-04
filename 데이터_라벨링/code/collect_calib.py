#!/usr/bin/env python3
"""
카메라-라이다 캘리브레이션 데이터 수집 (Daheng SDK + VLP-16 UDP, ROS 불필요)

- 카메라 1대씩 진행. 보드 자세를 바꿀 때마다 SPACE로 저장
- 저장 시 라이다를 --lidar-sec 초 동안 누적(정지 상태 가정 → 점 밀도 증가)
- 저장 결과: out/<cam_name>/pose_000.png, pose_000_lidar.npy (x,y,z,intensity,ring), meta.json

라이다 좌표계: x=앞(방위각 0°), y=왼쪽, z=위 (ROS velodyne 드라이버와 동일)

사용 예)
  python3 collect_calib.py --cam-index 1 --cam-name CAM4_front --board 8x5 --square 0.1
  python3 collect_calib.py --cam-sn FCS22010123 --cam-name CAM1_left --board 8x5 --square 0.1

키: SPACE 저장 / d 마지막 저장 삭제 / q 종료
"""
import argparse
import json
import os
import socket
import struct
import threading
import time

import cv2
import numpy as np

import aruco_target as AT

# ---------------- VLP-16 파싱 ----------------
VLP16_VERT_DEG = np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15], dtype=np.float64)
VLP16_RING = np.argsort(np.argsort(VLP16_VERT_DEG))  # laser id -> ring(아래 0 ~ 위 15)
_SIN_W = np.sin(np.deg2rad(VLP16_VERT_DEG))
_COS_W = np.cos(np.deg2rad(VLP16_VERT_DEG))


def parse_vlp16_packet(data):
    """1206바이트 데이터 패킷 → (N,5) [x,y,z,intensity,ring]"""
    if len(data) != 1206:
        return None
    blocks = np.frombuffer(data[:1200], dtype=np.uint8).reshape(12, 100)
    flags = blocks[:, 0].astype(np.uint16) | (blocks[:, 1].astype(np.uint16) << 8)
    if not np.all(flags == 0xEEFF):
        return None
    az = (blocks[:, 2].astype(np.float64) + blocks[:, 3].astype(np.float64) * 256) / 100.0  # deg
    # 블록 내 두 번째 발사열의 방위각 = 다음 블록과의 중간값으로 보간
    d = np.diff(az)
    d = np.where(d < 0, d + 360, d)
    d = np.append(d, d[-1] if len(d) else 0)
    ch = blocks[:, 4:].reshape(12, 32, 3)
    dist = (ch[:, :, 0].astype(np.float64) + ch[:, :, 1].astype(np.float64) * 256) * 0.002  # m
    inten = ch[:, :, 2].astype(np.float64)
    az_all = np.concatenate([np.repeat(az[:, None], 16, 1), np.repeat((az + d / 2)[:, None], 16, 1)], axis=1) % 360
    laser = np.tile(np.arange(16), 2)[None, :].repeat(12, 0)
    m = dist > 0.3
    r, a, l, it = dist[m], np.deg2rad(az_all[m]), laser[m], inten[m]
    xy = r * _COS_W[l]
    x = xy * np.cos(a)
    y = -xy * np.sin(a)
    z = r * _SIN_W[l]
    return np.column_stack([x, y, z, it, VLP16_RING[l]])


class LidarReceiver(threading.Thread):
    def __init__(self, port=2368, host=""):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.buf = []
        self.pkt_count = 0
        self.running = True

    def run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            with self.lock:
                self.buf.append((time.time(), data))
                self.pkt_count += 1
                if len(self.buf) > 20000:  # 약 25초분 초과 시 앞부분 버림
                    self.buf = self.buf[-10000:]

    def grab(self, seconds):
        """지금부터 seconds 동안 받은 패킷을 점군으로 반환"""
        t0 = time.time()
        time.sleep(seconds)
        with self.lock:
            pkts = [d for t, d in self.buf if t >= t0]
        pts = [p for p in (parse_vlp16_packet(d) for d in pkts) if p is not None]
        return (np.vstack(pts) if pts else np.zeros((0, 5))), len(pkts)

    def stop(self):
        self.running = False


# ---------------- Daheng 카메라 ----------------
class DahengCam:
    def __init__(self, index=None, sn=None, exposure=None):
        import gxipy as gx
        self.gx = gx
        self.dm = gx.DeviceManager()
        n, info = self.dm.update_device_list()
        if n == 0:
            raise RuntimeError("Daheng 카메라를 찾지 못했습니다.")
        print("[카메라 목록]")
        for i, d in enumerate(info):
            print(f"  index {i + 1}: SN={d.get('sn')} model={d.get('model_name')}")
        self.cam = self.dm.open_device_by_sn(sn) if sn else self.dm.open_device_by_index(index)
        self.cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
        if exposure:
            # 자동노출이 켜져 있으면 ExposureTime이 읽기전용이므로 먼저 끈다
            try:
                self.cam.ExposureAuto.set(gx.GxAutoEntry.OFF)
            except Exception:
                pass
            try:
                self.cam.ExposureTime.set(float(exposure))
            except Exception as e:
                print(f"[경고] 노출 설정 실패({e}). 카메라 현재 설정을 그대로 씁니다.")
        self.cam.stream_on()

    def read(self):
        raw = self.cam.data_stream[0].get_image(timeout=1000)
        if raw is None or raw.get_status() != self.gx.GxFrameStatusList.SUCCESS:
            return None
        rgb = raw.convert("RGB")
        if rgb is None:
            return None
        return cv2.cvtColor(rgb.get_numpy_array(), cv2.COLOR_RGB2BGR)

    def set_exposure(self, us):
        """실행 중 노출 변경. 자동노출이 켜져 있으면 먼저 끈다."""
        us = float(max(20.0, min(1000000.0, us)))
        try:
            self.cam.ExposureAuto.set(self.gx.GxAutoEntry.OFF)
        except Exception:
            pass
        try:
            self.cam.ExposureTime.set(us)
            return self.cam.ExposureTime.get()
        except Exception as e:
            print(f"[경고] 노출 변경 실패: {e}")
            return None

    def get_exposure(self):
        try:
            return self.cam.ExposureTime.get()
        except Exception:
            return None

    def close(self):
        try:
            self.cam.stream_off()
        finally:
            self.cam.close_device()


# ---------------- 메인 ----------------
def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--cam-index", type=int, help="Daheng 카메라 번호(1부터)")
    g.add_argument("--cam-sn", help="Daheng 카메라 시리얼")
    p.add_argument("--cam-name", required=True, help="저장 폴더 이름 (예: CAM4_front)")
    p.add_argument("--target", help="ArUco 보드 정의 yaml (예: target_aruco.yaml). "
                                    "지정하면 체커보드 대신 ArUco 보드를 씁니다")
    p.add_argument("--params", help="카메라 내부 파라미터 yaml. 주면 화면에 보드 기울기/거리를 "
                                    "실시간 표시합니다(자세 다양성 확보용)")
    p.add_argument("--board", help="[체커보드] 내부 코너 수 가로x세로 (예: 8x5)")
    p.add_argument("--square", type=float, help="[체커보드] 칸 한 변(m)")
    p.add_argument("--lidar-port", type=int, default=2368)
    p.add_argument("--lidar-sec", type=float, default=1.0, help="저장 시 라이다 누적 시간(초)")
    p.add_argument("--exposure", type=float, help="노출 시간(us), 미지정 시 카메라 설정 유지")
    p.add_argument("--auto", action="store_true",
                   help="자동 촬영으로 시작 (혼자 작업할 때: 보드를 새 자세로 들고 가만히 있으면 저장)")
    p.add_argument("--auto-novelty", type=float, default=8.0,
                   help="자동 촬영 시 기존 자세와 최소 법선 각도차(도)")
    p.add_argument("--auto-hold", type=float, default=1.2,
                   help="자동 촬영 전 정지 유지 시간(초)")
    p.add_argument("--out", default="calib_data")
    args = p.parse_args()

    target = detector = None
    cols = rows = None
    if args.target:
        target = AT.load_target(args.target)
        detector = AT.make_detector(target)
        print(f"[타겟] ArUco 보드: 마커 {target['marker_size'] * 1000:.0f}mm, "
              f"id {sorted(target['positions'])}, 최소 {target['min_markers']}개 필요")
    else:
        if not args.board or args.square is None:
            sys.exit("[오류] --target 을 쓰거나, 체커보드면 --board 와 --square 를 지정하세요.")
        cols, rows = map(int, args.board.lower().split("x"))
    board = (cols, rows) if cols else None

    K = D = None
    if args.params:
        K, D = AT.load_intrinsics(args.params)
        print("[정보] 내부 파라미터 로드 - 화면에 기울기/거리를 표시합니다 (기울기 30도 이상 권장)")
    out_dir = os.path.join(args.out, args.cam_name)
    os.makedirs(out_dir, exist_ok=True)

    lidar = LidarReceiver(args.lidar_port)
    lidar.start()
    cam = DahengCam(index=args.cam_index, sn=args.cam_sn, exposure=args.exposure)

    idx = len([f for f in os.listdir(out_dir) if f.endswith(".png")])
    last_pkt, last_t, pps = 0, time.time(), 0.0
    print("[안내] SPACE 저장 / d 마지막 삭제 / q 종료  (저장 중에는 보드·차량 정지)")
    print("       노출 조절: [ 어둡게 / ] 밝게 (20%씩)")
    print("       화면의 board 값(보드 영역 평균 밝기)이 120~200, mk 4/4 가 되게 맞추세요")
    print("       a: 자동 촬영 토글 (보드를 새 자세로 들고 가만히 있으면 알아서 저장)")
    exposure_us = cam.get_exposure()
    auto = args.auto
    saved_normals = []      # 저장된 자세의 보드 법선 (자동모드 중복 방지)
    if target and K is not None and idx > 0:
        # 이어서 찍는 경우: 기존 자세의 법선을 읽어 자동모드가 중복 촬영하지 않게 한다
        import glob as _glob
        for _f in sorted(_glob.glob(os.path.join(out_dir, "pose_*.png"))):
            _g = cv2.imread(_f, cv2.IMREAD_GRAYSCALE)
            if _g is None:
                continue
            _r = AT.detect(_g, target, detector)
            if _r is None:
                continue
            _p = AT.board_pose(_r[0], _r[1], K, D)
            if _p:
                _n = _p[0][:, 2]
                saved_normals.append(_n if _n @ _p[1] > 0 else -_n)
        print(f"[자동촬영] 기존 {len(saved_normals)}자세의 법선을 읽었습니다 (중복 방지용)")
    steady_since = None     # 정지 시작 시각
    last_normal = None      # 직전 프레임 법선
    last_auto_save = 0.0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                continue
            small = cv2.resize(frame, None, fx=0.5, fy=0.5)
            view = small.copy()
            board_stats = None
            cur_normal = None
            if target:
                # ArUco는 전체 해상도에서도 16ms라 축소 없이 검출한다
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                r = AT.detect(gray, target, detector)
                found = r is not None
                if found:
                    # 노출 판단은 화면 전체가 아니라 '보드 영역'으로 해야 한다.
                    # 전체 포화율은 천장 조명이 대부분이라 노출 지표로 쓸 수 없다.
                    u = r[1]
                    x0, x1 = max(0, int(u[:, 0].min())), min(gray.shape[1], int(u[:, 0].max()))
                    y0, y1 = max(0, int(u[:, 1].min())), min(gray.shape[0], int(u[:, 1].max()))
                    if x1 > x0 and y1 > y0:
                        b = gray[y0:y1, x0:x1]
                        board_stats = (float(b.mean()), 100.0 * float((b >= 250).mean()), len(r[2]))
                    for k in range(0, len(r[1]), 4):      # 마커 4코너씩 외곽선
                        q = (r[1][k:k + 4] * 0.5).astype(np.int32)
                        cv2.polylines(view, [q], True, (0, 255, 0), 2)
                    cv2.putText(view, f"ids {r[2]}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if K is not None:
                        pose = AT.board_pose(r[0], r[1], K, D)
                        if pose:
                            Rb, tb, rms = pose
                            cur_normal = Rb[:, 2] if Rb[:, 2] @ tb > 0 else -Rb[:, 2]
                            dist = float(np.linalg.norm(tb))
                            tilt = float(np.degrees(np.arccos(
                                min(1.0, abs(Rb[:, 2] @ (tb / dist))))))
                            good = tilt >= 30
                            cv2.putText(view, f"tilt {tilt:4.0f} deg  {'OK' if good else '<< TILT MORE'}",
                                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                        (0, 255, 0) if good else (0, 165, 255), 2)
                            cv2.putText(view, f"dist {dist:.2f} m   reproj {rms:.1f} px",
                                        (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            else:
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCorners(gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
                if found:
                    cv2.drawChessboardCorners(view, board, corners, found)

            now = time.time()
            if now - last_t >= 1.0:
                pps = (lidar.pkt_count - last_pkt) / (now - last_t)
                last_pkt, last_t = lidar.pkt_count, now
            lid_ok = pps > 500
            cv2.putText(view, f"board: {'OK' if found else '--'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.putText(view, f"lidar: {pps:.0f} pkt/s", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if lid_ok else (0, 0, 255), 2)
            cv2.putText(view, f"saved: {idx}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if exposure_us:
                if board_stats:
                    bmean, bsat, nmk = board_stats
                    if bsat > 2.0:
                        msg, colr = "<< TOO BRIGHT, press [", (0, 0, 255)
                    elif bmean < 90:
                        msg, colr = "<< TOO DARK, press ]", (0, 165, 255)
                    elif bmean > 215:
                        msg, colr = "<< a bit bright", (0, 165, 255)
                    else:
                        msg, colr = "OK", (0, 255, 0)
                    cv2.putText(view, f"exp {exposure_us / 1000:.0f}ms  board {bmean:.0f} "
                                      f"sat {bsat:.1f}%  mk {nmk}/4  {msg}",
                                (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colr, 2)
                else:
                    cv2.putText(view, f"exp {exposure_us / 1000:.0f}ms   (board not detected)",
                                (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.imshow("collect_calib", view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key in (ord("["), ord("]")) and hasattr(cam, "set_exposure"):
                cur = exposure_us or cam.get_exposure() or 30000.0
                new = cur * (0.8 if key == ord("[") else 1.25)
                got = cam.set_exposure(new)
                if got:
                    exposure_us = got
                    print(f"[노출] {got / 1000:.1f} ms")
            if key == ord("d") and idx > 0:
                idx -= 1
                for suf in (".png", "_lidar.npy"):
                    f = os.path.join(out_dir, f"pose_{idx:03d}{suf}")
                    if os.path.exists(f):
                        os.remove(f)
                print(f"[삭제] pose_{idx:03d}")
            def do_save(quiet=False):
                """현재 프레임 + 라이다 1초 누적 저장. 성공하면 True."""
                nonlocal idx
                img_before = frame
                pts, n = lidar.grab(args.lidar_sec)
                img_after = cam.read()
                if img_after is not None:
                    diff = cv2.absdiff(cv2.cvtColor(img_before, cv2.COLOR_BGR2GRAY),
                                       cv2.cvtColor(img_after, cv2.COLOR_BGR2GRAY)).mean()
                    if diff > 8:
                        if not quiet:
                            print(f"[경고] 저장 중 움직임 감지(diff={diff:.1f}). 다시 시도하세요.")
                        return False
                name = f"pose_{idx:03d}"
                cv2.imwrite(os.path.join(out_dir, name + ".png"), img_before)
                np.save(os.path.join(out_dir, name + "_lidar.npy"), pts.astype(np.float32))
                print(f"[저장] {name}  라이다 {len(pts)}점 / 패킷 {n}"
                      + ("  (자동)" if quiet else ""))
                idx += 1
                return True

            if key == ord("a"):
                auto = not auto
                steady_since = None
                print(f"[자동촬영] {'켜짐 - 보드를 새 자세로 들고 가만히 계세요' if auto else '꺼짐'}")

            if key == ord(" "):
                if not found:
                    print("[경고] 보드가 검출되지 않아 저장하지 않았습니다.")
                    continue
                if not lid_ok:
                    print("[경고] 라이다 패킷이 부족합니다(연결/IP 확인).")
                    continue
                if do_save() and cur_normal is not None:
                    saved_normals.append(cur_normal)

            # ---- 자동 촬영 ----
            if auto:
                ready, why = False, ""
                if not found or cur_normal is None:
                    why = "board not found"
                elif not lid_ok:
                    why = "lidar"
                elif time.time() - last_auto_save < 2.0:
                    why = "cooldown"
                else:
                    moved = (last_normal is None or
                             np.degrees(np.arccos(np.clip(cur_normal @ last_normal, -1, 1))) > 1.5)
                    if moved:
                        steady_since = time.time()
                    elif steady_since is None:
                        steady_since = time.time()
                    held = time.time() - steady_since
                    if saved_normals:
                        d_new = min(np.degrees(np.arccos(np.clip(cur_normal @ sn, -1, 1)))
                                    for sn in saved_normals)
                    else:
                        d_new = 180.0
                    if d_new < args.auto_novelty:
                        why = f"too similar ({d_new:.0f} deg)"
                        steady_since = time.time()
                    elif held < args.auto_hold:
                        why = f"hold still {args.auto_hold - held:.1f}s"
                    else:
                        ready = True
                last_normal = cur_normal
                cv2.putText(view, f"AUTO  {'CAPTURING...' if ready else why}",
                            (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if ready else (0, 200, 255), 2)
                cv2.imshow("collect_calib", view)
                cv2.waitKey(1)
                if ready:
                    if do_save(quiet=True):
                        saved_normals.append(cur_normal)
                    last_auto_save = time.time()
                    steady_since = None
    finally:
        cam.close()
        lidar.stop()
        cv2.destroyAllWindows()
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            meta = {"cam_name": args.cam_name, "cam_index": args.cam_index, "cam_sn": args.cam_sn,
                    "lidar_frame": "x=forward(az 0), y=left, z=up", "num_poses": idx}
            if target:
                meta["target_type"] = "aruco"
                meta["target_yaml"] = os.path.abspath(args.target)
            else:
                meta["target_type"] = "checkerboard"
                meta["board_inner_corners"] = [cols, rows]
                meta["square_m"] = args.square
            json.dump(meta, f, indent=2, ensure_ascii=False)
        print(f"[완료] {out_dir} 에 {idx}개 자세 저장")


if __name__ == "__main__":
    main()
