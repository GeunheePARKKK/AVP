#!/usr/bin/env python3
"""
기존 카메라 내부 파라미터 검증 스크립트

링(초점/조리개)을 고정한 뒤 체커보드를 5~10장 새로 찍어서,
기존 K(카메라 행렬), D(왜곡계수)를 '고정'한 채 보드 자세만 추정하고 재투영 오차를 계산한다.

판정
  - 재투영 RMS가 이전 캘리브레이션 오차와 비슷(약 0.5px 이하)  → 통과
  - 눈에 띄게 커짐(1px 이상)                                   → 재캘리브레이션

주의: 재투영 오차만으로는 초점거리 변화를 놓칠 수 있다.
  보드를 정면으로 작게 찍으면 초점거리가 10% 틀려도 RMS는 0.05px밖에 안 오른다.
  solvePnP가 초점거리 오차를 '거리'로 흡수해 버리기 때문이다.
  그래서 이 스크립트는 촬영 구성마다 민감도(초점 1% 변화당 RMS 증가량)를 직접
  측정해 보여주고, --ref-dist(줄자 실측 거리)로 초점거리 변화를 따로 잡아낸다.

사용 예)
  사진:    python3 verify_intrinsics.py --params cam1.yaml --images ./cam1_verify
  실측거리: python3 verify_intrinsics.py --params cam1.yaml --images ./cam1_verify --ref-dist 3.00
  실시간:  python3 verify_intrinsics.py --params cam1.yaml --cam-index 1
  어안:    위 명령에 --fisheye 추가
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

PASS_PX = 0.5      # 이하면 통과
RECAL_PX = 1.0     # 이상이면 재캘리브레이션


# ---------------- 파라미터 로드 ----------------
def load_params(path):
    """K, D와 함께 yaml에 들어 있는 부가정보(이전 RMS, 보드 규격, 해상도)를 함께 돌려준다."""
    meta = {}

    # 1) OpenCV FileStorage 형식 (!!opencv-matrix)
    #    주의: 형식이 안 맞으면 cv2.error가 아니라 SystemError가 올라오므로 Exception으로 받는다
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            K = fs.getNode("camera_matrix").mat()
            D = fs.getNode("distortion_coefficients").mat()
            for key, name in (("rms_reprojection_error", "prev_rms"), ("board_cols", "cols"),
                              ("board_rows", "rows"), ("square_size_mm", "square_mm"),
                              ("image_width", "width"), ("image_height", "height")):
                node = fs.getNode(key)
                if not node.empty() and (node.isReal() or node.isInt()):
                    meta[name] = node.real()
            fs.release()
            if K is not None and D is not None:
                return K.astype(np.float64), D.ravel().astype(np.float64), meta
    except Exception:
        pass

    if yaml is None:
        sys.exit("[오류] PyYAML이 필요합니다: pip install pyyaml")
    with open(path) as f:
        d = yaml.safe_load(f)

    for key, name in (("rms_reprojection_error", "prev_rms"), ("board_cols", "cols"),
                      ("board_rows", "rows"), ("square_size_mm", "square_mm"),
                      ("image_width", "width"), ("image_height", "height")):
        if isinstance(d, dict) and key in d:
            meta[name] = float(d[key])

    # 2) ROS camera_calibration 형식
    if "camera_matrix" in d:
        K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)
        return K, D, meta

    # 3) Kalibr camchain 형식 (첫 번째 카메라 사용)
    cam = d.get("cam0", d)
    if "intrinsics" in cam:
        fu, fv, pu, pv = cam["intrinsics"]
        K = np.array([[fu, 0, pu], [0, fv, pv], [0, 0, 1]], dtype=np.float64)
        D = np.array(cam["distortion_coeffs"], dtype=np.float64)
        if "resolution" in cam:
            meta["width"], meta["height"] = map(float, cam["resolution"])
        return K, D, meta

    sys.exit(f"[오류] 파라미터 형식을 인식하지 못했습니다: {path}")


# ---------------- 체커보드 검출 ----------------
def detect(gray, board):
    flags_sb = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, board, flags_sb)
    if ok:
        return corners.reshape(-1, 2).astype(np.float64)
    ok, corners = cv2.findChessboardCorners(gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    return corners.reshape(-1, 2).astype(np.float64)


# ---------------- 자세 추정 / 재투영 (K, D 고정) ----------------
def solve_pose(obj, img_pts, K, D, fisheye):
    if fisheye:
        D4 = np.zeros(4)
        D4[:min(4, len(D))] = D[:4]
        norm = cv2.fisheye.undistortPoints(img_pts.reshape(-1, 1, 2), K, D4.reshape(4, 1))
        ok, rvec, tvec = cv2.solvePnP(obj, norm, np.eye(3), np.zeros(4), flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D4.reshape(4, 1))
    else:
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    if not ok:
        return None
    err = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
    return rvec, tvec, err


def board_geometry(obj, rvec, tvec):
    """보드 중심까지의 거리(m)와 광축 대비 기울기(deg)"""
    R, _ = cv2.Rodrigues(rvec)
    center_cam = R @ obj.mean(axis=0) + tvec.ravel()
    dist = float(np.linalg.norm(center_cam))
    normal = R[:, 2]
    view = center_cam / dist
    tilt = float(np.degrees(np.arccos(np.clip(abs(normal @ view), 0.0, 1.0))))
    return dist, tilt


def coverage_of(pts, w, h):
    """코너 볼록껍질이 화면 면적에서 차지하는 비율(%)"""
    hull = cv2.convexHull(pts.astype(np.float32).reshape(-1, 1, 2))
    return 100.0 * cv2.contourArea(hull) / float(w * h)


def focal_sensitivity(obj, img_pts, K, D, fisheye, base_rms, pct=1.0):
    """초점거리를 pct% 바꾼 K로 자세를 다시 맞췄을 때 '추가로' 생기는 RMS(px).
    원래 오차와 독립이라고 보고 제곱합에서 빼낸다.
    이 값이 작으면 그 사진은 초점거리 변화를 잡아내지 못한다."""
    K2 = K.copy()
    K2[0, 0] *= 1 + pct / 100.0
    K2[1, 1] *= 1 + pct / 100.0
    out = solve_pose(obj, img_pts, K2, D, fisheye)
    if out is None:
        return float("nan")
    pert = float(np.mean(out[2] ** 2))
    return float(np.sqrt(max(0.0, pert - base_rms ** 2)))


def region_of(pts, w, h):
    """보드 중심이 화면 중앙(가운데 50%)인지 가장자리인지"""
    cx, cy = pts.mean(axis=0)
    return "center" if (0.25 * w < cx < 0.75 * w and 0.25 * h < cy < 0.75 * h) else "edge"


def measure(obj, pts, K, D, fisheye, w, h, name):
    out = solve_pose(obj, pts, K, D, fisheye)
    if out is None:
        return None
    rvec, tvec, err = out
    dist, tilt = board_geometry(obj, rvec, tvec)
    rms = float(np.sqrt(np.mean(err ** 2)))
    return dict(name=name, rms=rms, max=float(err.max()),
                dist=dist, tilt=tilt, cover=coverage_of(pts, w, h), region=region_of(pts, w, h),
                sens=focal_sensitivity(obj, pts, K, D, fisheye, rms))


# ---------------- 판정 / 요약 ----------------
def verdict(rms, prev_rms):
    if rms >= RECAL_PX:
        return "재캘리브레이션 권장"
    if rms <= PASS_PX:
        # 이전 오차 대비 2배 넘게 나빠졌으면 통과시키지 않는다
        if prev_rms and rms > 2 * prev_rms:
            return f"주의(이전 {prev_rms:.2f}px 대비 상승)"
        return "통과"
    return "주의"


def summarize(results, prev_rms, ref_dist, K):
    if not results:
        print("\n[결과] 검출된 체커보드가 없습니다.")
        return
    print("\n=================== 검증 결과 ===================")
    print(f"{'이미지':<24}{'RMS(px)':>9}{'최대(px)':>10}{'거리(m)':>9}{'기울기':>8}{'화면%':>8}  위치")
    for r in results:
        print(f"{r['name']:<24}{r['rms']:>9.3f}{r['max']:>10.3f}{r['dist']:>9.3f}"
              f"{r['tilt']:>7.0f}°{r['cover']:>8.1f}  {r['region']}")

    rms_all = np.array([r["rms"] for r in results])
    total = float(np.sqrt(np.mean(rms_all ** 2)))

    print(f"\n[장수] {len(results)}장" + ("" if 5 <= len(results) <= 10 else "  ※ 5~10장을 권장합니다"))
    for reg in ("center", "edge"):
        v = [r["rms"] for r in results if r["region"] == reg]
        if v:
            print(f"[{reg:<6}] 평균 RMS {np.sqrt(np.mean(np.square(v))):.3f} px ({len(v)}장)")
    if not any(r["region"] == "edge" for r in results):
        print("  ※ 가장자리 촬영이 없습니다. 왜곡 변화는 가장자리에서 잘 드러나니 추가 촬영을 권장합니다.")

    print(f"\n[전체 RMS] {total:.3f} px", end="")
    if prev_rms:
        print(f"   (이전 {prev_rms:.3f} px, 변화 {total - prev_rms:+.3f} px)")
    else:
        print("   (비교할 이전 오차 없음: --prev-rms 로 지정 가능)")
    print(f"[판정] {verdict(total, prev_rms)}"
          f"   기준: {PASS_PX}px 이하 통과 / {RECAL_PX}px 이상 재캘리브레이션")

    # ---- 초점거리 검출력 ----
    sens = np.array([r["sens"] for r in results if np.isfinite(r["sens"])])
    print("\n--------- 초점거리 변화 검출력 ---------")
    if sens.size:
        best = float(sens.max())
        print(f"초점 1% 변화 시 추가 RMS: 최대 {best:.3f} px (사진별 {sens.min():.3f}~{best:.3f})")
        if best > 1e-6:
            detectable = PASS_PX / best
            print(f"→ 이 촬영 구성으로 잡아낼 수 있는 최소 초점거리 변화: 약 {detectable:.1f}%")
            if detectable > 3:
                print("  ※ 검출력이 낮습니다. 보드를 화면에 더 크게(화면% 30 이상), 더 비스듬히")
                print("     (기울기 30° 이상) 놓고 다시 찍으면 초점거리 변화를 잡아낼 수 있습니다.")
    else:
        print("민감도를 계산하지 못했습니다.")

    # ---- 줄자 실측 거리 비교 ----
    print("\n--------- 줄자 실측 거리 대조 ---------")
    if ref_dist:
        est = np.array([r["dist"] for r in results])
        ratio = est.mean() / ref_dist
        print(f"추정 거리 평균 {est.mean():.3f} m (편차 {est.std():.3f}) vs 실측 {ref_dist:.3f} m")
        if est.std() > 0.05 * ref_dist:
            print("  ※ 사진마다 거리가 제각각입니다(편차 5% 초과). 이 대조는 모든 사진을")
            print("     같은 거리에서 찍었을 때만 유효합니다. 보드를 고정하고 다시 찍으세요.")
            return
        print(f"→ 초점거리 변화 추정: {100 * (ratio - 1):+.2f}%  "
              f"(fx {K[0, 0]:.1f} → {K[0, 0] * ratio:.1f})")
        if abs(ratio - 1) > 0.02:
            print("  ※ 2% 넘게 어긋났습니다. 재투영 RMS가 낮더라도 재캘리브레이션을 권장합니다.")
        else:
            print("  초점거리는 유지된 것으로 보입니다.")
    else:
        print("--ref-dist 로 줄자 실측 거리(m)를 주면 초점거리 변화를 직접 확인할 수 있습니다.")
        print("(모든 사진을 같은 거리에서 찍고, 카메라 렌즈 앞면에서 보드까지 재세요)")


# ---------------- 실행 모드 ----------------
def run_images(args, K, D, obj):
    files = sorted(sum([glob.glob(os.path.join(args.images, e))
                        for e in ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.PNG", "*.JPG")], []))
    if not files:
        sys.exit(f"[오류] 이미지가 없습니다: {args.images}")
    results = []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"[건너뜀] 읽기 실패: {os.path.basename(f)}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        check_size(args, w, h)
        pts = detect(gray, args.board)
        if pts is None:
            print(f"[건너뜀] 보드 미검출: {os.path.basename(f)}")
            continue
        r = measure(obj, pts, K, D, args.fisheye, w, h, os.path.basename(f))
        if r is None:
            print(f"[건너뜀] 자세 추정 실패: {os.path.basename(f)}")
            continue
        results.append(r)
    return results


def set_exposure(cam, gx, exposure):
    """노출 지정. 자동노출이 켜져 있으면 ExposureTime이 읽기전용이라 먼저 꺼야 한다."""
    if not exposure:
        return
    try:
        cam.ExposureAuto.set(gx.GxAutoEntry.OFF)
    except Exception:
        pass
    try:
        cam.ExposureTime.set(float(exposure))
    except Exception as e:
        print(f"[경고] 노출 설정 실패({e}). 카메라 현재 설정을 그대로 씁니다.")


def open_camera(args):
    if args.v4l2 is not None:
        cap = cv2.VideoCapture(args.v4l2)
        if not cap.isOpened():
            sys.exit(f"[오류] 카메라를 열 수 없습니다: /dev/video{args.v4l2}")
        return lambda: (cap.read()[1]), cap.release
    # Daheng SDK (collect_calib.py와 동일한 방식)
    try:
        import gxipy as gx
    except ImportError:
        sys.exit("[오류] gxipy(Daheng SDK)가 없습니다. USB 웹캠이면 --v4l2 0 을 쓰세요.")
    dm = gx.DeviceManager()
    n, info = dm.update_device_list()
    if n == 0:
        sys.exit("[오류] Daheng 카메라를 찾지 못했습니다.")
    print("[카메라 목록]")
    for i, d in enumerate(info):
        print(f"  index {i + 1}: SN={d.get('sn')} model={d.get('model_name')}")
    cam = dm.open_device_by_sn(args.cam_sn) if args.cam_sn else dm.open_device_by_index(args.cam_index)
    cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
    set_exposure(cam, gx, args.exposure)
    cam.stream_on()

    def read():
        raw = cam.data_stream[0].get_image(timeout=1000)
        if raw is None or raw.get_status() != gx.GxFrameStatusList.SUCCESS:
            return None
        rgb = raw.convert("RGB")
        return None if rgb is None else cv2.cvtColor(rgb.get_numpy_array(), cv2.COLOR_RGB2BGR)

    def close():
        try:
            cam.stream_off()
        finally:
            cam.close_device()

    # DeviceManager가 GC되면 Galaxy API 자체가 해제되어 read()가 NotInitApi로 죽는다.
    # 클로저가 dm을 참조하지 않으므로 함수 객체에 매달아 수명을 붙들어 둔다.
    read._keepalive = (dm, gx)
    close._keepalive = (dm, gx)
    return read, close


def detect_preview(gray_small, board):
    """미리보기용 빠른 검출. 정확도보다 속도 우선(축소 영상, 폴백 없음)."""
    ok, c = cv2.findChessboardCornersSB(gray_small, board, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    return c.reshape(-1, 2).astype(np.float64)


def run_live(args, K, D, obj):
    read, close = open_camera(args)
    os.makedirs(args.save_dir, exist_ok=True)
    results, idx = [], 0
    print("[안내] SPACE: 현재 프레임 기록 / u: 왜곡보정 토글 / d: 마지막 기록 삭제 / q: 종료 후 요약")
    show_undist = False
    scale = args.preview_scale
    try:
        while True:
            frame = read()
            if frame is None:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            check_size(args, w, h)

            # 미리보기는 축소 영상에서 빠르게 검출한다.
            # 전체 해상도 정밀 검출은 570ms가 걸려 루프가 1.7fps로 떨어지고,
            # 그러면 waitKey가 키를 거의 다 놓쳐 SPACE가 안 먹는다.
            small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            pts_s = detect_preview(small, args.board)
            view = frame.copy()
            info = "board: not found"
            prev = None
            if pts_s is not None:
                pts_approx = pts_s / scale
                prev = measure(obj, pts_approx, K, D, args.fisheye, w, h, "preview")
                cv2.drawChessboardCorners(view, args.board,
                                          pts_approx.reshape(-1, 1, 2).astype(np.float32), True)
                if prev:
                    info = (f"RMS~{prev['rms']:.2f}px  dist {prev['dist']:.2f}m  "
                            f"tilt {prev['tilt']:.0f}deg  area {prev['cover']:.0f}%  "
                            f"sens {prev['sens']:.2f}px/1%")
            if show_undist:
                if args.fisheye:
                    view = cv2.fisheye.undistortImage(view, K, D[:4].reshape(4, 1), Knew=K)
                else:
                    view = cv2.undistort(view, K, D)
            colr = (0, 255, 0) if prev else (0, 0, 255)
            cv2.putText(view, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colr, 2)
            cv2.putText(view, f"saved: {len(results)}  [SPACE] record  [q] finish",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            disp = view if w <= 1280 else cv2.resize(view, None, fx=1280 / w, fy=1280 / w)
            cv2.imshow("verify_intrinsics", disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("u"):
                show_undist = not show_undist
            if key == ord("d") and results:
                gone = results.pop()
                os.remove(os.path.join(args.save_dir, gone["name"]))
                print(f"[삭제] {gone['name']}")
                idx -= 1
            if key == ord(" "):
                if pts_s is None:
                    print("[경고] 보드가 보이지 않아 기록하지 않았습니다.")
                    continue
                # 기록할 때만 전체 해상도로 정밀 검출
                pts = detect(gray, args.board)
                if pts is None:
                    print("[경고] 정밀 검출 실패. 보드를 조금 더 크게/또렷하게 잡아주세요.")
                    continue
                name = f"img_{idx:03d}.png"
                cur = measure(obj, pts, K, D, args.fisheye, w, h, name)
                if cur is None:
                    print("[경고] 자세 추정 실패.")
                    continue
                cv2.imwrite(os.path.join(args.save_dir, name), frame)
                results.append(cur)
                print(f"[기록] {name}  RMS {cur['rms']:.3f}px  tilt {cur['tilt']:.0f}deg  "
                      f"area {cur['cover']:.0f}%  ({cur['region']})")
                idx += 1
    finally:
        close()
        cv2.destroyAllWindows()
    return results


def check_size(args, w, h):
    """해상도가 캘리브레이션 당시와 다르면 K 자체가 무효"""
    if args._size_warned or not args.calib_size:
        return
    cw, ch = args.calib_size
    if (w, h) != (int(cw), int(ch)):
        print(f"[경고] 해상도가 다릅니다: 현재 {w}x{h}, 캘리브레이션 당시 {int(cw)}x{int(ch)}")
        print("       해상도가 바뀌면 K를 그대로 쓸 수 없습니다. 같은 해상도로 다시 찍으세요.")
    args._size_warned = True


def main():
    p = argparse.ArgumentParser(description="기존 내부 파라미터 검증 (K, D 고정 후 재투영 오차)")
    p.add_argument("--params", required=True, help="파라미터 yaml 경로")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--images", help="체커보드 사진 폴더")
    src.add_argument("--cam-index", type=int, help="Daheng 카메라 번호(1부터)")
    src.add_argument("--cam-sn", help="Daheng 카메라 시리얼")
    src.add_argument("--v4l2", type=int, help="USB 웹캠 장치 번호 (예: 0)")
    p.add_argument("--board", help="내부 코너 수 가로x세로 (예: 9x8). 미지정 시 yaml에서 읽음")
    p.add_argument("--square", type=float, help="칸 한 변 길이(m). 미지정 시 yaml에서 읽음")
    p.add_argument("--prev-rms", type=float, help="이전 캘리브레이션 RMS(px). 미지정 시 yaml에서 읽음")
    p.add_argument("--ref-dist", type=float, help="보드까지 줄자 실측 거리(m) - 초점거리 변화 확인용")
    p.add_argument("--fisheye", action="store_true", help="어안 모델 사용")
    p.add_argument("--exposure", type=float, help="노출 시간(us), Daheng 전용")
    p.add_argument("--save-dir", default="verify_captures", help="실시간 모드 저장 폴더")
    p.add_argument("--preview-scale", type=float, default=0.5,
                   help="실시간 미리보기 검출 배율(작을수록 빠름). 기록은 항상 전체 해상도")
    args = p.parse_args()

    K, D, meta = load_params(args.params)

    # 보드 규격: 인자 > yaml
    if args.board:
        cols, rows = map(int, args.board.lower().split("x"))
    elif "cols" in meta and "rows" in meta:
        cols, rows = int(meta["cols"]), int(meta["rows"])
        print(f"[정보] 보드 규격을 yaml에서 읽었습니다: {cols}x{rows}")
    else:
        sys.exit("[오류] --board 를 지정하세요 (예: --board 9x8)")
    args.board = (cols, rows)

    if args.square is None:
        if "square_mm" in meta:
            args.square = meta["square_mm"] / 1000.0
            print(f"[정보] 칸 크기를 yaml에서 읽었습니다: {args.square * 1000:.1f} mm")
        else:
            sys.exit("[오류] --square 를 지정하세요 (예: --square 0.123)")

    prev_rms = args.prev_rms if args.prev_rms is not None else meta.get("prev_rms")
    args.calib_size = (meta["width"], meta["height"]) if "width" in meta and "height" in meta else None
    args._size_warned = False

    obj = np.zeros((cols * rows, 3), np.float64)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square

    print(f"[K]\n{K}\n[D] {D}")
    if prev_rms:
        print(f"[이전 RMS] {prev_rms:.3f} px")

    results = run_images(args, K, D, obj) if args.images else run_live(args, K, D, obj)
    summarize(results, prev_rms, args.ref_dist, K)


if __name__ == "__main__":
    main()
