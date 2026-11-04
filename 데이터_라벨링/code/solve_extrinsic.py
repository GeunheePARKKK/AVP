#!/usr/bin/env python3
"""
카메라-라이다 외부 파라미터 계산 (평면 매칭 방식)

입력: collect_calib.py 결과 폴더 (pose_XXX.png + pose_XXX_lidar.npy + meta.json)
      + 카메라 내부 파라미터 yaml
처리: 1) 카메라: solvePnP → 보드 평면 (카메라 기준)
      2) 라이다: 초기값으로 보드 주변만 잘라낸 뒤 RANSAC → 보드 평면 (라이다 기준)
      3) 평면 법선/거리로 초기해 → 점-평면 거리 최소화로 정밀화 (2회 반복)
출력: calib_result/<cam>_extrinsic.json  (R, t: 라이다 → 카메라)
      calib_result/<cam>_overlay/  (라이다 점을 영상에 투영한 확인용 이미지)

사용 예)
  python3 solve_extrinsic.py --data calib_data/CAM4_front --params cam4.yaml --cam-pos 0.20 0.0 -0.10
  (--yaw 미지정 시 폴더 이름으로 자동: CAM1 +90, CAM2 -90, CAM3 180, CAM4 0)

좌표계: 라이다 x=앞, y=왼쪽, z=위 / 카메라 x=오른쪽, y=아래, z=앞
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import aruco_target as AT

try:
    import yaml
except ImportError:
    yaml = None

DEFAULT_YAW = {"CAM1": 90.0, "CAM2": -90.0, "CAM3": 180.0, "CAM4": 0.0}


# ---------------- 내부 파라미터 로드 ----------------
def load_params(path):
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            K = fs.getNode("camera_matrix").mat()
            D = fs.getNode("distortion_coefficients").mat()
            fs.release()
            if K is not None and D is not None:
                return K.astype(np.float64), D.ravel().astype(np.float64)
    except Exception:  # 형식 불일치 시 cv2.error가 아닌 SystemError가 올라온다
        pass
    if yaml is None:
        sys.exit("[오류] PyYAML이 필요합니다: pip install pyyaml")
    with open(path) as f:
        d = yaml.safe_load(f)
    if "camera_matrix" in d:
        return (np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3),
                np.array(d["distortion_coefficients"]["data"], dtype=np.float64))
    cam = d.get("cam0", d)
    if "intrinsics" in cam:
        fu, fv, pu, pv = cam["intrinsics"]
        return (np.array([[fu, 0, pu], [0, fv, pv], [0, 0, 1]], dtype=np.float64),
                np.array(cam["distortion_coeffs"], dtype=np.float64))
    sys.exit(f"[오류] 파라미터 형식을 인식하지 못했습니다: {path}")


# ---------------- 초기값 ----------------
def initial_extrinsic(yaw_deg, cam_pos):
    """카메라가 라이다 기준 yaw 방향을 보고 cam_pos에 있을 때 (R, t): p_cam = R p_lidar + t"""
    th = np.deg2rad(yaw_deg)
    z_c = np.array([np.cos(th), np.sin(th), 0.0])   # 카메라 앞
    x_c = np.array([np.sin(th), -np.cos(th), 0.0])  # 카메라 오른쪽
    y_c = np.array([0.0, 0.0, -1.0])                # 카메라 아래
    R = np.vstack([x_c, y_c, z_c])                  # lidar → cam
    t = -R @ np.asarray(cam_pos, dtype=np.float64)
    return R, t


# ---------------- 카메라 쪽: 보드 평면 ----------------
def detect(gray, board):
    ok, c = cv2.findChessboardCornersSB(gray, board, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:
        return c.reshape(-1, 2).astype(np.float64)
    ok, c = cv2.findChessboardCorners(gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return c.reshape(-1, 2).astype(np.float64)


def board_pose(obj, img_pts, K, D, fisheye):
    if fisheye:
        norm = cv2.fisheye.undistortPoints(img_pts.reshape(-1, 1, 2), K, D[:4].reshape(4, 1))
        ok, rvec, tvec = cv2.solvePnP(obj, norm, np.eye(3), np.zeros(4))
        proj, _ = cv2.fisheye.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, D[:4].reshape(4, 1))
    else:
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, D)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, axis=1))))
    Rb, _ = cv2.Rodrigues(rvec)
    return Rb, tvec.ravel(), err


# ---------------- 라이다 쪽: 보드 평면 ----------------
def ransac_plane(pts, thresh=0.02, iters=500, rng=np.random.default_rng(0)):
    best = None
    n_pts = len(pts)
    if n_pts < 3:
        return None
    for _ in range(iters):
        s = pts[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        inl = np.abs((pts - s[0]) @ n) < thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        return None
    # 내점으로 최소자승 재적합
    P = pts[best]
    c = P.mean(axis=0)
    n = np.linalg.svd(P - c)[2][2]
    inl = np.abs((pts - c) @ n) < thresh
    return inl


def crop_board_points(lidar_xyz, R, t, Rb, tb, extent, margin_plane, margin_normal):
    """현재 외부 파라미터 추정으로 라이다 점을 보드 좌표계로 옮겨 보드 근처만 남김"""
    p_cam = lidar_xyz @ R.T + t
    p_board = (p_cam - tb) @ Rb  # 보드 좌표계 (x,y 평면, z 법선)
    (x0, x1), (y0, y1) = extent
    m = ((p_board[:, 0] > x0 - margin_plane) & (p_board[:, 0] < x1 + margin_plane) &
         (p_board[:, 1] > y0 - margin_plane) & (p_board[:, 1] < y1 + margin_plane) &
         (np.abs(p_board[:, 2]) < margin_normal))
    return m


# ---------------- 최적화 ----------------
def solve_closed_form(nc_list, dc_list, nl_list, dl_list):
    # 회전: n_c = R n_l (Kabsch)
    A = np.array(nl_list)
    B = np.array(nc_list)
    U, _, Vt = np.linalg.svd(A.T @ B)
    Sfix = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ Sfix @ U.T
    # 이동: n_c · t = d_c - d_l
    t, *_ = np.linalg.lstsq(B, np.array(dc_list) - np.array(dl_list), rcond=None)
    return R, t


def _plane_basis(n):
    """n 에 수직인 정규직교 기저 (u, v)"""
    a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, a)
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def refine(R0, t0, poses, fix_height=None):
    """점-평면 거리 최소화.

    fix_height 가 주어지면 카메라의 라이다 기준 높이를 그 값으로 '고정'한다.
    cam_pos = -R^T t 이므로 cam_pos_z = -(R[:,2] · t). 즉 R[:,2]·t = -Z 라는 제약이다.
    t 를 t = -Z·n + a·u + b·v (n = R[:,2], u⊥v⊥n) 로 두면 제약이 항상 만족되고
    자유도가 3 -> 2 로 줄어든다. 높이는 법선 분포상 가장 약하게 결정되는 축이라,
    줄자 실측이 있으면 고정하는 편이 낫다.
    """
    if fix_height is None:
        x0 = np.concatenate([Rotation.from_matrix(R0).as_rotvec(), t0])

        def resid(x):
            R = Rotation.from_rotvec(x[:3]).as_matrix()
            t = x[3:]
            return np.concatenate([(P["pts"] @ R.T + t) @ P["nc"] - P["dc"] for P in poses])

        r = least_squares(resid, x0, loss="huber", f_scale=0.02)
        return Rotation.from_rotvec(r.x[:3]).as_matrix(), r.x[3:]

    Z = float(fix_height)
    n0 = R0[:, 2]
    u0, v0 = _plane_basis(n0)
    w0 = t0 - (n0 @ t0) * n0                     # t0 에서 n 성분 제거
    x0 = np.concatenate([Rotation.from_matrix(R0).as_rotvec(), [w0 @ u0, w0 @ v0]])

    def t_of(R, a, b):
        n = R[:, 2]
        u, v = _plane_basis(n)
        return -Z * n + a * u + b * v

    def resid(x):
        R = Rotation.from_rotvec(x[:3]).as_matrix()
        t = t_of(R, x[3], x[4])
        return np.concatenate([(P["pts"] @ R.T + t) @ P["nc"] - P["dc"] for P in poses])

    r = least_squares(resid, x0, loss="huber", f_scale=0.02)
    R = Rotation.from_rotvec(r.x[:3]).as_matrix()
    return R, t_of(R, r.x[3], r.x[4])


# ---------------- 투영 확인 이미지 ----------------
def draw_overlay(img, lidar_xyz, R, t, K, D, fisheye, board_mask=None):
    p_cam = lidar_xyz @ R.T + t
    front = p_cam[:, 2] > 0.3
    p = p_cam[front]
    if len(p) == 0:
        return img
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    if fisheye:
        uv, _ = cv2.fisheye.projectPoints(p.reshape(-1, 1, 3), rvec, tvec, K, D[:4].reshape(4, 1))
    else:
        uv, _ = cv2.projectPoints(p, rvec, tvec, K, D)
    uv = uv.reshape(-1, 2)
    dist = np.linalg.norm(p, axis=1)
    col = cv2.applyColorMap(np.clip((dist - 1) / 9 * 255, 0, 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)
    out = img.copy()
    h, w = out.shape[:2]
    bm = board_mask[front] if board_mask is not None else np.zeros(len(p), bool)
    for (u, v), c, b in zip(uv, col, bm):
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(out, (int(u), int(v)), 4 if b else 2, (255, 255, 255) if b else tuple(int(x) for x in c), -1)
    return out


# ---------------- 메인 ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="collect_calib 결과 폴더 (예: calib_data/CAM4_front)")
    ap.add_argument("--params", required=True, help="해당 카메라 내부 파라미터 yaml")
    ap.add_argument("--cam-pos", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"),
                    help="줄자로 잰 카메라 위치(m), 라이다 기준 x앞 y왼쪽 z위")
    ap.add_argument("--yaw", type=float, help="카메라가 보는 방향(deg), 미지정 시 폴더 이름으로 자동")
    ap.add_argument("--target", help="ArUco 보드 정의 yaml (미지정 시 meta.json 기록값 사용)")
    ap.add_argument("--fix-height", type=float, metavar="Z",
                    help="카메라 높이(라이다 기준 z, m)를 줄자 실측값으로 고정. "
                         "높이는 법선 분포상 가장 약하게 결정되는 축이라 실측이 있으면 권장")
    ap.add_argument("--fisheye", action="store_true")
    ap.add_argument("--out", default="calib_result")
    args = ap.parse_args()

    with open(os.path.join(args.data, "meta.json")) as f:
        meta = json.load(f)
    cam_name = meta["cam_name"]
    is_aruco = meta.get("target_type") == "aruco"
    target = detector = None
    board = obj = None
    if is_aruco:
        tpath = args.target or meta.get("target_yaml")
        if not tpath or not os.path.exists(tpath):
            sys.exit("[오류] ArUco 타겟 정의 yaml을 찾지 못했습니다. --target 으로 지정하세요.")
        target = AT.load_target(tpath)
        detector = AT.make_detector(target)
        extent = target["extent"]
        print(f"[타겟] ArUco 보드 {target['board_width']}x{target['board_height']}m, "
              f"마커 {target['marker_size'] * 1000:.0f}mm, id {sorted(target['positions'])}")
    else:
        cols, rows = meta["board_inner_corners"]
        sq = meta["square_m"]
        board = (cols, rows)
        obj = np.zeros((cols * rows, 3), np.float64)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
        extent = ((-sq, cols * sq), (-sq, rows * sq))  # 코너 영역 + 한 칸

    yaw = args.yaw
    if yaw is None:
        key = cam_name.upper()[:4]
        if key not in DEFAULT_YAW:
            sys.exit("[오류] --yaw 를 지정하세요 (폴더 이름에서 CAM1~4를 찾지 못함)")
        yaw = DEFAULT_YAW[key]
    K, D = load_params(args.params)
    R, t = initial_extrinsic(yaw, args.cam_pos)
    print(f"[{cam_name}] yaw={yaw}°, 초기 위치={args.cam_pos}")

    # 1) 카메라 쪽 평면
    poses = []
    for img_path in sorted(glob.glob(os.path.join(args.data, "pose_*.png"))):
        name = os.path.basename(img_path)[:-4]
        lid_path = os.path.join(args.data, name + "_lidar.npy")
        if not os.path.exists(lid_path):
            continue
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if is_aruco:
            r = AT.detect(gray, target, detector)
            if r is None:
                print(f"  [제외] {name}: 마커 부족")
                continue
            pose = AT.board_pose(r[0], r[1], K, D, args.fisheye)
            if pose is None:
                print(f"  [제외] {name}: 자세 추정 실패")
                continue
            Rb, tb, err = pose
        else:
            pts2d = detect(gray, board)
            if pts2d is None:
                print(f"  [제외] {name}: 보드 미검출")
                continue
            Rb, tb, err = board_pose(obj, pts2d, K, D, args.fisheye)
        nc = Rb[:, 2]
        dc = float(nc @ tb)
        if dc < 0:
            nc, dc = -nc, -dc
        lidar = np.load(lid_path)
        poses.append(dict(name=name, img=img, Rb=Rb, tb=tb, nc=nc, dc=dc, lidar=lidar, reproj=err))
    if len(poses) < 3:
        sys.exit(f"[오류] 유효 자세가 {len(poses)}개입니다. 최소 3개(권장 10개 이상) 필요합니다.")

    # 2)~3) 라이다 평면 추출 + 풀이 (마진을 줄여가며 반복)
    for it, (mp, mn) in enumerate([(0.35, 0.35), (0.10, 0.10), (0.05, 0.05)]):
        used = []
        for P in poses:
            xyz = P["lidar"][:, :3].astype(np.float64)
            m = crop_board_points(xyz, R, t, P["Rb"], P["tb"], extent, mp, mn)
            cand = xyz[m]
            P["mask"] = np.zeros(len(xyz), bool)
            if len(cand) < 30:
                P["ok"] = False
                continue
            inl = ransac_plane(cand)
            if inl is None or inl.sum() < 30:
                P["ok"] = False
                continue
            bp = cand[inl]
            rings = np.unique(P["lidar"][m][inl][:, 4]).size
            if rings < 3:
                P["ok"] = False
                continue
            c = bp.mean(axis=0)
            nl = np.linalg.svd(bp - c)[2][2]
            dl = float(nl @ c)
            if dl < 0:
                nl, dl = -nl, -dl
            idx = np.where(m)[0][inl]
            P["mask"][idx] = True
            P.update(ok=True, pts=bp, nl=nl, dl=dl, rings=rings)
            used.append(P)
        if len(used) < 3:
            if it == 0:
                sys.exit(f"[오류] 라이다에서 보드를 찾은 자세가 {len(used)}개뿐입니다. "
                         "초기 위치(--cam-pos)/--yaw 또는 보드 높이를 확인하세요.")
            sys.exit(f"[오류] {it + 1}번째 정밀화에서 보드를 찾은 자세가 {len(used)}개로 줄었습니다.\n"
                     "       앞 단계의 해가 발산한 것입니다. 위의 '법선 다양성' 경고를 확인하세요.")
        normals = np.array([P["nc"] for P in used])
        # 이동 성분은 보드 법선이 3차원을 고루 덮을 때만 풀린다.
        # 법선이 한 평면에 몰리면(=최소 특이값 0) 그 직교 방향의 이동은 수학적으로 결정되지 않는다.
        sv = np.linalg.svd(normals)[1]
        diversity = sv[2] / np.sqrt(len(normals))
        if it == 0:
            print(f"  법선 다양성 지수: {diversity:.3f} (특이값 {np.round(sv, 3)})")
        if diversity < 0.04:
            sys.exit("[오류] 보드 법선이 거의 한 평면에 몰려 있습니다 "
                     f"(다양성 지수 {diversity:.3f} < 0.04).\n"
                     "       이 상태에서는 회전은 풀려도 '이동'이 수학적으로 결정되지 않아\n"
                     "       엉뚱한 카메라 위치가 나옵니다. 보드를 좌우로 돌린 자세뿐 아니라\n"
                     "       위아래로 젖힌 자세(앞으로 숙이기/뒤로 눕히기)도 섞어서 다시 찍으세요.")
        if diversity < 0.12 and it == 0:
            print(f"  [경고] 보드 기울기가 다양하지 않습니다 (다양성 지수 {diversity:.3f}). "
                  "위아래로 젖힌 자세를 더 추가하면 위치 정확도가 올라갑니다.")
        if it == 0:
            R, t = solve_closed_form([P["nc"] for P in used], [P["dc"] for P in used],
                                     [P["nl"] for P in used], [P["dl"] for P in used])
        R, t = refine(R, t, used, args.fix_height)

    # 결과 평가
    print(f"\n{'자세':<10}{'재투영(px)':>11}{'라이다점':>9}{'링수':>6}{'점-평면 RMS(mm)':>17}")
    all_res = []
    for P in poses:
        if not P.get("ok"):
            print(f"{P['name']:<10}{P['reproj']:>11.3f}{'-':>9}{'-':>6}{'제외':>17}")
            continue
        r = (P["pts"] @ R.T + t) @ P["nc"] - P["dc"]
        all_res.append(r)
        print(f"{P['name']:<10}{P['reproj']:>11.3f}{len(P['pts']):>9}{P['rings']:>6}{np.sqrt(np.mean(r ** 2)) * 1000:>17.1f}")
    all_res = np.concatenate(all_res)
    rms_mm = float(np.sqrt(np.mean(all_res ** 2)) * 1000)

    cam_pos = -R.T @ t
    rpy = Rotation.from_matrix(R.T).as_euler("xyz", degrees=True)  # 카메라 자세 (라이다 기준)
    print(f"\n[결과] 전체 점-평면 RMS: {rms_mm:.1f} mm  (VLP-16 노이즈 수준 약 20~30mm면 정상)")
    print(f"  카메라 위치(라이다 기준, m): x={cam_pos[0]:.3f}  y={cam_pos[1]:.3f}  z={cam_pos[2]:.3f}")
    print(f"  줄자 측정값과 차이(cm): {np.round((cam_pos - np.array(args.cam_pos)) * 100, 1)}")

    os.makedirs(args.out, exist_ok=True)
    res = {
        "cam_name": cam_name,
        "description": "p_cam = R * p_lidar + t  (lidar: x fwd, y left, z up / cam: x right, y down, z fwd)",
        "R_lidar_to_cam": R.tolist(),
        "t_lidar_to_cam": t.tolist(),
        "cam_position_in_lidar_m": cam_pos.tolist(),
        "cam_rpy_in_lidar_deg": rpy.tolist(),
        "point_to_plane_rms_mm": rms_mm,
        "num_poses_used": int(sum(1 for P in poses if P.get("ok"))),
    }
    out_json = os.path.join(args.out, f"{cam_name}_extrinsic.json")
    with open(out_json, "w") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)

    ov_dir = os.path.join(args.out, f"{cam_name}_overlay")
    os.makedirs(ov_dir, exist_ok=True)
    for P in poses:
        ov = draw_overlay(P["img"], P["lidar"][:, :3].astype(np.float64), R, t, K, D, args.fisheye, P.get("mask"))
        cv2.imwrite(os.path.join(ov_dir, P["name"] + ".jpg"), ov)
    print(f"  저장: {out_json}\n  확인 이미지: {ov_dir}/ (흰 점 = 보드로 쓰인 라이다 점, 보드 위에 있어야 정상)")


if __name__ == "__main__":
    main()
