#!/usr/bin/env python3
"""
ArUco 보드 검출/자세추정 공용 모듈 (collect_calib.py, solve_extrinsic.py 에서 함께 사용)

체커보드 대신 네 귀퉁이에 ArUco 마커가 붙은 평면 보드를 쓴다.
보드 좌표계: 중심이 원점, +x 오른쪽, +y 위, +z 보드 법선(앞면 방향)
"""
import sys

import cv2
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None


def load_target(path):
    """target_aruco.yaml 읽기 -> dict"""
    if yaml is None:
        sys.exit("[오류] PyYAML이 필요합니다: pip install pyyaml")
    with open(path) as f:
        d = yaml.safe_load(f)
    ms = float(d["marker_size"])
    pos = {int(k): (float(v[0]), float(v[1])) for k, v in d["marker_positions"].items()}
    return {
        "dictionary": d.get("dictionary", "DICT_4X4_250"),
        "marker_size": ms,
        "positions": pos,
        "min_markers": int(d.get("min_markers", 3)),
        "board_width": float(d.get("board_width", 0.915)),
        "board_height": float(d.get("board_height", 0.610)),
        # 보드 평면에서 라이다 점을 잘라낼 범위 (중심 기준 ±)
        "extent": ((-d.get("board_width", 0.915) / 2, d.get("board_width", 0.915) / 2),
                   (-d.get("board_height", 0.610) / 2, d.get("board_height", 0.610) / 2)),
    }


def make_detector(target):
    """검출기 생성.

    기본 적응형 이진화 창(최대 23)은 이 보드처럼 마커가 화면에서 클 때
    귀퉁이 마커를 놓친다. 실측에서 winSizeMax=53으로 올리니 4개 모두 잡혔다.
    """
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, target["dictionary"]))
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 7
    p.cornerRefinementMaxIterations = 50
    p.cornerRefinementMinAccuracy = 0.01
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    return cv2.aruco.ArucoDetector(dic, p)


def marker_object_points(target, mid):
    """마커 하나의 4코너를 보드 좌표(m)로. ArUco 코너 순서와 동일: 좌상,우상,우하,좌하"""
    x, y = target["positions"][mid]
    s = target["marker_size"]
    return np.array([[x, y, 0], [x + s, y, 0], [x + s, y - s, 0], [x, y - s, 0]], np.float64)


def detect(gray, target, detector=None):
    """보드 검출 -> (obj_pts(N,3), img_pts(N,2), 검출된 id 목록) 또는 None"""
    det = detector or make_detector(target)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return None
    obj, img, found = [], [], []
    for cc, i in zip(corners, ids.ravel()):
        i = int(i)
        if i not in target["positions"]:
            continue
        obj.append(marker_object_points(target, i))
        img.append(cc.reshape(-1, 2).astype(np.float64))
        found.append(i)
    if len(found) < target["min_markers"]:
        return None
    return np.vstack(obj), np.vstack(img), sorted(found)


def board_pose(obj_pts, img_pts, K, D, fisheye=False):
    """보드 자세 -> (Rb, tb, 재투영 RMS px). Rb의 3열이 보드 법선."""
    if fisheye:
        D4 = np.zeros(4)
        D4[:min(4, len(D))] = D[:4]
        norm = cv2.fisheye.undistortPoints(img_pts.reshape(-1, 1, 2), K, D4.reshape(4, 1))
        ok, rvec, tvec = cv2.solvePnP(obj_pts, norm, np.eye(3), np.zeros(4),
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.fisheye.projectPoints(obj_pts.reshape(-1, 1, 3), rvec, tvec, K, D4.reshape(4, 1))
    else:
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            rvec, tvec = cv2.solvePnPRefineLM(obj_pts, img_pts, K, D, rvec, tvec)
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    if not ok:
        return None
    err = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
    Rb, _ = cv2.Rodrigues(rvec)
    return Rb, tvec.ravel(), float(np.sqrt(np.mean(err ** 2)))


def draw(img, gray, target, detector=None):
    """미리보기용: 검출된 마커 표시 + 보드 외곽선"""
    det = detector or make_detector(target)
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return img, []
    out = cv2.aruco.drawDetectedMarkers(img, corners, ids)
    return out, sorted(int(i) for i in ids.ravel())


def load_intrinsics(path):
    """카메라 내부 파라미터 yaml -> (K, D). OpenCV / ROS 형식 모두 지원."""
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            K = fs.getNode("camera_matrix").mat()
            D = fs.getNode("distortion_coefficients").mat()
            fs.release()
            if K is not None and D is not None:
                return K.astype(np.float64), D.ravel().astype(np.float64)
    except Exception:   # 형식 불일치 시 cv2.error가 아닌 SystemError가 올라온다
        pass
    if yaml is None:
        sys.exit("[오류] PyYAML이 필요합니다: pip install pyyaml")
    with open(path) as f:
        d = yaml.safe_load(f)
    if "camera_matrix" in d:
        return (np.array(d["camera_matrix"]["data"], np.float64).reshape(3, 3),
                np.array(d["distortion_coefficients"]["data"], np.float64))
    sys.exit(f"[오류] 파라미터 형식을 인식하지 못했습니다: {path}")
