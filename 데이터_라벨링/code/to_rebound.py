"""세션을 ReBound 포맷으로 변환한다.

    python3 to_rebound.py <세션> --poses <deskew_on> --keyframes <keyframes.json> \
        --detections <detections.json> -o <출력>

ReBound 포맷 (github.com/ajedgley/ReBound wiki)
    pointcloud/LIDAR_TOP/<n>.pcd   차량 좌표계
    cameras/<이름>/<n>.jpg          + extrinsics.json, intrinsics.json
    ego/<n>.json                    전역 좌표계에서의 차량 자세
    bounding/<n>/boxes.json         정답 — 사람이 채운다
    pred_bounding/<n>/boxes.json    예측 — 검출기 초안을 넣는다
    metadata.json, timestamps.json

좌표계
    우리 리그에는 별도의 차량 좌표계가 없다. **라이다 좌표계를 차량 좌표계로
    삼는다** (x=앞, y=왼쪽, z=위). 따라서 라이다 extrinsic 은 항등이고,
    카메라 extrinsic 은 T_cam_lidar 의 역행렬이다.

왜곡
    intrinsics.json 에 왜곡계수 칸이 없다 — ReBound 는 **왜곡 보정된 이미지**를
    전제한다. 그래서 이미지를 cv2.undistort 로 펴서 저장한다.

포인트클라우드
    검출기가 본 것과 같은 **10 스윕 누적** 클라우드를 쓴다. 단일 스캔보다
    10 배 촘촘해서 사람이 차를 알아보기 쉽다. 누적은 키프레임 시점의 좌표계로
    ego pose 를 써서 모은다.
"""

import argparse
import json
import os
import struct
import sys

import cv2
import numpy as np
import yaml

CALIB = "/home/acelab/Documents/Lidar_Camera_Calibration/calib_result/extrinsics_all.yaml"
AVP = "/home/acelab/Documents/TalkFile_AVP_calib"
NAME = {"front": "CAM_FRONT", "left": "CAM_LEFT",
        "right": "CAM_RIGHT", "rear": "CAM_BACK"}
BAYER = {"BayerRG8": cv2.COLOR_BayerRG2BGR, "BayerGB8": cv2.COLOR_BayerGB2BGR,
         "BayerGR8": cv2.COLOR_BayerGR2BGR, "BayerBG8": cv2.COLOR_BayerBG2BGR}


def quat_from_R(R):
    """3x3 회전행렬 → (w, x, y, z)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w, x, y, z = (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s
    return [float(w), float(x), float(y), float(z)]


def write_pcd(path, xyzi):
    """binary PCD (x y z intensity, float32)."""
    n = len(xyzi)
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
              "TYPE F F F F\nCOUNT 1 1 1 1\n"
              f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(xyzi.astype(np.float32).tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--poses", required=True)
    ap.add_argument("--keyframes", required=True)
    ap.add_argument("--detections", default=None)
    ap.add_argument("--extra", default=None,
                    help="같은 키프레임 구조의 추가 검출 (예: cones.json). 합쳐 넣는다")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--n-sweep", type=int, default=10)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(here)),
                                    "포인트보정", "code"))
    from vlp16_dataset import VLP16PcapDataset
    from analyze_deskew import load_poses

    ds = VLP16PcapDataset(os.path.join(args.session, "lidar.pcap"))
    poses = load_poses(args.poses)
    kf = json.load(open(args.keyframes))["keyframes"]
    det = json.load(open(args.detections)) if args.detections else None
    if det and args.extra:
        ex = {f["keyframe"]: f["boxes"] for f in json.load(open(args.extra))["frames"]}
        for f in det["frames"]:
            f["boxes"] = f["boxes"] + ex.get(f["keyframe"], [])
    meta = json.load(open(os.path.join(args.session, "meta.json")))
    cal = yaml.safe_load(open(CALIB))["cameras"]

    for d in ("pointcloud/LIDAR_TOP", "ego"):
        os.makedirs(os.path.join(args.out, d), exist_ok=True)

    # ---- 카메라 설정 (프레임마다 같다) ----
    cams = {}
    for pos, c in cal.items():
        name = NAME[pos]
        cdir = os.path.join(args.out, "cameras", name)
        os.makedirs(cdir, exist_ok=True)
        T = np.array(c["T_cam_lidar"]["data"]).reshape(4, 4)
        T_lidar_cam = np.linalg.inv(T)            # 차량(=라이다) 좌표계에서의 카메라 자세
        fs = cv2.FileStorage(os.path.join(AVP, c["intrinsics"]), cv2.FILE_STORAGE_READ)
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat().ravel()
        fs.release()
        json.dump({"translation": [float(x) for x in T_lidar_cam[:3, 3]],
                   "rotation": quat_from_R(T_lidar_cam[:3, :3])},
                  open(os.path.join(cdir, "extrinsics.json"), "w"), indent=1)
        json.dump({"matrix": [[float(v) for v in row] for row in K]},
                  open(os.path.join(cdir, "intrinsics.json"), "w"), indent=1)
        sn = c["serial"]
        info = [x for x in meta["cameras"] if x["sn"] == sn][0]
        cams[name] = dict(dir=cdir, K=K, D=D, serial=sn,
                          fmt=info["pixel_format"], meta_name=info["name"])

    ts = []
    for i, f in enumerate(kf):
        k = f["scan"]
        ts.append(str(f["time_unix"]))

        # ---- 포인트클라우드 (10 스윕 누적, 키프레임 좌표계) ----
        Tk_inv = np.linalg.inv(poses[k])
        acc = []
        for j in range(max(0, k - args.n_sweep + 1), k + 1):
            a, b = ds._spans[j]
            T = Tk_inv @ poses[j]
            q = ds._xyz[a:b] @ T[:3, :3].T + T[:3, 3]
            acc.append(np.column_stack([q, ds.intensity[a:b]]))
        pc = np.vstack(acc)
        pc = pc[np.linalg.norm(pc[:, :3], axis=1) < 50]
        write_pcd(os.path.join(args.out, "pointcloud/LIDAR_TOP", f"{i}.pcd"), pc)

        # ---- ego ----
        T = poses[k]
        json.dump({"translation": [float(x) for x in T[:3, 3]],
                   "rotation": quat_from_R(T[:3, :3])},
                  open(os.path.join(args.out, "ego", f"{i}.json"), "w"), indent=1)

        # ---- 이미지 (왜곡 보정) ----
        for name, c in cams.items():
            fi = f["cameras"][c["meta_name"]]
            raw = np.load(os.path.join(args.session, fi["folder"],
                                       f"{fi['frame_id']:08d}.npy"))
            lo, hi = np.percentile(raw, (0.1, 99.9))
            y = np.clip((raw.astype(np.float32) - lo) / max(hi - lo, 1e-6), 0, 1)
            bgr = cv2.cvtColor((np.power(y, 0.35) * 255).astype(np.uint8),
                               BAYER[c["fmt"]])
            lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
            l, a_, b_ = cv2.split(lab)
            l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
            bgr = cv2.cvtColor(cv2.merge([l, a_, b_]), cv2.COLOR_LAB2BGR)
            bgr = cv2.undistort(bgr, c["K"], c["D"])      # ReBound 는 보정된 영상을 전제
            cv2.imwrite(os.path.join(c["dir"], f"{i}.jpg"), bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, 92])

        # ---- 박스 ----
        for sub, boxes in (("bounding", []),
                           ("pred_bounding", det["frames"][i]["boxes"] if det else [])):
            bd = os.path.join(args.out, sub, str(i))
            os.makedirs(bd, exist_ok=True)
            out = []
            for n, bx in enumerate(boxes):
                cx, cy, cz = bx["center"]
                l_, w_, h_ = bx["size"]
                yaw = bx["yaw"]
                out.append({"origin": [cx, cy, cz + h_ / 2],   # 바닥중심 → 중심
                            # ReBound 는 W,L,H 순서다 (dataformat_utils.py:147).
                            # mmdet3d 는 dx(길이), dy(폭) 순이므로 맞바꾼다.
                            "size": [w_, l_, h_],
                            "rotation": [float(np.cos(yaw / 2)), 0.0, 0.0,
                                         float(np.sin(yaw / 2))],
                            "annotation": bx["class"],
                            "confidence": int(round(bx["score"] * 100)),
                            # refine_boxes.py 를 거쳤으면 트랙 ID 가 들어 있다.
                            # 프레임이 바뀌어도 같은 객체는 같은 ID 를 갖는다.
                            "id": bx.get("track_id", f"{i:03d}_{n:03d}"),
                            "internal_pts": bx.get("points", 0),
                            "data": {k: v for k, v in bx.items()
                                     if k in ("views", "track_id")}})
            json.dump({"boxes": out}, open(os.path.join(bd, "boxes.json"), "w"),
                      indent=1)
            json.dump({}, open(os.path.join(bd, "description.json"), "w"))
        print(f"  {i+1}/{len(kf)}  점 {len(pc):,}  예측박스 "
              f"{len(det['frames'][i]['boxes']) if det else 0}", flush=True)

    json.dump({"annotation_map": {}},
              open(os.path.join(args.out, "pred_bounding", "annotation_map.json"), "w"))
    json.dump({"source-format": "cam_lidar_recording (VLP-16 + Daheng x4)",
               "filenames": [os.path.basename(args.session.rstrip("/"))]},
              open(os.path.join(args.out, "metadata.json"), "w"), ensure_ascii=False, indent=1)
    json.dump({"timestamps": ts},
              open(os.path.join(args.out, "timestamps.json"), "w"), indent=1)
    print(f"완료: {args.out}")


if __name__ == "__main__":
    main()
