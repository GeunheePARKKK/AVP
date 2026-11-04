# 코드 전체 — 단계별 안내

녹화부터 3D bounding box 라벨링까지 쓰는 코드를 **전부 이 폴더에 모았다.**
전에는 `시간동기화/code`, `포인트보정/code`, `데이터_라벨링/code` 셋으로 나뉘어
있었는데, 서로 import 하고 있어서 한곳에 두는 편이 낫다.

설치와 실행 순서는 [`설치_방법.md`](설치_방법.md) 를 본다.

---

## 단계별

| 단계 | 하는 일 | 코드 |
|---|---|---|
| **1** | **녹화** | **`record_freerun.py`** ← 025~041 전부 이걸로 찍었다<br>`record.py` (SessionWriter·공용), `lidar_capture.py`, `vlp16.py`, `lidar_config.py`<br>`record_session.py` (MCU 하드웨어 동기용, 지금은 안 씀)<br>**`daheng_ws_1/`** — 카메라 제어 (`sync_cam.py`·`dhcam.py`·`collect.py`) + `vendor/gxipy` |
| | 녹화 확인 | `check_session.py` (숫자 검산), `preview_session.py` (4분할 미리보기 영상)<br>`sync_report.py`, `trim_session.py` (앞부분 잘라내기) |
| **2** | **포인트 보정 · 오도메트리** | **`run_kiss_icp.py`** — KISS-ICP (deskew 켜고 `--voxel 0.2 --max-range 25`)<br>`vlp16_dataset.py` (pcap → 스캔+점별 타임스탬프), `analyze_deskew.py` (포즈 읽기·기하 유틸)<br>`deskew_to_shutter.py`, `shutter_real.py` (셔터 시각 보정), `export_scans.py` |
| **3** | **드리프트 측정** | `measure_drift.py` — 포즈 정렬 후 ICP 로 오차를 잰다. 전파 간격을 정하는 근거 |
| **4** | **클래스 체계 · 메트릭** | 코드 없음 → [`../dataset_config.yaml`](../dataset_config.yaml) |
| **5** | **키프레임 선별** | `select_keyframes.py` — 1 m 이동 또는 3 초 경과마다 선택 |
| **6** | **자동 검출 (초안)** | **`auto_label.py`** — nuScenes 사전학습 PointPillars. **10 스윕 입력 고정**<br>(20·30 으로 올려도 나아지지 않는다. 학습 조건과 달라진다) |
| **7** | **초안 다듬기** | **`refine_boxes.py`** — 묶기·오탐 제거·전파·트랙 ID·공백 메우기 |
| **7.5** | **꼬깔콘 검출** | **`detect_cones.py`** — 기하 규칙. 검출기에는 쓸 만한 꼬깔콘 출력이 없다 |
| **8** | **포맷 변환** | **`to_rebound.py`** — ReBound 포맷. 포인트클라우드는 **30 스윕** |
| **9** | **툴 적재** | `run_rebound.sh` + [`rebound.patch`](rebound.patch) |
| **10** | **사람이 라벨링** | ReBound 에서 수작업 |
| **11** | **품질 검사** | 아직 코드 없음 |
| — | **전 단계 일괄** | **`run_session.sh`** — 2·5·6·7·7.5·8 과 확인 영상을 차례로 |

## 확인용 그림·영상

| 코드 | 내용 |
|---|---|
| **`render_lidar_3d.py`** | **3D 박스 확인 영상** (`lidar3d.mp4`). 캘리브레이션을 안 쓴다 — 박스 품질만 보기에 이쪽이 정확 |
| `render_boxes_video.py` | 카메라 4대에 박스 투영 (캘리브레이션 필요) |
| `project_overlay.py` | 라이다 점을 카메라에 투영 — 캘리브레이션 검증용 |
| `compare_overlay.py` | deskew 켜고/끄고 비교 |
| `export_images.py` | raw `.npy` → JPEG (밝기 보정) |
| `make_quad_video.py` | 4분할 영상 |
| `plot_results.py` | 그래프 |
| `to_rosbag.py` | rosbag2 변환 |

## 캘리브레이션 (별도 작업)

| 코드 | 내용 |
|---|---|
| `collect_calib.py` | 보드 자세 수집 |
| `solve_extrinsic.py` | 외부 파라미터 추정 → `../../calib_result/` |
| `verify_intrinsics.py` | 내부 파라미터 검증 |
| `aruco_target.py`, `target_aruco.yaml` | ArUco 보드 정의 |
| `intrinsics/cam1~4.yaml` | **카메라 내부 파라미터** (원래 레포 밖에 있던 것) |

---

## 장소마다 달라지는 값

`--max-range` 는 **고정값이 아니다.** 30 스윕 누적 기준 박스 안 점 중앙값으로 정한다.

| 장소 | 세션 | 범위 | 20~25 m | 25~30 m | 30~35 m |
|---|---|---|---|---|---|
| 대구공항 (지하) | 031 · 034 | **20 m** | 173 | 48 | 58 |
| 파티마병원 (지하) | 037 | **25 m** | 326 | 156 | 95 |
| 신세계 (트인 곳) | 041 | **30 m** | 622 | 422 | 330 |

20 m 를 그대로 쓰면 041 은 검출의 **84 %** 를 버려 프레임당 5.0 개만 남는다 (30 m 면 12.8 개).
