# 포인트보정 — KISS-ICP deskew 용 라이다 원본

주차장에서 20초 주행하며 찍은 VLP-16 스캔 한 세션. **점별 시각이 붙은 포인트클라우드**를 KISS-ICP 에 넣어 모션 왜곡(skew)을 펴고 오도메트리를 얻는 데 필요한 것만 담았다.

녹화: 2026-09-13 14:13:22 ~ 14:13:42 · 세션 `session_013`

---

## 왜 카메라가 없나

이 세션은 **하드웨어 동기도 캘리브레이션도 하지 않은 자유 구동(free-run)** 녹화다 (`data/session_013/meta.json` 의 `"mode"`). deskew 는 라이다 한 대의 점별 시각만으로 성립하므로 카메라 프레임은 쓰이지 않는다.

원본 세션은 3.9 GB 다. 대부분이 카메라다.

| 구성 | 용량 | 여기 포함 |
|---|---|---|
| `rosbag_0.db3` (카메라 4대 + 라이다) | 2.0 GB | ✗ |
| `cam1`~`cam4` 원본 `.npy` 200장 × 4 | 1.9 GB | ✗ |
| `full_20s.mp4`, `preview.mp4` | 35 MB | ✗ |
| **`lidar.pcap`** | **20 MB** | ✓ |
| `meta.json` | 4 KB | ✓ |

카메라 이미지와 rosbag 은 연구실 노트북 `~/ros2_ws/cam_lidar_recording/sessions/session_013` 에 그대로 있다.

---

## 점별 시각은 어디서 나오나

VLP-16 은 점마다 타임스탬프를 패킷에 써주지 않는다. 대신 발사 순서가 고정이라 패킷 타임스탬프에서 정확히 계산된다.

- 시퀀스 간격 55.296 µs, 채널 간격 2.304 µs
- 패킷당 12블록 × 2시퀀스 × 16채널 = 384점, 블록 간격 110.592 µs
- 점별 시각 = 패킷 타임스탬프 + 블록·시퀀스·채널 오프셋

`code/vlp16.py` 가 이 계산을 해서 `Decoded.time` (회전 시작 기준 상대 초) 에 넣고, `code/vlp16_dataset.py` 가 그것을 KISS-ICP 규약인 0~1 정규화 값으로 바꿔 넘긴다. 방위각도 같은 방식으로 블록 사이를 보간하므로 회전 왜곡이 이미 한 번 펴진 상태다.

---

## 데이터

```
data/session_013/
├── lidar.pcap     20 MB · 15,087 패킷 · 데이터 포트 2368
└── meta.json      센서 설정, 패킷 통계, 프레임 수
```

디코딩 검증 결과 (`python3 code/vlp16.py data/session_013/lidar.pcap`):

| 항목 | 값 |
|---|---|
| 모델 / 반사 모드 | VLP-16 / Strongest (단일 반사) |
| 총 점 | 5,400,646 |
| 회전(스캔) 수 | **201** |
| 회전당 점 수 | 중앙값 26,930 (최소 10,762, 최대 27,413) |
| 스캔 내 점별 시각 | 0 ~ 0.0999 초, 단조증가 |
| 스캔 간격 | 평균 0.0999 초 (10 Hz) |
| 링 | 16개 전부 존재 |
| 패킷 유실 | 0 |

첫 회전과 마지막 회전은 녹화 시작·종료가 회전 중간에 걸려 반 토막이다. `vlp16_dataset.py` 가 `min_points` 로 걸러낸다.

---

## 돌리는 법

```bash
pip install kiss-icp numpy

cd code
python3 run_kiss_icp.py ../data/session_013/lidar.pcap
python3 run_kiss_icp.py ../data/session_013/lidar.pcap --no-deskew   # 비교용
```

`../results/session_013/` 에 `poses_kitti.txt`, `poses_tum.txt`, `summary.txt` 가 남는다. `--save-deskewed` 를 주면 보정된 스캔도 `deskewed/00000.npy` 로 저장된다 (스캔당 약 300 KB).

보정된 점만 따로 쓰려면:

```bash
python3 export_scans.py ../data/session_013/lidar.pcap -o /tmp/scans   # (x,y,z,t) npy
```

점검 · 그림 · 카메라 셔터 시각 기준 보정(발표자료 Step 3 ~ 5):

```bash
python3 analyze_deskew.py  ../data/session_013/lidar.pcap --poses ../results/deskew_on
python3 plot_results.py    ../data/session_013/lidar.pcap --on ../results/deskew_on --off ../results/deskew_off
python3 deskew_to_shutter.py ../data/session_013/lidar.pcap --poses ../results/deskew_on --save
```

`deskew_to_shutter.py` 가 카메라 4대분 '셔터 순간 포인트클라우드' 를 만든다. session_013 은 free-run 이라
셔터 시각은 A안 가정값(0 / 25 / 50 / 75 ms)이다. 실제 로그가 있으면 `CAMERAS` 상수만 바꾸면 된다.

수집과 가공 때 확인할 것: [`docs/수집_가공_체크리스트.md`](docs/수집_가공_체크리스트.md)
2026-09-29 실행 기록과 그림: [`results/2026-09-29_session_013/RESULTS.md`](results/2026-09-29_session_013/RESULTS.md)

---

## 실제로 돌려본 결과

KISS-ICP 1.3.0, 기본 설정, `max_range=100`, 201 스캔 전부.

| | deskew 켬 | 끔 |
|---|---|---|
| 이동 거리 | **33.256 m** | 32.493 m |
| 스캔당 이동 (평균 / 최대) | 0.1663 / 0.2623 m | 0.1625 / 0.2620 m |
| 시작 → 끝 직선거리 | 28.164 m | 28.106 m |

20초에 33 m 이므로 평균 약 6 km/h. 저속 주행이라 deskew 차이가 2.3% 수준으로 작다. 스캔당 최대 0.26 m 를 움직였는데 한 스캔이 0.1초에 걸쳐 찍히므로, 보정하지 않으면 한 스캔 안에서 최대 그만큼 점이 끌린다. **차이는 직선 구간보다 회전 구간에서 크게 벌어진다.**

---

## KISS-ICP 버전 주의

`KissICP` 의 pose 접근 방식이 버전마다 다르다.

| 버전 | 누적 pose |
|---|---|
| 1.3.x | `odom.last_pose` (매 프레임 직접 모아야 함), `register_frame` 이 `(보정된 스캔, 정합에 쓴 점)` 반환 |
| 1.0 ~ 1.2 | `odom.poses` 에 전부 쌓임 |

`run_kiss_icp.py` 는 양쪽을 다 처리한다. `load_config` 의 인자 형태도 버전별로 갈려서 같이 분기해 두었다.

또한 **KISS-ICP 의 기본 로더는 Velodyne pcap 을 읽지 못한다** (rosbag, KITTI, Ouster pcap, `.ply`/`.bin` 디렉터리만 지원). 그래서 `vlp16_dataset.py` 가 필요하다.

---

## 다른 세션

같은 날 같은 조건으로 `session_008` ~ `session_013` 여섯 개를 찍었다 (각 20초). 전부 연구실 노트북에 있고, 필요하면 pcap 만 같은 방식으로 꺼내면 된다 — 세션당 20 MB.

---

## 한계 — 오도메트리이지 SLAM 이 아니다

KISS-ICP 는 **라이다 오도메트리**다. 직전 자세 대비 상대 변화를 누적할 뿐, 전역 지도를 만들지도 과거 자세를 되돌려 고치지도 않는다.

| | 오도메트리 (우리) | SLAM |
|---|---|---|
| 하는 일 | 상대 변화 누적 | 전역 일관된 지도 + 위치 |
| 루프 클로저 | 없음 | 있음 |
| 전역 최적화 | 없음 | 포즈 그래프 최적화 |
| 오차 | **계속 누적** | 루프에서 보정 |

KISS-ICP 도 복셀 지도를 들고 있지만 그것은 현재 위치 주변만 유지하는 **정합용 지역 지도**다.

### 다른 데이터셋과의 비교

| 데이터셋 | 위치 추정 방식 | 정확도 |
|---|---|---|
| KITTI | RTK GPS + INS | cm 급 |
| nuScenes | GPS + HD 지도 + Monte Carlo Localization | ≤ 10 cm |
| **우리** | **라이다만** | **1.5 % / 이동거리** |

nuScenes 는 미리 만든 HD 지도에 매 프레임을 맞추므로 누적이 생기지 않는다. 우리는 참조점이 없어 오차가 쌓이기만 한다.

### 실측 드리프트 (session_025, voxel 0.2 m / max_range 25 m)

| 시간 간격 | 이동 거리 | 드리프트 |
|---|---|---|
| 1 초 | 1.1 m | 2.0 cm |
| 5 초 | 5.7 m | 3.4 cm |
| 10 초 | 11 m | 13.4 cm |
| 15 초 | 17 m | **26.3 cm** (이동거리의 1.5 %) |

GPS·루프 클로저 없는 순수 라이다 오도메트리에서 0.5 ~ 2 % 는 흔한 범위다. 16 빔이고 기둥·주차열이 반복되는 지하 주차장이라 정합에 특히 불리하다.

### 영향

| | 영향 |
|---|---|
| 포인트 보정 (deskew · 셔터 시각) | **없음.** 0.1 초 이내 구간만 쓴다 |
| 3D 박스 라벨 정확도 | **없음.** 전파는 초안일 뿐이고 사람이 5 초마다 확인·수정한다 |
| 라벨링 작업량 | 있음. 드리프트가 작았다면 재조정 간격을 늘릴 수 있었다 |
| **ego pose 를 궤적 정답으로 쓰는 것** | **불가.** 데이터셋 문서에 "ego pose 는 라벨링 보조용이며 궤적 정답이 아님" 을 명시한다 |

### 개선하려면

| 방법 | 효과 | 비용 |
|---|---|---|
| 루프 클로저 (포즈 그래프 SLAM) | 큼 | KISS-ICP 에 없다. 다른 구현 필요. session_025 는 거의 직선이라 쓸 루프가 없고, 120 초짜리 session_034 는 가능성이 있다 |
| 지도 기반 재정합 | 큼 | 지도를 한 번 만들고 각 프레임을 거기에 맞춘다. 구현 필요 |
| IMU 융합 | 중간 | **IMU 가 없다** |
| 재조정 간격 단축 (5 → 3 초) | 작음 | 라벨링 작업량 증가 |

지금은 5 초 재조정으로 진행한다. SLAM 교체는 4 ~ 8 단계를 전부 다시 해야 하므로, 먼저 session_034 에 루프가 있는지 확인한 뒤 따로 판단한다.
