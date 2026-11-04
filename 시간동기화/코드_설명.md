# code

| 파일 | 역할 |
|---|---|
| `record.py` | 수집. ESP32 핸드셰이크(GO/STOP) + 카메라 + 라이다를 한 세션으로. `--lidar-only`로 라이다만 |
| `lidar_capture.py` | Velodyne UDP 2368/8308 → pcap. 일반 UDP 소켓으로 받아 헤더를 다시 붙이므로 root 권한 불필요 |
| `vlp16.py` | VLP-16 pcap 디코더 (numpy 벡터화, 방위각 보간). 링 고도각 오차 0.0000°, 285만 점 / 0.31 s |
| `check_session.py` | 세션 검산 — ESP32 트리거·노출 카운트, 라이다 유실·회전·PPS, 카메라 FrameID |
| `sync_report.py` | 동기 판정 — 라이다 위상, 카메라↔라이다 간격, GPRMC 시각 대조, **셔터 순간 라이다 방위각**, **카메라 트리거 → 노출 지연** |
| `lidar_config.py` | 라이다 설정 조회·변경 (rpm, Phase Lock), 플래시 저장, PPS/NMEA 실시간 감시 |
| `to_rosbag.py` | 세션 → rosbag2 (`/velodyne_points`, `/cam<N>/image_raw`) |
| `preview_session.py` | 카메라 프레임 + 라이다 평면도·측면도를 PNG / GIF / MP4로 |

## 필요한 것

- Python 3.10, numpy, pyserial, opencv-python, Pillow
- ROS 2 Humble (`to_rosbag.py`만)
- 카메라 모드: Daheng Galaxy SDK + `~/Documents/daheng_ws_1`의 `sync_cam.py`, `collect.py`, `vendor/gxipy` (이 레포에는 없음, `--daheng-ws`로 경로 지정)

## 흐름

```bash
python3 lidar_config.py --watch                # PPS / NMEA 실시간 (배선할 때)
python3 lidar_config.py --phase-lock on --offset 0 --save --yes

python3 record.py --duration 20                # sessions/session_NNN/
python3 check_session.py sessions/session_NNN
python3 sync_report.py   sessions/session_NNN
python3 to_rosbag.py     sessions/session_NNN  # sessions/session_NNN/rosbag
python3 preview_session.py sessions/session_NNN --mp4
```

## 세션 폴더

```
session_NNN/
├── meta.json      설정, 카메라 정보, 라이다 설정·상태 스냅샷(Phase Lock offset 포함), 요약
├── mcu_log.txt    ESP32 시리얼 원문 — T(트리거), PPS, NMEA, CNT
├── lidar.pcap     UDP 2368 + 8308 원본 (VeloView로도 열림)
└── cam<N>/
    ├── frames.csv     frame_id, cam_timestamp, host_recv_unix
    └── 00000000.npy   raw Bayer, 파일명 = FrameID
```

`cam<N>`의 N은 `sync_cam.CAMERA_SNS` 안의 **물리 슬롯 번호**입니다 (도착 순서가 아님).

## 설계에서 중요한 것

- **ESP32 시리얼을 항상 읽습니다.** 현재 펌웨어는 USB 출력이 막히면 `loop()`가 멈춰 PPS를 내리지 못합니다. `record.py`는 라이다 단독 모드에서도 명령 없이 시리얼을 읽고, MCU 연결 직후 PPS가 `Locked`로 돌아올 때까지 기다린 뒤 녹화합니다.
- **유실은 라이다 타임스탬프로 셉니다.** 호스트 도착 시각으로 세면 스케줄링 지터를 유실로 오판합니다.
- **수집 중에는 파싱하지 않습니다.** 원본(pcap, npy, 로그)만 저장하고 판정·변환은 나중에 합니다.
- **라이다 설정은 되읽어 확인합니다.** 이 장비의 웹 서버는 잘못된 요청에도 200/204를 줄 수 있어서 `lidar_config.py`는 쓴 뒤 항상 설정을 다시 읽어 대조합니다. Phase Lock offset은 0.01° 단위로 변환해 보냅니다.
