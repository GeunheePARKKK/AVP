# 카메라 + 라이다 동시 수집

Daheng MER2-240-159U3C ×4 (하드웨어 트리거) 와 Velodyne VLP-16 을 한 세션으로
수집하고, 그 세션을 rosbag2 로 변환한다.

> **통합 워크스페이스 (2026-09-21)**
> 카메라·MCU 파이프라인(`daheng_ws_1`)과 라이다 파이프라인(별도 워크스페이스)을
> `daheng_ws_1` 한 폴더로 합쳤다. 라이다 쪽 스크립트가 예전에는
> `/home/acelab/Documents/daheng_ws_1` 경로로 카메라 모듈을 불러왔지만, 이제는
> 같은 폴더에서 바로 import 한다. `--daheng-ws` 옵션은 남아 있으나 쓸 일이 없다.
>
> 캘리브레이션 문서는 `README.md` 에 그대로 있다. 이 파일은 수집·동기화 전용이다.

## 시차 발사 (A 방식) — 2026-09-21 부터 기본

라이다는 100 ms 에 걸쳐 360° 를 훑으므로 4대를 동시에 찍으면 앞 카메라만 라이다와
맞고 뒤 카메라는 50 ms 어긋난다. 그래서 **라이다가 각 카메라 쪽을 지날 때 그
카메라를 찍는다** (nuScenes 방식). 대가로 카메라끼리는 동시가 아니다.

    FSYNC ─┬─ CAM1   +0 ms   앞      0°
           ├─ CAM2  +25 ms   오른쪽  90°
           ├─ CAM3  +50 ms   뒤    180°
           └─ CAM4  +75 ms   왼쪽  270°

- **배선 변경 없음.** GPIO4 하나가 4대에 같은 펄스를 보내고, 카메라가 `TriggerDelay`
  만큼 기다렸다 찍는다 (MER2-240 최대 3초, 실측 확인).
- 설정은 `sync_cam.py` 의 `CAMERA_AZIMUTH_DEG` 한 곳. 공간 캘리브레이션 결과가
  나오면 이 표만 고친다. `STAGGERED_TRIGGER = False` 면 예전 동시 발사.
- **노출 상한** — 지연 + 노출 + 센서 읽기(8 ms) < 100 ms. CAM4 는 **17 ms** 가
  한계다. 넘기면 `sync_cam` 이 거부한다. 어두우면 게인·조리개로 보완한다.
- **라이다 Phase Lock offset — `record.py` 가 자동으로 맞춘다.** 라이다 통과 순간을
  노출 **중앙**에 맞추려면 offset 을 노출 절반만큼 당긴다 (노출 10 ms → **342°**,
  공식 `360 − 1.8 × 노출ms`). `record.py` 가 녹화 전에 라이다 설정을 읽어 권장값과
  다르면 고치고 플래시에 저장한 뒤, 위상이 자리 잡도록 60초 기다린다. 이미 맞으면
  바로 진행한다. 노출을 바꿔도 사람이 할 일이 없다.
  - 끄려면 `--no-phaselock-auto`, 대기를 건너뛰려면 `--phase-settle 0`
  - 수동으로 할 때: `python3 lidar_config.py --phase-lock on --offset 342 --save --yes`

- **검증** — `sync_report.py` 의 "카메라별 노출 중앙의 라이다 방위각" 이 네 카메라
  모두 0 근처면 정상. 네 대가 같은 만큼 어긋나면 offset 을 권해 주고, 카메라마다
  다르면 TriggerDelay 나 장착 각도 문제라고 알려 준다.
- **결합할 때 주의** — `matched.csv` 의 한 줄(같은 N)은 같은 FSYNC 를 받은 네 장이지
  같은 순간이 아니다. 라이다와는 반드시 카메라별 시각(`camK_lidar_us`,
  `camK_unix`)으로 맞춘다.

```
                    ESP32-S3
        10 Hz FSYNC ──→ 카메라 Line0 (트리거)
        1 PPS ────────→ 라이다 I/F Box (GPS PULSE)
        GPRMC ────────→ 라이다 I/F Box (GPS RECEIVE)
        Line1 ←──────── 카메라 (노출 확인용)
              │
              │ USB 시리얼 (GO / STOP / 로그)
              ▼
           record.py ──→ sessions/session_NNN/ ──→ to_rosbag.py ──→ *.db3
              ▲
              │ USB3 (이미지)          이더넷 (UDP 브로드캐스트)
           카메라                      라이다
```

## 빠른 시작

```bash
source /opt/ros/humble/setup.bash

python3 record.py --duration 20                 # 20초 세션 1회
python3 check_session.py sessions/session_001   # 반드시 검산할 것
python3 to_rosbag.py sessions/session_001       # rosbag2 로 변환
```

라이다만 받아 볼 때 (MCU·카메라 없이):

```bash
python3 record.py --lidar-only --duration 10
python3 lidar_capture.py --duration 5           # 파일 없이 수신만 확인
```

## 파일

| 파일 | 역할 |
|---|---|
| `record.py` | 수집 오케스트레이션. MCU 핸드셰이크 + 카메라 + 라이다를 한 세션으로 묶는다 |
| `lidar_capture.py` | Velodyne UDP(2368/8308) → pcap. 단독 실행하면 수신 상태만 확인한다 |
| `vlp16.py` | VLP-16 pcap 디코더. 회전 단위 포인트클라우드를 만든다 |
| `to_rosbag.py` | 세션 폴더 → rosbag2 (`/velodyne_points`, `/cam<N>/image_raw`) |
| `check_session.py` | 세션 검산. 카메라·MCU·라이다를 한 번에 본다 |
| `sync_report.py` | 카메라↔라이다의 주파수·위상·표류 측정 |
| `lidar_config.py` | 라이다 설정 조회/변경 (rpm, Phase Lock) |
| `preview_session.py` | 세션을 눈으로 확인 (PNG / GIF / MP4) |

### 카메라 · MCU 쪽 (라이다 스크립트가 공용으로 쓴다)

| 파일 | 역할 |
|---|---|
| `sync_cam.py` | 트리거 카메라 래퍼. `CameraArray`, 카메라별 수신 스레드 |
| `collect.py` | 카메라 전용 수집. `McuLink` / `find_port` / `Preview` 는 `record.py` 도 쓴다 |
| `mcu_parse.py` | MCU 로그 / frames.csv 파서 (`check_session.py` 가 쓴다) |
| `timebase.py` | **시간축 변환 공용 모듈.** 카메라 셔터 · 라이다 점 → MCU 시계 → UNIX |
| `merge.py` | 결합표 `matched.csv`. 펄스 N 한 줄에 카메라별 셔터 시각(MCU · 라이다 · UNIX) |
| `list_cams.py` | 연결된 카메라 시리얼 확인 |
| `snap_multi.py` | 여러 대 프리뷰 · 스냅샷 · USB 대역폭 측정 (MCU 불필요) |
| `check_trigger_delay.py` | 카메라 TriggerDelay 지원 여부 확인 |
| `view_npy.py` | .npy 원본을 창으로 보거나 PNG 로 변환 |
| `preflight.py` | 우분투 환경 점검 |
| `method2_gprmc_line1_handshake.ino` | ESP32-S3 펌웨어 |

**`sync_cam.py` 를 고칠 때 주의** — 아래 이름과 시그니처를 다른 스크립트가 쓴다.

- `record.py` : `CameraArray(sns=...)`, `.queue`, `.latest`, `.slots`, `.print_stats()`,
  `CAMERA_SNS`, `DEFAULT_EXPOSURE_US`, `DEFAULT_GAIN_DB`
- `record_freerun.py` : `_must_enum`, `_must_float`(반환값 사용), `_try_enum`, `_read_*`,
  `_reader_loop`

## 세션 구조

```
sessions/session_001/
    meta.json          설정 + 카메라/라이다 요약 (수집이 끝나면 갱신된다)
    mcu_log.txt        MCU 시리얼 원문 (T / EXP / CNT / PPS / NMEA)
    lidar.pcap         UDP 2368 + 8308 원본. VeloView 로도 열린다
    cam3/
        frames.csv     frame_id, cam_timestamp, host_recv_unix
        00000000.npy   파일명 = FrameID (도착 순서가 아님)
```

**폴더 이름은 물리 슬롯 번호다.** `sync_cam.CAMERA_SNS` 안에서의 위치(1부터)로
폴더를 짓는다. 이 번호가 곧 ESP32 Line1 채널 번호이고 로그의 `EXP,<N>` 이다.
4대 중 3번만 꽂혀 있으면 `cam3/` 에 저장된다.

예전 `collect.py` 는 도착 순서로 `cam1` 부터 붙여서, 3번만 꽂으면 `cam1/` 에
들어가 어느 GPIO 가 쏜 것인지 알 수 없게 되었다. 통합하면서 `collect.py` 도
슬롯 번호를 쓰도록 고쳤다 (`CameraArray.slots`). 이제 `collect.py` 와
`record.py` 의 폴더 이름이 같다.

## 수집 순서

1. 라이다 UDP 수신 시작 — **소켓은 세션 사이에도 닫지 않는다.** 닫았다 열면 그
   틈에 커널 수신 버퍼가 넘쳐 다음 세션 첫 회전이 깨진 채로 시작한다.
2. MCU `READY` 대기
3. 카메라 열기 → 트리거 대기 (Line0 에 신호가 오기 전까지 한 장도 찍지 않는다)
4. 세션마다: pcap 열기 → `GO` → N초 → `STOP` → 큐 비우기 → pcap 닫기

수집 중에는 파싱도 계산도 하지 않는다. 예외 하나에 세션이 통째로 날아가기
때문이다. 검증은 전부 `check_session.py` 로 미룬다.

## 타임스탬프

두 센서 모두 **호스트 수신 시각(UNIX)** 을 rosbag 메시지 스탬프로 쓴다.
라이다 PPS 가 물려 있지 않은 동안에는 이것이 유일한 공통 시계다.

| | 스탬프 | 오차 요인 |
|---|---|---|
| 라이다 | 회전 첫 패킷의 호스트 수신 시각 | 이더넷 도착 + 스케줄링 지터 (실측 최대 4 ms) |
| 카메라 | `frames.csv` 의 `host_recv_unix` | USB3 전송 + 스케줄링 지터 |

더 정밀하게 맞추려면 PPS 를 물린 뒤 `mcu_log.txt` 의 `T` / `PPS` / `NMEA` 줄로
보정해야 한다.

> **2026-09-21 — `to_rosbag.py` 가 MCU 시계 기준으로 바뀌었다.** 위 표는 예전
> 방식(`--host-time`)이다. 지금 기본은 이렇다.
>
> | | 스탬프 |
> |---|---|
> | 카메라 | 노출 중앙 = `T[N]` + 카메라별 지연(Line1 실측) + 노출/2 |
> | 라이다 | 회전 첫 점의 라이다 시각 → PPS·GPRMC 대응표로 MCU 시각 |
> | UNIX | MCU + 세션 앵커 (라이다 패킷 최소 수신 지연으로 잡음) |
>
> 상대 시각은 µs, 절대 시각은 ms 수준. 두 센서가 같은 앵커를 쓰므로 앵커
> 오차는 센서 사이 정합에 영향이 없다. MCU 로그가 없거나(프리런) 라이다가
> GPRMC 를 받지 않았으면 자동으로 호스트 시각으로 내려가며 경고한다.
>
> 합성 세션 검증: rosbag 시간축만으로 역산한 '사진 순간 라이다 방위각' 이 네
> 카메라 모두 목표 대비 0.07° 이내. 같은 세션을 `--host-time` 으로 넣으면 47°
> (13 ms) 어긋나고 2.5° 흔들린다. 그 값은 원본 그대로 세션에 남아 있으므로 나중에 다시 풀 수 있다.
결합 규칙은 `N = FrameID + offset` 으로 `T,<N>,<us>` 를 조회하는 것이고,
`offset` 은 `check_session.py` 가 MCU 의 첫 `EXP` 줄에서 확정해 준다.

## 검증된 수치

이 장비에서 실측한 값이다 (2026-09-09, 10 Hz 단일반사).

| 항목 | 값 |
|---|---|
| 라이다 패킷 유실 | **0** (라이다 자체 타임스탬프 기준, 8초 세션 6261 패킷) |
| 라이다 회전 주기 | 100.00 ± 0.03 ms → 10.000 Hz |
| 회전당 점 | 평균 28,466 (거리 0 제외) |
| ring 고도각 오차 | 16채널 전부 **0.0000°** (사양 대조) |
| 디코딩 속도 | 285만 점 / 0.31 s |
| 카메라 | 6초에 60장, FrameID 끊김 0, SDK lost 0 |
| 라이다 시계 드리프트 | +0.5 ~ +1.3 ms / 8초 (60~185 ppm) |
| 카메라↔라이다 표류 (Phase Lock 후) | −0.0066 ms/s → 253분마다 1프레임 |
| 카메라 지연 (노출 + 전송) | 노출시간 + 7.1 ms (호스트 도착 기준) |
| 트리거 → 노출 시작 | 12.8 µs (09-15), 2~10 µs (09-08). 값이 다른 원인 확인 중 |
| MCU 트리거 · PPS 지터 | ±1 µs |
| 셔터 순간 라이다 방위각 | 평균 −0.007°, 표준편차 0.34° (= 94 µs), 200회 |
| FrameID 시작 번호 | 0 (offset = 1) |

`유실 패킷` 은 **라이다가 패킷에 찍은 시각** 으로 센다. 호스트 도착 시각으로
세면 안 된다 — 리눅스가 수신 스레드를 늦게 깨우면 패킷은 멀쩡히 다 왔는데도
도착 시각에 몇 ms 짜리 구멍이 생겨서, 실제로는 유실이 0인데 7곳이 비었다고
보고하게 된다.

## 지금 이 리그의 미해결 사항

### 1. ~~두 센서의 위상이 묶여 있지 않다~~ → 해결 (2026-09-09)

**하드웨어 동기가 걸렸다.** PPS `Locked`, Phase Lock `On`, 모터 `lock=On`.

무엇을 했나

1. ESP32 GPIO5 → **74AHCT125** 비반전 5V 버퍼 → I/F Box `GPS PULSE`
   (맨 GPIO 직결은 레벨이 부족해 `Absent` 였다)
2. **라이다 전원 재투입** — 이게 결정적이었다. 라이다가 켜진 상태에서
   PPS 선을 새로 물렸더니 입력단이 인식하지 못하는 상태로 남아 있었다.
   배선 후 전원을 내렸다 올리자 곧바로 `Locked` 이 되었다.
   *배선은 라이다 전원을 내리고 할 것.*
3. `python3 lidar_config.py --phase-lock on --offset 0 --yes`

Phase Lock 은 PPS 펄스가 들어오는 순간 라이다가 지정한 방위각을 보고
있도록 회전 위상을 맞춘다. 600 rpm 이 정확히 10 Hz 이므로 한 번 맞으면
그 초 안의 회전 10번이 전부 결정된 시각에 일어난다. ESP32 가 FSYNC 와
PPS 를 같은 타이머에서 만들기 때문에, 이때부터 카메라 셔터와 라이다
방위각이 한 시계 위에 놓인다.

측정값 (`sync_report.py`)

| | Phase Lock 전 | 후 |
|---|---|---|
| 표류 | −0.057 ms/s | **−0.0066 ms/s** |
| 한 프레임 밀리기까지 | 29분 | **253분** |
| 카메라↔라이다 간격 | 72.2 ms | 15.0 ms |

**남은 15 ms 는 어긋남이 아니라 카메라의 지연이다.** 노출을 8 ms 에서
4 ms 로 줄이자 간격이 15.0 → 11.1 ms 로 3.9 ms 만큼 같이 줄었다.

    간격 = 노출시간 + 7.1 ms   (센서 읽기 + USB3 전송, 2.46 MB)

즉 셔터가 열리는 순간과 라이다가 0도를 지나는 순간은 이미 일치하고,
`frames.csv` 의 `host_recv_unix` 가 프레임이 **도착한** 시각이라 그만큼
늦게 찍힐 뿐이다. 결합할 때는 이 상수를 빼면 된다.

### 1-b. ~~GPRMC 는 아직 안 들어온다~~ → 해결 (MAX3232)

처음에는 아래처럼 2N2222 반전 회로로 시도했으나 박스 수신기가 음전압을 요구해
통과하지 못했다. **MAX3232 (TTL → RS-232) 로 바꾼 뒤 정상**이다.

    ESP32 GPIO17 → MAX3232 TXD 입력,  MAX3232 VCC 5V,  GND 공통
    MAX3232 DB9 2번 → I/F Box GPS RECEIVE

`lidar-pps-gprmc-wiring.md` 의 2N2222 절은 기록으로만 남아 있다.

<details><summary>당시 기록 (2N2222 시절)</summary>


PPS 는 `Locked` 인데 NMEA 필드는 전부 `00` 이고 `gps.position` 이 비어
있다. **위상 동기에는 영향이 없다** — GPRMC 는 절대 UTC 시각을 넣어주는
역할이라, 없으면 라이다 타임스탬프의 '정시' 기준이 서지 않을 뿐이다.

2N2222 반전 회로 쪽을 봐야 한다. 확인 순서는
`lidar-pps-gprmc-wiring.md` 참고. 풀업 4.7 kΩ 은 박스 입력이 RS-232
수신기(입력 임피던스 3~7 kΩ)일 경우 분압되어 HIGH 가 2.6 V 까지
내려간다. **1 kΩ 으로 낮추면 4.2 V 가 되어 여유가 생긴다.**

</details>

### 1-c. ~~시리얼을 안 읽으면 PPS 가 끊긴다~~ → 펌웨어 수정 (하드웨어 검증 필요)

노트북이 ESP32 시리얼을 읽지 않으면 USB 송신 버퍼가 차서 `Serial.printf` 가
막히고, `loop()` 가 멈춘 동안 PPS 가 HIGH 로 고정되었다
(2026-09-14 실측: 미수신 시 Absent 1093/1093, 계속 읽으면 2.3초 만에 Locked).
그래서 `record.py` 의 `SerialDrain` 이 녹화 사이에도 시리얼을 읽어 줘야 했다.

펌웨어에서 두 가지를 고쳤다.

1. USB 로그를 비블로킹으로 (`ulogf`). 자리를 최대 5 ms 기다리는 동안에도 펄스를
   내리고, 그래도 막히면 그 줄을 버린다. 녹화 중에는 노트북이 읽고 있으므로
   5 ms 안에 비워져 손실이 없다.
2. 타이머 ISR 에서 PPS 틱이 아닐 때 PPS 를 강제로 내린다. 무슨 일이 있어도
   매초 상승 엣지는 생긴다.

**검증 방법** — 펌웨어를 올린 뒤 노트북에서 시리얼을 열지 않은 상태로 두고
`python3 lidar_config.py` 로 PPS 가 `Locked` 를 유지하는지 확인한다.
`SerialDrain` 은 그대로 두어도 해가 없다.

### 2. Line1 노출 카운터가 다른 채널에 잡힌다 / 과다 계수

**현재 Line1 입력 핀: CAM1=GPIO6, CAM2=GPIO15, CAM3=GPIO38, CAM4=GPIO39**
(보드 양쪽 헤더에 나눠 배치하려고 7/15/16 에서 바꿨다. 38/39 는 PSRAM 유무와
무관하게 항상 쓸 수 있다.)

원인이 여럿 섞여 있었다.

- **배선 순서** — `CAMERA_SNS` 의 슬롯 순서와 실제 GPIO 배선이 다르면 다른
  채널이 센다. 펌웨어 `L` 명령으로 각 핀의 레벨과 계수를 바로 볼 수 있다.
- **브레드보드 접촉** — 카메라 케이블 연선을 브레드보드에 직접 꽂으면 매 녹화마다
  되는 채널이 바뀌었다. 나사 단자대로 옮길 것.
- **레벨 확인 필터** — 펌웨어의 `EXP_LEVEL_CHECK` 를 켰을 때 계수가 0 이 되었다.
  기본값을 0 (끔) 으로 바꾸고 락아웃(50 ms)만으로 막는다. 조건은
  `노출시간 < EXP_LOCKOUT_US < 100 ms`.

<details><summary>당시 기록</summary>


`CNT,60,0,104,0,0` — 60번 쐈고 3번 카메라만 꽂혀 있는데, 노출 신호는 채널 3
(GPIO38) 이 아니라 **채널 2 (GPIO15)** 가 세고 있다. 둘 중 하나다.

- 카메라의 Line1 배선이 GPIO15 쪽 가지에 물려 있다
- `sync_cam.CAMERA_SNS` 의 SN↔슬롯 순서가 실제 배선과 다르다

또 60번 쐈는데 **104번** 세었다. 입력 핀 채터링(풀업 4.7k / 노이즈)을
의심할 만하다. 사진 자체는 정확히 60장, FrameID 끊김 0으로 정상이므로
**촬영에는 영향이 없고 사후 결합용 `offset` 확정만 막힌다.**

`check_session.py` 가 이 두 경우를 구분해서 알려 준다.

</details>

### 3. 두 센서의 시야가 겹치지 않는다

카메라가 천장과 벽 모서리를 올려다보고 있어 라이다 수평면과 공통 시야가 거의
없다. 함께 쓰려면 카메라 향을 라이다 평면 쪽으로 맞춰야 하고, 그 방위각이
정해져야 Phase Lock 의 `offset` 도 정할 수 있다.

### 4. 카메라 노출

기본값 8000 us / 8 dB 는 지금 조명에서 어둡다 (화소 평균 10.6/255).
`--exposure` / `--gain` 으로 올리되, 트리거 주기 100 ms 의 80% 인 **80 ms 를
넘기면 sync_cam 이 거부한다** — 노출이 주기보다 길면 다음 트리거를 씹는다.

## rosbag2

```bash
python3 to_rosbag.py sessions/session_001              # <session>/rosbag 에 생성
python3 to_rosbag.py sessions/session_001 -o ~/ros2_ws/cam_lidar_recording
python3 to_rosbag.py sessions/session_001 --debayer    # 바로 보이는 BGR (용량 3배)
python3 to_rosbag.py sessions/session_001 --storage mcap
```

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 회전당 1개. `x,y,z,intensity,ring,time` |
| `/cam<N>/image_raw` | `sensor_msgs/Image` | 기본은 Bayer 원본 (`bayer_gbrg8`) |

`frame_id` 는 각각 `velodyne`, `cam<N>`. 좌표계는 `velodyne_pointcloud` 와 같은
ROS 관례(x 전방, y 좌측, z 상방)로 변환해서 넣는다.

```bash
ros2 bag info  sessions/session_001/rosbag
ros2 bag play  sessions/session_001/rosbag
rviz2   # Fixed Frame 을 velodyne 로, /velodyne_points 추가
```

앞뒤로 잘린 회전 2개는 제외하고 넣는다. 첫 회전은 캡처를 시작한 순간부터,
마지막 회전은 멈춘 순간까지만 담겨 있어 360°가 아니다.

`ros2 bag record` 를 쓰지 않는 이유: 라이다 드라이버(`ros-humble-velodyne`)와
Daheng 카메라용 ROS2 노드가 둘 다 없고, 카메라의 하드웨어 트리거 타임스탬프를
ROS 시간에 싣는 작업이 별도로 필요하다. 원본을 먼저 안전하게 받아 두고 변환은
나중에 다시 돌릴 수 있게 나눴다.

## 환경 요구사항

`preflight.py` 로 점검한다. 이 장비는 현재 전부 충족한다.

| 항목 | 필요 | 현재 |
|---|---|---|
| Galaxy SDK | `/usr/lib/libgxiapi.so` | 설치됨 |
| `usbfs_memory_mb` | 1000 (4대 동시) | 1000 |
| 시리얼 권한 | `dialout` 그룹 | 속함 |
| 라이다 인터페이스 | 192.168.1.0/24 | `enx80691ae96d22` = 192.168.1.100 |
| 디스크 | 20초 세션당 약 2 GB (4대) | 1.7 TB 여유 |

UDP 포트 2368/8308 을 다른 프로그램(VeloView, `velodyne_driver`)이 잡고 있으면
캡처가 열리지 않는다. `ss -ulpn 'sport = :2368'` 로 확인한다.

## 예전 세션 데이터

라이다 워크스페이스에서 찍은 세션(`session_003`, `session_017`, `session_020` 등)은
그쪽 `sessions/` 에 그대로 있다. 이 폴더의 스크립트로 그대로 읽을 수 있다.

    python3 check_session.py <라이다_ws>/sessions/session_020
    python3 sync_report.py   <라이다_ws>/sessions/session_020

새로 찍는 세션은 이 폴더의 `sessions/` 에 쌓인다.
