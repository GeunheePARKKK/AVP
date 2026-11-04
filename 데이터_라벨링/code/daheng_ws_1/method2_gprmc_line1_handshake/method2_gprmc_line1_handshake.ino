/* =============================================================================
 *  방식 2 + GPRMC + Line1 + 핸드셰이크
 *  -----------------------------------------------------------------------
 *  기존 method2_with_gprmc.ino 에서 바뀐 것
 *
 *  [1] Line1 (ExposureActive) 4채널 입력 추가
 *      - 카메라 4대가 "찍었다"고 알려주는 신호를 GPIO 인터럽트로 받는다.
 *      - 시각을 정밀하게 재는 것이 아니라 "몇 번 노출했나"를 세는 것이 목적.
 *
 *  [2] EXP / CNT 로그를 기존 USB 시리얼로 함께 송신
 *      - 별도 포트 없이 줄 앞 태그로 구분한다.
 *
 *  [3] 핸드셰이크
 *      - 부팅해도 FSYNC 를 쏘지 않고 대기한다. 노트북의 "GO" 를 기다린다.
 *      - PPS 와 GPRMC 는 부팅 즉시 시작한다 (라이다는 카메라와 무관하게
 *        계속 시각을 받아야 하므로).
 *
 *  [4] 시각을 64비트로 (기존 32비트는 71분 35초마다 0으로 되돌아감)
 *
 *  [5] USB 를 읽는 쪽이 없어도 PPS / GPRMC 가 정상으로 나간다
 *      예전에는 노트북이 시리얼을 안 읽으면 로그 출력이 막혀 loop() 가 서고,
 *      PPS 가 HIGH 로 고정되어 라이다가 PPS 를 잃었다 (Absent). 그래서
 *      record.py 가 녹화 사이에도 시리얼을 계속 읽어 줘야 했다.
 *      → USB 로그를 비블로킹으로 바꾸고(ulogf), ISR 에 PPS 강제 하강을 넣었다.
 *        라이다를 켜 둔 채 노트북 없이 대기해도 Locked 가 유지되어야 한다.
 *
 *  대상 보드 : ESP32-S3 (Arduino-ESP32 core 2.x / 3.x 공용)
 *  ※ Arduino IDE 에서 USB CDC On Boot 설정에 따라 Serial 이 네이티브 USB(C타입)
 *    인지 UART0 인지 달라진다. C to USB 로 로그를 받으려면 "USB CDC On Boot:
 *    Enabled" 로 두고 C타입 포트에 꽂을 것.
 *
 *  배선  (5V 로우사이드 방식 — 회로도 PNG 참조)
 *    [출력] GPIO4  : 10 Hz FSYNC  → R1 1kΩ → Q1~Q4 베이스 (1핀이 4채널 공통)
 *                                   +5V → R3 → 카메라 Pin1(보라, Line0+)
 *                                   카메라 Pin3(파랑, Line0−) → Q 컬렉터 → GND
 *           GPIO5  : 1 PPS        → 버퍼 → I/F Box GPS PULSE (노랑)
 *           GPIO17 : GPRMC TX     → MAX3232 → I/F Box GPS RECEIVE (흰색)
 *    [입력] GPIO6  : CAM1 Line1
 *           GPIO15 : CAM2 Line1
 *           GPIO38 : CAM3 Line1
 *           GPIO39 : CAM4 Line1
 *           각 채널 3.3V 풀업 4.7k.
 *           카메라 Pin8(주황, Line1+) → 풀업 노드 → GPIO
 *           카메라 Pin7(초록, Line1−) → 공통 GND
 *           옵토 절연 출력이므로 노출 중이 LOW → FALLING 엣지로 계수.
 *
 *  카메라 커넥터 (HR25 8핀 / 서드파티 케이블 실측)
 *    Pin1 보라 Line0+   Pin2 검정 GND(미사용)   Pin3 파랑 Line0−
 *    Pin4 빨강 NC       Pin5 노랑 Line2(미사용) Pin6 갈색 Line3(미사용)
 *    Pin7 초록 Line1−   Pin8 주황 Line1+
 *    ※ 정품 케이블과 색 배열이 다르다. 반드시 핀 번호 기준으로 배선할 것.
 *    ※ 노랑·갈색은 비절연 3.3V 로직이므로 5V 접촉 시 카메라 손상. 개별 절연 필수.
 *    GND    : 공통 접지
 *
 *  ----------------------------------------------------------------------
 *  노트북 → MCU 명령 (개행으로 끝냄)
 *    GO    : 촬영 시작 (FSYNC 송출 개시, 카운터 리셋)
 *    STOP  : 촬영 정지 (FSYNC 정지, CNT 요약 출력)
 *    DLY,a,b,c,d : 카메라별 TriggerDelay(us). Line1 엣지를 이 위상 근처만 받는다
 *    PING  : READY 재출력 (연결 확인용)
 *    L     : Line1 배선 진단 (각 핀 레벨과 누적 계수 출력)
 *    R     : 통계 리셋
 *
 *  MCU → 노트북 로그
 *    READY                          부팅 완료. 이 줄을 받고 GO 를 보낼 것
 *    MODE,RUN / MODE,IDLE           명령 수신 확인
 *    T,<N>,<us>                     트리거 발사
 *    PPS,<seq>,<us>                 1PPS 발사
 *    NMEA,<PPS엣지us>,<전송시작us>,<지연us>,<송신소요us>,<PPS번호>,<문장>
 *    EXP,<cam>,<N>,<us>             카메라 노출 감지 (세션 앞부분만)
 *    CNT,<N>,<c1>,<c2>,<c3>,<c4>    쏜 횟수 vs 카메라별 노출 횟수
 *    # ...                          주석 / 통계
 * ========================================================================== */

#include <Arduino.h>
#include <stdarg.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include <WiFi.h>

// ---------------------------- 사용자 설정 -----------------------------------
#define PIN_10HZ         4       // 10 Hz FSYNC 출력 핀 (0~31)
#define PIN_PPS          5       // 1 PPS 출력 핀 (0~31)
#define PIN_NMEA_TX      17      // GPRMC 송신 핀 (UART1 TX)

// --- Line1 (ExposureActive) 입력 4채널 ---
#define N_CAM            4
static const uint8_t PIN_EXP[N_CAM] = { 6, 15, 38, 39 };  // CAM1~CAM4
// ※ 보드 양쪽 헤더에 나눠 배치하기 위한 배열.
//   GPIO38/39 는 모듈 종류(PSRAM 유무)와 무관하게 항상 쓸 수 있다.
//   GPIO33~37 은 옥탈 PSRAM 모듈에서 내부 점유되므로 피한다.
//   GPIO39 는 JTAG(MTCK) 겸용이나 GPIO 사용에는 문제 없다.

// 옵토(오픈컬렉터) 출력 + 외부 풀업이면 노출 중이 LOW 이므로 FALLING.
// 자체 점퍼 테스트(GPIO4 → GPIO6 직결)로 검증할 때는 RISING 으로 바꿀 것.
#define EXP_EDGE         FALLING

// 외부 4.7k 풀업을 달았으면 0, 저항 없이 내부 풀업(약 45k)을 쓰려면 1
#define EXP_INTERNAL_PULLUP  0

// 한 번의 노출이 여러 인터럽트로 세지는 것을 막는 최소 간격.
// 조건 :  노출시간 < EXP_LOCKOUT_US < 트리거 주기(100,000 us)
//   - 노출시간보다 커야 노출 종료 지점의 튐이 걸러진다
//   - 트리거 주기보다 작아야 진짜 다음 노출을 놓치지 않는다
// 노출 20 ms 기준으로 50 ms 는 양쪽 조건을 모두 만족한다.
// 노출을 60 ms 이상으로 올리려면 이 값도 함께 올릴 것 (예: 80,000).
#define EXP_LOCKOUT_US   50000LL

// 글리치 필터: 하강 엣지 인터럽트가 걸린 직후 핀이 실제로 활성 레벨인지
// 확인한다.
//   0 = 끔 (기본)   1 = 켬
//
// [기본값을 0 으로 둔 이유]
// 인터럽트 진입부터 이 코드가 실행되기까지 수 us 가 걸린다. 옵토 출력은
// 상승/하강이 느려서 그 사이에 레벨이 아직 확정되지 않았거나 이미 복귀해
// 있을 수 있고, 그러면 진짜 노출까지 걸러져 계수가 0 이 된다. 실제로 이
// 필터를 켠 뒤 CNT 가 0 으로 나오는 현상이 관측되었다.
//
// 노출 종료 지점의 튐은 아래 EXP_LOCKOUT_US 로 막는다. 노출 시간이
// 락아웃보다 짧으면 (예: 노출 20 ms < 락아웃 50 ms) 종료 지점이 락아웃
// 안에 들어와 자동으로 걸러지므로 이 필터가 필요 없다.
//
// 노출을 락아웃보다 길게(70 ms 이상) 쓰면서 유령 계수가 생기면, 이 값을
// 1 로 켜기 전에 EXP_LOCKOUT_US 를 노출시간보다 크고 트리거 주기(100 ms)
// 보다 작은 값으로 먼저 조정할 것.
#define EXP_LEVEL_CHECK  0

// Line1 엣지를 '기대 위상' 근처에서만 받아들인다.
//
// [왜 필요한가]
// FSYNC 가 나가는 순간 Line0 전류가 바뀌면서 Line1 입력에 가짜 하강 엣지가
// 유도된다. 그걸 노출로 세면 진짜 노출이 가려지고, CNT 가 트리거 누락을
// 감지하지 못한다.
//   2026-09-22 실측 (session_004/006/007/008)
//     CAM1(GPIO6)  잡음 +3 us, 진짜 +25 ms
//     CAM3(GPIO38) 잡음 없음, 진짜 +5 us     ← 시간만으로는 잡음과 구분 불가
//     CAM4(GPIO39) 잡음 +3 us 와 가끔 +1 ms, 진짜 +50 ms
//     CAM2(GPIO15) 잡음 없음, 진짜 +75 ms
//
// 잡음(3 us)과 CAM3 의 진짜 노출(5 us)이 겹치므로 '이른 엣지 = 잡음' 규칙은
// 쓸 수 없다. 대신 카메라마다 노출이 올 시점(TriggerDelay)을 알고 그 근처만
// 받는다. 기대 위상은 노트북이 GO 전에 DLY 명령으로 알려준다 (기본 0).
#define EXP_MATCH_US     1500UL

// 세션 시작 후 카메라당 이 개수까지만 EXP 줄을 출력한다 (offset 확정용).
// 이후에는 세기만 하고 CNT 로 요약한다. 0 = 전부 출력.
#define EXP_LOG_LIMIT    10

#define NMEA_BAUD        9600    // Velodyne 표준 (변경 금지 권장)

#define FREQ_HZ          10      // 기준 주파수
#define PPS_DIVIDER      10      // FREQ_HZ / PPS_DIVIDER = 1 Hz
#define TICK_US          (1000000UL / FREQ_HZ)

#define FSYNC_WIDTH_US   1000LL  // 10 Hz High 폭 (1 ms)
#define PPS_WIDTH_US     20000LL // 1 PPS High 폭 (20 ms)

// --- 시작 시각 (UTC) : 테스트용 임의값. 첫 PPS 가 이 시각으로 나간다 ---
#define START_HH         12
#define START_MM         0
#define START_SS         0
#define START_DAY        14
#define START_MON        8
#define START_YEAR       26     // 두 자리

// --- 고정 더미 좌표 : 대구 (35.8714 N, 128.6014 E) ---
#define FIX_LAT          "3552.2840"
#define FIX_NS           "N"
#define FIX_LON          "12836.0840"
#define FIX_EW           "E"

#define SERIAL_BAUD      921600
#define STAT_PERIOD_MS   10000
#define PRINT_EVENTS     1       // 1 = T / PPS 줄 출력, 0 = 통계만
#define PRINT_NMEA       1       // 1 = 전송한 GPRMC 원문도 USB 로 출력
// ---------------------------------------------------------------------------

#if (PIN_10HZ > 31) || (PIN_PPS > 31)
  #error "PIN_10HZ / PIN_PPS 는 GPIO0~31 범위를 사용하세요"
#endif
// Line1 입력은 GPIO32 이상도 쓸 수 있다 (expISR 이 두 뱅크를 모두 처리).

#define MASK_10HZ  (1UL << PIN_10HZ)
#define MASK_PPS   (1UL << PIN_PPS)

#define EV_FSYNC 0
#define EV_PPS   1
#define EV_EXP   2
#define LOG_N    256

/* ===========================================================================
 *  타입 정의
 * ======================================================================== */
struct LogEvt {
  uint8_t  type;
  uint8_t  cam;      // EV_EXP 일 때만 사용 (0~3)
  uint32_t seq;      // EV_FSYNC: N / EV_PPS: PPS번호 / EV_EXP: 그 시점의 N
  int64_t  t_us;
};

struct Stat {
  uint32_t n;
  uint32_t mn;
  uint32_t mx;
  uint64_t sum;
};

struct UtcClock {
  uint8_t hh, mm, ss;
  uint8_t day, mon, yr;
};

enum Mode { M_IDLE = 0, M_RUN = 1 };

/* ===========================================================================
 *  전역 변수
 * ======================================================================== */
static LogEvt            g_log[LOG_N];
static volatile uint16_t g_wr = 0, g_rd = 0;
static volatile uint32_t g_drop = 0;
static portMUX_TYPE      g_mux = portMUX_INITIALIZER_UNLOCKED;

static hw_timer_t *g_timer = NULL;

// --- 모드 ---
static volatile Mode     g_mode = M_IDLE;

// --- 카운터 ---
// g_tick   : 타이머가 몇 번 울렸나 (PPS 분주용, 계속 증가)
// g_fsyncN : 실제로 몇 발 쐈나 (= 트리거 번호 N, GO 마다 1부터)
static volatile uint32_t g_tick   = 0;
static volatile uint32_t g_fsyncN = 0;
static volatile uint32_t g_ppsSeq = 0;

static volatile int64_t  g_fsyncFallAt  = 0;
static volatile int64_t  g_ppsFallAt    = 0;
static volatile bool     g_fsyncPending = false;
static volatile bool     g_ppsPending   = false;
static volatile int64_t  g_ppsEdgeUs    = 0;
static volatile uint32_t g_fsyncAt32    = 0;   // 마지막 FSYNC 시각 (하위 32비트)

// --- Line1 (노출 감지) ---
static volatile uint32_t g_expCnt[N_CAM]    = {0, 0, 0, 0};   // 세션 내 노출 횟수
static volatile int64_t  g_expLastUs[N_CAM] = {0, 0, 0, 0};   // 락아웃 판정용
static volatile uint32_t g_expLogged[N_CAM] = {0, 0, 0, 0};   // 출력한 EXP 줄 수
static volatile uint32_t g_expOther[N_CAM]  = {0, 0, 0, 0};   // 기대 밖이라 버린 엣지
static volatile uint32_t g_expExpect[N_CAM] = {0, 0, 0, 0};   // 기대 위상 (us, DLY 로 설정)

static UtcClock g_clk = { START_HH, START_MM, START_SS,
                          START_DAY, START_MON, START_YEAR };
static bool     g_firstPps = true;
static uint32_t g_nmeaHandled = 0;
static uint32_t g_nmeaSent = 0;
static uint32_t g_nmeaSkipped = 0;
static int64_t  g_nmeaMaxLagUs = 0;

static Stat     g_sF, g_sP;
static int64_t  g_lastF = 0, g_lastP = 0;
static uint32_t g_statMs = 0;

static char g_sentence[96];

// 명령 수신 버퍼
// DLY,25000,75000,0,50000 처럼 긴 명령이 들어온다. 짧으면 조용히 잘려서
// 뒤쪽 카메라의 기대 위상이 0 으로 들어간다 (2026-09-22 session_009 에서
// CAM4 가 50 ms 대신 0 으로 잡혀 잡음을 셌다).
static char   g_cmd[64];
static uint8_t g_cmdLen = 0;
static bool    g_cmdOverflow = false;

// ------------------------- 로그 링버퍼 (ISR → loop) -------------------------
static inline void IRAM_ATTR pushLog(uint8_t type, uint8_t cam,
                                     uint32_t seq, int64_t t) {
  portENTER_CRITICAL_ISR(&g_mux);
  uint16_t nxt = (uint16_t)((g_wr + 1) % LOG_N);
  if (nxt == g_rd) {
    g_drop++;
  } else {
    g_log[g_wr].type = type;
    g_log[g_wr].cam  = cam;
    g_log[g_wr].seq  = seq;
    g_log[g_wr].t_us = t;
    g_wr = nxt;
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

static bool popLog(LogEvt &e) {
  bool ok = false;
  portENTER_CRITICAL(&g_mux);
  if (g_rd != g_wr) {
    e = g_log[g_rd];
    g_rd = (uint16_t)((g_rd + 1) % LOG_N);
    ok = true;
  }
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

// ------------------------------ 타이머 ISR ----------------------------------
// PPS 는 모드와 무관하게 항상 나간다 (라이다 시계 유지).
// FSYNC 는 M_RUN 일 때만 나간다 (핸드셰이크).
void IRAM_ATTR onTimer() {
  uint32_t n       = ++g_tick;
  bool     ppsTick = ((n % PPS_DIVIDER) == 1);
  bool     fsyncTick = (g_mode == M_RUN);

  // 안전장치: PPS 는 원래 loop() 가 20 ms 뒤에 내린다. 어떤 이유로든
  // loop() 가 멈추면 PPS 가 HIGH 로 고정되어 다음 초의 상승 엣지가 사라지고
  // 라이다가 PPS 를 잃는다. PPS 틱이 아닌 모든 틱(100 ms 뒤)에 강제로
  // 내려서, 최악의 경우에도 매초 상승 엣지는 반드시 생기게 한다.
  // 정상 동작에서는 이미 내려가 있으므로 아무 일도 하지 않는다.
  if (!ppsTick) REG_WRITE(GPIO_OUT_W1TC_REG, MASK_PPS);

  uint32_t mask = (fsyncTick ? MASK_10HZ : 0) | (ppsTick ? MASK_PPS : 0);
  if (mask) REG_WRITE(GPIO_OUT_W1TS_REG, mask);   // ← 펄스가 여기서 남

  int64_t t = esp_timer_get_time();

  if (fsyncTick) {
    // 32비트로 둔다. ISR 두 곳에서 읽고 쓰므로 64비트면 찢어질 수 있다.
    g_fsyncAt32    = (uint32_t)t;
    g_fsyncFallAt  = t + FSYNC_WIDTH_US;
    g_fsyncPending = true;
    pushLog(EV_FSYNC, 0, ++g_fsyncN, t);
  }

  if (ppsTick) {
    g_ppsFallAt  = t + PPS_WIDTH_US;
    g_ppsPending = true;
    g_ppsEdgeUs  = t;
    pushLog(EV_PPS, 0, ++g_ppsSeq, t);
  }
}

// --------------------- Line1 핀 레벨 읽기 -----------------------------------
// ESP32-S3 는 GPIO 입력 레지스터가 두 뱅크로 나뉜다.
//   GPIO_IN_REG  : GPIO0~31 / GPIO_IN1_REG : GPIO32~48 (비트 = 핀번호 - 32)
static inline bool IRAM_ATTR expLevel(uint8_t pin) {
  if (pin < 32) return (REG_READ(GPIO_IN_REG)  & (1UL << pin)) != 0;
  else          return (REG_READ(GPIO_IN1_REG) & (1UL << (pin - 32))) != 0;
}

// --------------------- Line1 (노출) 인터럽트 --------------------------------
// 하는 일은 카운트 하나 올리는 것이 전부. 시각 정밀도는 필요 없다.
static void IRAM_ATTR expISR(void *arg) {
  uint32_t i = (uint32_t)(uintptr_t)arg;      // 0=CAM1 … 3=CAM4
  int64_t  t = esp_timer_get_time();

  // 기대 위상에서 EXP_MATCH_US 이상 벗어난 엣지는 잡음으로 보고 버린다.
  // 버린 엣지는 락아웃도 걸지 않으므로 진짜 노출을 가리지 않는다.
  uint32_t ph = (uint32_t)t - g_fsyncAt32;
  uint32_t ex = g_expExpect[i];
  uint32_t gap = (ph > ex) ? (ph - ex) : (ex - ph);
  if (gap > EXP_MATCH_US) {
    g_expOther[i]++;
    return;
  }

  if (t - g_expLastUs[i] < EXP_LOCKOUT_US) return;   // 엣지 튐 방지

#if EXP_LEVEL_CHECK
  // 핀이 실제로 활성 레벨인지 확인. 글리치는 이 시점에 이미 복귀해 있다.
  bool lvl = expLevel(PIN_EXP[i]);
  if (EXP_EDGE == FALLING) { if (lvl)  return; }     // LOW 여야 진짜
  else                     { if (!lvl) return; }     // HIGH 여야 진짜
#endif

  g_expLastUs[i] = t;
  g_expCnt[i]++;
  pushLog(EV_EXP, (uint8_t)i, g_fsyncN, t);   // 지금 몇 번 펄스 중인지 함께
}

// --------------------------- 펄스 하강 처리 ---------------------------------
static inline void servicePulses() {
  int64_t now = esp_timer_get_time();
  if (g_fsyncPending && now >= g_fsyncFallAt) {
    REG_WRITE(GPIO_OUT_W1TC_REG, MASK_10HZ);
    g_fsyncPending = false;
  }
  if (g_ppsPending && now >= g_ppsFallAt) {
    REG_WRITE(GPIO_OUT_W1TC_REG, MASK_PPS);
    g_ppsPending = false;
  }
}

/* ===========================================================================
 *  비블로킹 USB 로그
 * ---------------------------------------------------------------------------
 *  USB 시리얼을 읽는 쪽이 없으면 송신 버퍼가 차고, 그때 Serial.printf 는
 *  자리가 날 때까지 멈춘다. 그동안 loop() 가 서 있으니 PPS 를 20 ms 뒤에
 *  내리지 못하고 GPRMC 도 못 보낸다. 라이다는 HIGH 로 고정된 PPS 를 보고
 *  Absent 로 판정한다.
 *    2026-09-14 실측: 시리얼 미수신 시 PPS Absent 1093/1093, GPIO5 3.3 V 고정
 *                     계속 읽어 줄 때 2.3 초 만에 Locked
 *
 *  모든 USB 로그를 여기로 보낸다.
 *    - 버퍼에 자리가 나기를 최대 LOG_WAIT_US 동안 기다린다.
 *      기다리는 동안에도 servicePulses() 를 돌려 펄스는 제때 내린다.
 *    - 그래도 자리가 없으면 그 줄을 버린다 (g_logDropped).
 *  노트북이 읽고 있으면 5 ms 안에 충분히 비워지므로 녹화 중 손실은 없다.
 *  읽는 사람이 없을 때 로그가 빠지는 것은 문제가 아니다. 타이밍이 밀리는
 *  것이 문제다.
 * ======================================================================== */
#define LOG_WAIT_US   5000UL
#define LOG_LINE_MAX  256

static volatile uint32_t g_logDropped = 0;

static void ulogWrite(const char *buf, int n) {
  if (n <= 0) return;
  uint32_t t0 = micros();
  while (Serial.availableForWrite() < n) {
    servicePulses();                          // 기다리는 동안에도 펄스는 내린다
    if ((uint32_t)(micros() - t0) > LOG_WAIT_US) {
      g_logDropped++;
      return;
    }
  }
  Serial.write((const uint8_t *)buf, n);
}

static void ulogf(const char *fmt, ...) {
  char buf[LOG_LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  ulogWrite(buf, n);
}

static void ulogln(const char *str = "") {
  char buf[LOG_LINE_MAX];
  int n = snprintf(buf, sizeof(buf), "%s\n", str);
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  ulogWrite(buf, n);
}

/* ===========================================================================
 *  UTC 시계 — PPS 한 번에 1초씩 전진
 * ======================================================================== */
static uint8_t daysInMonth(uint8_t mon, uint8_t yr) {
  static const uint8_t d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (mon == 2) {
    uint16_t y = 2000 + yr;
    bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
    return leap ? 29 : 28;
  }
  return d[mon - 1];
}

static void clockTick() {
  if (++g_clk.ss < 60) return;
  g_clk.ss = 0;
  if (++g_clk.mm < 60) return;
  g_clk.mm = 0;
  if (++g_clk.hh < 24) return;
  g_clk.hh = 0;
  if (++g_clk.day <= daysInMonth(g_clk.mon, g_clk.yr)) return;
  g_clk.day = 1;
  if (++g_clk.mon <= 12) return;
  g_clk.mon = 1;
  g_clk.yr++;
}

/* ===========================================================================
 *  NMEA 문장 생성
 * ======================================================================== */
static uint8_t nmeaChecksum(const char *body) {
  uint8_t cs = 0;
  while (*body) cs ^= (uint8_t)(*body++);
  return cs;
}

static void buildGPRMC(char *out, size_t cap) {
  char body[80];
  snprintf(body, sizeof(body),
           "GPRMC,%02u%02u%02u,A,%s,%s,%s,%s,000.0,000.0,%02u%02u%02u,,,A",
           g_clk.hh, g_clk.mm, g_clk.ss,
           FIX_LAT, FIX_NS, FIX_LON, FIX_EW,
           g_clk.day, g_clk.mon, g_clk.yr);
  snprintf(out, cap, "$%s*%02X\r\n", body, nmeaChecksum(body));
}

// ------------------------------- 통계 ---------------------------------------
static void statReset(Stat &s) {
  s.n = 0; s.mn = 0xFFFFFFFF; s.mx = 0; s.sum = 0;
}

static void statAdd(Stat &s, uint32_t d) {
  s.n++; s.sum += d;
  if (d < s.mn) s.mn = d;
  if (d > s.mx) s.mx = d;
}

static void statPrint(const char *tag, Stat &s, uint32_t expect) {
  if (s.n == 0) { ulogf("# %s : (데이터 없음)\n", tag); return; }
  uint32_t avg = (uint32_t)(s.sum / s.n);
  ulogf("# %s n=%lu  avg=%lu us (목표 %lu)  min=%lu  max=%lu  jitter=%lu us\n",
                tag, (unsigned long)s.n, (unsigned long)avg, (unsigned long)expect,
                (unsigned long)s.mn, (unsigned long)s.mx,
                (unsigned long)(s.mx - s.mn));
}

// ------------------ BLANK : FSYNC 직후에 걸러낸 잡음 엣지 -------------------
static void printBlanked() {
  uint32_t b[N_CAM];
  portENTER_CRITICAL(&g_mux);
  for (int i = 0; i < N_CAM; i++) b[i] = g_expOther[i];
  portEXIT_CRITICAL(&g_mux);
  if (!(b[0] | b[1] | b[2] | b[3])) return;      // 잡음이 없으면 조용히
  ulogf("BLANK,%lu,%lu,%lu,%lu\n", (unsigned long)b[0], (unsigned long)b[1],
        (unsigned long)b[2], (unsigned long)b[3]);
}

// ---------------------- CNT : 쏜 횟수 vs 노출 횟수 --------------------------
static void printCnt() {
  uint32_t n, c[N_CAM];
  portENTER_CRITICAL(&g_mux);
  n = g_fsyncN;
  for (int i = 0; i < N_CAM; i++) c[i] = g_expCnt[i];
  portEXIT_CRITICAL(&g_mux);

  ulogf("CNT,%lu,%lu,%lu,%lu,%lu\n",
                (unsigned long)n,
                (unsigned long)c[0], (unsigned long)c[1],
                (unsigned long)c[2], (unsigned long)c[3]);
}

// --------------------------- 무선(Wi-Fi) 비활성화 ---------------------------
static void disableRadios() {
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

/* ===========================================================================
 *  모드 전환
 * ======================================================================== */
// STOP 직후 바로 CNT 를 찍으면, 시차 발사에서 늦게 찍는 카메라의 마지막
// 노출이 아직 일어나지 않아 하나 모자라게 나온다 (CAM2 는 트리거 +75 ms).
//   2026-09-22 session_006: 사진은 4대 모두 200장인데 CNT 는 [200,199,200,201]
// 마지막 펄스의 노출이 모두 끝날 때까지 기다렸다가 찍는다.
#define CNT_DELAY_US   200000LL
static volatile int64_t g_cntAt = 0;      // 0 = 예약 없음

static void startRun() {
  if (g_mode == M_RUN) { ulogln("# 이미 RUN 상태"); return; }

  // 세션 카운터 초기화. N 은 이 세션의 1번부터 다시 센다.
  portENTER_CRITICAL(&g_mux);
  g_fsyncN = 0;
  for (int i = 0; i < N_CAM; i++) {
    g_expCnt[i]    = 0;
    g_expLogged[i] = 0;
    g_expOther[i] = 0;
    g_expLastUs[i] = 0;
  }
  g_mode = M_RUN;
  g_cntAt = 0;                       // 이전 세션의 예약이 남아 있으면 취소
  portEXIT_CRITICAL(&g_mux);

  statReset(g_sF);
  g_lastF = 0;
  ulogln("MODE,RUN");
}

static void stopRun() {
  portENTER_CRITICAL(&g_mux);
  g_mode = M_IDLE;
  portEXIT_CRITICAL(&g_mux);

  // FSYNC 핀을 확실히 내려둔다
  REG_WRITE(GPIO_OUT_W1TC_REG, MASK_10HZ);
  g_fsyncPending = false;

  ulogln("MODE,IDLE");
  // 늦게 찍는 카메라의 마지막 노출까지 기다렸다가 CNT 를 찍는다.
  g_cntAt = esp_timer_get_time() + CNT_DELAY_US;
}

// --------------------------- 명령 처리 --------------------------------------
static void handleCommand(const char *cmd) {
  if      (!strcmp(cmd, "GO"))   startRun();
  else if (!strcmp(cmd, "STOP")) stopRun();
  else if (!strncmp(cmd, "DLY,", 4) || !strncmp(cmd, "dly,", 4)) {
    // DLY,<CAM1 us>,<CAM2>,<CAM3>,<CAM4>
    // 카메라 TriggerDelay 를 알려준다. Line1 엣지를 이 위상 근처에서만 받는다.
    uint32_t v[N_CAM] = {0, 0, 0, 0};
    const char *p = cmd + 4;
    int n = 0;
    while (n < N_CAM && *p) {
      v[n++] = (uint32_t)strtoul(p, NULL, 10);
      const char *c = strchr(p, ',');
      if (!c) break;
      p = c + 1;
    }
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < N_CAM; i++) g_expExpect[i] = v[i];
    portEXIT_CRITICAL(&g_mux);
    ulogf("DLY,%lu,%lu,%lu,%lu\n", (unsigned long)v[0], (unsigned long)v[1],
          (unsigned long)v[2], (unsigned long)v[3]);
  }
  else if (!strcmp(cmd, "PING") || !strcmp(cmd, "ping"))
    ulogln("READY");
  else if (!strcmp(cmd, "L") || !strcmp(cmd, "l")) {
    // Line1 배선 진단. 각 핀의 현재 레벨과 누적 계수를 출력한다.
    //   idle=HIGH  → 풀업 정상. 노출이 오면 LOW 로 떨어져야 한다.
    //   idle=LOW   → 배선 단락이거나 카메라가 계속 노출 중
    //   계수 0 인데 HIGH → 주황선(Pin8) 미연결 가능성이 가장 높다
    ulogln("# --- Line1 진단 ---");
    for (int i = 0; i < N_CAM; i++) {
      bool lv = expLevel(PIN_EXP[i]);
      ulogf("# CAM%d  GPIO%-2u  level=%s  기대 %lu us  count=%lu  버림=%lu\n",
                    i + 1, PIN_EXP[i], lv ? "HIGH" : "LOW ",
                    (unsigned long)g_expExpect[i],
                    (unsigned long)g_expCnt[i], (unsigned long)g_expOther[i]);
    }
    ulogln("# 대기 중에는 전부 HIGH 여야 정상입니다.");
  }
  else if (!strcmp(cmd, "R") || !strcmp(cmd, "r")) {
    statReset(g_sF); statReset(g_sP); g_drop = 0; g_nmeaMaxLagUs = 0;
    ulogln("# 통계 리셋");
  }
  else if (cmd[0] != '\0') {
    ulogf("# [WARN] 알 수 없는 명령 : %s\n", cmd);
  }
}

static void pollCommand() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (g_cmdLen > 0) {
        g_cmd[g_cmdLen] = '\0';
        if (g_cmdOverflow) {
          ulogf("# [WARN] 명령이 너무 길어 잘렸습니다 (%u자 제한): %s\n",
                (unsigned)(sizeof(g_cmd) - 1), g_cmd);
        }
        handleCommand(g_cmd);
        g_cmdLen = 0;
      }
      g_cmdOverflow = false;
    } else if (g_cmdLen < sizeof(g_cmd) - 1) {
      g_cmd[g_cmdLen++] = ch;
    } else {
      // 버퍼를 넘었다. 조용히 잘리면 DLY 뒤쪽 값이 0 으로 들어가므로 알린다.
      g_cmdOverflow = true;
    }
  }
}

// ------------------------------- setup --------------------------------------
void setup() {
  disableRadios();

  Serial.begin(SERIAL_BAUD);
  delay(1500);

  // GPRMC 송신용 UART1 (RX 미사용)
  Serial1.begin(NMEA_BAUD, SERIAL_8N1, -1, PIN_NMEA_TX);

  // --- 출력 핀 ---
  pinMode(PIN_10HZ, OUTPUT);
  pinMode(PIN_PPS,  OUTPUT);
  REG_WRITE(GPIO_OUT_W1TC_REG, MASK_10HZ | MASK_PPS);

  // --- Line1 입력 4채널 ---
  for (uint32_t i = 0; i < N_CAM; i++) {
#if EXP_INTERNAL_PULLUP
    pinMode(PIN_EXP[i], INPUT_PULLUP);
#else
    pinMode(PIN_EXP[i], INPUT);
#endif
    attachInterruptArg(digitalPinToInterrupt(PIN_EXP[i]),
                       expISR, (void *)(uintptr_t)i, EXP_EDGE);
  }

  // --- 타이머 : 모드와 무관하게 항상 돈다 (PPS 유지) ---
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  g_timer = timerBegin(1000000);
  timerAttachInterrupt(g_timer, &onTimer);
  timerAlarm(g_timer, TICK_US, true, 0);
#else
  g_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer, &onTimer, true);
  timerAlarmWrite(g_timer, TICK_US, true);
  timerAlarmEnable(g_timer);
#endif

  statReset(g_sF); statReset(g_sP);
  g_statMs = millis();

  buildGPRMC(g_sentence, sizeof(g_sentence));
  size_t len = strlen(g_sentence);

  ulogln("# ===== 방식 2 + GPRMC + Line1 + 핸드셰이크 =====");
  ulogln("# Wi-Fi 비활성화됨");
  ulogf("# %d Hz -> GPIO%d / 1 PPS -> GPIO%d / GPRMC -> GPIO%d @ %d bps\n",
                FREQ_HZ, PIN_10HZ, PIN_PPS, PIN_NMEA_TX, NMEA_BAUD);
  ulogf("# Line1 입력 : CAM1=GPIO%d CAM2=GPIO%d CAM3=GPIO%d CAM4=GPIO%d (%s)\n",
                PIN_EXP[0], PIN_EXP[1], PIN_EXP[2], PIN_EXP[3],
                (EXP_EDGE == FALLING) ? "FALLING" : "RISING");
  ulogf("# Line1 필터 : 락아웃 %lld us, 기대위상 매칭 ±%lu us, 레벨확인 %s\n",
                (long long)EXP_LOCKOUT_US, (unsigned long)EXP_MATCH_US,
                EXP_LEVEL_CHECK ? "ON" : "OFF");
  ulogln("# 기대 위상은 노트북이 GO 전에 DLY 로 알려준다 (없으면 전부 0)");
  ulogf("# 시작 시각(UTC) %02u:%02u:%02u  %02u/%02u/%02u\n",
                START_HH, START_MM, START_SS, START_DAY, START_MON, START_YEAR);
  ulogf("# 예시 문장 (%u byte, 약 %lu ms 소요) : %s",
                (unsigned)len, (unsigned long)(len * 10UL * 1000UL / NMEA_BAUD),
                g_sentence);
  ulogln("# PPS/GPRMC 는 지금부터 송출. FSYNC 는 GO 를 기다림.");
  ulogln("# 명령 : GO / STOP / DLY,a,b,c,d / PING / L / R");
  ulogln("READY");
}

// -------------------------------- loop --------------------------------------
void loop() {
  servicePulses();
  pollCommand();

  // ---- PPS 직후 GPRMC 전송 (모드와 무관하게 항상) ----
  uint32_t seqNow;
  int64_t  edge;
  portENTER_CRITICAL(&g_mux);
  seqNow = g_ppsSeq;
  edge   = g_ppsEdgeUs;
  portEXIT_CRITICAL(&g_mux);

  if (seqNow != g_nmeaHandled) {
    uint32_t missed = seqNow - g_nmeaHandled;
    g_nmeaHandled = seqNow;

    if (g_firstPps) {
      g_firstPps = false;
      missed--;
    }
    for (uint32_t i = 0; i < missed; i++) clockTick();

    if (missed > 1) g_nmeaSkipped += (missed - 1);

    buildGPRMC(g_sentence, sizeof(g_sentence));

    int64_t txStart = esp_timer_get_time();
    Serial1.print(g_sentence);
    g_nmeaSent++;

    int64_t lag = txStart - edge;
    if (lag > g_nmeaMaxLagUs) g_nmeaMaxLagUs = lag;

    uint32_t txDurUs = (uint32_t)strlen(g_sentence) * 10UL * 1000000UL / NMEA_BAUD;

#if PRINT_NMEA
    ulogf("NMEA,%lld,%lld,%lld,%lu,%lu,%s",
                  (long long)edge,
                  (long long)txStart,
                  (long long)lag,
                  (unsigned long)txDurUs,
                  (unsigned long)seqNow,
                  g_sentence);
#endif
  }

  // ---- 로그 출력 ----
  LogEvt e;
  while (popLog(e)) {
    servicePulses();

    if (e.type == EV_FSYNC) {
      int64_t d = e.t_us - g_lastF;
      g_lastF = e.t_us;
      if (e.seq > 1) statAdd(g_sF, (uint32_t)d);
#if PRINT_EVENTS
      ulogf("T,%lu,%lld\n", (unsigned long)e.seq, (long long)e.t_us);
#endif

    } else if (e.type == EV_PPS) {
      int64_t d = e.t_us - g_lastP;
      g_lastP = e.t_us;
      if (e.seq > 1) statAdd(g_sP, (uint32_t)d);
#if PRINT_EVENTS
      ulogf("PPS,%lu,%lld\n", (unsigned long)e.seq, (long long)e.t_us);
#endif

    } else {  // EV_EXP
      uint8_t i = e.cam;
      if (EXP_LOG_LIMIT == 0 || g_expLogged[i] < EXP_LOG_LIMIT) {
        g_expLogged[i]++;
        // EXP,<카메라번호 1~4>,<그 시점의 트리거번호 N>,<시각us>
        ulogf("EXP,%u,%lu,%lld\n",
                      (unsigned)(i + 1),
                      (unsigned long)e.seq,
                      (long long)e.t_us);
      }
    }
  }

  // ---- 예약된 CNT (STOP 후) ----
  if (g_cntAt && esp_timer_get_time() >= g_cntAt) {
    g_cntAt = 0;
    printCnt();                      // 세션 요약
    printBlanked();
    if (g_drop) ulogf("# [WARN] 로그 유실 %lu 건\n", (unsigned long)g_drop);
  }

  // ---- 주기 통계 ----
  if (millis() - g_statMs >= STAT_PERIOD_MS) {
    g_statMs = millis();
    ulogln("# ---------------- 통계 ----------------");
    ulogf("# MODE=%s\n", (g_mode == M_RUN) ? "RUN" : "IDLE");
    statPrint("FSYNC", g_sF, TICK_US);
    statPrint("PPS  ", g_sP, 1000000UL);
    ulogf("# GPRMC 전송 %lu 건, PPS→전송시작 최대지연 %lld us",
                  (unsigned long)g_nmeaSent, (long long)g_nmeaMaxLagUs);
    if (g_nmeaSkipped)
      ulogf(", [WARN] 전송 누락 %lu 건 (시계는 보정됨)",
                    (unsigned long)g_nmeaSkipped);
    ulogln();
    if (g_drop) ulogf("# [WARN] 로그 유실 %lu 건\n", (unsigned long)g_drop);
    if (g_logDropped)
      ulogf("# USB 로그 생략 %lu 줄 (읽는 쪽이 없던 동안. 타이밍엔 영향 없음)\n",
            (unsigned long)g_logDropped);
    if (g_mode == M_RUN) { printCnt(); printBlanked(); }
    ulogln("# --------------------------------------");
    statReset(g_sF); statReset(g_sP);
    g_nmeaMaxLagUs = 0;
  }
}
