/* ===========================================================================
 *  [사본] PPS 극성 반전본 —  원본: method2_gprmc_line1_handshake.ino
 *  ---------------------------------------------------------------------
 *  무엇이 다른가
 *    PPS 만 극성을 뒤집었다. FSYNC(카메라), GPRMC, Line1 계수, GO/STOP
 *    핸드셰이크는 전부 원본과 같다.
 *
 *        원본     평소 LOW,  1초마다 20 ms HIGH   (상승 엣지가 기준)
 *        이 사본  평소 HIGH, 1초마다 20 ms LOW    (하강 엣지가 기준)
 *
 *  왜 만들었나
 *    ESP32 GPIO5 를 I/F Box 의 GPS PULSE 에 점퍼선으로 직결한 상태에서
 *    라이다가 PPS 를 전혀 못 보고 있다 (pps_state = Absent).
 *    신호원은 정상이므로 남은 것은 인터페이스 회로인데, 후보 중
 *    두 가지 — MAX3232 의 남는 채널, 로우사이드 트랜지스터 — 는
 *    신호를 반전시킨다. 그 경우 원본 펌웨어로는 극성이 거꾸로 들어간다.
 *
 *    반전 회로를 달았을 때 이 사본을 쓰면 라이다가 보는 극성이 원본과
 *    같아진다. 레벨 문제인지 극성 문제인지를 분리해서 확인할 수 있다.
 *
 *  어느 것을 쓸지
 *        비반전 (74HCT125 등 5V 버퍼)        → 원본
 *        반전   (MAX3232, 로우사이드 TR)     → 이 사본
 *
 *  지금 어느 펌웨어가 올라가 있는지는 부팅 로그로 확인한다.
 *    이 사본은 부팅할 때 "# BUILD: PPS-INVERTED" 를 출력한다.
 *
 *        python3 -c "import serial;s=serial.Serial('/dev/ttyACM0',921600);\
 *                    [print(s.readline().decode(errors='replace').strip()) \
 *                     for _ in range(20)]"
 *
 *    또는 저장소의 도구로:
 *        python3 ~/ros2_ws/cam_lidar_recording/lidar_config.py --watch
 *        (라이다 쪽 PPS 상태가 Absent 에서 바뀌는지 본다)
 *
 *  바뀐 곳은 세 군데뿐이고 전부 "[PPS 반전]" 주석이 달려 있다.
 *    1. onTimer()       상승 대신 하강으로 엣지를 낸다
 *    2. servicePulses() 20 ms 뒤 유휴(HIGH)로 되돌린다
 *    3. setup()         부팅 직후 유휴 레벨을 HIGH 로 둔다
 * ========================================================================== */

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

// --- Line1 (노출 감지) ---
static volatile uint32_t g_expCnt[N_CAM]    = {0, 0, 0, 0};   // 세션 내 노출 횟수
static volatile int64_t  g_expLastUs[N_CAM] = {0, 0, 0, 0};   // 락아웃 판정용
static volatile uint32_t g_expLogged[N_CAM] = {0, 0, 0, 0};   // 출력한 EXP 줄 수

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
static char   g_cmd[16];
static uint8_t g_cmdLen = 0;

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

  // [PPS 반전] PPS 는 평소 HIGH 로 두고 내려가는 엣지를 기준으로 쓴다.
  // 그래서 FSYNC 는 그대로 올리고(W1TS), PPS 는 반대로 내린다(W1TC).
  // 두 번의 레지스터 쓰기로 나뉘지만 사이 간격은 수 ns 라 무시할 수 있다.
  // PPS 를 먼저 쓴다 — 라이다의 시각 기준이라 엣지를 앞에 두는 편이 낫다.
  if (ppsTick)   REG_WRITE(GPIO_OUT_W1TC_REG, MASK_PPS);    // ← 기준 엣지 (하강)
  if (fsyncTick) REG_WRITE(GPIO_OUT_W1TS_REG, MASK_10HZ);   // ← 카메라 트리거 (상승)

  int64_t t = esp_timer_get_time();

  if (fsyncTick) {
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
  uint32_t i = (uint32_t)arg;                 // 0=CAM1 … 3=CAM4
  int64_t  t = esp_timer_get_time();

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
  // [PPS 반전] 20 ms 뒤에 내리는 것이 아니라 유휴 상태(HIGH)로 되돌린다.
  // 변수 이름은 g_ppsFallAt 그대로 두었다 (원본과 대조하기 쉽게).
  // 반전본에서는 '하강 시각'이 아니라 '복귀 시각'을 뜻한다.
  if (g_ppsPending && now >= g_ppsFallAt) {
    REG_WRITE(GPIO_OUT_W1TS_REG, MASK_PPS);
    g_ppsPending = false;
  }
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
  if (s.n == 0) { Serial.printf("# %s : (데이터 없음)\n", tag); return; }
  uint32_t avg = (uint32_t)(s.sum / s.n);
  Serial.printf("# %s n=%lu  avg=%lu us (목표 %lu)  min=%lu  max=%lu  jitter=%lu us\n",
                tag, (unsigned long)s.n, (unsigned long)avg, (unsigned long)expect,
                (unsigned long)s.mn, (unsigned long)s.mx,
                (unsigned long)(s.mx - s.mn));
}

// ---------------------- CNT : 쏜 횟수 vs 노출 횟수 --------------------------
static void printCnt() {
  uint32_t n, c[N_CAM];
  portENTER_CRITICAL(&g_mux);
  n = g_fsyncN;
  for (int i = 0; i < N_CAM; i++) c[i] = g_expCnt[i];
  portEXIT_CRITICAL(&g_mux);

  Serial.printf("CNT,%lu,%lu,%lu,%lu,%lu\n",
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
static void startRun() {
  if (g_mode == M_RUN) { Serial.println("# 이미 RUN 상태"); return; }

  // 세션 카운터 초기화. N 은 이 세션의 1번부터 다시 센다.
  portENTER_CRITICAL(&g_mux);
  g_fsyncN = 0;
  for (int i = 0; i < N_CAM; i++) {
    g_expCnt[i]    = 0;
    g_expLogged[i] = 0;
    g_expLastUs[i] = 0;
  }
  g_mode = M_RUN;
  portEXIT_CRITICAL(&g_mux);

  statReset(g_sF);
  g_lastF = 0;
  Serial.println("MODE,RUN");
}

static void stopRun() {
  portENTER_CRITICAL(&g_mux);
  g_mode = M_IDLE;
  portEXIT_CRITICAL(&g_mux);

  // FSYNC 핀을 확실히 내려둔다
  REG_WRITE(GPIO_OUT_W1TC_REG, MASK_10HZ);
  g_fsyncPending = false;

  Serial.println("MODE,IDLE");
  printCnt();                      // 세션 요약
  if (g_drop) Serial.printf("# [WARN] 로그 유실 %lu 건\n", (unsigned long)g_drop);
}

// --------------------------- 명령 처리 --------------------------------------
static void handleCommand(const char *cmd) {
  if      (!strcmp(cmd, "GO"))   startRun();
  else if (!strcmp(cmd, "STOP")) stopRun();
  else if (!strcmp(cmd, "PING") || !strcmp(cmd, "ping"))
    Serial.println("READY");
  else if (!strcmp(cmd, "L") || !strcmp(cmd, "l")) {
    // Line1 배선 진단. 각 핀의 현재 레벨과 누적 계수를 출력한다.
    //   idle=HIGH  → 풀업 정상. 노출이 오면 LOW 로 떨어져야 한다.
    //   idle=LOW   → 배선 단락이거나 카메라가 계속 노출 중
    //   계수 0 인데 HIGH → 주황선(Pin8) 미연결 가능성이 가장 높다
    Serial.println("# --- Line1 진단 ---");
    for (int i = 0; i < N_CAM; i++) {
      bool lv = expLevel(PIN_EXP[i]);
      Serial.printf("# CAM%d  GPIO%-2u  level=%s  count=%lu\n",
                    i + 1, PIN_EXP[i], lv ? "HIGH" : "LOW ",
                    (unsigned long)g_expCnt[i]);
    }
    Serial.println("# 대기 중에는 전부 HIGH 여야 정상입니다.");
  }
  else if (!strcmp(cmd, "R") || !strcmp(cmd, "r")) {
    statReset(g_sF); statReset(g_sP); g_drop = 0; g_nmeaMaxLagUs = 0;
    Serial.println("# 통계 리셋");
  }
  else if (cmd[0] != '\0') {
    Serial.printf("# [WARN] 알 수 없는 명령 : %s\n", cmd);
  }
}

static void pollCommand() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (g_cmdLen > 0) {
        g_cmd[g_cmdLen] = '\0';
        handleCommand(g_cmd);
        g_cmdLen = 0;
      }
    } else if (g_cmdLen < sizeof(g_cmd) - 1) {
      g_cmd[g_cmdLen++] = ch;
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
  // [PPS 반전] 부팅 직후의 유휴 레벨. FSYNC 는 LOW, PPS 는 HIGH 다.
  // 이 줄을 빼면 첫 펄스가 나갈 때까지 PPS 가 LOW 로 떠 있어서,
  // 라이다가 부팅 순간을 긴 펄스 하나로 오해할 수 있다.
  REG_WRITE(GPIO_OUT_W1TC_REG, MASK_10HZ);
  REG_WRITE(GPIO_OUT_W1TS_REG, MASK_PPS);

  // --- Line1 입력 4채널 ---
  for (uint32_t i = 0; i < N_CAM; i++) {
#if EXP_INTERNAL_PULLUP
    pinMode(PIN_EXP[i], INPUT_PULLUP);
#else
    pinMode(PIN_EXP[i], INPUT);
#endif
    attachInterruptArg(digitalPinToInterrupt(PIN_EXP[i]),
                       expISR, (void *)i, EXP_EDGE);
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

  Serial.println("# ===== 방식 2 + GPRMC + Line1 + 핸드셰이크 =====");
  Serial.println("# BUILD: PPS-INVERTED  (PPS 평소 HIGH, 1초마다 20 ms LOW)");
  Serial.println("# Wi-Fi 비활성화됨");
  Serial.printf("# %d Hz -> GPIO%d / 1 PPS -> GPIO%d / GPRMC -> GPIO%d @ %d bps\n",
                FREQ_HZ, PIN_10HZ, PIN_PPS, PIN_NMEA_TX, NMEA_BAUD);
  Serial.printf("# Line1 입력 : CAM1=GPIO%d CAM2=GPIO%d CAM3=GPIO%d CAM4=GPIO%d (%s)\n",
                PIN_EXP[0], PIN_EXP[1], PIN_EXP[2], PIN_EXP[3],
                (EXP_EDGE == FALLING) ? "FALLING" : "RISING");
  Serial.printf("# Line1 필터 : 락아웃 %lld us, 레벨확인 %s\n",
                (long long)EXP_LOCKOUT_US, EXP_LEVEL_CHECK ? "ON" : "OFF");
  Serial.printf("# 시작 시각(UTC) %02u:%02u:%02u  %02u/%02u/%02u\n",
                START_HH, START_MM, START_SS, START_DAY, START_MON, START_YEAR);
  Serial.printf("# 예시 문장 (%u byte, 약 %lu ms 소요) : %s",
                (unsigned)len, (unsigned long)(len * 10UL * 1000UL / NMEA_BAUD),
                g_sentence);
  Serial.println("# PPS/GPRMC 는 지금부터 송출. FSYNC 는 GO 를 기다림.");
  Serial.println("# 명령 : GO / STOP / PING / L / R");
  Serial.println("READY");
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
    Serial.printf("NMEA,%lld,%lld,%lld,%lu,%lu,%s",
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
      Serial.printf("T,%lu,%lld\n", (unsigned long)e.seq, (long long)e.t_us);
#endif

    } else if (e.type == EV_PPS) {
      int64_t d = e.t_us - g_lastP;
      g_lastP = e.t_us;
      if (e.seq > 1) statAdd(g_sP, (uint32_t)d);
#if PRINT_EVENTS
      Serial.printf("PPS,%lu,%lld\n", (unsigned long)e.seq, (long long)e.t_us);
#endif

    } else {  // EV_EXP
      uint8_t i = e.cam;
      if (EXP_LOG_LIMIT == 0 || g_expLogged[i] < EXP_LOG_LIMIT) {
        g_expLogged[i]++;
        // EXP,<카메라번호 1~4>,<그 시점의 트리거번호 N>,<시각us>
        Serial.printf("EXP,%u,%lu,%lld\n",
                      (unsigned)(i + 1),
                      (unsigned long)e.seq,
                      (long long)e.t_us);
      }
    }
  }

  // ---- 주기 통계 ----
  if (millis() - g_statMs >= STAT_PERIOD_MS) {
    g_statMs = millis();
    Serial.println("# ---------------- 통계 ----------------");
    Serial.printf("# MODE=%s\n", (g_mode == M_RUN) ? "RUN" : "IDLE");
    statPrint("FSYNC", g_sF, TICK_US);
    statPrint("PPS  ", g_sP, 1000000UL);
    Serial.printf("# GPRMC 전송 %lu 건, PPS→전송시작 최대지연 %lld us",
                  (unsigned long)g_nmeaSent, (long long)g_nmeaMaxLagUs);
    if (g_nmeaSkipped)
      Serial.printf(", [WARN] 전송 누락 %lu 건 (시계는 보정됨)",
                    (unsigned long)g_nmeaSkipped);
    Serial.println();
    if (g_drop) Serial.printf("# [WARN] 로그 유실 %lu 건\n", (unsigned long)g_drop);
    if (g_mode == M_RUN) printCnt();
    Serial.println("# --------------------------------------");
    statReset(g_sF); statReset(g_sP);
    g_nmeaMaxLagUs = 0;
  }
}
