#include "odometry.h" // ★ 추가
#include <PS2X_lib.h>

// PS2 핀 정의
#define PS2_DAT 52
#define PS2_CMD 51
#define PS2_SEL 53
#define PS2_CLK 50

// 모터 핀 정의
#define PWMA 12
#define DIRA1 34
#define DIRA2 35
#define PWMB 8
#define DIRB1 37
#define DIRB2 36
#define PWMC 9
#define DIRC1 43
#define DIRC2 42
#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

// 엔코더 핀 정의
#define ENCA_A 18
#define ENCA_B 3
#define ENCA_C 2
#define ENCA_D 19

volatile long encA = 0, encB = 0, encC = 0, encD = 0;
float speedA = 0, speedB = 0, speedC = 0, speedD = 0;
unsigned long lastSpeedTime = 0;

unsigned long lastRecvTime = 0; // 마지막 통신 수신 시점 (워치독용)

// ── 주행 속도 및 가감속 제어 변수 ────────────────────────────
int Motor_PWM = 125;            // 기본 주행 PWM 기준값
float targetSpeed = 125.0;      // PID 엔코더 목표 속도
int currentBasePWM = 0;         // 램프 가감속용 현재 PWM
const int RAMP_ACCEL_STEP = 20; // 100ms당 가속 폭 (약 0.6초에 목표 속도 도달)
const int RAMP_DECEL_STEP = 30; // 100ms당 감속 폭 (부드러운 정지)

int pwmA = 0, pwmB = 0, pwmC = 0, pwmD = 0;

// 헤딩 보정 (Heading Lock) 전용 변수
float targetHeading = 0.0;
// 각도 1도 오차당 목표 속도 보정량 (튜닝 가능)
// [실측] 3.0에서는 전진 정상오차 약 1도, 후진 약 4도로 수렴(발산은 아님).
// 후진 편차가 전진의 약 3배라 같은 게인이면 정상오차도 3배로 남는다.
const float KP_HEADING = 6.0;

// 적분 게인: P만으로는 출발 시 생긴 오프셋을 되돌리지 못하고 정상오차로 남는다.
// 그 오차가 다음 구간의 targetHeading으로 굳어져 누적되므로 I로 제거한다.
// (0으로 두면 기존 P 전용 동작으로 되돌아감)
const float KI_HEADING = 3.0;
const float HEADING_I_LIMIT = 10.0; // 적분 누적 제한 (안티 와인드업)
float headingIntegral = 0.0;

// ── [진단 전용] 휠 속도 트레이스 ────────────────────────────────
// 내부 휠 PID 는 목표 속도로 Motor_PWM(=125) 을 그대로 쓰는데, 이 값은 PWM 단위이지
// 엔코더 카운트율 단위가 아니다. 실측 속도가 목표에 얼마나 못 미치는지 확인하기 위한
// 관측 전용 코드로, 주행 동작에는 전혀 영향을 주지 않는다.
// 진단이 끝나면 0 으로 바꾸면 컴파일에서 통째로 빠진다.
#define HEADING_SPEED_TRACE 1

#if HEADING_SPEED_TRACE
float traceTargetLeft = 0.0;   // 직전 제어 주기의 좌측 목표 속도
float traceTargetRight = 0.0;  // 직전 제어 주기의 우측 목표 속도
#endif

// ── [이상 상태 감지 / 탈출 시도] ────────────────────────────────
#define FAULT_DETECT_ENABLE 1

// fault 코드 (POS: 3번째 필드로 나노 -> BLE -> PC 까지 전달)
#define FAULT_NONE 0
#define FAULT_STALL 1   // 모터가 물리적으로 안 돎 (엔코더 ~ 0). locked-rotor 전류 상태이므로 출력 상향 금지
#define FAULT_HEADING 2 // 헤딩 보정이 포화된 채 오차가 지속. 턱에 비스듬히 걸린 경우 -> 출력 상향 시도 가치 있음
#define FAULT_WHEEL 3   // 한 바퀴만 나머지와 따로 놂 (들림/걸림). 출력 상향은 무의미

// 감지 임계값.
// ★ STALL_SPEED_THRESH 는 정상 주행 속도(엔코더 counts/100ms)에 의존한다.
//   오도메트리 역산 추정치가 62~84 이라 그 20% 인 15 를 잠정값으로 둔다.
//   [SPD] 트레이스로 실측한 뒤 반드시 보정할 것.
const int STALL_SPEED_THRESH = 15;
const int STALL_MIN_BASE_PWM = 60;    // 실제로 밀고 있을 때만 스톨로 본다
const uint8_t STALL_TICKS = 5;        // 0.5초 연속
const float HEADING_FAULT_DEG = 8.0;  // 정상 주행 실측 최대 오차 2.3도 대비 충분한 여유
const uint8_t HEADING_TICKS = 10;     // 1.0초 연속
const float WHEEL_LOW_RATIO = 0.40;   // 나머지 3개 중앙값 대비 (걸린 바퀴)
const float WHEEL_HIGH_RATIO = 2.00;  // (들려서 헛도는 바퀴)
const uint8_t WHEEL_TICKS = 5;

// 출발 직후엔 엔코더가 0에서 시작하고 currentBasePWM 도 40부터 램프업하므로
// 이 시간이 지나기 전에는 스톨로 오판하지 않는다.
const unsigned long FAULT_GRACE_MS = 1000;

// 자율 주행에서 래치가 영원히 안 풀려 로봇이 벽돌이 되는 것을 막는 안전망.
// (PC 가 구버전이라 fault 를 못 읽는 경우 등)
const unsigned long FAULT_LATCH_TIMEOUT_MS = 10000;

uint8_t faultCode = FAULT_NONE;
unsigned long faultTime = 0;
unsigned long straightStartTime = 0;
uint8_t stallCount = 0, headingFaultCount = 0, wheelFaultCount = 0, healthyCount = 0;

// PS2 수동 조작 중 조건부 자동 부스트 사다리
//   0 = 정상 출력 -> 1 = 출력 상향 시도(최대 1.5초) -> 2 = 탈출 실패, 출력 제한
// 버튼에서 손을 떼면 0 으로 복귀한다.
const int PWM_BOOST = 170;
const int PWM_PROTECT = 60;
const unsigned long BOOST_MAX_MS = 1500;
uint8_t boostStage = 0;
unsigned long boostStartTime = 0;

// ── REVERSE: 이상 상태 탈출용 짧은 후진 ─────────────────────────
// 거리(오도메트리)와 시간 둘 다로 끊는다. 이 명령을 쓰는 상황이 곧 슬립/스톨이라
// 오도메트리를 못 믿을 수 있으므로, 시간 제한이 실질적인 안전장치다.
// 후방에는 센서가 전혀 없으므로(라이다 FOV 180도, 초음파 좌/우) 반드시 짧아야 한다.
const float REVERSE_TARGET_M = 0.25;
const unsigned long REVERSE_MAX_MS = 1500;
bool isReversing = false;
unsigned long reverseStartTime = 0;
float reverseStartX = 0.0, reverseStartY = 0.0;
// ★ 후진 종료 직후의 재시작 방지.
// PC 는 REVERSE 를 400ms 하트비트로 계속 보내는데, 메가가 1.5초에 자체 종료하면
// 곧바로 도착하는 다음 하트비트가 새 후진을 시작시켜 의도의 2배(최대 50cm)를
// 물러나게 된다. 후방에 센서가 하나도 없으므로 거리는 메가가 못박아야 한다.
unsigned long reverseCooldownUntil = 0;
const unsigned long REVERSE_COOLDOWN_MS = 2500;

// 세 값의 중앙값 (바퀴 편차 판정용)
float medianOf3(float a, float b, float c) {
  float t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

const char *faultName(uint8_t code) {
  if (code == FAULT_STALL) return "모터 스톨(엔코더 정지)";
  if (code == FAULT_HEADING) return "헤딩 보정 포화 지속";
  if (code == FAULT_WHEEL) return "바퀴 편차(들림/걸림)";
  return "정상";
}

// PS2 수동 조작 중 조작자가 요청한 출력에 부스트 사다리를 적용한다.
// 조작자가 방향 버튼을 계속 누르고 있는 동안에만 호출되므로, 손을 떼면 자동으로 멈춘다.
int applyBoostLadder(int cmdPWM, unsigned long now) {
  if (faultCode == FAULT_STALL) {
    // 모터가 물리적으로 막힌 상태. 지금도 PID 가 PWM 을 195 까지 밀어올려 locked-rotor
    // 전류가 흐르고 있으므로, 더 주면 권선/H-브리지가 탄다. 부스트 단계를 건너뛰고 바로 제한.
    if (boostStage != 2) {
      boostStage = 2;
      Serial.println("[BOOST] 모터 스톨 감지 - 출력 상향 금지, 보호 제한 진입");
    }
    return min(cmdPWM, PWM_PROTECT);
  }

  if (faultCode == FAULT_NONE) {
    if (boostStage == 1) {
      boostStage = 0;
      Serial.println("[BOOST] 이상 해소 - 정상 출력 복귀");
    }
    // 보호 제한(2단계)은 버튼에서 손을 뗄 때까지 유지한다
    return (boostStage == 2) ? min(cmdPWM, PWM_PROTECT) : cmdPWM;
  }

  // FAULT_HEADING / FAULT_WHEEL : 출력 상향으로 탈출 시도
  if (boostStage == 0) {
    boostStage = 1;
    boostStartTime = now;
    Serial.print("[BOOST] 이상 감지(");
    Serial.print(faultName(faultCode));
    Serial.println(") - 출력 상향으로 탈출 시도");
  }
  if (boostStage == 1) {
    if (now - boostStartTime < BOOST_MAX_MS) {
      return max(cmdPWM, PWM_BOOST);
    }
    boostStage = 2;
    Serial.println("[BOOST] 상향 시도 실패 - 보호 제한 (버튼을 놓으면 복귀)");
  }
  return min(cmdPWM, PWM_PROTECT);
}

// 각도 변수 (나노 BLE에서 YAW 수신)
float yaw = 0;
unsigned long lastYawTime = 0; // 마지막 YAW 수신 시점 (수신 두절 감지용)

bool isCalibrated =
    true; // 나노가 부팅 시 칼리브레이션을 수행하므로 메가는 상시 참으로 시작
PS2X ps2x;
int ps2_error = 0;
byte ps2_type = 0;
byte vibrate = 0;
bool wasPs2Controlled = false;
bool isTurning = false;
float destinationAngle = 0.0;
const float ANGLE_TOLERANCE = 2.0;

// 엔코더 인터럽트 (ISR)
void isrEncA() { encA++; }
void isrEncB() { encB++; }
void isrEncC() { encC++; }
void isrEncD() { encD++; }

class SimplePID {
public:
  float kp, ki, kd;
  float error, lastError, integral;
  unsigned long lastT;
  float lastOutput;

  SimplePID() {
    kp = 0;
    ki = 0;
    kd = 0;
    error = lastError = integral = 0;
    lastT = 0;
    lastOutput = 0;
  }

  SimplePID(float _kp, float _ki, float _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    error = lastError = integral = 0;
    lastT = 0;
    lastOutput = 0;
  }

  float compute(float target, float current) {
    unsigned long now = millis();

    // 첫 호출이거나 시간이 갱신되지 않은 경우 방어
    if (lastT == 0) {
      lastT = now;
      lastError = target - current;
      return 0;
    }

    float dt = (now - lastT) / 1000.0;
    // dt가 0 이하이거나 비정상적으로 긴 경우(0.5초 초과) 기본 0.1초로 제한하여
    // 적분 폭주 방지
    if (dt <= 0.0 || dt > 0.5)
      dt = 0.1;

    error = target - current;
    integral += error * dt;
    if (integral > 50)
      integral = 50;
    if (integral < -50)
      integral = -50;

    float derivative = (error - lastError) / dt;
    float out = kp * error + ki * integral + kd * derivative;

    lastError = error;
    lastT = now;
    lastOutput = out;
    return out;
  }

  void setKp(float _kp) { kp = _kp; }
  void setKi(float _ki) { ki = _ki; }
  void setKd(float _kd) { kd = _kd; }

  void reset() {
    integral = 0;
    lastError = 0;
    lastT = millis();
    lastOutput = 0;
  }

  void printGains() {
    Serial.print("Kp=");
    Serial.print(kp, 3);
    Serial.print(" Ki=");
    Serial.print(ki, 3);
    Serial.print(" Kd=");
    Serial.print(kd, 3);
  }
};

SimplePID pidA(0.3, 0.02, 0.08);
SimplePID pidB(0.3, 0.02, 0.08);
SimplePID pidC(0.3, 0.02, 0.08);
SimplePID pidD(0.3, 0.02, 0.08);

// 각 바퀴의 현재 회전 방향 (+1: 전진, -1: 후진, 0: 정지)
int dirSignA = 0, dirSignB = 0, dirSignC = 0, dirSignD = 0;

// 직진/후진 주행 모드의 진행 방향 (+1: 전진, -1: 후진, 0: 미주행)
int driveDir = 0;

void motorDrive(int pwmPin, int d1, int d2, int pwm) {
  pwm = constrain(pwm, -255, 255);

  int sign = 0;
  if (pwm > 0)
    sign = 1;
  else if (pwm < 0)
    sign = -1;

  // 하드웨어 배선에 맞춘 바퀴별 방향 판별 (ADVANCE 전진 시 +1, BACK 후진 시 -1)
  // PWMA/PWMC는 음수 PWM이 전진, PWMB/PWMD는 양수 PWM이 전진
  if (pwmPin == PWMA)
    dirSignA = -sign;
  else if (pwmPin == PWMB)
    dirSignB = sign;
  else if (pwmPin == PWMC)
    dirSignC = -sign;
  else if (pwmPin == PWMD)
    dirSignD = sign;

  if (pwm > 0) {
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
    analogWrite(pwmPin, -pwm);
  } else {
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void ADVANCE() {
  motorDrive(PWMA, DIRA1, DIRA2, -pwmA);
  motorDrive(PWMB, DIRB1, DIRB2, pwmB);
  motorDrive(PWMC, DIRC1, DIRC2, -pwmC);
  motorDrive(PWMD, DIRD1, DIRD2, pwmD);
}

void STOP() {
  motorDrive(PWMA, DIRA1, DIRA2, 0);
  motorDrive(PWMB, DIRB1, DIRB2, 0);
  motorDrive(PWMC, DIRC1, DIRC2, 0);
  motorDrive(PWMD, DIRD1, DIRD2, 0);
}

void TURN_RIGHT(int speed = 0) {
  int spd = (speed == 0) ? Motor_PWM : speed;
  motorDrive(PWMA, DIRA1, DIRA2, -spd);
  motorDrive(PWMB, DIRB1, DIRB2, -spd);
  motorDrive(PWMC, DIRC1, DIRC2, -spd);
  motorDrive(PWMD, DIRD1, DIRD2, -spd);
}

void TURN_LEFT(int speed = 0) {
  int spd = (speed == 0) ? Motor_PWM : speed;
  motorDrive(PWMA, DIRA1, DIRA2, spd);
  motorDrive(PWMB, DIRB1, DIRB2, spd);
  motorDrive(PWMC, DIRC1, DIRC2, spd);
  motorDrive(PWMD, DIRD1, DIRD2, spd);
}

void BACK() {
  motorDrive(PWMA, DIRA1, DIRA2, pwmA);
  motorDrive(PWMB, DIRB1, DIRB2, -pwmB);
  motorDrive(PWMC, DIRC1, DIRC2, pwmC);
  motorDrive(PWMD, DIRD1, DIRD2, -pwmD);
}

void initStraightMode(int dir = 1) { // dir: +1 전진, -1 후진
  pidA.reset();
  pidB.reset();
  pidC.reset();
  pidD.reset();
  driveDir = dir;
  headingIntegral = 0.0; // 구간마다 적분 초기화
  targetSpeed = (float)Motor_PWM;
  currentBasePWM = 40; // 최소 기동 토크(부드러운 가속 시작점)
  pwmA = pwmB = pwmC = pwmD = currentBasePWM;
  encA = encB = encC = encD = 0;
  lastSpeedTime = millis();
  // 출발 직후는 엔코더가 0이고 PWM 도 40부터 램프업하므로 스톨 판정을 유예한다
  straightStartTime = millis();
  stallCount = headingFaultCount = wheelFaultCount = healthyCount = 0;
  targetHeading = yaw; // ★ 직진/후진 시작 시점의 각도를 목표 헤딩으로 기억
  Serial.print("PID/가감속/헤딩락 초기화 (");
  Serial.print(dir > 0 ? "전진" : "후진");
  Serial.print(", 목표 각도: ");
  Serial.print(targetHeading, 1);
  Serial.println("도)");
}

void print_all_data(uint32_t t) {
  String dir = "";
  if (yaw > 337.5 || yaw <= 22.5)
    dir = "북(정면)";
  else if (yaw > 22.5 && yaw <= 67.5)
    dir = "북동";
  else if (yaw > 67.5 && yaw <= 112.5)
    dir = "동";
  else if (yaw > 112.5 && yaw <= 157.5)
    dir = "남동";
  else if (yaw > 157.5 && yaw <= 202.5)
    dir = "남";
  else if (yaw > 202.5 && yaw <= 247.5)
    dir = "남서";
  else if (yaw > 247.5 && yaw <= 292.5)
    dir = "서";
  else if (yaw > 292.5 && yaw <= 337.5)
    dir = "북서";

  Serial.print("Y(Heading): ");
  Serial.print(yaw, 1);
  Serial.print(" [");
  Serial.print(dir);
  Serial.print("]");

  // ★ 위치 좌표 출력 추가
  Serial.print("  |  X: ");
  Serial.print(pose.x, 3);
  Serial.print("m  Y: ");
  Serial.print(pose.y, 3);
  Serial.println("m");

  // ★ 나노 YAW 수신 상태 점검 (수신이 끊기면 헤딩락이 조용히 무력화된다)
  if (lastYawTime == 0 || (t - lastYawTime) > 1000) {
    Serial.print("  [경고] 나노 YAW 수신 없음 (");
    if (lastYawTime == 0) {
      Serial.print("부팅 후 한 번도 수신 못함");
    } else {
      Serial.print((t - lastYawTime) / 1000.0, 1);
      Serial.print("초 경과");
    }
    Serial.println(") - 헤딩락 비활성. 나노 IMU/배선 확인 필요");
  }

  // ★ 직진/후진 주행 중일 때 헤딩 락 실시간 보정 상태 출력
  if (pwmA > 0 && !isTurning && driveDir != 0) {
    float hErr = targetHeading - yaw;
    if (hErr > 180.0)
      hErr -= 360.0;
    if (hErr < -180.0)
      hErr += 360.0;
    Serial.print("  └─► [H-Lock ");
    Serial.print(driveDir > 0 ? "전진" : "후진");
    Serial.print("] 목표: ");
    Serial.print(targetHeading, 1);
    Serial.print("° (오차: ");
    Serial.print(hErr, 1);
    Serial.print("°, 적분: ");
    Serial.print(headingIntegral, 1);
    Serial.print(") | 좌측PWM(A): ");
    Serial.print(pwmA);
    Serial.print("  우측PWM(B): ");
    Serial.println(pwmB);

#if HEADING_SPEED_TRACE
    // 목표 속도(좌/우) vs 엔코더 실측 속도(A~D) vs 실제 출력 PWM
    // corrX = pwmX - base 로 역산 가능하므로 base 도 함께 찍는다.
    Serial.print("  └─► [SPD] 목표 L");
    Serial.print(traceTargetLeft, 1);
    Serial.print(" R");
    Serial.print(traceTargetRight, 1);
    Serial.print(" | 실측 A");
    Serial.print(speedA, 0);
    Serial.print(" B");
    Serial.print(speedB, 0);
    Serial.print(" C");
    Serial.print(speedC, 0);
    Serial.print(" D");
    Serial.print(speedD, 0);
    Serial.print(" | base ");
    Serial.print(currentBasePWM);
    Serial.print(" | PWM A");
    Serial.print(pwmA);
    Serial.print(" B");
    Serial.print(pwmB);
    Serial.print(" C");
    Serial.print(pwmC);
    Serial.print(" D");
    Serial.println(pwmD);
#endif
  }

  // ★ Nano(Serial2)로 좌표 데이터 전송 (YAW는 나노가 직접 계산하므로 제외)
  Serial2.print("POS:");
  Serial2.print(pose.x, 3);
  Serial2.print(",");
  Serial2.print(pose.y, 3);
  Serial2.print(",");
  Serial2.println(faultCode); // 3번째 필드: 이상 상태 코드 (나노 -> BLE -> PC)
}

// ── 경량 JSON 값 추출 함수 (키 기반 파싱) ─────────────────────
String extractJsonValue(String json, String key) {
  int keyIndex = json.indexOf("\"" + key + "\"");
  if (keyIndex == -1) return "";

  int colonIndex = json.indexOf(':', keyIndex);
  if (colonIndex == -1) return "";

  int startIdx = colonIndex + 1;
  while (startIdx < (int)json.length() && (json[startIdx] == ' ' || json[startIdx] == '\t')) {
    startIdx++;
  }
  if (startIdx >= (int)json.length()) return "";

  if (json[startIdx] == '\"') {
    startIdx++;
    int endIdx = json.indexOf('\"', startIdx);
    if (endIdx != -1) {
      return json.substring(startIdx, endIdx);
    }
  } else {
    int endIdx = startIdx;
    while (endIdx < (int)json.length() && json[endIdx] != ',' && json[endIdx] != '}' && json[endIdx] != ' ' && json[endIdx] != '\r' && json[endIdx] != '\n') {
      endIdx++;
    }
    return json.substring(startIdx, endIdx);
  }
  return "";
}

// ── 주행 및 부가 명령 통합 처리 함수 ───────────────────────────
void processCommand(String cmd, uint32_t now) {
  static int lastCommand = -9999;
  static unsigned long lastCmdTime = 0;
  const unsigned long CMD_COOLDOWN = 500;

  cmd.trim();
  if (cmd.length() == 0) return;

  // 1. JSON 형식인 경우: {"S-signal":"STOP", "R-signal":"RESET_ODO"}
  if (cmd.startsWith("{") && cmd.endsWith("}")) {
    String sSignal = extractJsonValue(cmd, "S-signal");
    String rSignal = extractJsonValue(cmd, "R-signal");

    Serial.print("[JSON 명령 수신] S-signal: '");
    Serial.print(sSignal);
    Serial.print("', R-signal: '");
    Serial.print(rSignal);
    Serial.println("'");

    // ★ PC 패킷이 도착했다는 사실 자체가 통신 생존 증거이므로 워치독을 갱신한다.
    // (아래 개별 분기의 쿨다운에 걸려 명령이 무시되더라도 링크는 살아있는 것)
    lastRecvTime = now;

    // R-signal 처리 (부가 명령어: RESET_ODO)
    if (rSignal.equalsIgnoreCase("RESET_ODO")) {
      lastRecvTime = now;
      resetOdometry();
      Serial.println("위치 초기화 완료 (0, 0)");
      Serial2.println("POS:0.000,0.000,0");
    }

    // S-signal 처리 (주행/조향 명령)
    if (sSignal.length() > 0) {
      if (sSignal.equalsIgnoreCase("STOP") || sSignal == "-1") {
        lastRecvTime = now;
        faultCode = FAULT_NONE; // PC 가 정지를 지시했다 = 이상 상태를 인지했다
        isReversing = false;
        currentBasePWM = 0;
        STOP();
        isTurning = false;
        driveDir = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        Serial.println("정지 명령(STOP) 수신 및 정지 완료");
      } else if (sSignal.equalsIgnoreCase("REVERSE")) {
        lastRecvTime = now;
        // 직진과 다른 동작이므로 fault 래치를 해제하고 실행한다 (상위의 탈출 시도)
        faultCode = FAULT_NONE;
        if (!isReversing && now >= reverseCooldownUntil) {
          isTurning = false;
          initStraightMode(-1);
          isReversing = true;
          reverseStartTime = now;
          reverseStartX = pose.x;
          reverseStartY = pose.y;
          BACK();
          Serial.println("후진 명령(REVERSE) 수신 - 최대 25cm / 1.5초 후진 시작");
        }
        // 이미 후진 중이면 하트비트로만 취급 (워치독 갱신 후 무시)
      } else if (sSignal == "0") {
        lastRecvTime = now;
        isReversing = false; // 전진 명령이 오면 후진 취소
        // ★ 래치된 이상 상태에서는 실패했던 그 동작(직진)만 거부한다.
        // STOP / 회전 명령은 그대로 통과하므로 PC 가 탈출을 지시할 수 있다.
        if (faultCode != FAULT_NONE) {
          if (now - faultTime < FAULT_LATCH_TIMEOUT_MS) {
            static unsigned long lastFaultLog = 0;
            if (now - lastFaultLog > 1000) {
              lastFaultLog = now;
              Serial.print("[FAULT ");
              Serial.print(faultCode);
              Serial.print("] 직진 명령 거부 - ");
              Serial.println(faultName(faultCode));
            }
            return;
          }
          // 안전망: PC 가 끝내 인지하지 못해도 로봇이 영구히 멈춰 있지는 않도록 자동 해제
          Serial.println("[FAULT] 래치 타임아웃 - 자동 해제 후 직진 재시도 허용");
          faultCode = FAULT_NONE;
        }
        if (isTurning || pwmA == 0) {
          initStraightMode();
        }
        isTurning = false;
        ADVANCE();
        Serial.println("직진 명령(0) 수신 및 직진 시작");
      } else {
        // 각도 회전 명령 (예: 90, -90 등)
        int target = sSignal.toInt();
        // ★ 이미 회전 중이면 새 각도 명령을 무시한다.
        // PC 는 매 프레임 "남은 오차"를 다시 계산해 보내므로 값이 계속 바뀌는데,
        // 그때마다 새 회전으로 받아들이면 STOP + delay(100) + 목표각 재설정이
        // 반복돼 회전 내내 덜컹거린다. 메가가 destinationAngle 까지 자체
        // 폐루프로 수렴시키게 두고, 탈출은 STOP / "0"(정렬 완료) 이 담당한다.
        if (isTurning) {
          Serial.println("회전 중 - 새 각도 명령 무시");
        } else if (!(target == lastCommand && (now - lastCmdTime) < CMD_COOLDOWN)) {
          faultCode = FAULT_NONE; // 직진과 다른 동작이므로 허용 (PC 의 탈출 시도)
          isReversing = false;
          lastCommand = target;
          lastCmdTime = now;
          Serial.print("회전 각도 수신: ");
          Serial.println(target);

          STOP();
          driveDir = 0;
          pwmA = pwmB = pwmC = pwmD = 0;
          delay(100);
          destinationAngle = yaw + target;
          if (destinationAngle >= 360.0) destinationAngle -= 360.0;
          if (destinationAngle < 0.0)   destinationAngle += 360.0;
          isTurning = true;
        }
      }
    }
    return;
  }

  // 2. 단일 텍스트 명령 처리 (하위 호환)
  if (cmd.equalsIgnoreCase("RESET_ODO")) {
    lastRecvTime = now;
    resetOdometry();
    Serial.println("위치 초기화 완료 (0, 0)");
    Serial2.println("POS:0.000,0.000,0");
  } else if (cmd.equalsIgnoreCase("STOP") || cmd == "-1") {
    lastRecvTime = now;
    faultCode = FAULT_NONE; // 정지 지시 = 이상 상태 인지
    isReversing = false;
    currentBasePWM = 0;
    STOP();
    isTurning = false;
    driveDir = 0;
    pwmA = pwmB = pwmC = pwmD = 0;
    Serial.println("정지 명령(STOP) 수신 및 정지 완료");
  } else if (cmd.equalsIgnoreCase("REVERSE")) {
    lastRecvTime = now;
    faultCode = FAULT_NONE;
    if (!isReversing && now >= reverseCooldownUntil) {
      isTurning = false;
      initStraightMode(-1);
      isReversing = true;
      reverseStartTime = now;
      reverseStartX = pose.x;
      reverseStartY = pose.y;
      BACK();
      Serial.println("후진 명령(REVERSE) 수신 - 최대 25cm / 1.5초 후진 시작");
    }
  } else if (cmd == "0") {
    lastRecvTime = now;
    isReversing = false; // 전진 명령이 오면 후진 취소
        // ★ 래치된 이상 상태에서는 실패했던 그 동작(직진)만 거부한다.
        // STOP / 회전 명령은 그대로 통과하므로 PC 가 탈출을 지시할 수 있다.
        if (faultCode != FAULT_NONE) {
          if (now - faultTime < FAULT_LATCH_TIMEOUT_MS) {
            static unsigned long lastFaultLog = 0;
            if (now - lastFaultLog > 1000) {
              lastFaultLog = now;
              Serial.print("[FAULT ");
              Serial.print(faultCode);
              Serial.print("] 직진 명령 거부 - ");
              Serial.println(faultName(faultCode));
            }
            return;
          }
          // 안전망: PC 가 끝내 인지하지 못해도 로봇이 영구히 멈춰 있지는 않도록 자동 해제
          Serial.println("[FAULT] 래치 타임아웃 - 자동 해제 후 직진 재시도 허용");
          faultCode = FAULT_NONE;
        }
    if (isTurning || pwmA == 0) {
      initStraightMode();
    }
    isTurning = false;
    ADVANCE();
    Serial.println("직진 명령(0) 수신 및 직진 시작");
  } else {
    // 숫자 각도 검사
    bool isValidNumber = true;
    for (unsigned int i = 0; i < cmd.length(); i++) {
      if (i == 0 && cmd[i] == '-') continue;
      if (!isDigit(cmd[i])) {
        isValidNumber = false;
        break;
      }
    }
    if (isValidNumber) {
      int target = cmd.toInt();
      // 쿨다운에 걸려 명령이 무시되더라도 통신은 살아있으므로 워치독 먼저 갱신
      lastRecvTime = now;
      // ★ 회전 중에는 새 각도 명령 무시 (JSON 경로와 동일한 이유)
      if (isTurning) {
        Serial.println("회전 중 - 새 각도 명령 무시");
      } else if (!(target == lastCommand && (now - lastCmdTime) < CMD_COOLDOWN)) {
        faultCode = FAULT_NONE; // 직진과 다른 동작이므로 허용 (PC 의 탈출 시도)
        isReversing = false;
        lastCommand = target;
        lastCmdTime = now;
        Serial.print("회전 각도 수신: ");
        Serial.println(target);

        STOP();
        driveDir = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        delay(100);
        destinationAngle = yaw + target;
        if (destinationAngle >= 360.0) destinationAngle -= 360.0;
        if (destinationAngle < 0.0)   destinationAngle += 360.0;
        isTurning = true;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // 나노와 고속 통신 설정 (115200bps)

  delay(300); // PS2 무선 모듈 기동 대기
  ps2_error =
      ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  if (ps2_error == 0) {
    ps2_type = ps2x.readType();
    Serial.println("PS2 컨트롤러 연결 성공");
  } else {
    Serial.print("PS2 컨트롤러 감지 실패, 에러 코드: ");
    Serial.println(ps2_error);
  }

  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  STOP();

  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCA_C, INPUT_PULLUP);
  pinMode(ENCA_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_A), isrEncA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_B), isrEncB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_C), isrEncC, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_D), isrEncD, RISING);

  // I2C는 나노가 담당하므로 메가에서 제거됨

  while (Serial2.available() > 0)
    Serial2.read();

  STOP();
  yaw = 0;
  resetOdometry(); // ★ 위치 초기화
  isCalibrated = true;
  lastRecvTime = millis(); // 워치독 타이머 초기화
  Serial.println("준비 완료");
}

void loop() {
  uint32_t now = millis();

  // ── 데이터 전송 및 출력 주기 제어 ──────────────────────
  static uint32_t p_ms = 0;
  if (now - p_ms >= 500) {
    print_all_data(now);
    p_ms = now;
  }

  // ── PS2 컨트롤러 처리 ──────────────────────────────────
  static bool wasPs2Advancing = false;
  static bool wasPs2Backing = false;
  bool ps2Controlled = false;
  bool ps2StraightHeld = false; // 이번 루프에 전진/후진 버튼이 유지되고 있는지 (부스트 사다리용)
  if (ps2_error == 0 && ps2_type != 2) {
    ps2x.read_gamepad(false, vibrate);

    bool buttonPressed =
        ps2x.Button(PSB_START) || ps2x.Button(PSB_PAD_UP) ||
        ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT) ||
        ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_SELECT) ||
        ps2x.Button(PSB_PINK) || ps2x.Button(PSB_RED) ||
        ps2x.Button(PSB_GREEN) || ps2x.Button(PSB_BLUE) ||
        ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1);

    if (buttonPressed) {
      // 수동 조작이 항상 우선: PS2 조작을 "시작하는 순간"에만 자율 주행에서 걸린
      // 래치를 해제한다. 매 틱 해제하면 감지 -> 즉시 해제가 반복돼 부스트 사다리가
      // 1단계로 올라가지 못한다. (PS2 중에는 감지 로직이 래치하지 않으므로,
      //  이후의 해제는 "조건이 사라지면 자동 복귀" 경로가 담당한다)
      if (!wasPs2Controlled && faultCode != FAULT_NONE) {
        Serial.println("[FAULT] PS2 수동 조작 시작 - 래치 해제");
        faultCode = FAULT_NONE;
        boostStage = 0;
      }
      ps2Controlled = true;
      wasPs2Controlled = true;
      isTurning = false;
      isReversing = false; // 수동 조작이 자동 후진보다 우선
      lastRecvTime = now; // 워치독 방지

      // ── 직진 (헤딩 보정 및 PID 적용) ──
      if (ps2x.Button(PSB_START) || ps2x.Button(PSB_PAD_UP) ||
          ps2x.Button(PSB_GREEN)) {
        wasPs2Backing = false;
        ps2StraightHeld = true;
        Motor_PWM = applyBoostLadder(125, now);
        if (!wasPs2Advancing || pwmA == 0) {
          initStraightMode(); // 헤딩 락 기준 각도 캡처 및 PID 리셋
          wasPs2Advancing = true;
        }
        ADVANCE();
      } else if (ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_BLUE)) {
        wasPs2Advancing = false;
        ps2StraightHeld = true;
        Motor_PWM = applyBoostLadder(125, now);
        if (!wasPs2Backing || pwmA == 0) {
          initStraightMode(-1); // 헤딩 락 기준 각도 캡처 및 PID 리셋 (후진)
          wasPs2Backing = true;
        }
        BACK();
      } else if (ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PINK)) {
        wasPs2Advancing = false;
        wasPs2Backing = false;
        driveDir = 0;
        pwmA = 0;
        Motor_PWM = 125;
        TURN_LEFT();
      } else if (ps2x.Button(PSB_PAD_RIGHT) || ps2x.Button(PSB_RED)) {
        wasPs2Advancing = false;
        wasPs2Backing = false;
        driveDir = 0;
        pwmA = 0;
        Motor_PWM = 125;
        TURN_RIGHT();
      } else if (ps2x.Button(PSB_SELECT)) {
        wasPs2Advancing = false;
        wasPs2Backing = false;
        driveDir = 0;
        currentBasePWM = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        STOP();
      } else if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
        int LY = ps2x.Analog(PSS_LY);
        int LX = ps2x.Analog(PSS_LX);

        if (LY < 127) { // 아날로그 전진 (헤딩 보정 적용)
          ps2StraightHeld = true;
          Motor_PWM = applyBoostLadder((int)(1.5 * (127 - LY)), now);
          wasPs2Backing = false;
          if (!wasPs2Advancing || pwmA == 0) {
            initStraightMode();
            wasPs2Advancing = true;
          }
          ADVANCE();
        } else if (LY > 127) { // 아날로그 후진 (헤딩 보정 적용)
          wasPs2Advancing = false;
          ps2StraightHeld = true;
          Motor_PWM = applyBoostLadder((int)(1.5 * (LY - 128)), now);
          if (!wasPs2Backing || pwmA == 0) {
            initStraightMode(-1);
            wasPs2Backing = true;
          }
          BACK();
        } else if (LX < 128) {
          wasPs2Advancing = false;
          wasPs2Backing = false;
          driveDir = 0;
          pwmA = 0;
          Motor_PWM = 1.5 * (127 - LX);
          TURN_LEFT();
        } else if (LX > 128) {
          wasPs2Advancing = false;
          wasPs2Backing = false;
          driveDir = 0;
          pwmA = 0;
          Motor_PWM = 1.5 * (LX - 128);
          TURN_RIGHT();
        } else {
          wasPs2Advancing = false;
          wasPs2Backing = false;
          driveDir = 0;
          currentBasePWM = 0;
          pwmA = pwmB = pwmC = pwmD = 0;
          STOP();
        }
      }
      delay(20);
    } else {
      if (wasPs2Controlled) {
        wasPs2Advancing = false;
        wasPs2Backing = false;
        driveDir = 0;
        currentBasePWM = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        STOP();
        wasPs2Controlled = false;
      }
    }
  }

  // 전진/후진 버튼에서 손을 떼면 부스트 사다리를 초기화한다.
  // (보호 제한(2단계)이 걸려 있어도 여기서 풀리므로, 조작자가 버튼을 놓았다
  //  다시 누르면 정상 출력으로 재시도할 수 있다)
  if (!ps2StraightHeld && boostStage != 0) {
    boostStage = 0;
    Serial.println("[BOOST] 직진 조작 해제 - 출력 사다리 초기화");
  }

  // ── 나노(Serial2) 수신: PS2 조작 중에도 반드시 실행 ────────────
  // yaw를 여기서만 갱신하므로, 이 블록이 막히면 헤딩락이 정지된 각도로
  // 동작해 보정이 전혀 이뤄지지 않는다. (주행 명령만 수동 조작 중 무시)
  static String inputBuffer2 = "";
  while (isCalibrated && Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      inputBuffer2.trim();
      if (inputBuffer2.length() > 0) {
        // YAW 데이터 파싱 (나노로부터 공급받음)
        if (inputBuffer2.startsWith("YAW:")) {
          float tempYaw = inputBuffer2.substring(4).toFloat();
          if (!isnan(tempYaw) && !isinf(tempYaw)) {
            yaw = tempYaw;
            // ★ 여기서 lastRecvTime 을 갱신하면 안 된다.
            // 나노는 BLE 가 끊겨도 IMU YAW 를 계속 보내므로, YAW 로 워치독을
            // 되살리면 PC 링크가 죽어도 워치독이 영원히 발동하지 않는다.
            // 워치독(lastRecvTime)은 "주행 명령 수신"에서만 갱신한다.
            lastYawTime = now;  // YAW 수신 상태 갱신 (헤딩락 유효성 점검 전용)
          }
        } else if (!ps2Controlled) {
          // PS2 수동 조작 중에는 자동 주행 명령을 무시
          processCommand(inputBuffer2, now);
        }
      }
      inputBuffer2 = "";
    } else if (c != '\r') {
      inputBuffer2 += c;
      if (inputBuffer2.length() > 100)
        inputBuffer2 = "";
    }
  }

  // ── 엔코더/오도메트리/PID 제어: 100ms 주기 (상시 실행) ──
  if (now - lastSpeedTime >= 100) {
    float dt = (now - lastSpeedTime) / 1000.0;
    if (dt <= 0.0)
      dt = 0.1;

    // 인터럽트 변수 안전 복사 및 리셋
    noInterrupts();
    long rawA = encA;
    encA = 0;
    long rawB = encB;
    encB = 0;
    long rawC = encC;
    encC = 0;
    long rawD = encD;
    encD = 0;
    interrupts();

    // 모터 회전 방향(전진/후진) 부호 반영
    long cA = rawA * dirSignA;
    long cB = rawB * dirSignB;
    long cC = rawC * dirSignC;
    long cD = rawD * dirSignD;

    // ★ 위치 좌표 업데이트
    updateOdometry(cA, cB, cC, cD, yaw, isTurning);

    speedA = (abs(cA) / 10.0) / dt;
    speedB = (abs(cB) / 10.0) / dt;
    speedC = (abs(cC) / 10.0) / dt;
    speedD = (abs(cD) / 10.0) / dt;
    lastSpeedTime = now;

    // 직진/후진 주행 중일 때 100ms 주기로 PID 연산 및 출력 반영 (자동/리모컨 공통)
    if (!isTurning && driveDir != 0 && pwmA > 0) {
      // ── 1. 가감속(Ramp) 점진적 속도 증가/감소 (목표: Motor_PWM) ──
      if (currentBasePWM < Motor_PWM) {
        currentBasePWM = min(currentBasePWM + RAMP_ACCEL_STEP, Motor_PWM);
      } else if (currentBasePWM > Motor_PWM) {
        currentBasePWM = max(currentBasePWM - RAMP_DECEL_STEP, Motor_PWM);
      }

      // ── 2. 헤딩 오차 계산 (-180 ~ +180도 정규화) ──
      float headingError = targetHeading - yaw;
      if (headingError > 180.0)
        headingError -= 360.0;
      if (headingError < -180.0)
        headingError += 360.0;

      // 적분 누적 (출발 시 생긴 오프셋을 시간이 지나며 되돌리는 역할)
      float integralNext =
          constrain(headingIntegral + headingError * dt, -HEADING_I_LIMIT,
                    HEADING_I_LIMIT);
      float rawCorrection =
          headingError * KP_HEADING + integralNext * KI_HEADING;

      // [안티 와인드업 - 조건부 적분]
      // KP=6 이라 |오차| > 5도면 P 항만으로 이미 출력이 ±30 에 걸린다. 그 상태에서도
      // 적분을 계속 쌓으면 (오차 10도 기준 약 1초면 리밋 10 도달) 슬립/문턱/끼임처럼
      // 바퀴가 헛도는 구간을 빠져나온 직후, 오차가 0을 지나도 I항이 홀로 ±30 을
      // 유지하며 로봇을 반대편으로 계속 밀어 크게 휘청이게 만든다.
      // 출력이 포화됐고 오차가 그 포화를 더 키우는 방향일 때만 적분을 멈춘다.
      // 포화되지 않은 정상 주행 구간에서는 기존 코드와 완전히 동일하게 동작한다.
      bool headingSaturated =
          (rawCorrection > 30.0f && headingError > 0.0f) ||
          (rawCorrection < -30.0f && headingError < 0.0f);
      if (!headingSaturated) {
        headingIntegral = integralNext;
      }

#if FAULT_DETECT_ENABLE
      // ── 이상 상태 감지 (출발 직후 유예 시간 이후에만) ──────────
      if ((now - straightStartTime) > FAULT_GRACE_MS) {
        // A. 모터 스톨: 실제로 밀고 있는데 네 바퀴 평균이 거의 안 돎
        float avgSpeed = (speedA + speedB + speedC + speedD) / 4.0f;
        bool stallNow =
            (currentBasePWM >= STALL_MIN_BASE_PWM && avgSpeed < STALL_SPEED_THRESH);

        // B. 헤딩 싸움: 보정이 포화된 채로 큰 오차가 지속
        bool headingNow =
            (headingSaturated && fabs(headingError) > HEADING_FAULT_DEG);

        // C. 바퀴 편차: 한 바퀴만 나머지 3개의 중앙값에서 크게 벗어남.
        //    좌우 비교로는 판정할 수 없다 - 내부 휠 PID 가 명령을 크게 감쇠시켜서
        //    정상 주행에서도 명령한 좌우 속도차와 실측이 늘 어긋나기 때문.
        float sp[4] = {speedA, speedB, speedC, speedD};
        bool wheelNow = false;
        for (int i = 0; i < 4; i++) {
          float others[3];
          int k = 0;
          for (int j = 0; j < 4; j++) {
            if (j != i) others[k++] = sp[j];
          }
          float med = medianOf3(others[0], others[1], others[2]);
          // 나머지가 실제로 돌고 있을 때만 의미가 있다 (전체 정지는 A 의 몫)
          if (med > STALL_SPEED_THRESH) {
            if (sp[i] < med * WHEEL_LOW_RATIO || sp[i] > med * WHEEL_HIGH_RATIO) {
              wheelNow = true;
            }
          }
        }

        stallCount = stallNow ? (uint8_t)(stallCount + 1) : 0;
        headingFaultCount = headingNow ? (uint8_t)(headingFaultCount + 1) : 0;
        wheelFaultCount = wheelNow ? (uint8_t)(wheelFaultCount + 1) : 0;

        uint8_t newFault = FAULT_NONE;
        if (stallCount >= STALL_TICKS) newFault = FAULT_STALL;
        else if (headingFaultCount >= HEADING_TICKS) newFault = FAULT_HEADING;
        else if (wheelFaultCount >= WHEEL_TICKS) newFault = FAULT_WHEEL;

        if (newFault != FAULT_NONE && faultCode != newFault) {
          faultCode = newFault;
          faultTime = now;
          Serial.print("[FAULT ");
          Serial.print(faultCode);
          Serial.print("] ");
          Serial.print(faultName(faultCode));
          Serial.print(" | 평균속도 ");
          Serial.print(avgSpeed, 0);
          Serial.print(" (A");
          Serial.print(speedA, 0);
          Serial.print(" B");
          Serial.print(speedB, 0);
          Serial.print(" C");
          Serial.print(speedC, 0);
          Serial.print(" D");
          Serial.print(speedD, 0);
          Serial.print(") | 헤딩오차 ");
          Serial.print(headingError, 1);
          Serial.print("° | base ");
          Serial.println(currentBasePWM);
        }

        // PS2 수동 조작 중에는 래치하지 않는다. 사람이 보고 있고, 부스트 사다리가
        // 조작자의 버튼 유지 여부로 이미 통제되기 때문. 조건이 사라지면 바로 해제.
        if (ps2Controlled) {
          if (!stallNow && !headingNow && !wheelNow) {
            healthyCount++;
            if (healthyCount >= STALL_TICKS && faultCode != FAULT_NONE) {
              Serial.println("[FAULT] 이상 조건 해소 - 상태 정상 복귀");
              faultCode = FAULT_NONE;
            }
          } else {
            healthyCount = 0;
          }
        } else if (faultCode != FAULT_NONE) {
          // 자율 주행: 즉시 정지하고 래치. 이후 "0"(직진) 명령은 거부된다.
          // 탈출 판단은 라이다/초음파를 가진 PC 의 몫이다 (메가는 후방이 완전 맹점).
          currentBasePWM = 0;
          STOP();
          driveDir = 0;
          pwmA = pwmB = pwmC = pwmD = 0;
        }
      }
#endif

      // 헤딩 보정량 계산 (오차가 클 때 과도한 보정 방지를 위해 ±30.0으로 제한)
      float headingCorrection = constrain(rawCorrection, -30.0f, 30.0f);

      // 좌/우 바퀴의 목표 속도 차등 적용
      // [실측 검증] 전진 중 우측을 증속하면 yaw가 감소한다. 따라서 yaw를
      // 올려야 하는 상황(headingError > 0)에서는 좌측을 증속해야 한다.
      // 후진 시에는 바퀴 회전 방향이 반대라 좌/우 속도차가 yaw에 반대 부호로
      // 작용하므로, driveDir을 곱해 보정 방향을 다시 반전시킨다.
      float targetSpeedLeft = (float)Motor_PWM + driveDir * headingCorrection;
      float targetSpeedRight = (float)Motor_PWM - driveDir * headingCorrection;

#if HEADING_SPEED_TRACE
      traceTargetLeft = targetSpeedLeft;
      traceTargetRight = targetSpeedRight;
#endif

      float corrA =
          constrain(pidA.compute(targetSpeedLeft, speedA), -70.0, 70.0);
      float corrB =
          constrain(pidB.compute(targetSpeedRight, speedB), -70.0, 70.0);
      float corrC =
          constrain(pidC.compute(targetSpeedLeft, speedC), -70.0, 70.0);
      float corrD =
          constrain(pidD.compute(targetSpeedRight, speedD), -70.0, 70.0);

      pwmA = constrain(currentBasePWM + (int)corrA, 0, 255);
      pwmB = constrain(currentBasePWM + (int)corrB, 0, 255);
      pwmC = constrain(currentBasePWM + (int)corrC, 0, 255);
      pwmD = constrain(currentBasePWM + (int)corrD, 0, 255);

      if (driveDir > 0)
        ADVANCE();
      else
        BACK();
    }
  }

  if (!ps2Controlled) {
    // ── 안전 워치독 (1.5초 이상 통신 두절 시 자동 비상 정지) ──
    if (now - lastRecvTime > 1500) {
      if (pwmA > 0 || isTurning || isReversing) {
        currentBasePWM = 0;
        STOP();
        isTurning = false;
        isReversing = false;
        driveDir = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        static unsigned long lastWdLog = 0;
        if (now - lastWdLog > 2000) {
          lastWdLog = now;
          Serial.println("[경고] 통신 두절로 인한 비상 정지 (Watchdog)");
        }
      }
    }

    // PC USB 시리얼(Serial) 직접 입력 수신
    static String inputBuffer1 = "";
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n') {
        inputBuffer1.trim();
        if (inputBuffer1.length() > 0) {
          processCommand(inputBuffer1, now);
        }
        inputBuffer1 = "";
      } else if (c != '\r') {
        inputBuffer1 += c;
        if (inputBuffer1.length() > 100)
          inputBuffer1 = "";
      }
    }
    // ── 후진 탈출 종료 판정 (거리 또는 시간, 먼저 걸리는 쪽) ──
    // 실제 구동은 아래 PID 블록이 driveDir = -1 로 BACK() 을 유지하며 담당한다.
    if (isReversing) {
      float movedBack = sqrt((pose.x - reverseStartX) * (pose.x - reverseStartX) +
                             (pose.y - reverseStartY) * (pose.y - reverseStartY));
      bool byDist = (movedBack >= REVERSE_TARGET_M);
      bool byTime = ((now - reverseStartTime) >= REVERSE_MAX_MS);
      if (byDist || byTime) {
        STOP();
        isReversing = false;
        reverseCooldownUntil = now + REVERSE_COOLDOWN_MS; // 하트비트로 인한 재시작 차단
        driveDir = 0;
        currentBasePWM = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        Serial.print("후진 종료 (");
        Serial.print(byDist ? "목표 거리 도달" : "시간 초과 - 슬립으로 실제로는 못 물러났을 수 있음");
        Serial.print(", 이동 ");
        Serial.print(movedBack, 3);
        Serial.println("m) -> 정지 유지");
      }
    }

    // 제자리 회전 시퀀스 (오차 비례 부드러운 감속 회전)
    if (isTurning) {
      float error = destinationAngle - yaw;
      if (error > 180.0)
        error -= 360.0;
      if (error < -180.0)
        error += 360.0;

      if (abs(error) <= ANGLE_TOLERANCE) {
        // ★ 회전 완료 후 스스로 전진하지 않는다.
        // 이전에는 여기서 initStraightMode()+ADVANCE() 를 실행해, PC 가 "제자리
        // 회전"만 지시했는데도 로봇이 앞으로 기어나갔다 (헤딩 정렬 버튼,
        // ㄷ자 회피의 90도 선회 단계 등). 정지 상태를 유지하고 다음 명령
        // ("0" = 직진)을 기다린다.
        STOP();
        isTurning = false;
        driveDir = 0;
        currentBasePWM = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        Serial.println("회전 완료 -> 정지 유지 (다음 명령 대기)");
      } else {
        // 남은 오차(2도 ~ 45도)에 따라 회전 PWM을 60 ~ Motor_PWM 사이로 비례
        // 감속
        int turnPwm = constrain((int)(abs(error) * 2.0f + 55), 60, Motor_PWM);

        if (error > 0)
          TURN_RIGHT(turnPwm);
        else
          TURN_LEFT(turnPwm);
      }
    }
  }
}