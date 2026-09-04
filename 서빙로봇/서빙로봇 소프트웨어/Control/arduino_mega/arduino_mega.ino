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
  }

  // ★ Nano(Serial2)로 좌표 데이터 전송 (YAW는 나노가 직접 계산하므로 제외)
  Serial2.print("POS:");
  Serial2.print(pose.x, 3);
  Serial2.print(",");
  Serial2.println(pose.y, 3);
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

    // R-signal 처리 (부가 명령어: RESET_ODO)
    if (rSignal.equalsIgnoreCase("RESET_ODO")) {
      lastRecvTime = now;
      resetOdometry();
      Serial.println("위치 초기화 완료 (0, 0)");
      Serial2.println("POS:0.000,0.000,0.0");
    }

    // S-signal 처리 (주행/조향 명령)
    if (sSignal.length() > 0) {
      if (sSignal.equalsIgnoreCase("STOP") || sSignal == "-1") {
        lastRecvTime = now;
        currentBasePWM = 0;
        STOP();
        isTurning = false;
        driveDir = 0;
        pwmA = pwmB = pwmC = pwmD = 0;
        Serial.println("정지 명령(STOP) 수신 및 정지 완료");
      } else if (sSignal == "0") {
        lastRecvTime = now;
        if (isTurning || pwmA == 0) {
          initStraightMode();
        }
        isTurning = false;
        ADVANCE();
        Serial.println("직진 명령(0) 수신 및 직진 시작");
      } else {
        // 각도 회전 명령 (예: 90, -90 등)
        int target = sSignal.toInt();
        if (!(target == lastCommand && (now - lastCmdTime) < CMD_COOLDOWN)) {
          lastCommand = target;
          lastCmdTime = now;
          lastRecvTime = now;
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
    Serial2.println("POS:0.000,0.000,0.0");
  } else if (cmd.equalsIgnoreCase("STOP") || cmd == "-1") {
    lastRecvTime = now;
    currentBasePWM = 0;
    STOP();
    isTurning = false;
    driveDir = 0;
    pwmA = pwmB = pwmC = pwmD = 0;
    Serial.println("정지 명령(STOP) 수신 및 정지 완료");
  } else if (cmd == "0") {
    lastRecvTime = now;
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
      if (!(target == lastCommand && (now - lastCmdTime) < CMD_COOLDOWN)) {
        lastCommand = target;
        lastCmdTime = now;
        lastRecvTime = now;
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
      ps2Controlled = true;
      wasPs2Controlled = true;
      isTurning = false;
      lastRecvTime = now; // 워치독 방지

      // ── 직진 (헤딩 보정 및 PID 적용) ──
      if (ps2x.Button(PSB_START) || ps2x.Button(PSB_PAD_UP) ||
          ps2x.Button(PSB_GREEN)) {
        wasPs2Backing = false;
        Motor_PWM = 125;
        if (!wasPs2Advancing || pwmA == 0) {
          initStraightMode(); // 헤딩 락 기준 각도 캡처 및 PID 리셋
          wasPs2Advancing = true;
        }
        ADVANCE();
      } else if (ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_BLUE)) {
        wasPs2Advancing = false;
        Motor_PWM = 125;
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
          Motor_PWM = 1.5 * (127 - LY);
          wasPs2Backing = false;
          if (!wasPs2Advancing || pwmA == 0) {
            initStraightMode();
            wasPs2Advancing = true;
          }
          ADVANCE();
        } else if (LY > 127) { // 아날로그 후진 (헤딩 보정 적용)
          wasPs2Advancing = false;
          Motor_PWM = 1.5 * (LY - 128);
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
            lastRecvTime = now; // 워치독 타이머 갱신
            lastYawTime = now;  // YAW 수신 상태 갱신
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
      headingIntegral += headingError * dt;
      headingIntegral =
          constrain(headingIntegral, -HEADING_I_LIMIT, HEADING_I_LIMIT);

      // 헤딩 보정량 계산 (오차가 클 때 과도한 보정 방지를 위해 ±30.0으로 제한)
      float headingCorrection =
          constrain(headingError * KP_HEADING + headingIntegral * KI_HEADING,
                    -30.0f, 30.0f);

      // 좌/우 바퀴의 목표 속도 차등 적용
      // [실측 검증] 전진 중 우측을 증속하면 yaw가 감소한다. 따라서 yaw를
      // 올려야 하는 상황(headingError > 0)에서는 좌측을 증속해야 한다.
      // 후진 시에는 바퀴 회전 방향이 반대라 좌/우 속도차가 yaw에 반대 부호로
      // 작용하므로, driveDir을 곱해 보정 방향을 다시 반전시킨다.
      float targetSpeedLeft = (float)Motor_PWM + driveDir * headingCorrection;
      float targetSpeedRight = (float)Motor_PWM - driveDir * headingCorrection;

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
      if (pwmA > 0 || isTurning) {
        currentBasePWM = 0;
        STOP();
        isTurning = false;
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
    // 제자리 회전 시퀀스 (오차 비례 부드러운 감속 회전)
    if (isTurning) {
      float error = destinationAngle - yaw;
      if (error > 180.0)
        error -= 360.0;
      if (error < -180.0)
        error += 360.0;

      if (abs(error) <= ANGLE_TOLERANCE) {
        STOP();
        delay(150);
        isTurning = false;
        initStraightMode();
        ADVANCE();
        Serial.println("직진 시작");
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