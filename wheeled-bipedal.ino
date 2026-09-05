#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "TWAI_CAN_MI_Motor.h"

// ==========================================================
//  PIN & CONFIG CONSTANTS
// ==========================================================
// ---- BTS7960 wheel drivers (replaces the L298N) ----
// Right motor - BTS7960 #1
const int R_RPWM = 25;
const int R_LPWM = 26;
// Left motor - BTS7960 #2
const int L_RPWM = 32;
const int L_LPWM = 33;
// R_EN / L_EN / VCC on both modules are wired straight to 3.3V,
// so both drivers are always enabled - nothing to drive for that in software.

// ---- Encoders ----
// Pin<->side mapping confirmed by the hardware test sketch: enc1 reads
// the RIGHT wheel and lives on 16/17, enc2 reads the LEFT wheel and
// lives on 34/35 (crossed from what the schematic would suggest).
const int M1_A = 16, M1_B = 17;   // right
const int M2_A = 34, M2_B = 35;   // left

const int I2C_SDA = 18, I2C_SCL = 19;
const byte IMU_ADDR = 0x68;

const int PWM_FREQ = 20000, PWM_RES = 10;
const int PWM_MAX  = (1 << PWM_RES) - 1;

// Gear ratio changed 270:1 -> 50:1. Assumes the encoder disc is still
// 64 CPR pre-gearbox like the old motors - verify against the new
// motor's datasheet if position/velocity numbers look off by a
// constant factor.
// The 37D "64 CPR" spec already includes 4x quadrature decoding (and
// attachFullQuad also decodes 4x), so counts/output-rev = 64 * 50 = 3200.
// (Was 64*4*50=12800, which double-counted quadrature and made every
// reported wheel speed read 4x too low - confirmed by a full-effort spin
// test reading 0.92 rev/s when the motor was actually doing ~3.7 rev/s.)
const float COUNTS_PER_OUTPUT_REV = 64.0 * 50.0;   // = 3200

const float CONTROL_HZ = 200.0;
const unsigned long CONTROL_US = 1000000UL / 200;

// ==========================================================
//  GLOBAL OBJECTS & STATE
// ==========================================================
ESP32Encoder enc1, enc2;
MI_Motor_ joint1, joint2;

struct RobotState {
  float pitch;
  float pitchRate;
  float wheelPosR, wheelPosL;
  float wheelVelR, wheelVelL;
};
RobotState state;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float gyroBiasEst = 0.0;   // ONLINE residual pitch-gyro bias, learned from the accel while
                           // near-upright (on top of the startup calibration); cancels the
                           // slow drift that made the estimate read "level" while leaning.
unsigned long loopCounter = 0;

// ==========================================================
//  JOINTS
// ==========================================================
const float J1_MIN = -0.319, J1_MAX = 0.233;
const float J2_MIN = -0.515, J2_MAX = 0.039;
float standJ1 = 0.21;
float standJ2 = -0.49;
const float POSE_SPEED = 0.3;
const float RELEASE_SPEED = 0.05;              // slower than POSE_SPEED, for a gentle lower
const unsigned long RELEASE_SETTLE_MS = 1500;  // time given to reach the rest position before cutting power

// "Down"/relaxed target angles - placeholders (currently just the opposite
// joint limits from the standing pose). Watch the first release and adjust
// restJ1/restJ2 to whatever position is actually "legs down" for your linkage.
float restJ1 = J1_MAX;
float restJ2 = J2_MIN;

bool jointsHolding = false;

float clampJ1(float v){ return v < J1_MIN ? J1_MIN : (v > J1_MAX ? J1_MAX : v); }
float clampJ2(float v){ return v < J2_MIN ? J2_MIN : (v > J2_MAX ? J2_MAX : v); }

// ==========================================================
//  BALANCE CONTROLLER + DEAD-ZONE COMPENSATION
// ==========================================================
bool  balanceEnabled = false;
float pitchSetpoint  = -4.0;

// NOTE: Kp/Kd/Ki, DEADZONE, and TIPOVER_LIMIT below are carried over
// unchanged from the 270:1 tune. The 50:1 gearboxes have roughly 5.4x
// less torque and 5.4x more free speed for the same effort value, so
// treat these as a starting point only - expect to re-tune all of
// them (and re-find DEADZONE's stiction point) once you start
// balancing on the new motors.
float Kp = 0.05;
float Kd = 0.05;
float Ki = 0.0;
float Kv = 0.0;        // wheel-velocity braking gain (opposes wheel spin to stop runaway; start 0, tune up)

float DEADZONE = 0.0;  // feedforward stiction comp. Leave 0 for balancing: any nonzero value
                       // injects a +/-DEADZONE bang-bang kick at the setpoint. Measured wheel
                       // stiction is ~0.14 if ever needed as a drive-mode feedforward.

float RATE_LP_ALPHA = 0.30;   // low-pass on gyro rate feeding the D term (0..1; lower = smoother, more lag)

// Overall control-sign multiplier. If the closed loop drives AWAY from the fall
// (robot dives off in one direction and diverges), the loop polarity is inverted
// somewhere (pitch or drive direction); set this to -1 to flip the whole control
// output at once. Live-tunable as "sign".
float CTRL_SIGN = 1.0;

// Dead-band (degrees). When the robot is within +/-PITCH_DB of the setpoint AND
// turning slowly, command zero effort so it stops fidgeting back and forth through
// the gearbox backlash (which sustains the jitter). 0 = off. Live-tunable 'pitchdb'.
float PITCH_DB = 0.0;

float integralError = 0.0;
float lastEffort = 0.0;
float lastRawEffort = 0.0;
float lastP = 0.0, lastD = 0.0, lastI = 0.0, lastV = 0.0;   // split-out control terms for telemetry
const float TIPOVER_LIMIT = 40.0;

// ---- Burst data logger ('L' captures a 2.5 s full-rate trace as CSV) ----
#define LOG_N 500
struct LogSample { float t, pitch, rate, rawEff, eff, P, D, V, wR, wL; };
LogSample logBuf[LOG_N];
int  logIdx = 0;
bool logging = false;
unsigned long logStartUs = 0;

void dumpLog() {
  Serial.println("---- LOG BEGIN (t_s,pitch,rate,rawEff,eff,P,D,V,wR,wL) ----");
  for (int i = 0; i < logIdx; i++) {
    Serial.printf("%.4f,%.3f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f\n",
                  logBuf[i].t, logBuf[i].pitch, logBuf[i].rate,
                  logBuf[i].rawEff, logBuf[i].eff, logBuf[i].P, logBuf[i].D,
                  logBuf[i].V, logBuf[i].wR, logBuf[i].wL);
  }
  Serial.println("---- LOG END ----");
}

// ==========================================================
//  MOTOR TEST MODE (bypasses balance to test raw motor response)
// ==========================================================
bool  motorTestMode = false;
float motorTestEffort = 0.0;

// ==========================================================
//  IMU
// ==========================================================
void imuInit() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
}

void imuReadRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, (uint8_t)14);
  int16_t rAx=(Wire.read()<<8)|Wire.read();
  int16_t rAy=(Wire.read()<<8)|Wire.read();
  int16_t rAz=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  int16_t rGx=(Wire.read()<<8)|Wire.read();
  int16_t rGy=(Wire.read()<<8)|Wire.read();
  int16_t rGz=(Wire.read()<<8)|Wire.read();
  ax=rAx/16384.0; ay=rAy/16384.0; az=rAz/16384.0;
  gx=rGx/131.0;   gy=rGy/131.0;   gz=rGz/131.0;
}

void calibrateGyro(int samples = 500) {
  float sx=0, sy=0, sz=0;
  float ax,ay,az,gx,gy,gz;
  for (int i = 0; i < samples; i++) {
    imuReadRaw(ax,ay,az,gx,gy,gz);
    sx+=gx; sy+=gy; sz+=gz;
    delay(3);
  }
  gyroBiasX = sx/samples;
  gyroBiasY = sy/samples;
  gyroBiasZ = sz/samples;
}

float accelPitch(float ax, float ay, float az) {
  return atan2(ax, az) * 180.0 / PI;
}

// ==========================================================
//  BALANCE FUNCTION (with dead-zone comp)
// ==========================================================
float computeBalance(float dt) {
  float error = pitchSetpoint - state.pitch;
  float P = Kp * error;
  float D = Kd * (-state.pitchRate);

  integralError += error * dt;
  if (integralError >  50) integralError =  50;
  if (integralError < -50) integralError = -50;
  float I = Ki * integralError;

  // Wheel-velocity braking: opposes how fast the wheels are actually spinning.
  // Pure pitch feedback lets the wheels accumulate speed and run away (drive one
  // way, over-translate, throw the body past vertical). This term pulls effort
  // back toward zero wheel speed so the robot settles instead of running off.
  // Uses the correctly-scaled encoder velocity, lightly low-passed (it's quantized).
  static float wheelVelFilt = 0.0f;
  // Right encoder (enc1, pins 16/17) is currently dead (reads ~0). Both wheels
  // get identical effort and spin together, so the LEFT wheel speed alone is a
  // valid proxy for robot velocity. Revert to 0.5*(wheelVelR+wheelVelL) once the
  // right encoder wiring is fixed.
  float avgVel = state.wheelVelL;
  wheelVelFilt += 0.20f * (avgVel - wheelVelFilt);
  float V = -Kv * wheelVelFilt;

  // CTRL_SIGN flips only the pitch-based terms (P/D/I). The wheel-velocity brake
  // V must keep its own sign so it always OPPOSES wheel speed regardless of the
  // pitch-loop polarity (otherwise flipping the loop turns the brake into a
  // wheel-speed accelerator -> runaway).
  float effort = CTRL_SIGN * (P + D + I) + V;
  lastRawEffort = effort;
  lastP = P; lastD = D; lastI = I; lastV = V;

  // Backlash dead-band: if already close to balance and moving slowly, stop driving
  // so the wheels don't buzz back and forth across the gear slack.
  if (PITCH_DB > 0.0f && fabsf(error) < PITCH_DB && fabsf(state.pitchRate) < 20.0f)
    effort = 0.0f;

  if (effort >  1.0) effort =  1.0;
  if (effort < -1.0) effort = -1.0;

  if (DEADZONE > 0.0 && fabs(effort) > 0.001) {
    if (effort > 0)  effort = DEADZONE + (1.0 - DEADZONE) * effort;
    else             effort = -DEADZONE + (1.0 - DEADZONE) * effort;
  }
  return effort;
}

// ==========================================================
//  STATE ESTIMATION
// ==========================================================
void updateState(float dt) {
  float ax,ay,az,gx,gy,gz;
  imuReadRaw(ax,ay,az,gx,gy,gz);
  gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;

  // NOTE: gy is negated here. The IMU's gy has the OPPOSITE sign to the
  // accelerometer-derived pitch direction, so during fast motion (when the
  // complementary filter leans on the gyro) the estimate would invert and the
  // controller would drive the wrong way -> it could hold gentle leans but
  // never catch a real fall. Negating makes gyro agree with the accel at all
  // speeds. (Confirmed: robot fell backward while logged pitch went negative.)
  float gyroRate = -gy - gyroBiasEst;   // subtract the online-learned residual bias

  // --- Gyro-led complementary filter with accelerometer gating ---
  // Pitch integrates the RAW gyro (no filter lag on the angle itself).
  state.pitch += gyroRate * dt;

  // Correct toward the accelerometer tilt ONLY when the measured acceleration
  // is close to 1 g. During fast wheel moves the accelerometer also picks up
  // the robot's horizontal acceleration, so atan2(ax,az) lies (it can even
  // point the wrong way). Trusting it then drags the pitch estimate backwards
  // and the controller chases a phantom angle -> runaway. Gating it keeps the
  // estimate honest exactly when balancing needs it most.
  const float ACC_TRUST_BAND = 0.20f;   // use accel only within 0.20 g of gravity-only
  const float ACC_CORRECT    = 0.02f;   // correction strength. (0.05 was tried to fight gyro-bias
                                        // drift but it leaned too hard on the noisy accel and made
                                        // the estimate oscillate; use a small ki for the drift instead.)
  float accMag = sqrtf(ax*ax + ay*ay + az*az);   // in g
  if (fabsf(accMag - 1.0f) < ACC_TRUST_BAND) {
    float accAngle = accelPitch(ax, ay, az);
    float aerr = accAngle - state.pitch;
    state.pitch += ACC_CORRECT * aerr;      // fast proportional correction (unchanged)
    // Slowly learn the residual gyro bias from the same error. This is the Mahony
    // integral term: it drives the long-term drift to zero WITHOUT the noise that a
    // big proportional accel gain adds. Time constant ~ seconds.
    const float BIAS_KI = 0.30f;
    // Only learn the bias when NEARLY STILL. During the balancing wobble the accel is
    // motion-contaminated, so learning then teaches a wrong bias that accumulates over
    // a few seconds and eventually drops the robot ("turns off"). Learn it while
    // settling (before balance), then hold it steady through the wobble.
    if (fabsf(gyroRate) < 3.0f) {
      gyroBiasEst -= BIAS_KI * aerr * dt;
      if (gyroBiasEst >  5.0f) gyroBiasEst =  5.0f;   // clamp to +/-5 deg/s of learned bias
      if (gyroBiasEst < -5.0f) gyroBiasEst = -5.0f;
    }
  }

  // The D term needs a smoother rate: the raw gyro is too noisy to multiply by
  // Kd without amplifying noise into full-throttle wheel commands. A light
  // first-order low-pass keeps the lead (anticipation) while killing the jitter.
  static float rateFilt = 0.0f;
  rateFilt += RATE_LP_ALPHA * (gyroRate - rateFilt);
  state.pitchRate = rateFilt;

  static int64_t lastC1 = 0, lastC2 = 0;
  int64_t c1 =  enc1.getCount();
  int64_t c2 = -enc2.getCount();
  state.wheelPosR = (float)c1 / COUNTS_PER_OUTPUT_REV;
  state.wheelPosL = (float)c2 / COUNTS_PER_OUTPUT_REV;
  state.wheelVelR = ((float)(c1 - lastC1) / COUNTS_PER_OUTPUT_REV) / dt;
  state.wheelVelL = ((float)(c2 - lastC2) / COUNTS_PER_OUTPUT_REV) / dt;
  lastC1 = c1; lastC2 = c2;
}

void initStateEstimate() {
  float ax,ay,az,gx,gy,gz;
  imuReadRaw(ax,ay,az,gx,gy,gz);
  state.pitch = accelPitch(ax, ay, az);
  state.pitchRate = 0;
}

// ==========================================================
//  WHEEL MOTORS (BTS7960: two PWM pins per motor, no direction pin)
// ==========================================================
void setWheelR(float effort) {
  effort = constrain(effort, -1.0f, 1.0f);
  int duty = (int)(fabs(effort) * PWM_MAX);
  if (effort > 0.001f) {
    ledcWrite(R_RPWM, duty);
    ledcWrite(R_LPWM, 0);
  } else if (effort < -0.001f) {
    ledcWrite(R_RPWM, 0);
    ledcWrite(R_LPWM, duty);
  } else {
    ledcWrite(R_RPWM, 0);
    ledcWrite(R_LPWM, 0);
  }
}
void setWheelL(float effort) {
  effort = constrain(effort, -1.0f, 1.0f);
  int duty = (int)(fabs(effort) * PWM_MAX);
  if (effort > 0.001f) {
    ledcWrite(L_RPWM, duty);
    ledcWrite(L_LPWM, 0);
  } else if (effort < -0.001f) {
    ledcWrite(L_RPWM, 0);
    ledcWrite(L_LPWM, duty);
  } else {
    ledcWrite(L_RPWM, 0);
    ledcWrite(L_LPWM, 0);
  }
}
void stopWheels() { setWheelR(0); setWheelL(0); }

// ==========================================================
//  JOINT MOTORS
// ==========================================================
void prepJoint(MI_Motor_ &m, uint8_t id) {
  m.Motor_Con_Init(id); delay(100);
  m.Motor_Reset();      delay(200);
}
void holdPose() {
  float t1 = clampJ1(standJ1);
  float t2 = clampJ2(standJ2);
  joint1.Change_Mode(POS_MODE); delay(50);
  joint1.Motor_Enable();        delay(50);
  joint2.Change_Mode(POS_MODE); delay(50);
  joint2.Motor_Enable();        delay(50);
  joint1.Set_PosMode(t1, POSE_SPEED);
  joint2.Set_PosMode(t2, POSE_SPEED);
  jointsHolding = true;
  Serial.printf(">>> HOLDING POSE: j1=%.3f j2=%.3f <<<\n", t1, t2);
}
void releaseJoints() {
  float r1 = clampJ1(restJ1);
  float r2 = clampJ2(restJ2);

  Serial.println(">>> LOWERING JOINTS <<<");
  joint1.Set_PosMode(r1, RELEASE_SPEED);
  joint2.Set_PosMode(r2, RELEASE_SPEED);

  unsigned long start = millis();
  while (millis() - start < RELEASE_SETTLE_MS) {
    pollJoint(joint1);
    pollJoint(joint2);
    delay(20);
  }

  joint1.Motor_Reset();
  joint2.Motor_Reset();
  jointsHolding = false;
  Serial.println(">>> JOINTS RELEASED <<<");
}
void pollJoint(MI_Motor_ &m) { m.Motor_Data_Updata(1); }

// ==========================================================
//  KEYBOARD
// ==========================================================
void handleKey(char c) {
  switch (c) {
    case 'H': holdPose();      break;
    case 'R': releaseJoints(); break;
    case 'B': balanceEnabled = true; motorTestMode = false; integralError = 0;
              Serial.println(">>> BALANCE ON <<<"); break;
    case ' ':
    case 'K': balanceEnabled = false; motorTestMode = false; stopWheels();
              Serial.println(">>> KILL <<<"); break;

    // ---- capture a 2.5 s full-rate trace of the balance response ----
    case 'L': logIdx = 0; logging = true; logStartUs = micros();
              Serial.println(">>> LOGGING 2.5s... <<<"); break;

    // ---- MOTOR TEST MODE: directly command wheel effort, bypass balance ----
    case 'T': motorTestMode = true; balanceEnabled = false; motorTestEffort = 0;
              Serial.println(">>> MOTOR TEST MODE (balance off) <<<"); break;
    case '1': motorTestEffort = 0.1;  Serial.println("effort=0.1"); break;
    case '2': motorTestEffort = 0.2;  Serial.println("effort=0.2"); break;
    case '3': motorTestEffort = 0.3;  Serial.println("effort=0.3"); break;
    case '5': motorTestEffort = 0.5;  Serial.println("effort=0.5"); break;
    case '9': motorTestEffort = 1.0;  Serial.println("effort=1.0"); break;
    case '0': motorTestEffort = 0.0;  Serial.println("effort=0.0"); break;
    case '-': motorTestEffort = -motorTestEffort;
              Serial.printf("effort=%.1f (flipped)\n", motorTestEffort); break;
  }
}

// ==========================================================
//  LIVE TUNING OVER SERIAL
// ==========================================================
// Type "<name> <value>" and press Enter to change a value without
// reflashing, e.g.  kp 0.08   or   deadzone 0.15
// Type "list" to print current values.
struct TunableParam {
  const char* name;
  float* value;
};

TunableParam tunables[] = {
  {"kp",       &Kp},
  {"kd",       &Kd},
  {"ki",       &Ki},
  {"kv",       &Kv},
  {"ratelp",   &RATE_LP_ALPHA},
  {"sign",     &CTRL_SIGN},
  {"pitchdb",  &PITCH_DB},
  {"deadzone", &DEADZONE},
  {"setpoint", &pitchSetpoint},
  {"standj1",  &standJ1},
  {"standj2",  &standJ2},
  {"effort",   &motorTestEffort},   // fine T-mode effort, e.g. "effort 0.06" (T mode must be on)
};
const int NUM_TUNABLES = sizeof(tunables) / sizeof(tunables[0]);

void printTunables() {
  Serial.println("---- current tuning values ----");
  for (int i = 0; i < NUM_TUNABLES; i++) {
    Serial.printf("  %-9s = %.4f\n", tunables[i].name, *(tunables[i].value));
  }
  Serial.println("Set with: <name> <value>   e.g. kp 0.08   (type 'list' to see this again)");
  Serial.println("--------------------------------");
}

bool namesMatch(const char* a, const char* b) {
  while (*a && *b) {
    if (tolower(*a) != tolower(*b)) return false;
    a++; b++;
  }
  return *a == '\0' && *b == '\0';
}

void processTuningLine(char* line) {
  while (*line == ' ') line++;
  if (strlen(line) == 0) return;

  if (namesMatch(line, "list") || namesMatch(line, "help")) {
    printTunables();
    return;
  }

  // split into "<name> <value>" on the first space or '='
  char* sep = line;
  while (*sep && *sep != ' ' && *sep != '=') sep++;
  if (*sep == '\0') {
    Serial.printf("Unknown command '%s' (try: list)\n", line);
    return;
  }
  *sep = '\0';
  char* valueStr = sep + 1;
  while (*valueStr == ' ' || *valueStr == '=') valueStr++;

  for (int i = 0; i < NUM_TUNABLES; i++) {
    if (namesMatch(line, tunables[i].name)) {
      float v = atof(valueStr);
      *(tunables[i].value) = v;
      Serial.printf(">>> %s = %.4f <<<\n", tunables[i].name, v);
      if (namesMatch(tunables[i].name, "standj1") || namesMatch(tunables[i].name, "standj2")) {
        Serial.println("(press H to move the joints to the new pose)");
      }
      return;
    }
  }
  Serial.printf("Unknown parameter '%s' (try: list)\n", line);
}

// Legacy single-key commands (H,R,B,K,space,T,digits,-,?) still act
// instantly, byte by byte, unchanged - this keeps the kill switch
// (space/K) instant with no Enter needed. Anything that starts with a
// letter is instead treated as the start of a typed tuning command
// (e.g. "kp 0.08") and buffered until Enter, since it needs several
// characters before it means anything.
char serialLineBuf[40];
int  serialLineLen = 0;
bool inTuningLine = false;

bool isLegacyKey(char c) {
  if (c == 'H' || c == 'R' || c == 'B' || c == 'K' || c == 'T' ||
      c == 'L' || c == ' ' || c == '-' || c == '?') return true;
  if (c >= '0' && c <= '9') return true;
  return false;
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    if (!inTuningLine) {
      if (c == '\n' || c == '\r') continue;
      if (isLegacyKey(c)) {
        handleKey(c);
        continue;
      }
      if (isalpha(c)) {
        inTuningLine = true;
        serialLineLen = 0;
        serialLineBuf[serialLineLen++] = c;
        continue;
      }
      continue;  // stray punctuation at line start - ignore
    }

    if (c == '\n' || c == '\r') {
      serialLineBuf[serialLineLen] = '\0';
      processTuningLine(serialLineBuf);
      inTuningLine = false;
      serialLineLen = 0;
    } else if (serialLineLen < (int)sizeof(serialLineBuf) - 1) {
      serialLineBuf[serialLineLen++] = c;
    }
  }
}

// ==========================================================
//  SETUP
// ==========================================================
void setup() {
  Serial.setTxBufferSize(1024);   // large TX buffer so the status print never blocks the 200 Hz loop
  Serial.begin(115200);
  delay(500);

  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  stopWheels();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc1.attachFullQuad(M1_A, M1_B);
  enc2.attachFullQuad(M2_A, M2_B);
  enc1.clearCount(); enc2.clearCount();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);   // 400 kHz fast-mode I2C: ~4x quicker IMU reads = less loop latency
  imuInit();

  Motor_CAN_Init();
  delay(100);
  prepJoint(joint1, MOTER_1_ID);
  prepJoint(joint2, MOTER_2_ID);

  Serial.println("Hold robot STILL - calibrating gyro...");
  calibrateGyro();
  initStateEstimate();
  Serial.printf("Gyro bias: %.3f %.3f %.3f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.println("Ready.");
  Serial.println("H=stand R=release B=balance SPACE/K=kill");
  Serial.println("T=motor test | 1/2/3/5/9=effort 0=stop -=reverse");
  Serial.println("Tune live: type '<name> <value>' + Enter (e.g. kp 0.08), or 'list'");
  printTunables();
}

// ==========================================================
//  MAIN CONTROL LOOP
// ==========================================================
void loop() {
  readSerialCommands();

  static unsigned long lastControlUs = 0;
  unsigned long nowUs = micros();

  if (nowUs - lastControlUs >= CONTROL_US) {
    float dt = (nowUs - lastControlUs) / 1000000.0;
    lastControlUs = nowUs;
    loopCounter++;

    updateState(dt);

    if (balanceEnabled &&
        fabs(state.pitch - pitchSetpoint) > TIPOVER_LIMIT) {
      balanceEnabled = false;
      integralError = 0;
    }

    if (motorTestMode) {
      lastEffort = motorTestEffort;
      setWheelR(motorTestEffort);
      setWheelL(motorTestEffort);
    } else if (balanceEnabled) {
      lastEffort = computeBalance(dt);
      setWheelR(lastEffort);
      setWheelL(lastEffort);
    } else {
      stopWheels();
      integralError = 0;
      lastEffort = 0.0;
    }

    // ---- burst logger: record every control tick while active ----
    if (logging) {
      if (logIdx < LOG_N) {
        logBuf[logIdx].t      = (nowUs - logStartUs) / 1000000.0;
        logBuf[logIdx].pitch  = state.pitch;
        logBuf[logIdx].rate   = state.pitchRate;
        logBuf[logIdx].rawEff = lastRawEffort;
        logBuf[logIdx].eff    = lastEffort;
        logBuf[logIdx].P      = lastP;
        logBuf[logIdx].D      = lastD;
        logBuf[logIdx].V      = lastV;
        logBuf[logIdx].wR     = state.wheelVelR;
        logBuf[logIdx].wL     = state.wheelVelL;
        logIdx++;
      } else {
        logging = false;
        dumpLog();
      }
    }
  }

  // ---- housekeeping ----
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll >= 100) {
    lastPoll = millis();
    pollJoint(joint1);
    pollJoint(joint2);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    const char* mode = motorTestMode ? "TEST" : (balanceEnabled ? "BAL" : "off");
    Serial.printf("pitch=%.2f rate=%.1f bias=%.2f P=%.2f D=%.2f V=%.2f rawEff=%.2f eff=%.2f wR=%.2f wL=%.2f | j1cmd=%.3f j1act=%.3f j2cmd=%.3f j2act=%.3f [%s]\n",
                  state.pitch, state.pitchRate, gyroBiasEst, lastP, lastD, lastV, lastRawEffort, lastEffort,
                  state.wheelVelR, state.wheelVelL,
                  standJ1, joint1.motor_rx_data.cur_angle,
                  standJ2, joint2.motor_rx_data.cur_angle, mode);
  }

  static unsigned long lastRateCheck = 0;
  static unsigned long lastLoopCount = 0;
  if (millis() - lastRateCheck >= 2000) {
    unsigned long rate = (loopCounter - lastLoopCount) / 2;
    lastLoopCount = loopCounter;
    lastRateCheck = millis();
    Serial.printf(">>> loop rate: %lu Hz <<<\n", rate);
  }
}