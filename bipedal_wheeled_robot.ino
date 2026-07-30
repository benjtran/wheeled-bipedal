#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <math.h>
#include "TWAI_CAN_MI_Motor.h"

// ==========================================================
//  PIN & CONFIG CONSTANTS
// ==========================================================
const int ENA = 25, IN1 = 26, IN2 = 27;      // right wheel (motor 1)
const int ENB = 32, IN3 = 33, IN4 = 14;      // left wheel  (motor 2)
const int M1_A = 34, M1_B = 35;              // right encoder
const int M2_A = 16, M2_B = 17;              // left encoder
const int I2C_SDA = 18, I2C_SCL = 19;
const byte IMU_ADDR = 0x68;

const int PWM_FREQ = 20000, PWM_RES = 10;
const int PWM_MAX  = (1 << PWM_RES) - 1;     // 1023

const float COUNTS_PER_OUTPUT_REV = 64.0 * 4.0 * 270.0;   // 69120

// Loop timing
const float CONTROL_HZ = 200.0;                       // balance loop rate
const unsigned long CONTROL_US = 1000000UL / 200;     // 5000 us period

// ==========================================================
//  GLOBAL OBJECTS & STATE
// ==========================================================
ESP32Encoder enc1, enc2;
MI_Motor_ joint1, joint2;

// --- estimated robot state (Layer 1 output) ---
struct RobotState {
  float pitch;        // fused tilt angle (deg) — 0 = upright
  float pitchRate;    // tilt angular velocity (deg/s)
  float wheelPosR;    // right wheel position (revs)
  float wheelPosL;    // left wheel position (revs)
  float wheelVelR;    // right wheel velocity (rev/s)
  float wheelVelL;    // left wheel velocity (rev/s)
};
RobotState state;

// --- gyro bias, found at startup ---
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ==========================================================
//  IMU  (raw read + fused angle)
// ==========================================================
void imuInit() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // wake
  Wire.endTransmission();
}

// raw read: accel in g, gyro in deg/s (bias NOT removed here)
void imuReadRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, (uint8_t)14);
  int16_t rAx=(Wire.read()<<8)|Wire.read();
  int16_t rAy=(Wire.read()<<8)|Wire.read();
  int16_t rAz=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();                 // skip temperature
  int16_t rGx=(Wire.read()<<8)|Wire.read();
  int16_t rGy=(Wire.read()<<8)|Wire.read();
  int16_t rGz=(Wire.read()<<8)|Wire.read();
  ax=rAx/16384.0; ay=rAy/16384.0; az=rAz/16384.0;
  gx=rGx/131.0;   gy=rGy/131.0;   gz=rGz/131.0;
}

// Measure gyro bias while robot is held perfectly still. Call once at startup.
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

// The accelerometer-only tilt angle (deg).
// NOTE: you must confirm which axes correspond to YOUR robot's fall direction.
// This assumes pitch about X using ay,az — verify empirically and change if needed.
float accelPitch(float ax, float ay, float az) {
  return atan2(ay, az) * 180.0 / PI;
}

// ==========================================================
//  STATE ESTIMATION  (Layer 1)
//  Fuses accel + gyro into state.pitch and state.pitchRate.
//  Call once per control cycle with the real dt.
// ==========================================================
void updateState(float dt) {
  float ax,ay,az,gx,gy,gz;
  imuReadRaw(ax,ay,az,gx,gy,gz);

  // remove gyro bias
  gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;

  // --- pitch: pick the gyro axis that matches your fall direction ---
  float gyroRate = gx;                 // <-- verify: which gyro axis is "tipping"?
  float accAngle = accelPitch(ax, ay, az);

  // ---- COMPLEMENTARY FILTER (your fusion lives here) ----
  // TODO(you): tune the 0.98/0.02 weighting. Higher gyro weight = smoother
  // but drifts more; higher accel weight = less drift but noisier.
  const float ALPHA = 0.98;
  state.pitch     = ALPHA * (state.pitch + gyroRate * dt) + (1.0 - ALPHA) * accAngle;
  state.pitchRate = gyroRate;

  // --- wheels ---
  static int64_t lastC1 = 0, lastC2 = 0;
  int64_t c1 =  enc1.getCount();
  int64_t c2 = -enc2.getCount();       // left encoder negated (leads swapped)
  state.wheelPosR = (float)c1 / COUNTS_PER_OUTPUT_REV;
  state.wheelPosL = (float)c2 / COUNTS_PER_OUTPUT_REV;
  state.wheelVelR = ((float)(c1 - lastC1) / COUNTS_PER_OUTPUT_REV) / dt;
  state.wheelVelL = ((float)(c2 - lastC2) / COUNTS_PER_OUTPUT_REV) / dt;
  lastC1 = c1; lastC2 = c2;
}

// seed pitch with the accelerometer angle so it doesn't start from 0
void initStateEstimate() {
  float ax,ay,az,gx,gy,gz;
  imuReadRaw(ax,ay,az,gx,gy,gz);
  state.pitch = accelPitch(ax, ay, az);
  state.pitchRate = 0;
}

// ==========================================================
//  WHEEL MOTORS  (Layer 0)
//  effort is -1.0 .. +1.0 ; sign = direction, magnitude = speed
// ==========================================================
void setWheelR(float effort) {
  int dir = (effort >= 0) ? 1 : -1;
  int duty = (int)(fabs(effort) * PWM_MAX);
  if (duty > PWM_MAX) duty = PWM_MAX;
  digitalWrite(IN1, dir >= 0 ? HIGH : LOW);
  digitalWrite(IN2, dir >= 0 ? LOW  : HIGH);
  ledcWrite(ENA, duty);
}
void setWheelL(float effort) {
  int dir = (effort >= 0) ? 1 : -1;
  int duty = (int)(fabs(effort) * PWM_MAX);
  if (duty > PWM_MAX) duty = PWM_MAX;
  digitalWrite(IN3, dir >= 0 ? HIGH : LOW);
  digitalWrite(IN4, dir >= 0 ? LOW  : HIGH);
  ledcWrite(ENB, duty);
}
void stopWheels() { setWheelR(0); setWheelL(0); }

// ==========================================================
//  JOINT MOTORS  (Layer 0)
// ==========================================================
void prepJoint(MI_Motor_ &m, uint8_t id) {
  m.Motor_Con_Init(id); delay(100);
  m.Motor_Reset();      delay(200);
  m.Change_Mode(SPEED_MODE); delay(200);
  m.Set_SpeedMode(0.0); delay(100);
}
void enableJoint(MI_Motor_ &m) {
  m.Change_Mode(SPEED_MODE); delay(50);
  m.Motor_Enable();          delay(50);
  m.Set_SpeedMode(0.0);
}
void disableJoint(MI_Motor_ &m) {
  m.Set_SpeedMode(0.0); delay(50);
  m.Motor_Reset();
}
void setJointSpeed(MI_Motor_ &m, float radPerSec) {
  m.Set_SpeedMode(radPerSec);
}
void pollJoint(MI_Motor_ &m) {
  m.Motor_Data_Updata(5);
}

// ==========================================================
//  SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  stopWheels();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc1.attachFullQuad(M1_A, M1_B);
  enc2.attachFullQuad(M2_A, M2_B);
  enc1.clearCount(); enc2.clearCount();

  Wire.begin(I2C_SDA, I2C_SCL);
  imuInit();

  Motor_CAN_Init();
  delay(100);
  prepJoint(joint1, MOTER_1_ID);
  prepJoint(joint2, MOTER_2_ID);

  Serial.println("Hold robot STILL — calibrating gyro...");
  calibrateGyro();
  initStateEstimate();
  Serial.printf("Gyro bias: %.3f %.3f %.3f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.println("Ready.");
}

// ==========================================================
//  MAIN CONTROL LOOP — fixed rate
// ==========================================================
void loop() {
  static unsigned long lastControlUs = 0;
  unsigned long nowUs = micros();

  if (nowUs - lastControlUs >= CONTROL_US) {
    float dt = (nowUs - lastControlUs) / 1000000.0;
    lastControlUs = nowUs;

    updateState(dt);

    // Control loop can go here

    pollJoint(joint1);
    pollJoint(joint2);
  }

  // slower housekeeping (telemetry, controller input) can go here,
  // gated by their own timers so they don't disturb the control loop
}