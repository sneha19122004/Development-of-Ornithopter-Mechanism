/**
 * ============================================================
 *  INTELLIGENT GIMBAL CONTROLLER — ESP32 WROOM
 *  PlatformIO / Arduino Framework
 *
 *  Architecture:
 *    RadioMaster Pocket → MSP over UART2 → ESP32 → Pitch Servo + Roll Servo
 *
 *  Channel assignment:
 *    CH1  — Roll     → UNUSED (fully automatic, derived from pitch)
 *    CH2  — Pitch    → UNUSED (fully automatic, derived from throttle)
 *    CH3  — Throttle → drives BOTH servos automatically
 *    CH4  — Yaw      → spare / expansion
 *
 *  Throttle → Pitch mapping (automatic):
 *    Throttle idle  (1000 µs / 0%)   → pitch servo at HOME (90°) — neutral/cruise
 *    Throttle full  (2000 µs / 100%) → pitch servo at PITCH_MAX (120°) — climb attitude
 *    Mapping is linear across the full throttle range.
 *
 *  Throttle → Roll mapping (automatic):
 *    Roll servo is NOT driven by the stick.
 *    Instead, roll automatically tracks the ACTUAL pitch servo position.
 *    As pitch rises from HOME toward PITCH_MAX, roll is biased toward
 *    ROLL_BIAS_AT_MAX_PITCH to counteract aerodynamic torque at high power.
 *    At HOME pitch, roll servo sits at HOME (90°) — no correction needed.
 *
 *  Dynamic roll guard (safety):
 *    Maximum ±roll authority shrinks as pitch moves away from HOME,
 *    preventing mechanism binding against the fuselage.
 *    Since roll is automatic here, the guard clamps the bias output.
 *
 *  Slew-rate limiters:
 *    Both axes are rate-limited (deg/ms) to protect gears from torque spikes.
 *
 *  Failsafe watchdog:
 *    If no valid MSP_RC packet arrives within MSP_TIMEOUT_MS, both servos
 *    are driven (via the slew limiter) back to HOME for best-glide attitude.
 * ============================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ─── PIN ASSIGNMENT ──────────────────────────────────────────
#define PIN_PITCH_SERVO   18   // Base / pitch servo PWM out
#define PIN_ROLL_SERVO    19   // Nested / roll servo PWM out
#define PIN_MSP_RX        16   // UART2 RX ← RadioMaster CRSF/MSP TX
#define PIN_MSP_TX        17   // UART2 TX → RadioMaster (optional telemetry)

// ─── SERVO GEOMETRY ──────────────────────────────────────────
#define SERVO_HOME          90
#define PITCH_MIN           60
#define PITCH_MAX          120
#define ROLL_BIAS_AT_MAX_PITCH   5.0f
#define ROLL_MAX_FULL       30
#define ROLL_MAX_LIMITED     8

// ─── THROTTLE MAPPING ────────────────────────────────────────
#define THR_MIN_US        1000
#define THR_RAMP_LO_US    1300
#define THR_RAMP_HI_US    1400
#define THR_MAX_US        2000

// ─── SLEW-RATE LIMITS ────────────────────────────────────────
#define SLEW_PITCH_DEG_PER_MS   0.12f
#define SLEW_ROLL_DEG_PER_MS    0.20f

// ─── FAILSAFE ────────────────────────────────────────────────
#define MSP_TIMEOUT_MS    500

// ─── MSP PROTOCOL ────────────────────────────────────────────
#define MSP_PREAMBLE_1   0x24
#define MSP_PREAMBLE_2   0x4D
#define MSP_RC           200

Servo pitchServo;
Servo rollServo;

struct RCChannels {
  uint16_t roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4;
};

RCChannels rc = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};

float currentPitchDeg = SERVO_HOME;
float currentRollDeg  = SERVO_HOME;
float targetPitchDeg  = SERVO_HOME;
float targetRollDeg   = SERVO_HOME;

uint32_t lastMspPacketMs = 0;
bool     failsafeActive  = false;

enum MspState {
  MSP_IDLE, MSP_PREAMBLE2, MSP_DIRECTION,
  MSP_SIZE, MSP_CMD, MSP_PAYLOAD, MSP_CRC
};

MspState mspState   = MSP_IDLE;
uint8_t  mspSize    = 0;
uint8_t  mspCmd     = 0;
uint8_t  mspCrc     = 0;
uint8_t  mspBuf[64];
uint8_t  mspBufIdx  = 0;

// ─── MATH HELPERS ────────────────────────────────────────────

inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline float slewToward(float current, float target, float maxDelta) {
  float diff = target - current;
  if (diff >  maxDelta) return current + maxDelta;
  if (diff < -maxDelta) return current - maxDelta;
  return target;
}

/**
 * throttleToPitchDeg — Three-zone soft ramp
 *
 *   Zone 1 (0–30%)  : HOME (90°) — stationary, no drag
 *   Zone 2 (30–40%) : Linear ramp 90° → 120°
 *   Zone 3 (40–100%): PITCH_MAX (120°) — held at climb trim
 *
 * Both zone boundaries are zero-discontinuity by design.
 * The slew limiter prevents servo snapping even at instant stick inputs.
 */
float throttleToPitchDeg(uint16_t thrUs) {
  if (thrUs < THR_RAMP_LO_US) return (float)SERVO_HOME;
  if (thrUs > THR_RAMP_HI_US) return (float)PITCH_MAX;

  float t = clampf(
    (float)(thrUs - THR_RAMP_LO_US) / (float)(THR_RAMP_HI_US - THR_RAMP_LO_US),
    0.0f, 1.0f
  );
  return (float)SERVO_HOME + t * (float)(PITCH_MAX - SERVO_HOME);
}

/**
 * dynamicRollMax — Safety guard
 *
 * Reduces max ±roll authority as pitch extends, preventing
 * the servo from driving the tail mechanism into a physical bind.
 *
 *   pitch at HOME →  ±ROLL_MAX_FULL (30°)
 *   pitch at MAX  →  ±ROLL_MAX_LIMITED (8°)
 */
float dynamicRollMax(float actualPitchDeg) {
  float offset   = fabsf(actualPitchDeg - (float)SERVO_HOME);
  float fraction = clampf(offset / (float)(PITCH_MAX - SERVO_HOME), 0.0f, 1.0f);
  return (float)ROLL_MAX_FULL - fraction * (float)(ROLL_MAX_FULL - ROLL_MAX_LIMITED);
}

/**
 * pitchToRollBiasDeg — Automatic torque compensation
 *
 * Generates a small roll correction proportional to pitch deflection,
 * countering propulsive torque coupling at high power settings.
 *
 *   pitch at HOME (90°) →  0° bias
 *   pitch at MAX  (120°) →  ROLL_BIAS_AT_MAX_PITCH degrees
 */
float pitchToRollBiasDeg(float actualPitchDeg) {
  float fraction = clampf(
    (actualPitchDeg - (float)SERVO_HOME) / (float)(PITCH_MAX - SERVO_HOME),
    0.0f, 1.0f
  );
  float bias = fraction * ROLL_BIAS_AT_MAX_PITCH;
  return clampf(bias, -dynamicRollMax(actualPitchDeg), dynamicRollMax(actualPitchDeg));
}

// ─── MSP PARSER ──────────────────────────────────────────────

void mspFeedByte(uint8_t b) {
  switch (mspState) {
    case MSP_IDLE:
      if (b == MSP_PREAMBLE_1) mspState = MSP_PREAMBLE2;
      break;
    case MSP_PREAMBLE2:
      mspState = (b == MSP_PREAMBLE_2) ? MSP_DIRECTION : MSP_IDLE;
      break;
    case MSP_DIRECTION:
      mspState = MSP_SIZE; mspCrc = 0;
      break;
    case MSP_SIZE:
      mspSize = b; mspCrc ^= b; mspBufIdx = 0; mspState = MSP_CMD;
      break;
    case MSP_CMD:
      mspCmd = b; mspCrc ^= b;
      mspState = (mspSize > 0) ? MSP_PAYLOAD : MSP_CRC;
      break;
    case MSP_PAYLOAD:
      if (mspBufIdx < sizeof(mspBuf)) mspBuf[mspBufIdx++] = b;
      mspCrc ^= b;
      if (mspBufIdx >= mspSize) mspState = MSP_CRC;
      break;
    case MSP_CRC:
      if (b == mspCrc && mspCmd == MSP_RC && mspSize >= 16) {
        rc.roll     = mspBuf[0]  | (mspBuf[1]  << 8);
        rc.pitch    = mspBuf[2]  | (mspBuf[3]  << 8);
        rc.throttle = mspBuf[4]  | (mspBuf[5]  << 8);
        rc.yaw      = mspBuf[6]  | (mspBuf[7]  << 8);
        rc.aux1     = mspBuf[8]  | (mspBuf[9]  << 8);
        rc.aux2     = mspBuf[10] | (mspBuf[11] << 8);
        rc.aux3     = mspBuf[12] | (mspBuf[13] << 8);
        rc.aux4     = mspBuf[14] | (mspBuf[15] << 8);
        lastMspPacketMs = millis();
        failsafeActive  = false;
      }
      mspState = MSP_IDLE;
      break;
  }
}

// ─── SETUP ───────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n[GIMBAL] Intelligent Gimbal Controller — Ornithopter"));
  Serial.printf("[GIMBAL] Pitch ramp: 0-30%% HOME=%d° | 30-40%% ramp | 40-100%% MAX=%d°\n",
                SERVO_HOME, PITCH_MAX);
  Serial.printf("[GIMBAL] Roll bias: 0° at HOME → %.1f° at PITCH_MAX\n",
                ROLL_BIAS_AT_MAX_PITCH);

  Serial2.begin(115200, SERIAL_8N1, PIN_MSP_RX, PIN_MSP_TX);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pitchServo.setPeriodHertz(50);
  rollServo.setPeriodHertz(50);
  pitchServo.attach(PIN_PITCH_SERVO, 500, 2400);
  rollServo.attach(PIN_ROLL_SERVO,   500, 2400);

  pitchServo.write(SERVO_HOME);
  rollServo.write(SERVO_HOME);

  currentPitchDeg = targetPitchDeg = (float)SERVO_HOME;
  currentRollDeg  = targetRollDeg  = (float)SERVO_HOME;
  lastMspPacketMs = millis();

  Serial.println(F("[GIMBAL] Ready — listening for MSP on UART2 (GPIO16)"));
}

// ─── MAIN LOOP ───────────────────────────────────────────────

void loop() {
  uint32_t now = millis();
  static uint32_t lastLoopMs  = 0;
  static uint32_t lastDebugMs = 0;

  // 1. Drain MSP UART
  while (Serial2.available()) mspFeedByte((uint8_t)Serial2.read());

  // 2. Failsafe watchdog
  if ((now - lastMspPacketMs) > MSP_TIMEOUT_MS) {
    if (!failsafeActive) {
      failsafeActive = true;
      Serial.println(F("[GIMBAL] * FAILSAFE — MSP link lost, returning to HOME *"));
    }
    targetPitchDeg = (float)SERVO_HOME;
    targetRollDeg  = (float)SERVO_HOME;
  }

  // 3. Compute targets (throttle only — fully automatic)
  if (!failsafeActive) {
    targetPitchDeg = throttleToPitchDeg(rc.throttle);
    targetRollDeg  = (float)SERVO_HOME + pitchToRollBiasDeg(currentPitchDeg);
  }

  // 4. Slew-rate limiter (1 ms resolution)
  uint32_t dt = now - lastLoopMs;
  if (dt >= 1) {
    lastLoopMs = now;

    currentPitchDeg = slewToward(currentPitchDeg, targetPitchDeg,
                                 SLEW_PITCH_DEG_PER_MS * (float)dt);
    currentRollDeg  = slewToward(currentRollDeg,  targetRollDeg,
                                 SLEW_ROLL_DEG_PER_MS  * (float)dt);

    // Hard clamps — last defence before hardware write
    currentPitchDeg = clampf(currentPitchDeg, (float)PITCH_MIN, (float)PITCH_MAX);
    currentRollDeg  = clampf(currentRollDeg,
                             (float)SERVO_HOME - ROLL_MAX_FULL,
                             (float)SERVO_HOME + ROLL_MAX_FULL);

    pitchServo.write((int)roundf(currentPitchDeg));
    rollServo.write ((int)roundf(currentRollDeg));
  }

  // 5. Serial telemetry (200 ms cadence)
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    Serial.printf(
      "[GIMBAL] FS=%-3s | THR=%4d | P_tgt=%5.1f P_cur=%5.1f | "
      "R_bias=%+5.2f R_tgt=%5.1f R_cur=%5.1f | rollMax=%4.1f\n",
      failsafeActive ? "YES" : "NO",
      rc.throttle,
      targetPitchDeg, currentPitchDeg,
      pitchToRollBiasDeg(currentPitchDeg),
      targetRollDeg, currentRollDeg,
      dynamicRollMax(currentPitchDeg)
    );
  }
}
