#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>

// ---------- Wiring ----------
constexpr int STEP_PIN = 17;
constexpr int DIR_PIN  = 16;
constexpr int EN_PIN   = 15;
constexpr int SDA_PIN  = 8;
constexpr int SCL_PIN  = 9;

constexpr uint8_t AS5600_ADDRESS = 0x36;

// ---------- Encoder / reference ----------
float angleOffset = -155.13f;
float targetAngle = 0.0f;  // Calibrated upright reference

float currentAngle = 0.0f;
float currentError = 0.0f;
bool encoderOK = false;

// ---------- Your tuned gains ----------
float Kp = 175.0f;
float Ki = 10.0f;
float Kd = 0.0f;

// ---------- Control limits ----------
constexpr float MAX_ALLOWED_ERROR = 20.0f;
constexpr float MAX_VELOCITY = 2000.0f; // STEP pulses/second
constexpr float MIN_VELOCITY = 5.0f;

// Maximum integral contribution, in pulses/second.
constexpr float MAX_I_OUTPUT = 2000.0f;

constexpr uint32_t CONTROL_PERIOD_US = 10000; // 100 Hz
constexpr uint32_t MAX_CONTROL_GAP_US = 50000;
constexpr uint32_t PLOT_PERIOD_MS = 50;       // 20 Hz

// Change only if the previously working motor direction is reversed.
constexpr bool INVERT_DIRECTION = true;

// ---------- Runtime state ----------
bool pidEnabled = false;
bool plotMode = false;

float integralError = 0.0f;
float previousAngle = 0.0f;
float filteredAngularRate = 0.0f;
float targetVelocity = 0.0f;

// Derivative filtering time constant.
constexpr float DERIVATIVE_FILTER_SECONDS = 0.03f;

uint32_t lastControlUs = 0;
uint32_t lastStepUs = 0;
uint32_t lastPlotMs = 0;
uint32_t stepIntervalUs = 0;

int stepDirection = 1;

// Commanded steps only: NOT a measured motor position.
long armSteps = 0;

// Nonblocking serial input.
char commandBuffer[80];
size_t commandLength = 0;
bool discardLine = false;


// ---------- Utilities ----------
float wrapAngle(float angle) {
  while (angle <= -180.0f) angle += 360.0f;
  while (angle > 180.0f) angle -= 360.0f;
  return angle;
}

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void resetController() {
  integralError = 0.0f;
  filteredAngularRate = 0.0f;
  previousAngle = currentAngle;

  targetVelocity = 0.0f;
  stepIntervalUs = 0;

  digitalWrite(STEP_PIN, LOW);

  lastControlUs = micros();
  lastStepUs = lastControlUs;
}

void stopControl(bool holdMotor) {
  pidEnabled = false;
  resetController();
  digitalWrite(EN_PIN, holdMotor ? LOW : HIGH);
}

void trip(const char *reason) {
  stopControl(false);
  plotMode = false;

  Serial.print("TRIP: ");
  Serial.println(reason);
  Serial.println("Motor released. Correct the cause, then send p.");
}


// ---------- Encoder ----------
bool readEncoder(float &angle) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C); // RAW ANGLE high register

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received =
      Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);

  if (received != 2 || Wire.available() < 2) {
    while (Wire.available()) Wire.read();
    return false;
  }

  int highByte = Wire.read();
  int lowByte = Wire.read();

  int raw = ((highByte & 0x0F) << 8) | lowByte;

  angle = wrapAngle(raw * (360.0f / 4096.0f) - angleOffset);
  return true;
}


// ---------- Step generation ----------
void setVelocity(float velocity) {
  targetVelocity = clampFloat(
      velocity, -MAX_VELOCITY, MAX_VELOCITY);

  if (fabsf(targetVelocity) < MIN_VELOCITY) {
    targetVelocity = 0.0f;
    stepIntervalUs = 0;
    return;
  }

  int newDirection = targetVelocity > 0.0f ? 1 : -1;

  if (stepIntervalUs == 0 || newDirection != stepDirection) {
    stepDirection = newDirection;

    bool pinHigh = stepDirection > 0;
    if (INVERT_DIRECTION) pinHigh = !pinHigh;

    digitalWrite(DIR_PIN, pinHigh ? HIGH : LOW);

    // Start a fresh interval after setting DIR.
    lastStepUs = micros();
  }

  stepIntervalUs =
      (uint32_t)(1000000.0f / fabsf(targetVelocity));
}

void handleStepper() {
  if (!pidEnabled || stepIntervalUs == 0) return;

  uint32_t now = micros();

  if ((uint32_t)(now - lastStepUs) >= stepIntervalUs) {
    // Do not emit a burst of catch-up pulses after a delay.
    lastStepUs = now;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);

    armSteps += stepDirection;
  }
}


// ---------- PID ----------
void updateControl() {
  uint32_t now = micros();

  if ((uint32_t)(now - lastControlUs) < CONTROL_PERIOD_US) {
    return;
  }

  float measuredAngle;
  encoderOK = readEncoder(measuredAngle);

  uint32_t sampleTime = micros();
  uint32_t elapsedUs = sampleTime - lastControlUs;
  lastControlUs = sampleTime;

  if (!encoderOK) {
    if (pidEnabled) trip("Encoder communication failed");
    return;
  }

  currentAngle = measuredAngle;
  currentError = wrapAngle(targetAngle - currentAngle);

  if (!pidEnabled) {
    previousAngle = currentAngle;
    return;
  }

  if (elapsedUs > MAX_CONTROL_GAP_US) {
    trip("Control loop delayed more than 50 ms");
    return;
  }

  if (fabsf(currentError) > MAX_ALLOWED_ERROR) {
    trip("Pendulum error exceeded 10 degrees");
    return;
  }

  float dt = elapsedUs * 0.000001f;

  // Derivative on measurement avoids a derivative kick when
  // the target changes. Angle wrapping avoids +/-180 jumps.
  float angularRate =
      wrapAngle(currentAngle - previousAngle) / dt;
  previousAngle = currentAngle;

  float filterWeight =
      dt / (DERIVATIVE_FILTER_SECONDS + dt);

  filteredAngularRate +=
      filterWeight * (angularRate - filteredAngularRate);

  float pTerm = Kp * currentError;
  float dTerm = -Kd * filteredAngularRate;

  // Conditional integration with an integral contribution limit.
  if (Ki > 0.0f) {
    float integralLimit = MAX_I_OUTPUT / Ki;

    float candidateIntegral = clampFloat(
        integralError + currentError * dt,
        -integralLimit,
        integralLimit);

    float candidateOutput =
        pTerm + Ki * candidateIntegral + dTerm;

    bool pushesFurtherIntoSaturation =
        (candidateOutput > MAX_VELOCITY &&
         currentError > 0.0f) ||
        (candidateOutput < -MAX_VELOCITY &&
         currentError < 0.0f);

    if (!pushesFurtherIntoSaturation) {
      integralError = candidateIntegral;
    }
  } else {
    integralError = 0.0f;
  }

  float output =
      pTerm + Ki * integralError + dTerm;

  if (!isfinite(output)) {
    trip("Invalid controller output");
    return;
  }

  setVelocity(output);
}


// ---------- Display ----------
void printHelp() {
  Serial.println("\n=== Rotary Pendulum Balance Test ===");
  Serial.println("Use Serial Monitor at 115200, Newline enabled.");
  Serial.println("p : Toggle balancing");
  Serial.println("x : Stop and release motor immediately");
  Serial.println("e : Stop balancing, hold motor");
  Serial.println("z : Set current pendulum angle as upright; PID OFF");
  Serial.println("r : Zero commanded arm-step count; PID OFF");
  Serial.println("a : Show angle, error, velocity and arm steps");
  Serial.println("v : Toggle plot output");
  Serial.println("t 0.0 : Set reference angle; PID OFF");
  Serial.println("k 230 100 0 : Set gains; PID OFF");
  Serial.println("h : Help; PID OFF");
  Serial.println("Start: hold pendulum upright, send z, then p.");
  Serial.println("Calibration and gains are not saved across reset.");
}

void printStatus() {
  Serial.printf(
      "Encoder:%s PID:%s Angle:%.3f Target:%.3f "
      "Error:%.3f Velocity:%.1f Steps:%ld I:%.1f\n",
      encoderOK ? "OK" : "FAIL",
      pidEnabled ? "ON" : "OFF",
      currentAngle, targetAngle, currentError,
      targetVelocity, armSteps, Ki * integralError);
}

void handlePlotter() {
  if (!plotMode) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastPlotMs) < PLOT_PERIOD_MS) return;
  lastPlotMs = now;

  char output[96];

  int length = snprintf(
      output, sizeof(output),
      "Target:%.3f\tAngle:%.3f\tError:%.3f\tPID:%d\tSensor:%d\n",
      targetAngle, currentAngle, currentError,
      pidEnabled ? 1 : 0, encoderOK ? 1 : 0);

  // Skip a plot sample if the transmit buffer is full.
  if (length > 0 &&
      length < (int)sizeof(output) &&
      Serial.availableForWrite() >= length) {
    Serial.write((const uint8_t *)output, length);
  }
}


// ---------- Commands ----------
void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;

  char command = tolower((unsigned char)*line);

  switch (command) {
    case 'p': {
      if (pidEnabled) {
        stopControl(false);
        if (!plotMode) Serial.println("PID OFF; motor released.");
        break;
      }

      float angle;
      encoderOK = readEncoder(angle);

      if (!encoderOK) {
        trip("Cannot enable: encoder unavailable");
        break;
      }

      currentAngle = angle;
      currentError = wrapAngle(targetAngle - currentAngle);

      if (fabsf(currentError) > MAX_ALLOWED_ERROR) {
        trip("Cannot enable: hold pendulum within 10 degrees");
        break;
      }

      resetController();
      digitalWrite(EN_PIN, LOW);
      pidEnabled = true;

      if (!plotMode) Serial.println("PID ON.");
      break;
    }

    case 'x':
      stopControl(false);
      if (!plotMode) Serial.println("Stopped; motor released.");
      break;

    case 'e':
      stopControl(true);
      if (!plotMode) Serial.println("PID OFF; motor holding.");
      break;

    case 'v':
      plotMode = !plotMode;
      if (!plotMode) Serial.println("Plot OFF.");
      break;

    case 'a':
      printStatus();
      break;

    case 'z': {
      if (pidEnabled) {
        Serial.println("Send x or e before calibration.");
        break;
      }

      float angle;
      encoderOK = readEncoder(angle);

      if (!encoderOK) {
        Serial.println("Calibration failed: encoder unavailable.");
        break;
      }

      angleOffset = wrapAngle(angleOffset + angle);
      currentAngle = 0.0f;
      targetAngle = 0.0f;
      currentError = 0.0f;
      resetController();

      Serial.printf("Upright zero set. Offset = %.4f\n", angleOffset);
      break;
    }

    case 'r':
      if (pidEnabled) {
        Serial.println("Stop PID before zeroing arm steps.");
      } else {
        armSteps = 0;
        Serial.println("Commanded arm steps zeroed.");
      }
      break;

    case 't': {
      if (pidEnabled) {
        Serial.println("Stop PID before changing reference.");
        break;
      }

      float value;
      char extra;

      if (sscanf(line + 1, "%f %c", &value, &extra) != 1 ||
          !isfinite(value) || fabsf(value) > 5.0f) {
        Serial.println("Use t value, between -5 and +5 degrees.");
        break;
      }

      targetAngle = value;
      currentError = wrapAngle(targetAngle - currentAngle);
      resetController();

      Serial.printf("Reference = %.3f degrees\n", targetAngle);
      break;
    }

    case 'k': {
      if (pidEnabled) {
        Serial.println("Stop PID before changing gains.");
        break;
      }

      float p, i, d;
      char extra;

      if (sscanf(line + 1, "%f %f %f %c",
                 &p, &i, &d, &extra) != 3 ||
          !isfinite(p) || !isfinite(i) || !isfinite(d) ||
          p < 0.0f || i < 0.0f || d < 0.0f) {
        Serial.println("Use k Kp Ki Kd with nonnegative values.");
        break;
      }

      Kp = p;
      Ki = i;
      Kd = d;
      resetController();

      Serial.printf("Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
      break;
    }

    case 'h':
      if (pidEnabled) {
        Serial.println("Stop PID before printing full help.");
      } else {
        printHelp();
      }
      break;

    default:
      Serial.println("Unknown command.");
      break;
  }
}

void handleSerial() {
  // Bounded work per loop; no parseFloat() waiting.
  for (int n = 0; n < 16 && Serial.available(); ++n) {
    char c = Serial.read();

    // Emergency release works even without a newline.
    if (c == 'x' || c == 'X') {
      stopControl(false);
      commandLength = 0;
      discardLine = false;
      if (!plotMode) Serial.println("Stopped; motor released.");
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (!discardLine && commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
      }

      commandLength = 0;
      discardLine = false;
      continue;
    }

    if (discardLine) continue;

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      discardLine = true;
    }
  }
}


// ---------- Setup / main loop ----------
void setup() {
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);

  Serial.begin(115200);

  bool busStarted = Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setTimeOut(5);

  delay(500);

  if (!busStarted) {
    Serial.println("I2C initialization failed.");
  }

  encoderOK = readEncoder(currentAngle);
  currentError = wrapAngle(targetAngle - currentAngle);

  resetController();
  printHelp();
  printStatus();
}

void loop() {
  handleSerial();
  handleStepper();

  updateControl();
  handleStepper();

  handlePlotter();
  handleStepper();
}