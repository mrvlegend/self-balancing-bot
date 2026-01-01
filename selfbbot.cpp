#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;

// ---------------- ENCODERS ----------------
#define encodPinAR 2
#define encodPinBR 3
#define encodPinAL 7
#define encodPinBL 8

volatile long wheel_pulse_count_left  = 0;
volatile long wheel_pulse_count_right = 0;
int prevA_right = LOW;
int prevA_left  = LOW;

// --------------- MOTORS (L298N) ---------------
const int motorIn1 = A2;   // Right motor IN1
const int motorIn2 = A3;   // Right motor IN2  
const int motorIn3 = 4;    // Left  motor IN3
const int motorIn4 = 9;    // Left  motor IN4
const int motorEN1 = 6;    // Right motor ENA (PWM)
const int motorEN2 = 5;    // Left  motor ENB (PWM)

// --------------- PID ----------------
double setPoint   = 0.0;   // target angle (upright)
double inputAngle = 0.0;   // measured angle
double outputPID  = 0.0;   // PID output → motor command

double Kp =50;
double Ki =450;
double Kd =1.3;


PID pid(&inputAngle, &outputPID, &setPoint, Kp, Ki, Kd, DIRECT);

// --------------- COMPLEMENTARY FILTER ---------------
unsigned long lastTime;
float angle   = 0.0;
float gyroRate;
float dt;
const float alpha = 0.98;  // standard for MPU6050 complementary filter[web:6]

// small deadband around 0° to avoid jitter
const float ANGLE_DEADBAND = 0;

// --------------- AUTO CALIBRATION ---------------
bool   calibrated  = false;
double angleOffset = 0.0;
double calibSum    = 0.0;
int    calibCount  = 0;
unsigned long calibStartMs;

// --------------- ENCODER UPDATE ----------------
void updateEncoders() {
  int aR = digitalRead(encodPinAR);
  if (prevA_right == LOW && aR == HIGH) {
    if (digitalRead(encodPinBR) == HIGH) wheel_pulse_count_right++;
    else                                wheel_pulse_count_right--;
  }
  prevA_right = aR;

  int aL = digitalRead(encodPinAL);
  if (prevA_left == LOW && aL == HIGH) {
    if (digitalRead(encodPinBL) == HIGH) wheel_pulse_count_left++;
    else                                wheel_pulse_count_left--;
  }
  prevA_left = aL;
}

// --------------- MOTOR DRIVE ----------------
void driveMotors(double cmd) {
  int speed = constrain((int)abs(cmd), 0, 110);   // enough torque but not full 255

  // deadzone for small commands
  if (abs(cmd) < 8) {
    analogWrite(motorEN1, 0);
    analogWrite(motorEN2, 0);
    return;
  }

  if (cmd > 0) {
    // positive PID → move forward
    analogWrite(motorEN1, speed);
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);

    analogWrite(motorEN2, speed);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
  } else {
    // negative PID → move backward
    analogWrite(motorEN1, speed);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);

    analogWrite(motorEN2, speed);
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);
  }
}

// --------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Encoder pins
  pinMode(encodPinAR, INPUT_PULLUP);
  pinMode(encodPinBR, INPUT_PULLUP);
  pinMode(encodPinAL, INPUT_PULLUP);
  pinMode(encodPinBL, INPUT_PULLUP);
  prevA_right = digitalRead(encodPinAR);
  prevA_left  = digitalRead(encodPinAL);

  // Motor pins
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(motorEN1, OUTPUT);
  pinMode(motorEN2, OUTPUT);

  // PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-90, 90);   // ±PWM
  pid.SetSampleTime(5);
  pid.SetTunings(Kp, Ki, Kd);

  lastTime = micros();

  // start auto‑calibration
  calibStartMs = millis();
  calibrated   = false;
  calibSum     = 0.0;
  calibCount   = 0;

  Serial.println("=== SELF‑BALANCING N20 BOT (23 cm, 440 g) ===");
  Serial.println("Hold robot PERFECTLY upright for 3 seconds...");
  delay(1000);
}

// MAIN LOOP 
void loop() {
  // 1. Read IMU
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // 2. Complementary filter: pitch around X (forward/backward)
  gyroRate = gx / 131.0;                          // deg/s, X‑gyro
  float accelAngle = atan2(ay, az) * 180.0 / PI;  // pitch from Y/Z

  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle;

  // raw angle: define forward tilt as positive
  double rawAngle = -angle;

  // 3. Auto‑calibration for first 3 seconds
  if (!calibrated) {
    if (millis() - calibStartMs < 3000) {
      calibSum += rawAngle;
      calibCount++;
    } else {
      angleOffset = calibSum / calibCount;
      calibrated  = true;
      Serial.print("Calibration done. angleOffset = ");
      Serial.println(angleOffset, 3);
    }
  }

  // apply offset once calibrated (if not, offset≈0)
  inputAngle = rawAngle - angleOffset;

  // 4. PID + motors
  if (abs(inputAngle - setPoint) < ANGLE_DEADBAND) {
    outputPID = 0;
    driveMotors(0);
  } else {
    updateEncoders();
    pid.Compute();
    driveMotors(outputPID);
  }

  // 5. Debug print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.print("Angle: ");
    Serial.print(inputAngle, 2);
    Serial.print("  PID: ");
    Serial.print(outputPID, 1);
    Serial.print("  L:");
    Serial.print(wheel_pulse_count_left);
    Serial.print("  R:");
    Serial.println(wheel_pulse_count_right);
    lastPrint = millis();
  }

  delay(3);
}
