## Self-Balancing Robot (N20 Bot)
Initial Arduino Nano code for a 23cm tall, 440g self-balancing robot using MPU6050 IMU, quadrature encoders, L298N motor driver, and PID control. Features complementary filter for angle estimation and 3-second auto-calibration.
https://vlegendbk-png.github.io/mech1/
# Features
Complementary filter (α=0.98) for robust pitch angle estimation
​

PID controller (Kp=50, Ki=450, Kd=1.3) tuned for N20 motors

Quadrature encoders on both wheels for position tracking

Auto-calibration during first 3 seconds (hold upright!)

Deadband filtering and motor deadzone to eliminate jitter


# Real-time serial debug (angle, PID output, encoder counts)
Platform: Arduino Nano
IMU:     MPU6050 (I²C)
Motors:  N20 gearmotors (left/right differential drive)
Driver:  L298N H-bridge
Height:  23 cm
Weight:  440 g
Encoders: Quadrature (pins 2,3,7,8)
_________________________________________
| Component     | Pin       | Nano Pin  |
| ------------- | --------- | --------- |
| MPU6050       | SDA       | A4        |
|               | SCL       | A5        |
| Right Encoder | A         | D2 (INT0) |
|               | B         | D3 (INT1) |
| Left Encoder  | A         | D7        |
|               | B         | D8        |
| L298N Right   | IN1       | A2        |
|               | IN2       | A3        |
|               | ENA (PWM) | D6        |
| L298N Left    | IN3       | D4        |
|               | IN4       | D9        |
|               | ENB (PWM) | D5        |
_________________________________________

MPU6050 → Nano: VCC=3.3V/5V, GND, SDA→A4, SCL→A5
Encoders → Nano: VCC=5V, GND, A/B channels as above
L298N → Nano: Input pins as above, motors to OUT1-4
Power: 7.4V LiPo → L298N VCC/GND (separate logic 5V)

# Required Libraries
Install via Arduino Library Manager:
MPU6050 by Electronic Cats or jrowberg/i2cdevlib
​PID_v1 by Brett Beauregard
Wire (built-in)

# Installation

# Arduino IDE: Sketch → Include Library → Manage Libraries
# Search/install: "MPU6050", "PID_v1"


# Tuning Parameters

// PID (start here for stability)
double Kp = 50;   // Proportional (response speed) [30-70]
double Ki = 450;  // Integral (steady-state error) [300-600] 
double Kd = 1.3;  // Derivative (damping) [0.8-2.0]

// Complementary filter
const float alpha = 0.98;  // Gyro trust (0.96-0.99)

// Motor limits
constrain((int)abs(cmd), 0, 110);  // Max PWM=110 for N20
_______________________________________________________________________
| Issue         | Cause              | Fix                             |
| ------------- | ------------------ | ------------------------------- |
| Won't balance | Poor calibration   | Hold straighter during first 3s |
| Oscillates    | PID too aggressive | Lower Kp/Ki, increase Kd        |
| Jittery       | Encoder noise      | Check wiring, increase deadband |
| No MPU        | Connection fail    | Verify I²C (A4/A5), 3.3V power  |
| Weak torque   | PWM too low        | Increase max speed to 130       |
________________________________________________________________________



