# Maze-Solving Robot (MPU6050 + PID Control)

An autonomous maze-solving robot built with Arduino, utilizing three ultrasonic sensors for spatial awareness and an MPU6050 IMU for precise directional control. The robot employs a wall-following algorithm combined with a dual-PID system to maintain stability and accuracy that used to compete in insat robolympix 4.0 in the autonomous challenge.

## Code explanation

```
#include <Wire.h>
#include <MPU6050_tockn.h>
```
Libraries needed for the MPU6050 to work

```
 MPU6050 mpu6050(Wire);
```
Initialazing the mpu6050

```
int in1 = 6;   // Left motor PWM forward
int in2 = 3;   // Left motor PWM backward
int in3 = 5;   // Right motor PWM forward
int in4 = 9;   // Right motor PWM backward

long t;
int led =11;
int distw = 24;

int echo1 = 13; int trig1 = 12;  // Front sensor
int echo2 = 10;  int trig2 = 4;   // Left sensor
int echo3 = 8; int trig3 = 7;  // Right sensor

float Kp = 15;
float Ki = 0.05;
float Kd = 0.1;  

float tangle = 0.0; 

float integral  = 0;
float derivative = 0;
float preverror  = 0;

int basespeed = 180;
int maxspeed  = 255;
int minspeed  = 100;

float lastfront;
float lastleft;
float lastright;

int sensorMax = 3*40;
int sensorMin = 2;
const int READINGS = 4;

float p_turn = 40.0; 
float d_turn = 3;
float last_turn_error = 0;

// Speed Settings
int turn_base_speed = 30; 
int max_pwm = 70;

// ─── Maze Solver Settings ─────────────────────────────────
#define WALL_THRESHOLD  18    // cm — treat anything closer as a wall
#define TURN_SPEED      170   // motor speed during gyro turns
#define FORWARD_CHECK   12    // cm — front wall triggers a turn decision
#define CELL_DRIVE_MS   300   // ms to drive into new cell after a turn (tune this)
```
```
long readSensor(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return sensorMax;
  long dist = duration * 0.034 / 2;
  return constrain(dist, sensorMin, sensorMax);
}
```
Reading ultasonic sensor value

```
float sfront() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) sum += readSensor(trig1, echo1);
  return sum / READINGS;
}

float sleft() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) sum += readSensor(trig2, echo2);
  return sum / READINGS;
}

float sright() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) sum += readSensor(trig3, echo3);
  return sum / READINGS;
}
```
Reading Average values for each sensor

```
void stopMotors() {
  analogWrite(in1, 0); analogWrite(in2, 0);
  analogWrite(in3, 0); analogWrite(in4, 0);
}

void driveForward(int spd) {
  analogWrite(in1, spd); analogWrite(in2, 0);
  analogWrite(in3, spd); analogWrite(in4, 0);
}

void setMotors(int l, int r) {
  analogWrite(in1, constrain(l, 0, 255)); analogWrite(in2, 0);
  analogWrite(in3, constrain(r, 0, 255)); analogWrite(in4, 0);
}
```
Functions to control motors

```
void pidStep() {
  mpu6050.update();
  float error = sleft() - sright();
  integral += error;
  integral = constrain(integral, -100, 100);
  derivative = error - preverror;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  int l = constrain(basespeed + correction, minspeed, maxspeed);
  int r = constrain(basespeed - correction, minspeed, maxspeed);
  setMotors(l, r);
  preverror = error;
}
```
The PID

```
void pivotToAngle(float target_angle) {
  t=millis();
  while (true) {
    mpu6050.update();
    float current_yaw = mpu6050.getAngleZ();
    float turn_error = target_angle - current_yaw;

    // 1. Exit condition (Within 1 degree)
    if (abs(turn_error) < 5) {
      stopMotors();
      break;
    }
    if(millis()-t>2500){
      break;
    }


    float turn_diff = turn_error - last_turn_error;
    float turn_correction = (p_turn * turn_error) + (d_turn * turn_diff);
    last_turn_error = turn_error;
    int motor_pwm = constrain(turn_base_speed + abs(turn_correction), 0, max_pwm);
    if (turn_error > 0) {
      // TURN LEFT: Left back, Right forward
      analogWrite(in1, motor_pwm);         analogWrite(in2, 0); // Left motor back
      analogWrite(in3, 0); analogWrite(in4, motor_pwm);         // Right motor forward
    } else {
      // TURN RIGHT: Left forward, Right back
      analogWrite(in1,0 ); analogWrite(in2, motor_pwm);         // Left motor forward
      analogWrite(in3, motor_pwm);         analogWrite(in4, 0); // Right motor back
    }
  }
  tangle=target_angle;
  integral=0;preverror=0;
}
```
Pivots to a specefic angle using PID to be precise while turning

```
void uTurn() {
  pivotToAngle(78);
  delay(100);
  pivotToAngle(78);
}

```
U turn

```
void setup() {
  Wire.begin();
  mpu6050.begin();

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT); pinMode(echo3, INPUT);
  pinMode(led,OUTPUT);
  pinMode(startButton , INPUT);



  Serial.begin(9600);
  
  mpu6050.calcGyroOffsets(true);

    while (digitalRead(startButton) == LOW) {
    digitalWrite(led, (millis() / 500) % 2); 
    delay(50); 
  } 

  delay(50); 
}
```
The setup

```
void loop() {

  pidStep();
  if(right()){
    
    digitalWrite(led,100);
    driveForward(basespeed);
    delay(50); 
    stopMotors();
    delay(150);
    pivotToAngle(tangle-79);
    digitalWrite(led,0);
    stopMotors();
    delay(150);
    t=millis();
    while(millis()-t<650){
      pidStep();
    }
    }
    else if(left() && !forward() && !right()){
        digitalWrite(led,100);
         driveForward(basespeed);
        delay(50);
        stopMotors();
        delay(150);
        pivotToAngle(tangle+79);
        digitalWrite(led,0);
        stopMotors();
        delay(100);
        t=millis();
        while(millis()-t<650){
          pidStep();
        }
    }

    else if(!left() && !forward() && !right()){
      digitalWrite(led,100);
      pivotToAngle(tangle-165);
      digitalWrite(led,0);
      delay(50);
    }
```
The algorithm


## 🛠 Navigation Logic: The Algorithm

The robot implements the **Right-Hand Rulex (Wall Follower). This is an "uninformed" maze-solving algorithm where the robot treats the maze walls as a single continuous surface, keeping the right wall at a constant relative distance.

### Decision Priority:
1. **Turn Right:** If the right sensor detects an opening (`dist > 24cm`), the robot moves forward slightly to clear the corner, then executes a $-79^\circ$ pivot.
2. **Drive Forward:** If a right turn isn't possible but the front path is clear, the robot continues forward using PID stabilization.
3. **Turn Left:** If both right and front paths are blocked but the left is open, the robot executes a $+79^\circ$ pivot.
4. **U-Turn:** If the robot hits a dead end (all three sides blocked), it performs a $\approx 165^\circ$ pivot to backtrack.

---

## 🧠 PID Control Implementation

The project uses two distinct PID controllers to handle different movement states.

### 1. Straight-Line "Walking" PID
When moving forward, the robot uses a full PID loop to ensure it travels in a perfectly straight line, correcting for motor inconsistencies or physical bumps.

* **Error calculation:** $Error = TargetAngle - CurrentYaw$.
* **Proportional (Kp):** Corrects the heading based on the current deviation.
* **Integral (Ki):** Eliminates steady-state error (e.g., if one motor is inherently weaker than the other).
* **Derivative (Kd):** Dampens the correction to prevent the robot from oscillating (wobbling) back and forth.
* **Output:** The correction is added to the left motor speed and subtracted from the right, nudging the robot back to the $0^\circ$ reference.

### 2. Rotational "Pivot" PID
Precise $90^\circ$ turns are difficult with simple timers due to battery voltage fluctuations. The `pivotToAngle()` function uses a **PD (Proportional-Derivative) controller** to turn accurately.

* **Dynamic Speed:** As the robot approaches the target angle, the error decreases, naturally slowing the motors down. This prevents overshooting the turn.
* **Inertia Compensation:** The `d_turn` (Derivative) component acts as a "brake," helping the robot stop exactly at the desired angle despite the momentum of the chassis.
* **Targeting:** The robot updates its global `tangle` variable after every turn, ensuring the "Walking PID" always has a fresh, accurate reference for the next straight path.

---

## 🔌 Hardware Mapping

| Component | Pin | Function |
| :--- | :--- | :--- |
| **MPU6050** | I2C (SDA/SCL) | Gyroscopic Heading |
| **Ultrasonic Front** | Trig 12 / Echo 13 | Obstacle detection |
| **Ultrasonic Left** | Trig 4 / Echo 10 | Wall detection |
| **Ultrasonic Right** | Trig 7 / Echo 8 | Wall detection |
| **Left Motor** | PWM 6 (Fwd), 3 (Bwd) | Locomotion |
| **Right Motor** | PWM 5 (Fwd), 9 (Bwd) | Locomotion |
| **Start Button** | 2 | Calibration & Launch |
| **LED** | 11 | Status Indicator |

---

## 🚀 Setup and Usage

1.  **Calibration:** On power-up, the robot waits for the `startButton`. During this time, the MPU6050 is calibrated. The robot must remain perfectly still.
2.  **PID Tuning:** * Adjust `Kp`, `Ki`, and `Kd` to stabilize forward walking.
    * Adjust `p_turn` and `d_turn` to sharpen the $90^\circ$ and $180^\circ$ maneuvers.
    * Modify `distw` (24cm) to change the sensitivity for wall detection.
3.  **Operation:** Once the button is pressed, the LED indicates the robot's state (turning vs. walking) as it solves the maze.
