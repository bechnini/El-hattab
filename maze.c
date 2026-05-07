#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);
int startButton = 2;

int in1 = 6;   // Left motor PWM forward
int in2 = 3;   // Left motor PWM backward
int in3 = 5;   // Right motor PWM forward
int in4 = 9;   // Right motor PWM backward

long t;
int led =11;
int distw = 28;

int echo1 = 13; int trig1 = 12;  // Front sensor
int echo2 = 10;  int trig2 = 4;   // Left sensor
int echo3 = 8; int trig3 = 7;  // Right sensor

float Kp = 10;
float Ki = 0.05;
float Kd = 2;  

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
const int READINGS = 3;

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

long readSensor(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  //if (duration == 0) return sensorMin;
  long dist = duration * 0.034 / 2;
  return constrain(dist, sensorMin, sensorMax);
}

float sfront() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) {
    delay(10);
    int v=readSensor(trig1, echo1);
  if(v>7*40)sum +=lastfront;
  else if(v==0)sum+=7*40;
  else{
    lastfront=v;
      sum +=v;
  }}
  return constrain( sum / READINGS,sensorMin,sensorMax);
}

bool right(){
  return sright()>distw;
}
bool left(){
  return sleft()>distw;
}
bool forward(){
  return sfront()>distw;
}

float sleft() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) {
    delay(10);
    int v=readSensor(trig2, echo2);
  if(v>7*40 )sum +=lastleft;
  else if(v==0)sum+=7*40;
  else{
    lastleft=v;
      sum +=v;
  }}
  return constrain( sum / READINGS,sensorMin,sensorMax);
}

float sright() {
  float sum = 0;
  for (int i = 0; i < READINGS; i++) {
    delay(10);
    int v=readSensor(trig3, echo3);
  if(v>7*40 )sum +=lastright;
  else if(v==0)sum+=7*40;
  else{
    lastright=v;
      sum +=v;
  }}
  return constrain( sum / READINGS,sensorMin,sensorMax);
}

void stopMotors() {
  analogWrite(in1, 0); analogWrite(in2, 0);
  analogWrite(in3, 0); analogWrite(in4, 0);
}

void driveForward(int spd) {
  analogWrite(in1, spd); analogWrite(in2, 0);
  analogWrite(in3, spd); analogWrite(in4, 0);
}

void setMotors(int l, int r) {
  analogWrite(in1, constrain(l, 0, 120)); analogWrite(in2, 0);
  analogWrite(in3, constrain(r, 0, 120)); analogWrite(in4, 0);
}

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

void pidStep() {
  mpu6050.update();
  
  float currentAngle = mpu6050.getAngleZ();

  float error = tangle - currentAngle; 
  

  integral += error;
  integral = constrain(integral, -100, 100);
  derivative = error - preverror;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // 4. Apply correction to motors
  int l = basespeed + correction;
  int r = basespeed - correction;

  setMotors(l, r);

  preverror = error;
}

#define TARGET_DEG   87.0   
#define GYRO_TIMEOUT 2000   

void uTurn() {
  pivotToAngle(78);
  delay(100);
  pivotToAngle(78);
}

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
  
