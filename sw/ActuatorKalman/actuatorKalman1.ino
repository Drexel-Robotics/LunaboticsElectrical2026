#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define GENERAL_DEBUG 1

#define SKIP_AUTO_HOMING 1
#define HOMING_DELAY_MS 30*1000

const int m2_hall = 20;
const int m2_pwm = 10;
const int m2_dir= 16;
const int m2_tflag = 25;

const int m1_hall = 19;
const int m1_pwm = 9;
const int m1_dir= 15;
const int m1_tflag = 24;

volatile long steps2 = 0;        // Pulses from  Hall Effect sensors
int trigDelay2 = 1500;            // Delay bewteen pulse in microseconds
unsigned long lastStepTime2 = 0; // Time stamp of last pulse

volatile long steps1 = 0;        // Pulses from  Hall Effect sensors
int trigDelay1 = 1500;            // Delay bewteen pulse in microseconds
unsigned long lastStepTime1 = 0; // Time stamp of last pulse

char* tokens[3] = {};

#define DELAY_MS 10000

int dir1=0;
int dir2=0;

float x1 = 0;   // filtered position actuator 1
float P1 = 1;

float x2 = 0;   // filtered position actuator 2
float P2 = 1;

float Q = 0.05;   // process noise
float R = 3.0;    // measurement noise

float kalmanUpdate(float measurement, float &x, float &P) {
  P = P + Q;
  float K = P / (P + R);
  x = x + K * (measurement - x);
  P = (1 - K) * P;
  return x;
}

void countSteps2(void) {
  // if (GENERAL_DEBUG) Serial.println("Running countSteps2");
  if(micros() - lastStepTime2 > trigDelay2) {
    if(dir2==1){
      steps2++;
    }
    if (dir2==-1){
      steps2--;
    }
    lastStepTime2 = micros();
  }
}

void countSteps1(void) {
  // if (GENERAL_DEBUG) Serial.println("Running countSteps1");
  if(micros() - lastStepTime1 > trigDelay1) {
    if(dir1==1){
      steps1++;
    }
    if (dir1==-1){
      steps1--;
    }
    lastStepTime1 = micros();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (GENERAL_DEBUG) Serial.println();

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_tflag, INPUT);
  pinMode(m2_hall, INPUT);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_tflag, INPUT);
  pinMode(m1_hall, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(m2_hall), countSteps2, RISING);
  attachInterrupt(digitalPinToInterrupt(m1_hall), countSteps1, RISING);
  
  if (SKIP_AUTO_HOMING == 0) {
    if (GENERAL_DEBUG) Serial.println("Waiting to home....");
    delay(HOMING_DELAY_MS);
    homing();
  } 
  else {
    if (GENERAL_DEBUG) Serial.println("Skipping auto-homing");
  }

  if (GENERAL_DEBUG) Serial.println("Entering MAIN LOOP");
}

void loop() {
  kalmanUpdate(steps1, x1, P1);
  kalmanUpdate(steps2, x2, P2);

  String input;
    
  if (Serial.available() > 0) {
    input = Serial.readString();
    tokenize(tokens, input);
  }
}

// Using for testing
void printStatus() {
  Serial.println("====STATUS====");
  
  Serial.print("m1 raw position: ");
  Serial.println(steps1);

  Serial.print("m1 filtered position: ");
  Serial.println(x1);

  Serial.print("m2 raw position: ");
  Serial.println(steps2);

  Serial.print("m2 filtered position: ");
  Serial.println(x2);
  
  Serial.println("==============");
}

void tokenize(char* tokens[], String input){
  if (GENERAL_DEBUG) Serial.println("Running tokenize");
  int count = 0;
  char* cinput = input.c_str();
  char* token = strtok(cinput, " \n");

  while (token != NULL) {
    Serial.print(token);
    tokens[count++]=token;
    token = strtok(NULL, " \n");
  }

  String cmd1;
  String cmd2;

  cmd1 = tokens[0];
  cmd1.toLowerCase();
  cmd2 = atof(tokens[1]);

  if (GENERAL_DEBUG) {
    Serial.println();

    Serial.print("cmd1 = ");
    Serial.println(cmd1);

    Serial.print("cmd2 = ");
    Serial.println(cmd2);
  }

  if (cmd1.equals("home") || cmd1.equals("homing") || cmd1.equals("h")) {
    if (GENERAL_DEBUG) Serial.println("HOMING!");
    homing();
  }
  else if (cmd1.equals("stop") || cmd1.equals("s") || cmd1.equals("0")) {
    if (GENERAL_DEBUG) Serial.println("STOP!");
    stop();
  }
  else if (cmd1.equals("status") || cmd1.equals("print") || cmd1.equals("p")) {
    printStatus();
  }
  else if (cmd1.equals("1") || cmd1.equals("f")) {
    if (GENERAL_DEBUG) Serial.println("FORWARD!");
    forward();
  }
  else if (cmd1.equals("2") || cmd1.equals("b")) {
    if (GENERAL_DEBUG) Serial.println("BACKWARD!");
    backward();
  }
}

void forward(){
  if (GENERAL_DEBUG) Serial.println("Running FORWARD");
  dir1=1;
  dir2=1;
  digitalWrite(m1_pwm, HIGH);
  digitalWrite(m1_dir, HIGH);
  digitalWrite(m2_pwm, HIGH);
  digitalWrite(m2_dir, HIGH);
}

void backward(){
  if (GENERAL_DEBUG) Serial.println("Running BACKWARD");
  dir1=-1;
  dir2=-1;
  digitalWrite(m1_pwm, HIGH);
  digitalWrite(m1_dir, LOW);
  digitalWrite(m2_pwm, HIGH);
  digitalWrite(m2_dir, LOW);
}

void stop(){
  if (GENERAL_DEBUG) Serial.println("Running STOP");
  dir1=0;
  dir2=0;
  digitalWrite(m1_pwm, LOW);
  digitalWrite(m1_dir, HIGH);
  digitalWrite(m2_pwm, LOW);
  digitalWrite(m2_dir, HIGH);

  // Lock Kalman state to current steps
  x1 = steps1;
  x2 = steps2;
}

void homing() {
  if (GENERAL_DEBUG) Serial.println("Running Homing Function");

  backward(); // BOTH actuators move together

  long lastAvg = (steps1 + steps2) / 2;
  unsigned long lastMoveTime = millis();

  while (true) {
    long avg = (steps1 + steps2) / 2;

    // detect movement
    if (avg != lastAvg) {
      lastAvg = avg;
      lastMoveTime = millis();
    }

    // stop when no movement detected for 1 second
    if (millis() - lastMoveTime > 1000) {
      break;
    }
  }

  stop(); // BOTH actuators stop

  steps1 = 0;
  steps2 = 0;

  // Reset Kalman filter
  x1 = 0;
  x2 = 0;

  if (GENERAL_DEBUG) Serial.println("All homed");
}
