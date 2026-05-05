#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define GENERAL_DEBUG 1

#define SKIP_AUTO_HOMING 1
#define HOMING_DELAY_MS 30*1000

bool disableDebug = false;

const int m2_hall = 20;
const int m2_pwm = 10;
const int m2_dir= 16;
const int m2_tflag = 25;

const int m1_hall = 19;
const int m1_pwm = 9;
const int m1_dir= 15;
const int m1_tflag = 24;

const float maxTicks = 1735.0;
const float ticksPerInch = (1735.0 / 10.0);
const float maxLengthInch = 4.25;
const float maxLengthTicks = maxLengthInch * ticksPerInch;

const float act_min = 0.0;
const float act_max = maxLengthTicks; 
const float min_error = 10;

// const int enA = 13;
const int in1 = 2; //12
const int in2 = 3; //11

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

bool stopCalled = false;
bool movePosLoop = false;
bool runHomingLoop = false;

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
  // if(micros() - lastStepTime2 > trigDelay2) {
  //   if(dir2==1){
  //     steps2++;
  //   }
  //   if (dir2==-1){
  //     steps2--;
  //   }
  //   lastStepTime2 = micros();
  // }
  if(dir2==1){
    steps2++;
  }
  if (dir2==-1){
    steps2--;
  }
}

void countSteps1(void) {
  // if (GENERAL_DEBUG) Serial.println("Running countSteps1");
  // if(micros() - lastStepTime1 > trigDelay1) {
  //   if(dir1==1){
  //     steps1++;
  //   }
  //   if (dir1==-1){
  //     steps1--;
  //   }
  //   lastStepTime1 = micros();
  // }
  if(dir1==1){
    steps1++;
  }
  if (dir1==-1){
    steps1--;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // maxLengthTicks = maxLengthInch * ticksPerInch;

  if (GENERAL_DEBUG) Serial.println();

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_dir, OUTPUT);
  pinMode(m2_tflag, INPUT);
  pinMode(m2_hall, INPUT);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_dir, OUTPUT);
  pinMode(m1_tflag, INPUT);
  pinMode(m1_hall, INPUT);

  // pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
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

  if ((dir1 = 1) && x1 > maxLengthTicks) {
    dir1 = 0;
    digitalWrite(m1_pwm, LOW);
    digitalWrite(m1_dir, HIGH);
  }
  if ((dir2 = 2) && x2 > maxLengthTicks) {
    dir2 = 0;
    digitalWrite(m2_pwm, LOW);
    digitalWrite(m2_dir, HIGH);
  }
  
  if (!stopCalled) {
    if (movePosLoop) {
      move_2_pos_loop();
    }
    if (runHomingLoop) {
      homingLoop();
    }
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
    stopCalled = false;
    if (GENERAL_DEBUG) Serial.println("HOMING!");
    homing();
  }
  else if (cmd1.equals("stop") || cmd1.equals("s") || cmd1.equals("0")) {
    stopCalled = true;
    if (GENERAL_DEBUG) Serial.println("STOP!");
    stop();
  }
  else if (cmd1.equals("status") || cmd1.equals("print") || cmd1.equals("p")) {
    printStatus();
  }
  else if ((cmd1.equals("1") || cmd1.equals("f")) && ((x1 <= maxLengthTicks) || (x2 <= maxLengthTicks))) {
    if (GENERAL_DEBUG) Serial.println("FORWARD!");
    forward();
  }
  else if (cmd1.equals("2") || cmd1.equals("b")) {
    stopCalled = false;
    if (GENERAL_DEBUG) Serial.println("BACKWARD!");
    backward();
  }
  else if (cmd1.equals("setpos")) {
    stopCalled = false;
    if (GENERAL_DEBUG) Serial.println("Moving to Pos!");
    move_2_pos(cmd2.toFloat());
  }

  else if (cmd1.equals("3") || cmd1.equals("he")) {
    if (GENERAL_DEBUG) Serial.print("OPENING!");
    outtakeExtend();
  }
  else if (cmd1.equals("4") || cmd1.equals("hr")) {
    if (GENERAL_DEBUG) Serial.print("CLOSING!");
    outtakeRetract();
  }
  else if (cmd1.equals("hstop") || cmd1.equals("5")) {
    if (GENERAL_DEBUG) Serial.println("OUTTAKE STOP!");
    outtakeStop();
  }

  // stopCalled = false;
}

void outtakeExtend() {
  if (GENERAL_DEBUG) Serial.println("Opening Hopper");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // analogWrite(enA, 200);
}
void outtakeRetract() {
  if (GENERAL_DEBUG) Serial.println("Closing Hopper");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // analogWrite(enA, 200);
}
void outtakeStop() {
  if (GENERAL_DEBUG) Serial.println("Running Outtake STOP");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  // analogWrite(enA, 200);
}

// void moveIntakeAct1(int dir, bool pwm, bool m_dir) {
//   dir1 = dir;
//   digitalWrite(m1_pwm, pwm);
//   digitalWrite(m1_dir, m_dir);
// }
// void moveIntakeAct2(int dir, bool pwm, bool m_dir) {
//   dir2 = dir;
//   digitalWrite(m2_pwm, pwm);
//   digitalWrite(m2_dir, m_dir);
// }

void forward(){
  if (GENERAL_DEBUG && !disableDebug) Serial.println("Running FORWARD");
  dir1=1;
  dir2=1;
  digitalWrite(m1_pwm, HIGH);
  digitalWrite(m1_dir, HIGH);
  digitalWrite(m2_pwm, HIGH);
  digitalWrite(m2_dir, HIGH);
}

void backward(){
  if (GENERAL_DEBUG && !disableDebug) Serial.println("Running BACKWARD");
  dir1=-1;
  dir2=-1;
  digitalWrite(m1_pwm, HIGH);
  digitalWrite(m1_dir, LOW);
  digitalWrite(m2_pwm, HIGH);
  digitalWrite(m2_dir, LOW);
}

void adjust_intake(float current_pos) {
  // float error = x1 - x2;
  if ((x1 - current_pos) < -min_error) {
    dir1=1;
    digitalWrite(m1_pwm, HIGH);
    digitalWrite(m1_dir, HIGH);
  } else if ((x1 - current_pos) > min_error) {
    dir1=-1;
    digitalWrite(m1_pwm, HIGH);
    digitalWrite(m1_dir, LOW);
  }
  if ((x2 - current_pos) < -min_error) {
    dir2=1;
    digitalWrite(m2_pwm, HIGH);
    digitalWrite(m2_dir, HIGH);
  } else if ((x1 - current_pos) > min_error) {
    dir2=-1;
    digitalWrite(m2_pwm, HIGH);
    digitalWrite(m2_dir, LOW);
  }
}

float pos_move_to = 0.0;
unsigned long posMove_lastMoveTime = 0;
void move_2_pos_loop() {
  float pos = pos_move_to;
  unsigned long lastMoveTime = posMove_lastMoveTime;
  bool run = true;
  do {
    if (movePosLoop) {
      long avg = (x1 + x2) / 2;
      // long avg = x2;
      float error = pos - avg;

      if (abs(error) < min_error) {
        stop();
        movePosLoop = false;
        break;
      }

      disableDebug = true;
      // if (error > min_error) {
      //   forward();
      // } else if (error < -min_error) {
      //   backward();
      // }
      float target_pos = avg + min(max(-20, error), 20);
      adjust_intake(pos);
      disableDebug = false;
      
      // stop when no movement detected for 1 second
      if (millis() - lastMoveTime > 30000) {
        movePosLoop = false;
        break;
      }
      run = false;
    }
  } while(run);
  if (!movePosLoop) {
    stop(); // BOTH actuators stop
    if (GENERAL_DEBUG) Serial.println("Reached Pos");
  }
}
void move_2_pos(float pos) {
  if (GENERAL_DEBUG) Serial.println("Running MOVE 2 POS");

  pos = constrain(pos, act_min, act_max);

  unsigned long lastMoveTime = millis();
  
  movePosLoop = true;
  pos_move_to = pos;
  posMove_lastMoveTime = lastMoveTime;
  move_2_pos_loop();

  // stop(); // BOTH actuators stop
  // if (GENERAL_DEBUG) Serial.println("Reached Pos");
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

long homing_lastAvg = 0;
unsigned long homing_lastMoveTime = 0;
void homingLoop() {
  // unsigned long lastMoveTime = homing_lastMoveTime;
  bool run = true;
  do {
    if (runHomingLoop) {
      long avg = (steps1 + steps2) / 2;

      // detect movement
      if (avg != homing_lastAvg) {
        homing_lastAvg = avg;
        homing_lastMoveTime = millis();
      }

      // stop when no movement detected for 1 second
      if (millis() - homing_lastMoveTime > 1000) {
        runHomingLoop = false;
        break;
      }
    }
    run = false;
  } while(run);
  if (!runHomingLoop) {
    stop(); // BOTH actuators stop

    steps1 = 0;
    steps2 = 0;

    // Reset Kalman filter
    x1 = 0;
    x2 = 0;

    if (GENERAL_DEBUG) Serial.println("All homed");
  }
}
void homing() {
  if (GENERAL_DEBUG) Serial.println("Running Homing Function");

  backward(); // BOTH actuators move together

  long lastAvg = (steps1 + steps2) / 2;
  unsigned long lastMoveTime = millis();

  homing_lastAvg = lastAvg;
  homing_lastMoveTime = lastMoveTime;
  runHomingLoop = true;
  homingLoop();
}// // 
