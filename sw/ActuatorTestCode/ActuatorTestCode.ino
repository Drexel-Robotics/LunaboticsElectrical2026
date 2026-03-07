#include <ezButton.h>

ezButton extendButton(7);
ezButton retractButton(6);

const int in3 = 9;
const int in4 = 8;
const int enB = 10;

void setup() {
  Serial.begin(9600);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  stopMotor();  // Start safely stopped
}

void loop() {

  extendButton.loop();
  retractButton.loop();

  int extendState = extendButton.getState();
  int retractState = retractButton.getState();

  // Buttons are usually LOW when pressed with ezButton
  bool extendPressed = (extendState == 0);
  bool retractPressed = (retractState == 0);

  if (extendPressed && !retractPressed) {
    moveForward();     // Extend
  }
  else if (retractPressed && !extendPressed) {
    moveBackward();    // Retract
  }
  else {
    stopMotor();       // Neither OR both pressed
  }
}

// ---------- FUNCTIONS ----------

// Please do NOT press one button immediately after the other. It is bad for the actuator.

void moveForward() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 255);
}

void moveBackward() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 255);
}

void stopMotor() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}
