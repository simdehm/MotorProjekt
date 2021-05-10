#include <math.h>

int motor = 1;
int incomingByte = 0;
int zielGeschwMotor1 = 0;
int zielGeschwMotor2 = 0;
int momentanGeschwMotor1 = 0;
int momentanGeschwMotor2 = 0;
int startGeschwMotor1 = 0;
int startGeschwMotor2 = 0;
int x1 = 0;
int x2 = 0;
int xMax = 1000000;
int motor1Pin = 10;
int motor2Pin = 12;
int interruptPin = 2;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, RISING);
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
}

void loop() {

  if (Serial.available() == 0 ) {
    incomingByte  = Serial.read();
    if (motor == 1) {
      zielGeschwMotor1 = incomingByte * 4;
      Serial.write(zielGeschwMotor1);
      startGeschwMotor1 = momentanGeschwMotor1;
      x1 = 0;
      motor = 2;
    } else {
      zielGeschwMotor2 = incomingByte * 4;
      Serial.write(zielGeschwMotor2);
      startGeschwMotor2 = momentanGeschwMotor2;
      x2 = 0;
      motor = 1;
    }
  }
  if (momentanGeschwMotor1 != zielGeschwMotor1 ) {
   // geschwMotor1();
  }
  if (momentanGeschwMotor2 != zielGeschwMotor2) {
   // geschwMotor2();
  }
}

void geschwMotor1() {
  if (abs(zielGeschwMotor1 - momentanGeschwMotor1) < 5) {
    momentanGeschwMotor1 = zielGeschwMotor1;
  }
  else {
    momentanGeschwMotor1 = funktionA(startGeschwMotor1, zielGeschwMotor1, 1, x1);
  }
  analogWrite(motor1Pin, momentanGeschwMotor1);
}

void geschwMotor2() {
  if (abs(zielGeschwMotor2 - momentanGeschwMotor2) < 5) {
    momentanGeschwMotor2 = zielGeschwMotor2;
  }
  else {
    momentanGeschwMotor2 = funktionA(startGeschwMotor2, zielGeschwMotor2, 2, x2);
  }
  analogWrite(motor2Pin, momentanGeschwMotor2);
}

void interrupt () {
  momentanGeschwMotor1 = 0;
  momentanGeschwMotor2 = 0;
  analogWrite(motor1Pin, momentanGeschwMotor1);
  analogWrite(motor2Pin, momentanGeschwMotor2);
  zielGeschwMotor1 = 0;
  zielGeschwMotor2 = 0;
  delay(5000);
}


int funktionA (int startGeschw, int zielGeschw, int motor, int x) {
  int endGeschw;
  int m = (zielGeschw - startGeschw) / xMax;
  endGeschw = m * x + startGeschw;
  if (motor = 1) {
    x1 = x1 + 1;
  }
  else if (motor = 2) {
    x2 = x2 + 1;
  }
  return endGeschw;
}

int funktionB (int startGeschw, int zielGeschw, int motor, int x) {
  int endGeschw;
  int m = (zielGeschw - startGeschw) / (xMax * xMax);
  endGeschw = m * (x * x) + startGeschw;
  if (motor = 1) {
    x1 = x1 + 1;
  }
  else if (motor = 2) {
    x2 = x2 + 1;
  }
  return endGeschw;
}

int funktionC (int startGeschw, int zielGeschw, int motor, int x) {
  int endGeschw;
  int m = (zielGeschw - startGeschw) / sqrt(xMax);
  endGeschw = m * sqrt(x) + startGeschw;
  if (motor = 1) {
    x1 = x1 + 1;
  }
  else if (motor = 2) {
    x2 = x2 + 1;
  }
  return endGeschw;
}

int funktionD (int startGeschw, int zielGeschw, int motor, int x) {
  int endGeschw;
  int f0 = 1
  int g = zielGeschw+1-startGeschw+1;
  int m = (0-log(((g / (zielGeschw-startGeschw+1)) - 1) / ((g / f0) - 1))) / (g * xMax);
  endGeschw = g / (1 + 3 ^ ((0 - m) * g * x) * (g / f0 - 1)) + startGeschw-1;
  if (motor = 1) {
    x1 = x1 + 1;
  }
  else if (motor = 2) {
    x2 = x2 + 1;
  }
  return endGeschw;
}
