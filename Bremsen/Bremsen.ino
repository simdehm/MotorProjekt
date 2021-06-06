#include <math.h>

#define MOTOR_0_PIN 5
#define MOTOR_1_PIN 14
#define MOTOR_DIR_0_PIN 4
#define MOTOR_DIR_1_PIN 12
#define INTERRUPT_PIN 2
#define INTERRUPT_MANUELL_PIN 10

#define SENSOR_LINKS_VORNE_PIN 3
#define SENSOR_LINKS_HINTEN_PIN 4
#define SENSOR_RECHTS_VORNE_PIN 5
#define SENSOR_RECHTS_HINTEN_PIN 6
#define SENSOR_MITTE_VORNE_PIN 7
#define SENSOR_MITTE_HINTEN_PIN 8

#define LINKS_VORNE_STOP 0b11001100
#define LINKS_HINTEN_STOP 0b11000011
#define RECHTS_VORNE_STOP 0b00111100
#define RECHTS_HINTEN_STOP 0b00110011
#define MITTE_VORNE_STOP 0b11111100
#define MITTE_HINTEN_STOP 0b11110011

int wmotor = 1;
int incomingByte = 0;
int zielGeschwMotor [2];
int momentanGeschwMotor[2];
int startGeschwMotor [2];
int x [2];
int xMax = 100000;
bool manuellInterrupt = false;


void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);
  pinMode(MOTOR_0_PIN, OUTPUT);
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_DIR_0_PIN, OUTPUT);
  pinMode(MOTOR_DIR_1_PIN, OUTPUT);
  analogWriteFreq(333);
}

void loop() {

  readSerial();
  geschwMotor1();
  geschwMotor2();

  if (digitalRead(INTERRUPT_MANUELL_PIN) == HIGH) {
    manuellInterrupt = true;
    interrupt();
  }

  seerialPrintGeschw ();
}

//-------------------------------------------------------------------------------------------------------------------------------------

void readSerial() {

  if (Serial.available() > 0 ) {
    incomingByte  = Serial.read();
    if (wmotor == 1) {
      zielGeschwMotor [0] = incomingByte * 4;
      Serial.write(momentanGeschwMotor [0]);
      startGeschwMotor [0] = momentanGeschwMotor [0];
      x[0] = 0;
      wmotor = 2;
    } else {
      zielGeschwMotor [1] = incomingByte * 4;
      Serial.write(momentanGeschwMotor [1]);
      startGeschwMotor [1] = momentanGeschwMotor [1];
      x[1] = 0;
      wmotor = 1;
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void geschwMotor1() {
  if (momentanGeschwMotor [0] != momentanGeschwMotor [0]) {
    if (abs(zielGeschwMotor [0] - momentanGeschwMotor [0]) < 5) {
      momentanGeschwMotor [0] = zielGeschwMotor [0];
    }
    else {
      momentanGeschwMotor [0] = linear(startGeschwMotor [0], zielGeschwMotor [0], 1, x[0]);
    }
    if (momentanGeschwMotor [0] < 0) {
      digitalWrite(MOTOR_DIR_1_PIN, LOW);
    }
    else {
      digitalWrite(MOTOR_DIR_1_PIN, HIGH);
    }
    analogWrite(MOTOR_0_PIN, abs(momentanGeschwMotor [0]));
  }
}

void geschwMotor2() {
  if (momentanGeschwMotor [1] != momentanGeschwMotor [1]) {
    if (abs(zielGeschwMotor [1] - momentanGeschwMotor [1]) < 5) {
      momentanGeschwMotor [1] = zielGeschwMotor [1];
    }
    else {
      momentanGeschwMotor [1] = linear(startGeschwMotor [1], zielGeschwMotor [1], 2, x[1]);
    }
    if (momentanGeschwMotor [1] < 0) {
      digitalWrite(MOTOR_DIR_1_PIN, LOW);
    }
    else {
      digitalWrite(MOTOR_DIR_1_PIN, HIGH);
    }
    analogWrite(MOTOR_1_PIN, abs(momentanGeschwMotor [1]));
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void interrupt() {
  momentanGeschwMotor [0] = 0;
  momentanGeschwMotor [1] = 0;
  analogWrite(MOTOR_0_PIN, momentanGeschwMotor [0]);
  analogWrite(MOTOR_1_PIN, momentanGeschwMotor [1]);
  if (manuellInterrupt != true) {
    sendinterrupt();
  }
  else {
    manuellInterrupt = false;
  }
  delay(5000);
}

void sendinterrupt() {
  byte woStop = 0b00000000 ;
  if (digitalRead(SENSOR_LINKS_VORNE_PIN)) {
    woStop = LINKS_VORNE_STOP;
  }
  else if (digitalRead(SENSOR_LINKS_HINTEN_PIN)) {
    woStop = LINKS_HINTEN_STOP;
  }
  else if (digitalRead(SENSOR_RECHTS_VORNE_PIN)) {
    woStop = RECHTS_VORNE_STOP;
  }
  else if (digitalRead(SENSOR_RECHTS_HINTEN_PIN)) {
    woStop = RECHTS_HINTEN_STOP;
  }
  else if (digitalRead(SENSOR_MITTE_VORNE_PIN)) {
    woStop = MITTE_VORNE_STOP;
  }
  else if (digitalRead(SENSOR_MITTE_HINTEN_PIN)) {
    woStop = MITTE_HINTEN_STOP;
  }
  Serial.write(woStop);
}

//-------------------------------------------------------------------------------------------------------------------------------------

void serialPrintGeschw() {
  Serial.print("Zeit ");
  Serial.print(x[0]);
  Serial.print(" ");
  Serial.print(x[1]);
  Serial.print(" Geschwindigkeit ");
  Serial.print(momentanGeschwMotor[0]);
  Serial.print(" ");
  Serial.println(momentanGeschwMotor[1]);
}

//-------------------------------------------------------------------------------------------------------------------------------------

int linear (int startGeschw, int zielGeschw, int motor, int xf) {
  int endGeschw;
  int m = (zielGeschw - startGeschw) / xMax;
  endGeschw = m * xf + startGeschw;
  if (motor == 1) {
    x [0] = x [0] + 1;
  }
  else if (motor == 2) {
    x [1] = x [1] + 1;
  }
  delay(10);
  return endGeschw;
}

int quadratisch (int startGeschw, int zielGeschw, int motor, int xf) {
  int endGeschw;
  double m = (double) (zielGeschw - startGeschw) / (double) (xMax * xMax);
  endGeschw = (int) (m * (xf * xf) + startGeschw);
  if (motor == 1) {
    x [0] = x [0] + 1;
  }
  else if (motor == 2) {
    x [1] = x [1] + 1;
  }
  delay(10);
  return endGeschw;
}

int wurzel (int startGeschw, int zielGeschw, int motor, int xf) {
  int endGeschw;
  int m = (zielGeschw - startGeschw) / sqrt(xMax);
  endGeschw = m * sqrt(xf) + startGeschw;
  if (motor == 1) {
    x [0] = x [0] + 1;
  }
  else if (motor == 2) {
    x [1] = x [1] + 1;
  }
  delay(10);
  return endGeschw;
}

int logistisch (int startGeschw, int zielGeschw, int motor, int xf) {
  int endGeschw;
  int f0 = 1;
  int g = zielGeschw + 1 - startGeschw + 1;
  double m = ((0 - log(((g / (double)(zielGeschw - startGeschw + 1)) - 1) / ((g / (double)f0) - 1))) / (double) (g * xMax));
  endGeschw = (int) ((g / (1 + pow(3.0, ((0 - m) * g * xf * (g / f0 - 1)))) + startGeschw - 1));
  if (motor == 1) {
    x [0] = x [0] + 1;
  }
  else if (motor == 2) {
    x [1] = x [1] + 1;
  }
  delay(10);
  return endGeschw;
}
