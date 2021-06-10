#include <math.h>

#define MOTOR_0_PIN 5
#define MOTOR_1_PIN 14
#define MOTOR_DIR_0_PIN 4
#define MOTOR_DIR_1_PIN 12
/* #define INTERRUPT_PIN 2
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
  #define MITTE_HINTEN_STOP 0b11110011 */

int wmotor = 0;
int incomingByte = 0;
int zielGeschwMotor [2];
double momentanGeschwMotor[2];
bool manuellInterrupt = false;
long letzterBefehlZeit = 0;
float tau = 0.01;

void setup() {
  Serial.begin(9600);
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);
  pinMode(MOTOR_0_PIN, OUTPUT);
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_DIR_0_PIN, OUTPUT);
  pinMode(MOTOR_DIR_1_PIN, OUTPUT);
 // analogWriteFreq(333);
  pinMode(13, OUTPUT);
}

void loop() {
  readSerial();
  geschwMotor0();
  geschwMotor1();
  if (millis() - letzterBefehlZeit > 5000) {
    zielGeschwMotor[0] = 0;
    zielGeschwMotor[1] = 0;
  }
  /* geschwMotor0();
    geschwMotor1();
    if (digitalRead(INTERRUPT_MANUELL_PIN) == HIGH) {
     manuellInterrupt = true;
     interrupt();
    }
    serialPrintGeschw ();*/
}

//-------------------------------------------------------------------------------------------------------------------------------------

void readSerial() {
  if (Serial.available() > 0 ) {
    incomingByte  = Serial.read();
    if (wmotor == 0) {
      digitalWrite(13, HIGH);
      zielGeschwMotor [0] = *(int8_t*)&incomingByte ;
      wmotor = 1;
      letzterBefehlZeit = millis();

    } else {
      zielGeschwMotor [1] = *(int8_t*)&incomingByte;
      wmotor = 0;
      letzterBefehlZeit = millis();
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

void geschwMotor0() {
  if (abs(zielGeschwMotor[0] - momentanGeschwMotor[0]) < 4) {
    momentanGeschwMotor [0] = zielGeschwMotor [0];
  }
  else {
    momentanGeschwMotor[0] += (int) ((zielGeschwMotor[0]-momentanGeschwMotor[0]) * tau * 1/(1+exp(0.02 * (abs(zielGeschwMotor[0]-momentanGeschwMotor[0])-100))));
  }

  if (momentanGeschwMotor [0] < 0) {
    digitalWrite(MOTOR_DIR_0_PIN, LOW);
  }
  else {
    digitalWrite(MOTOR_DIR_0_PIN, HIGH);
  }
  analogWrite(MOTOR_0_PIN, abs(momentanGeschwMotor [0]));

}

void geschwMotor1() {
  if (abs(zielGeschwMotor[1] - momentanGeschwMotor[1]) < 4) {
    momentanGeschwMotor [1] = zielGeschwMotor [1];
  }
  else {
    momentanGeschwMotor[1] += (int) ((zielGeschwMotor[1]-momentanGeschwMotor[1]) * tau * 1/(1+exp(0.02 * (abs(zielGeschwMotor[1]-momentanGeschwMotor[1])-100))));
  }

  if (momentanGeschwMotor [1] < 0) {
    digitalWrite(MOTOR_DIR_1_PIN, LOW);
  }
  else {
    digitalWrite(MOTOR_DIR_1_PIN, HIGH);
  }
  analogWrite(MOTOR_1_PIN, abs(momentanGeschwMotor [1]));
}

//-------------------------------------------------------------------------------------------------------------------------------------

/*void interrupt() {
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
*/

//-------------------------------------------------------------------------------------------------------------------------------------
