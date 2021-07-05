#include <math.h>

#define MOTOR_0_PIN 5
#define MOTOR_1_PIN 14
#define MOTOR_DIR_0_PIN 4
#define MOTOR_DIR_1_PIN 12
/* #define INTERRUPT_PIN 13
  #define INTERRUPT_MANUELL_PIN 15

  #define SENSOR_LINKS_VORNE_PIN 3
  #define SENSOR_LINKS_HINTEN_PIN 6
  #define SENSOR_RECHTS_VORNE_PIN 7
  #define SENSOR_RECHTS_HINTEN_PIN 8
  #define SENSOR_MITTE_VORNE_PIN 9
  #define SENSOR_MITTE_HINTEN_PIN 10 */

  #define LINKS_VORNE_STOP 0b11001100
  #define LINKS_HINTEN_STOP 0b11000011
  #define RECHTS_VORNE_STOP 0b00111100
  #define RECHTS_HINTEN_STOP 0b00110011
  #define MITTE_VORNE_STOP 0b11111100
  #define MITTE_HINTEN_STOP 0b11110011 

int incomingByte = 0;
int zielGeschwMotor [2];
float momentanGeschwMotor[2];
bool manuellInterrupt = false;
long letzterBefehlZeit = 0;
double maxValueChange = 0.8;
double smoothness = 20;
int x = 0;
float tau = 0.002;

void setup() {
  Serial.begin(9600);
  
  /* pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);
  pinMode(INTERRUPT_MANUELL_PIN, INPUT);
  
  pinMode(SENSOR_LINKS_VORNE_PIN, INPUT);
  pinMode(SENSOR_LINKS_HINTEN_PIN, INPUT);
  pinMode(SENSOR_RECHTS_VORNE_PIN, INPUT);
  pinMode(SENSOR_RECHTS_HINTEN_PIN, INPUT);
  pinMode(SENSOR_MITTE_VORNE_PIN, INPUT);
  pinMode(SENSOR_MITTE_HINTEN_PIN, INPUT);  */
  
  pinMode(MOTOR_0_PIN, OUTPUT);
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_DIR_0_PIN, OUTPUT);
  pinMode(MOTOR_DIR_1_PIN, OUTPUT);
  
  analogWriteFreq(333);
}

void loop() {
  readSerial();
  geschwMotor(0, MOTOR_DIR_0_PIN, MOTOR_0_PIN);
  geschwMotor(1, MOTOR_DIR_1_PIN, MOTOR_1_PIN);
  if (millis() - letzterBefehlZeit > 5000) {
    zielGeschwMotor[0] = 0;
    zielGeschwMotor[1] = 0;
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------

  void readSerial() {
  if (Serial.available() > 0 ) {
    incomingByte  = Serial.read();
    x = x % 2;
    zielGeschwMotor [x] = *(int8_t*)&incomingByte;
    zielGeschwMotor [x] *= 2;
    x++;
    letzterBefehlZeit = millis();
  }
}
   

//-------------------------------------------------------------------------------------------------------------------------------------

void geschwMotor(int motor, int pinDir, int pin) {
  if (abs(zielGeschwMotor[motor] - momentanGeschwMotor[motor]) < 4) {
    momentanGeschwMotor [motor] = zielGeschwMotor [motor];
  }
  else {
    momentanGeschwMotor[motor] += constrain(((double)zielGeschwMotor[motor]-momentanGeschwMotor[motor])/smoothness,-maxValueChange,maxValueChange);
    //momentanGeschwMotor[motor] += ((zielGeschwMotor[motor] - momentanGeschwMotor[motor]) * tau * 1 / (1 + exp( 0.02 * (abs(zielGeschwMotor[motor] - momentanGeschwMotor[motor]) - 100))));
  }

  if (momentanGeschwMotor [motor] < 0) {
    digitalWrite(pinDir, LOW);
  }
  else {
    digitalWrite(pinDir, HIGH);
  }
  analogWrite(pin, abs(momentanGeschwMotor [motor]));

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
