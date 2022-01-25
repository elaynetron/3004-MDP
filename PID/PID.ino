#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
//Motor Left
#define encoder1A 3
#define encoder1B 5
//Motor Right
#define encoder2A 11
#define encoder2B 13

double targetRPM = 80;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  md.init();

  pinMode(encoder1A,INPUT);
  //pinMode(encoder1A,INPUT_PULLUP);
  //pinMode(encoder1B,INPUT);
  pinMode(encoder2A,INPUT);
  //pinMode(encoder2A,INPUT_PULLUP);
  //pinMode(encoder2B,INPUT);

  delay(2000);
  enableInterrupt(encoder1A, E1_ticks, RISING);
  enableInterrupt(encoder2A, E2_ticks, RISING);

  md.setSpeeds(calcRPM1(targetRPM),calcRPM2(targetRPM));  
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

// calculate RPM for E1
double calcRPM1(targetRPM) {
  
}
