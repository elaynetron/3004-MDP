//motor driver library
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

int encoder1A = 3; // motor 1, CW direction
int encoder1B = 5; // motor 1, ACW direction
int encoder2A = 11; // motor 2, CW direction
int encoder2B = 13; // motor 2, ACW direction

int sample = 500;
int pwm = 400;
unsigned long timewidth = 0;

void setup() {
  // put your setup code here, to run once:
  md.init();
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  Serial.begin(9600);
  Serial.println("PWM set: " + pwm);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (collectData){
    md.setM2Speed(pwm); //change
    if(sample>1){
      noInterrupts();
      timewidth = pulseIn(encoder2A, HIGH);
      interrupts();
  
      Serial.println("Time-width: "+ timewidth);
      sample--;
    }
    else{
      if(pwm<400){
        pwm += 20;
        sample = sampleSize;
        Serial.print("PWM set: ");
        Serial.println(pwm);
      }
      else{
        Serial.println("Done!");
        md.setM2Speed(0); //change
        collectData = false;
      }
    }
  }
}
