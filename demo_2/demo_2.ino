#include "DualVNH5019MotorShield.h"
#include <SharpIR.h>
#include <EnableInterrupt.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <RunningMedian.h>
DualVNH5019MotorShield md;

// Motor Left
#define encoder1A 3
#define encoder1B 5
// Motor Right
#define encoder2A 11
#define encoder2B 13
int val;
int encoder0Pos = 0; 
int encoder0PinALast = LOW;
int n= LOW;

double speed_L = 203;
double speed_R = 226;
double RPM_L = 0;
double RPM_R = 0;
volatile unsigned int tick_L = 0;
volatile unsigned int tick_R = 0;
int ticks_to_move = 0;

double OUTPUT_RPM_R = 0;
double OUTPUT_RPM_L = 0;
double setpoint_L = 120;
double setpoint_R = 123;
const double Kp_R = 2.6, K2_R = -0.3, K3_R = 0.0;
const double Kp_L = 2.6, K2_L = -0.3, K3_L = 0.0;
PID myPIDL(&RPM_L, &OUTPUT_RPM_L, &setpoint_L, Kp_L, K2_L, K3_L, DIRECT);
PID myPIDR(&RPM_R, &OUTPUT_RPM_R, &setpoint_R, Kp_R, K2_R, K3_R, DIRECT);

char string1[] = "1R1R1R1";
void setup()
{
  Serial.begin(9600);
  Serial.println("MDP group 4");
  md.init();
  
  pinMode(encoder1A,INPUT);
  pinMode(encoder2A,INPUT);
  
  delay(2000);
  enableInterrupt(encoder1A, E1_ticks, RISING);
  enableInterrupt(encoder2A, E2_ticks, RISING);
  
  myPIDL.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);
}

void loop()
{
  delay(1000);
  
  char *option = string1;
  while(*option != '\0'){
  switch (*option){
    case 'L':{
      Serial.println("---Turn 90 degrees left---");
      turnLeft(90);
      break;
    }
    case 'R':{
      Serial.println("---Turn 90 degrees right---");
      turnRight(90);
      break;
    }
    case '1':{
      moveForward(1);
      break;
    }
    case '2':{
      moveForward(2);
      break;
    }
    case '3':{
      moveForward(3);
      break;
    }
    case '4':{
      moveForward(4);
      break;
    }
    case '5':{
      moveForward(5);
      break;
    }
    case '6':{
      moveForward(6);
      break;
    }
    case '7':{
      moveForward(7);
      break;
    }
    case '8':{
      moveForward(8);
      break;
    }
    case '9':{
      moveForward(9);
      break;
    }
    case 't': {
      // obstacle avoidance test
      moveForward(13);
      delay(2000);
      turnLeft(45);
      delay(2000);
      moveForward(3);
      delay(2000);
      //delay(2000);
      turnRight(90);
      delay(2000);
      moveForward(2);
      delay(2000);
      turnLeft(45);
      delay(2000);
      moveForward(2);
      break;
    }
    default:{
      md.setSpeeds(0, 0);
      md.setBrakes(400, 400);
    }
  }
  option++;
  delay(1000);
  }
}

void moveForward(double grid){
  speed_L = 360;
  speed_R = 360;
  int total_ticks = 0;
  double tick_th = 298*grid;    // 10cm
  while (1){
    md.setSpeeds(speed_L-3,speed_R);
    tick_L = 0;
    tick_R = 0;
    delay(50);
      {
        RPM_L = tick_L/562.25/0.05*60;
        RPM_R = tick_R/562.25/0.05*60;
        Serial.print("  M1 Left RPM: ");
        Serial.print(RPM_L);
        Serial.print(",");
        Serial.print("  M2 Right RPM: ");
        Serial.println(RPM_R);
      }
    myPIDL.Compute();
    myPIDR.Compute();
    if (RPM_L<setpoint_L){
      speed_L += OUTPUT_RPM_L;
    }
    if (RPM_R<setpoint_R){
      speed_R += OUTPUT_RPM_R;
    }
    total_ticks += tick_L;
    if (total_ticks>tick_th){
      break;
    }
  }
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
}

//getDistance('f')<15)
void turnLeft(double angle) {
  speed_L = 360;
  speed_R = 360;
  int total_ticks = 0;
  int tick_th = 53*(angle/10);    // 90 degrees
  while (1){
    md.setSpeeds(-speed_L+5,speed_R);
    tick_L = 0;
    tick_R = 0;
    delay(50);
      {
        RPM_L = tick_L/562.25/0.05*60;
        RPM_R = tick_R/562.25/0.05*60;
        Serial.print("  M1Left RPM: ");
        Serial.print(RPM_L);
        Serial.print(",");
        Serial.print("  M2Right RPM: ");
        Serial.println(RPM_R);
      }
    myPIDL.Compute();
    myPIDR.Compute();
    if (RPM_L<setpoint_L){
      speed_L += OUTPUT_RPM_L;
    }
    if (RPM_R<setpoint_R){
      speed_R += OUTPUT_RPM_R;
    }
    total_ticks += tick_L;
    if (total_ticks>tick_th){
      break;
    }
  }
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
}

void turnRight(double angle) {
  speed_L = 360;
  speed_R = 360;
  int total_ticks = 0;
  int tick_th = 50*(angle/10)+50;       // angle:90 -> tick_th:405
  while (1){
    md.setSpeeds(speed_L-5,-speed_R);
    tick_L = 0;
    tick_R = 0;
    delay(50);
      {
        RPM_L = tick_L/562.25/0.05*60;
        RPM_R = tick_R/562.25/0.05*60;
        Serial.print("  M1Left RPM: ");
        Serial.print(RPM_L);
        Serial.print(",");
        Serial.print("  M2Right RPM: ");
        Serial.println(RPM_R);
      }
    myPIDL.Compute();
    myPIDR.Compute();
    if (RPM_L<setpoint_L){
      speed_L += OUTPUT_RPM_L;
    }
    if (RPM_R<setpoint_R){
      speed_R += OUTPUT_RPM_R;
    }
    total_ticks += tick_L;
    if (total_ticks>tick_th){
      break;
    }
  }
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
}

void E1_ticks()
{
  tick_L ++;
}
void E2_ticks()
{
  tick_R ++;
}
