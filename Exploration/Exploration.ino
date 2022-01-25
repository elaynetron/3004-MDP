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

// Sensors Init
SharpIR sharp1(SharpIR::GP2Y0A21YK0F, A0);
//SharpIR sharp2(SharpIR::GP2Y0A02YK0F, A1);
SharpIR sharp3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR sharp4(SharpIR::GP2Y0A21YK0F, A3);
SharpIR sharp5(SharpIR::GP2Y0A21YK0F, A4);
SharpIR sharp6(SharpIR::GP2Y0A21YK0F, A5);
//GP2Y0A02YK0F

RunningMedian sensor1Values(20);
//RunningMedian sensor2Values(30);
RunningMedian sensor3Values(20);
RunningMedian sensor4Values(20);
RunningMedian sensor5Values(20);
RunningMedian sensor6Values(20);

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
const double Kp_R = 2.6, K2_R = -0.7, K3_R = 0.0;
const double Kp_L = 2.6, K2_L = -0.3, K3_L = 0.0;
PID myPIDL(&RPM_L, &OUTPUT_RPM_L, &setpoint_L, Kp_L, K2_L, K3_L, DIRECT);
PID myPIDR(&RPM_R, &OUTPUT_RPM_R, &setpoint_R, Kp_R, K2_R, K3_R, DIRECT);
char movement;
double sensorA0, sensorA2, sensorA3, sensorA4, sensorA5;

void setup() {
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
  
  myPIDL.SetMode(AUTOMATIC);
//  myPIDL.SetOutputLimits(-350,350);
//  myPIDL.SetSampleTime(10);
  myPIDR.SetMode(AUTOMATIC);
//  myPIDR.SetOutputLimits(-350,350);
//  myPIDR.SetSampleTime(10);

  printSensors();
}

void loop() {
  delay(1000);

  if (Serial.available()){
    movement = (char)Serial.read();
  }
  
  switch (movement){
    case 'L':{
      turnLeft(92);
      break;
    }
    case 'R':{
      turnRight(86);
      break;
    }
    case'F':{
      moveForward(1);
    }
  }
  printSensors();
}

void moveForward(double grid){
  speed_L = 360;
  speed_R = 360;
  int total_ticks = 0;
  double tick_th = 260*grid;    // 10cm
  while (1){
    md.setSpeeds(speed_L-4,speed_R);
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
    if (total_ticks>tick_th || getDistance('f')<20){
      break;
    }
  }
  md.setSpeeds(0, 0);
  md.setBrakes(400, 350);
}

void turnLeft(double angle) {
  speed_L = 330;
  speed_R = 330;
  int total_ticks = 0;
  int tick_th = 4*angle+72;    // 90 degrees
  while (1){
    md.setSpeeds(-speed_L,speed_R);
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
  int tick_th = 4*angle+50;       // angle:90 -> tick_th:405
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

double getSensorMedianValue(int num){
  RunningMedian sensorValues(20);
  if (num==1){
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp1.getDistance());
      }
      else {
        sharp1.getDistance();
      }
    }
  }
  /*
  else if (num==2){
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp2.getDistance());
      }
      else {
        sharp2.getDistance();
      }
    }
  }*/
  else if (num==3){
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp3.getDistance());
      }
      else {
        sharp3.getDistance();
      }
    }
  }
  else if (num==4){
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp4.getDistance());
      }
      else {
        sharp4.getDistance();
      }
    }
  }
  else if (num==5){
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp5.getDistance());
      }
      else {
        sharp5.getDistance();
      }
    }
  }
  else{ // num==6
    for (int i=0; i<30; i++){
      if (i >= 10) {
      sensorValues.add(sharp6.getDistance());
      }
      else {
        sharp6.getDistance();
      }
    }
  }
  double dist = sensorValues.getMedian();
  return dist;
}

void printSensors(){
  double A0= getSensorMedianValue(1);
  double A4 =getSensorMedianValue(5);
  double A5= getSensorMedianValue(6);
  double A2 = getSensorMedianValue(3);
  double A3 = getSensorMedianValue(4);
  Serial.print((A2-1.2169)/0.9114);
  Serial.print(",");
  Serial.print(A0);
  Serial.print(",");
  Serial.print(A4);
  Serial.print(",");
  Serial.print(A5);
  Serial.print(",");
  Serial.println((A3+0.4583)/0.9682);
}

/*
double getDistance(char dir){
  if (dir=='f'){
    double A0= getSensorMedianValue(1);
    double A4 =getSensorMedianValue(5);
    double A5= getSensorMedianValue(6);
    if (A0<=17){
      return A0;
    }
    else if (A5 <= 30) {
      return A5;
    }
    else {
      return (A4-1.257)/0.7633;
    }
  }
  else if (dir=='l'){
    double A2 = getSensorMedianValue(3);
    return (A2-1.2169)/0.9114;
  }
  else if (dir=='r'){
    double A3 = getSensorMedianValue(4);
    return (A3+0.4583)/0.9682;
  }
}*/
