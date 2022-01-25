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
const double Kp_R = 2.6, K2_R = -0.3, K3_R = 0.0;
const double Kp_L = 2.6, K2_L = -0.3, K3_L = 0.0;
PID myPIDL(&RPM_L, &OUTPUT_RPM_L, &setpoint_L, Kp_L, K2_L, K3_L, DIRECT);
PID myPIDR(&RPM_R, &OUTPUT_RPM_R, &setpoint_R, Kp_R, K2_R, K3_R, DIRECT);

char option;
void setup()
{
  Serial.begin(9600);
  Serial.println("MDP group 4");
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
}

void loop()
{
  delay(1000);
  
  option = Serial.read();
  switch (option){
    case 'f':{
      Serial.println("---Move Forward---");
      moveForward(1);
      break;
    }
    case 'l':{
      Serial.println("---Turn 90 degrees left---");
      turnLeft(90);
      break;
    }
    case 'r':{
      Serial.println("---Turn 90 degrees right---");
      turnRight(90);
      break;
    }
    case '0':{
      Serial.println("---Sensor A0---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(1));
      break;
    }
    case '1':{
      Serial.println("---Sensor A1---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(2));
      break;
    }
    case '2':{
      Serial.println("---Sensor A2---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(3));
      break;
    }
    case '3':{
      Serial.println("---Sensor A3---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(4));
      break;
    }
    case '4':{
      Serial.println("---Sensor A4---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(5));
      break;
    }
    case '5':{
      Serial.println("---Sensor A5---");
      Serial.print("The sensor median value is ");
      Serial.println(getSensorMedianValue(6));
      break;
    }
    case '6':{
      Serial.println("---All Sensors---");
      getSensorMedianDist();
      break;
    }
    case '7': {
      Serial.println(getDistance('f'));
      break;
    }
  }
  delay(2000);
}

/*
 * Functions
PIDStepTest(); // step test for calculating PID
encoderTest();

getSensorValues();
getSensorMedianDist();

obstacleAvoidanceTest();
*/

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
    if (total_ticks>tick_th || getDistance('f')<20){
      break;
    }
  }
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
}

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

void obstacleAvoidanceTest(){
  moveForward(13);
  delay(2000);
  turnLeft(45);
  delay(2000);
  moveForward(3);
  delay(2000);
  turnRight(85);
  delay(2000);
  moveForward(3.45);
  delay(2000);
  turnLeft(40);
  delay(2000);
  moveForward(2);
}

void E1_ticks()
{
  tick_L ++;
}
void E2_ticks()
{
  tick_R ++;
}

void encoderTest(){
  n = digitalRead(encoder2A); 
  if ((encoder0PinALast == LOW) && (n == HIGH)) { 
    if (digitalRead(encoder2B) == LOW) {
      encoder0Pos--; 
    } else {
      encoder0Pos++; 
    } 
    Serial.print (encoder0Pos); 
    Serial.print ("/"); 
  }
  encoder0PinALast = n;

}
void printRPM(){
  for (int Mspeed=50;Mspeed>=0;){
    //md.setSpeeds(Mspeed+5,Mspeed);
    md.setSpeeds(Mspeed,Mspeed);
    delay(1000);
    for (int i=1;i<=5;i++){
      tick_L = 0;
      tick_R = 0;
      delay(50);
      {
        Serial.print("M1Left RPM: ");
        Serial.print(tick_L/562.25/0.05*60);
        Serial.print(",");
        Serial.print(tick_L);
        Serial.print(",");
        Serial.print("M2Right RPM: ");
        Serial.print(tick_R/562.25/0.05*60);
        Serial.print(",");
        Serial.println(tick_R);
      }
    }
  }
}

// for calculating RPM with time change and plot step change
void PIDStepTest(){
  Serial.println("Step test begins");
  for (int i=0;i<10;i++){
    md.setSpeeds(300,300);
    for (int i=1;i<=5;i++){
      tick_L = 0;
      tick_R = 0;
      delay(50);
      {
        Serial.print("Current time: ");
        Serial.print(millis());
        Serial.print("  M1Left RPM: ");
        Serial.print(tick_L/562.25/0.05*60);
        Serial.print(",");
        Serial.print(tick_L);
        Serial.print(",");
        Serial.print("M2Right RPM: ");
        Serial.print(tick_R/562.25/0.05*60);
        Serial.print(",");
        Serial.println(tick_R);
      }
    }
  }
  Serial.print("Start time: ");
  Serial.println(millis());
  for (int i=0;i<10;i++){
    md.setSpeeds(250,250);
    for (int i=1;i<=5;i++){
      tick_L = 0;
      tick_R = 0;
      delay(50);
      {
        Serial.print("Current time: ");
        Serial.print(millis());
        Serial.print("  M1Left RPM: ");
        Serial.print(tick_L/562.25/0.05*60);
        Serial.print(",");
        Serial.print(tick_L);
        Serial.print(",");
        Serial.print("M2Right RPM: ");
        Serial.print(tick_R/562.25/0.05*60);
        Serial.print(",");
        Serial.println(tick_R);
      }
    }
  }
  while(1);
}

// ====================== sensor ====================
// get reading for every sensor
void getSensorValues(){
  Serial.print(" 1: ");
  Serial.print(sharp1.getDistance());
  Serial.print(" 2: ");
  //Serial.print(sharp2.getDistance());
  Serial.print(" 3: ");
  Serial.print(sharp3.getDistance());
  Serial.print(" 4: ");
  Serial.print(sharp4.getDistance());
  Serial.print(" 5: ");
  Serial.print(sharp5.getDistance());
  Serial.print(" 6: ");
  Serial.println(sharp6.getDistance());  
  delay(1000);
}

// get median readings from all sensors
void getSensorMedianDist(){
  for (int i=0; i<30; i++){
    if (i >= 10) {
      sensor1Values.add(sharp1.getDistance());
      //sensor2Values.add(sharp2.getDistance());
      sensor3Values.add(sharp3.getDistance());
      sensor4Values.add(sharp4.getDistance());
      sensor5Values.add(sharp5.getDistance());
      sensor6Values.add(sharp6.getDistance());
    }
    else {
      sharp1.getDistance();
      //sharp2.getDistance();
      sharp3.getDistance();
      sharp4.getDistance();
      sharp5.getDistance();
      sharp6.getDistance();
    }
  }
  Serial.println("Sensors median readings:");
  Serial.print(" 1: ");
  Serial.print(sensor1Values.getMedian()); 
  /*
  Serial.print(" 2: ");
  Serial.println(sensor2Values.getMedian());  */
  Serial.print(" 3(long): ");
  Serial.println(sensor3Values.getMedian()); 
  Serial.print(" 4: ");
  Serial.println(sensor4Values.getMedian()); 
  Serial.print(" 5: ");
  Serial.println(sensor5Values.getMedian()); 
  Serial.print(" 6: ");
  Serial.println(sensor6Values.getMedian()); 
  delay(2000);
}

// get median distance from 20 values measured by sensors
// input: sensor number
// output: distance
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
}
