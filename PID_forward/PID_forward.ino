#include <EnableInterrupt.h>
#include <PID_v1.h>
#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;
// Define motor
#define M1_Encoder 3      //right motor pin A
#define M2_Encoder 11     //left motor pin A

/* Variable Declaration */
double leftEncoderValue = 0;
double rightEncoderValue = 0;
double output;
//double startLeftEncoderValue, startRightEncoderValue, 

// Specify the links and initial tuning parameters
double Kp=2.6, Ki=-0.3, Kd=0.0; // 2.6, -0.18, 0.003
//double Kp=1.42, Ki=-0.137, Kd=0.003;
PID myPID(&rightEncoderValue, &output, &leftEncoderValue, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(M1_Encoder, INPUT);
  pinMode(M2_Encoder, INPUT);
  /* Enable interrupt on rising edge */
  enableInterrupt(M2_Encoder, leftEncoderInc, RISING);
  enableInterrupt(M1_Encoder, rightEncoderInc, RISING);
  Serial.begin(9600);
  Serial.setTimeout(50);

  md.init();
  myPID.SetOutputLimits(-50, 50);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(5);
}

void loop() {
  //if (Serial.available() > 0) 
  PIDforward(15);
  delay(5000);
}

void PIDforward(float grid) {
 
  double fwdLValue = leftEncoderValue;
  double fwdRValue = rightEncoderValue;
  double distance_tick = (562.25 * (grid * 10)) / (6.3 * 3.1416); //6.3 originally

  while ((leftEncoderValue <= fwdLValue + distance_tick) || (rightEncoderValue <= fwdRValue + distance_tick)) {
    Serial.println("---Before----");
    Serial.println(leftEncoderValue);
    Serial.println(rightEncoderValue);
    //md.setSpeeds(300,300);
    md.setSpeeds(300 - output, 300 + output);
    /*if (myPID.Compute())
    {
      Serial.println("+++After+++");
      Serial.println(leftEncoderValue);
      Serial.println(rightEncoderValue);
      Serial.println(output);
      md.setSpeeds(300 - output, 300 + output);

    }*/
  }
  md.setBrakes(400, 400);
}

void leftEncoderInc(void) {
  leftEncoderValue++;
}

void rightEncoderInc(void) {
  rightEncoderValue++;
}
