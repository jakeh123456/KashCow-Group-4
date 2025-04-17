//===================================================//
//               Part B Robotics Project             //
//                  Company D Group 4                //
//                     Maze Solver                   //
//                   Jake Hallworth                  //
//                       V 1.5.3                     //
//===================================================//

//Include Libraries
#include <Servo.h>
#include <MKL_HCSR04.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Define pins
#define run1 A0
#define run2 A1
#define run3 A2
#define trig 3
#define enable 5
#define usf 9  //Ultrasonic front
#define usl 10 //Ultrasonic left
#define usr 11 //Ultrasonic right
#define svl 12 //Servo left
#define svr 13 //Servo right

//Define world-frame constants
#define GRAVITY_EARTH_MS2 9.80665 
#define RAD_TO_DEG 57.295779513082320876798154814105

//Library Setup
MKL_HCSR04 hc(trig, new int[3]{usf,usl,usr}, 3);
MPU6050 mpu;
Servo lw; //Left wheel servo
Servo rw; //Right wheel servo

//Define variables
int run = 1;
int deadend = 1;
/*
int frontThresh = 7;
int sideThresh = 3;
*/

int frontThresh = 10;
int sideThresh = 4;
int uThresh = 13;
bool inMaze = false;
bool DMPReady = false;
bool endRun = false;
float frontDist = 20.00;
float leftDist = 20.00;
float rightDist = 20.00;
float yaw;
float targAngle;
float euler[3];
float ypr[3];
uint8_t FIFOBuffer[64];
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;

//Define containers
Quaternion q;
VectorInt16 aa;
VectorInt16 gg;
VectorInt16 aaWorld;
VectorInt16 ggWorld;
VectorFloat gravity;

void setup(){
  //Setup serial connection
  Serial.begin(9600);
  IMUInit();
  //Confirm setup complete
  Serial.println("");
  Serial.print("Ready");
  Serial.println("");
  delay(500);
  IMU();
  Serial.println("");
  Serial.println("");
}

void loop(){
  if(digitalRead(enable) == HIGH && endRun == false){
    if(inMaze == false){
      Serial.print("Entering maze");
      Serial.println("");
      forward();
      while(leftDist > 10 && rightDist > 10){
        selfCheck();
      }
      inMaze = true;
      Serial.print("Entered maze");
      Serial.println("");
    }
    selfCheck();
    forward();
    while(frontDist > frontThresh){
      Serial.print("Proceed in maze");
      Serial.println("");
      selfCheck();
      if(rightDist <= sideThresh)slightLeft();
      if(leftDist <= sideThresh)slightRight();
      if(leftDist > uThresh && rightDist > uThresh){
        Serial.print("Exiting maze...");
        Serial.println("");
        delay(4000);
        end();
      }
    }
    if(endRun == false){
      Serial.print("Maze blocked");
      Serial.println("");
      Serial.print("Stop");
      Serial.println("");
      stop();

      if(leftDist < uThresh && rightDist < uThresh){
        Serial.print("DeadEnd detected");
        Serial.println("");
        Serial.print("Current Params: ");
        Serial.println("");
        Serial.print("Front Distance: ");
        Serial.print(frontDist);
        Serial.println("");
        Serial.print("Left Distance: ");
        Serial.print(leftDist);
        Serial.println("");
        Serial.print("Right Distance: ");
        Serial.print(rightDist);
        Serial.println("");
        deadend = run;
        Serial.print(run);
        Serial.println("");
        Serial.print("Deadend: ");
        Serial.print(deadend);
        Serial.println("");
        if(leftDist < rightDist) turn(true, 180);
        else turn(false, 180);
      }
      else if(leftDist < uThresh || rightDist < uThresh){
        Serial.print("Turn Detected");
        Serial.println("");
        Serial.print("Current Params: ");
        Serial.println("");
        Serial.print("Front Distance: ");
        Serial.print(frontDist);
        Serial.println("");
        Serial.print("Left Distance: ");
        Serial.print(leftDist);
        Serial.println("");
        Serial.print("Right Distance: ");
        Serial.print(rightDist);
        Serial.println("");
        if(leftDist < rightDist) turn(true, 90);
        else turn(false, 90);
      }
      else{
        Serial.print("T-Junction Detected");
        Serial.println("");
        Serial.print("Current Params: ");
        Serial.println("");
        Serial.print("Front Distance: ");
        Serial.print(frontDist);
        Serial.println("");
        Serial.print("Left Distance: ");
        Serial.print(leftDist);
        Serial.println("");
        Serial.print("Right Distance: ");
        Serial.print(rightDist);
        Serial.println("");
        Serial.print("RUN: ");
        Serial.print(run);
        Serial.println("");
        Serial.print("Deadend: ");
        Serial.print(deadend);
        Serial.println("");
        if(run == 1) turn(false, 90);
        else if(run == 2) turn(true, 90);     
        else if(deadend == 1) turn(true, 90);     
        else if(deadend == 2) turn(false, 90);
      }
    }
  }
  
  if(digitalRead(enable) == LOW && endRun == true){
    endRun = false;
    stop();
    runCheck();
  }

  else{
    stop();
    runCheck();
  }
}

void stop(){
  lw.detach();
  rw.detach();
}

void forward(){
  Serial.print("Forward");
  Serial.println("");
  rw.attach(svr);
  lw.attach(svl);
  rw.write(1);
  lw.write(101);
}

void left(){
  rw.attach(svr);
  lw.attach(svl);
  rw.write(1);
  lw.write(85);
}

void right(){
  rw.attach(svr);
  lw.attach(svl);
  rw.writeMicroseconds(1510);
  lw.write(101);
}

void leftSlow(){
  lw.writeMicroseconds(1435);
  rw.writeMicroseconds(1422);
  rw.attach(svr);
  lw.attach(svl);
}

void rightSlow(){
  lw.writeMicroseconds(1511);
  rw.writeMicroseconds(1511);
  rw.attach(svr);
  lw.attach(svl);
}

void slightLeft(){
  Serial.print("Left adjust");
  Serial.println("");
  lw.detach();
  delay(200);
  lw.attach(svl);
}

void slightRight(){
  Serial.print("Right adjust");
  Serial.println("");
  rw.detach();
  delay(200);
  rw.attach(svr);
}

void turn(bool dir, float turnAngle){
  Serial.print("Initiating Turn ");
  Serial.print(turnAngle);
  Serial.print("°");
  Serial.println("");
  IMU();
  const float initAngle = yaw;
  if(dir == false){
    Serial.print("Left");
    Serial.println("");
    targAngle = initAngle - turnAngle;
    if(targAngle <= 0){
      targAngle += 360;
    }
    leftSlow();
    while(abs(targAngle-yaw)>8){
      IMU();
    }
    Serial.print("Stop");
    Serial.println("");
    stop();
  }
  else{
    Serial.print("Right");
    Serial.println("");
    targAngle = initAngle + turnAngle;
    if(targAngle >= 360){
      targAngle -= 360;
    }
    rightSlow();
    while(abs(targAngle-yaw)>4){
      IMU();
    }
    Serial.print("Stop");
    Serial.println("");
    stop();
  }
}

void runCheck(){
  if(analogRead(run1) > 200){
    run = 1;
  }
  else if(analogRead(run2) > 200){
    run = 2;
  }
  else if(analogRead(run3) > 200){
    run = 3;
  }
  else{
    run = 0;
  }
}

void selfCheck(){
  delay(60);
  frontDist = hc.dist(0);
  if(frontDist < 1.00)frontDist = (frontThresh + 0.2);
  Serial.print("Front Distance: ");
  Serial.print(frontDist);
  Serial.println("");
  delay(60);
  leftDist = hc.dist(1);
  if(leftDist < 1.00)leftDist = (sideThresh + 0.2);
  Serial.print("Left Distance: ");
  Serial.print(leftDist);
  Serial.println("");
  delay(60);
  rightDist = hc.dist(2);
  if(rightDist < 1.00)rightDist = (sideThresh + 0.2);
  Serial.print("Right Distance: ");
  Serial.print(rightDist);
  Serial.println("");
  IMU();
}

void IMU() {
  if (!DMPReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)){
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = (ypr[0] * RAD_TO_DEG) + 180.0;
    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print("°");
    Serial.println("");
  }
}

void IMUInit() {
  Serial.print("Initialising IMU");
  Serial.println("");
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(633);
  mpu.setYAccelOffset(2998);
  mpu.setZAccelOffset(551);
  mpu.setXGyroOffset(37);
  mpu.setYGyroOffset(10);
  mpu.setZGyroOffset(-21);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.GetActiveOffsets();
    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  Serial.print("IMU Initialised");
  Serial.println("");
}

void end(){
  Serial.print("End run");
  Serial.println("");
  stop();
  inMaze = false;
  endRun = true;
  while(digitalRead(enable)==HIGH){
    Serial.println("Waiting for reset...");
    delay(1000);
  }
}
