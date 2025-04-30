//===================================================//
//               Part B Robotics Project             //
//                  Company D Group 4                //
//                     Maze Solver                   //
//                   Jake Hallworth                  //
//                       V 2.1.8                     //
//===================================================//

//Include Libraries
#include <Servo.h>
#include <MKL_HCSR04.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Define pins
#define runSel 2      //Run select switch
#define run2LED 3     //Indicator LED for run 2
#define run3LED 4     //Indicator LED for run 3
#define enableLED 5   //Indicator LED for enable
#define initLED 6     //Indicator LED for initialisation
#define softSerial 7  //ESP Comms
#define trig 8        //Ultrasonic trigger
#define usf 9         //Ultrasonic front
#define usl 10        //Ultrasonic left
#define usr 11        //Ultrasonic right
#define svl 12        //Servo left
#define svr 13        //Servo right
#define run1LED A0    //Indicator LED for run 1
#define enable A1     //Enable switch
/*Other Pins 
0  serialRX    //USB Serial receive (Programming)
1  serialTX    //USB Serial transmit (Debugging)
A2 UNASSIGNED  //Unassigned
A3 UNASSIGNED  //Unassigned
A4 I2C SDA     //IMU Data Line (I2C)
A5 I2C SCL     //IMU Clock Line (I2C)*/

//Define world-frame constants - Used for IMU calulations
#define GRAVITY_EARTH_MS2 9.80665 
#define RAD_TO_DEG 57.295779513082320876798154814105

//Library Setup
MKL_HCSR04 hc(trig, new int[3]{usf,usl,usr}, 3); //Instantiate Ultrasonic sensor objects
MPU6050 mpu;                                     //Instantiate IMU
Servo lw;                                        //Declare left wheel servo
Servo rw;                                        //Declare right wheel servo

//Define variables
int deadend = 1;         //Stores which run a deadend is detected on
int frontThresh = 11;    //Threshold for minimum allowable distance to an object infront of the robot
int sideThresh = 4;      //Threshold for minimum allowable distance to an object on either side of the robot
int uThresh = 13;        //Threshold for detecting u-Turns
bool inMaze = false;     //Flag that indicates whether the robot is inside or outside of the maze
bool DMPReady = false;   //Flag that indicates the IMU being properly initialised and callibrated
bool endRun = false;     //Flag that indicates the current run has ended
float frontDist = 20.00; //Stores the last measured distance from the front ultrasonic sensor
float leftDist = 20.00;  //Stores the last measured distance from the left ultrasonic sensor
float rightDist = 20.00; //Stores the last measured distance from the right ultrasonic sensor
float yaw;               //Stores the current yaw angle relative to the starting position which is set to 180°
float targAngle;         //Stores the target angle to complete a turn
float ypr[3];            //Container for yaw pitch and roll
uint8_t FIFOBuffer[64];  //Buffer container for the IMU
uint8_t MPUIntStatus;    //Stores the IMU status
uint8_t devStatus;       //Container used in the IMU library
uint16_t packetSize;     //Container used in the IMU library
volatile int run = 1;    //Flag that indicates the selected run

//Define containers
Quaternion q;
VectorInt16 aa;
VectorInt16 gg;
VectorInt16 aaWorld;
VectorInt16 ggWorld;
VectorFloat gravity;

void setup(){

  //Set pin modes (not necessary - just to make sure)
  pinMode(run1LED, OUTPUT);
  pinMode(run2LED, OUTPUT);
  pinMode(run3LED, OUTPUT);
  pinMode(initLED, OUTPUT);
  pinMode(enableLED, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(svl, OUTPUT);
  pinMode(svr, OUTPUT);
  pinMode(runSel, INPUT);
  pinMode(usf, INPUT);
  pinMode(usl, INPUT);
  pinMode(usr, INPUT);
  pinMode(enable, INPUT);

  //Set outputs
  digitalWrite(run1LED , HIGH);
  digitalWrite(run2LED , LOW);
  digitalWrite(run3LED , LOW);
  digitalWrite(initLED , LOW);
  digitalWrite(enableLED , LOW);
  digitalWrite(trig , LOW);
  digitalWrite(svl , LOW);
  digitalWrite(svr , LOW);

  //Enable interrupts for run switching and initialise IMU
  interrupts();
  attachInterrupt(digitalPinToInterrupt(runSel), runSelISR, RISING);
  IMUInit();

  //Confirm setup complete
  digitalWrite(initLED, HIGH);
}

void loop(){
  if(digitalRead(enable) == HIGH && endRun == false){
    digitalWrite(enableLED, HIGH);
    if(inMaze == false){
      forward();
      while(leftDist > 10 && rightDist > 10){
        distCheck();
      }
      inMaze = true;
    }
    distCheck();
    forward();
    while(frontDist > frontThresh && endRun == false){
      if(digitalRead(enable) == HIGH){
        distCheck();
        if(rightDist <= sideThresh)slightLeft();
        if(leftDist <= sideThresh)slightRight();
        if(leftDist > uThresh && rightDist > uThresh && (frontDist > 24 || frontDist == 0)){
         delay(100);
          if((frontDist > 30 || frontDist == 4.1) && leftDist > uThresh && rightDist > uThresh){
            delay(400);
            end();
          }
        }
      }
      else{
        endRun = true;
        end();
      }
    }
    if(endRun == false){
      stop();

      if(leftDist < uThresh && rightDist < uThresh){
        deadend = run;
        if(leftDist < rightDist) turn(false, 180);
        else turn(true, 180);
      }
      else if(leftDist < uThresh || rightDist < uThresh){
        if(leftDist < rightDist) turn(true, 90);
        else turn(false, 90);
      }
      else{
        if(run == 1) turn(false, 90);
        else if(run == 2) turn(true, 90);     
        else if(deadend == 1) turn(true, 90);     
        else if(deadend == 2) turn(false, 90);
      }
    }
  }
  
  if(digitalRead(enable) == LOW && endRun == true){
    digitalWrite(enableLED, LOW);
    endRun = false;
    inMaze = false;
    stop();
  }

  else{
    digitalWrite(enableLED, LOW);
    stop();
  }
}

void stop(){
  lw.detach();
  rw.detach();
}

void forward(){
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
  lw.writeMicroseconds(1455);
  rw.writeMicroseconds(1455);
  rw.attach(svr);
  lw.attach(svl);
}

void rightSlow(){
  lw.writeMicroseconds(1488);
  rw.writeMicroseconds(1488);
  rw.attach(svr);
  lw.attach(svl);
}

void slightLeft(){
  lw.detach();
  delay(200); //200 on Mac
  lw.attach(svl);
}

void slightRight(){
  rw.detach();
  delay(200); //200 on Mac
  rw.attach(svr);
}

void turn(bool dir, float turnAngle){
  IMU();
  const float initAngle = yaw;
  if(dir == false){
    targAngle = initAngle - turnAngle;
    if(targAngle <= 0){
      targAngle += 360;
    }
    left();
    while(abs(targAngle-yaw)>35){
      IMU();
    }
    leftSlow();
    while(abs(targAngle-yaw)>4){
      IMU();
    }
    stop();
  }
  else{
    targAngle = initAngle + turnAngle;
    if(targAngle >= 360){
      targAngle -= 360;
    }
    right();
    while(abs(targAngle-yaw)>35){
      IMU();
    }
    rightSlow();
    while(abs(targAngle-yaw)>3){
      IMU();
    }
    stop();
  }
}

void runSelISR(){
  if(run == 1){run = 2; digitalWrite(run1LED, LOW); digitalWrite(run2LED, HIGH);}
  else if(run == 2){run = 3; digitalWrite(run2LED, LOW); digitalWrite(run3LED, HIGH);}
  else {run = 1; digitalWrite(run3LED, LOW); digitalWrite(run1LED, HIGH);}
  delay(200);
}

void distCheck(){
  delay(60);
  frontDist = hc.dist(0);
  delay(60);
  leftDist = hc.dist(1);
  delay(60);
  rightDist = hc.dist(2);
  if(frontDist < 0.9)frontDist = (frontThresh + 0.1);
  if(leftDist < 0.9){if(rightDist < 14){leftDist = 13.5 - rightDist;}else {leftDist = (sideThresh + 0.1);}}
  if(rightDist < 0.9){if(leftDist < 14){rightDist = 13.5 - rightDist;}else {rightDist = (sideThresh + 0.1);}}
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
  }
}

void IMUInit() {
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
  IMU();
}

void end(){
  stop();
  inMaze = false;
  endRun = true;
  frontDist = 1.00;
  while(digitalRead(enable)==HIGH){}
}
