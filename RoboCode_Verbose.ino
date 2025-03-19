//===================================================//
//               Part B Robotics Project             //
//                  Company D Group 4                //
//        Maze Solver - Verbose (Serial enabled)     //
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

//Define world-frame constants
#define GRAVITY_EARTH_MS2 9.80665 
#define RAD_TO_DEG 57.295779513082320876798154814105

//Library Setup
MKL_HCSR04 hc(trig, new int[3]{usf,usl,usr}, 3);
MPU6050 mpu;
Servo lw; //Left wheel servo
Servo rw; //Right wheel servo

//Define variables
int deadend = 1;
int frontThresh = 12; //7 for Mac
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
float ypr[3];
uint8_t FIFOBuffer[64];
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
volatile int run = 1;

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

  //Setup serial connection
  Serial.begin(9600);

  //Confirm setup complete
  digitalWrite(initLED, HIGH);
  Serial.println("");
  Serial.print("Ready");
  Serial.println("");
  Serial.println("*------------------------------------------*");
  Serial.println("");

}

void loop(){
  if(digitalRead(enable) == HIGH && endRun == false){
    if(inMaze == false){
      Serial.print("Entering maze");
      Serial.println("");
      forward();
      while(leftDist > 10 && rightDist > 10){
        distCheck();
      }
      inMaze = true;
      Serial.print("Entered maze");
      Serial.println("");
    }
    distCheck();
    forward();
    while(frontDist > frontThresh){
      Serial.print("Proceed in maze");
      Serial.println("");
      distCheck();
      if(rightDist <= sideThresh)slightLeft();
      if(leftDist <= sideThresh)slightRight();
      if(frontDist > 30 &&leftDist > uThresh && rightDist > uThresh){
        delay(100);
        if(frontDist > 30 &&leftDist > uThresh && rightDist > uThresh){
          Serial.print("Exiting maze...");
          Serial.println("");
          delay(1000);
          end();
        }
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
        if(leftDist < rightDist) turn(false, 180);
        else turn(true, 180);
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
    Serial.print("endRun = false");
    Serial.println("");
    endRun = false;
    inMaze = false;
    stop();
  }

  else{
    stop();
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
  Serial.print("Left adjust");
  Serial.println("");
  lw.detach();
  delay(200); //200 on Mac
  lw.attach(svl);
}

void slightRight(){
  Serial.print("Right adjust");
  Serial.println("");
  rw.detach();
  delay(200); //200 on Mac
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
    left();
    while(abs(targAngle-yaw)>35){
      IMU();
    }
    leftSlow();
    while(abs(targAngle-yaw)>4){
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
    right();
    while(abs(targAngle-yaw)>35){
      IMU();
    }
    rightSlow();
    while(abs(targAngle-yaw)>3){
      IMU();
    }
    Serial.print("Stop");
    Serial.println("");
    stop();
  }
}

void runSelISR(){
  if(run == 1){run = 2; digitalWrite(run1LED, LOW); digitalWrite(run2LED, HIGH);}
  else if(run == 2){run = 3; digitalWrite(run2LED, LOW); digitalWrite(run3LED, HIGH);}
  else {run = 1; digitalWrite(run3LED, LOW); digitalWrite(run1LED, HIGH);}
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
  Serial.print("Front Distance: ");
  Serial.print(frontDist);
  Serial.println("");
  Serial.print("Left Distance: ");
  Serial.print(leftDist);
  Serial.println("");
  Serial.print("Right Distance: ");
  Serial.print(rightDist);
  Serial.println("");
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
  IMU();
}

void end(){
  Serial.print("End run");
  Serial.println("");
  stop();
  inMaze = false;
  endRun = true;
  frontDist = 1.00;
  while(digitalRead(enable)==HIGH){
    Serial.println("Waiting for reset...");
    delay(1000);
  }
}