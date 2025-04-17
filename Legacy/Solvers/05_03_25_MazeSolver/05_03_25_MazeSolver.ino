#include <Servo.h>              // Servo Library
#include <HCSR04.h>             // UltraSonic library for 1 trigger
#include "I2Cdev.h"                         // Library allows for I2C communication
#include "MPU6050_6Axis_MotionApps20.h"      // Library from Electronic Cats that will do most of the IMU processing

// Definitions / Numerical Constant Values
#define GRAVITY_EARTH_MS2 9.80665 
#define RAD_TO_DEG 57.295779513082320876798154814105


Servo servoLeft;                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// -----------------CAUTION-----------------
// TRIAL VERSION FOR PID STABILISER
// -----------------------------------------

// Distance Parameters & Initialisation
HCSR04 hc(3, new int[3]{9,10, 11}, 3); //initialisation class HCSR04 (trig pin , echo pin, number of sensor) (Front,Left,Right)

double distanceFRT, distanceLHS, distanceRHS;

const double thresholdDistanceFRT = 24.0;        // The left and right are pretty good now could make the front a bit bigger
const double secondthresholdDistanceFRT = 6;
const double thresholdDistanceLHS = 13.0;
const double thresholdDistanceRHS = 13.0;

const double wallRef = 6.0;

// Solving Bias
// 0 - Left Bias
// 1 - Right Bias
int turnBias = 1;

// Signal Pre-processing
double filteredDistanceFRT = 0.0;
double filteredDistanceLHS = 0.0;
double filteredDistanceRHS = 0.0;

const double default_smoothingFactor = 0.3;

// PID Params
double drift = 0.0;
double error = 0.0;
double prev_error = 0.0;
double deriv = 0.0;
double integ = 0;
double correction = 0.0;
double currentTime;
double dt;

double kp_internal;
double kd_internal;

double Kp = 12;           // Over-exaggerated motion
double Ki = 0.005;           // A lot of kick
double Kd = 0.5;            // TODO find optimal tuning.

// Turning Control System Declarations
float KpYaw = 0.16; // Proportional constant  | Recommended value =  {Needs adjusting and tested}
float KiYaw = 0.009; // Integral constant | Recommended value = {Needs adjusting and tested}
float KdYaw = 0.04; // Derivative constant  | Recommended value = {Needs adjusting and tested}

float previousError_Turning = 0.0; // To store previous error for derivative term
float integral_Turning = 0.0;      // To store accumulated error for integral term

double yawError = 0.0;
double previousTime = 0.0;
double prevYawError = 0.0;

double y_i_Max = 1.28;
double y_i_Min = -1.28;

// MPU6050 IMU Declarations
MPU6050 mpu;
    /*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
float yawAngle;

double leftservoVal;
double rightservoVal;

void setup() {
  Serial.begin(9600);           // Sets baud rate.
  attachServos();
  servoLeft.write(90);
  servoRight.write(90);
  MPU6050Initialise();
}

void loop() {
    // Reset the movedForward flag at the beginning of each cycle
    bool movedForward = false;
    Serial.println("LOOP START");

    // Read initial sensor values
    distanceFRT = hc.dist(0);
    delay(20);                   // DO NOT REMOVE
    distanceLHS = hc.dist(1);
    delay(20);                   // DO NOT REMOVE
    distanceRHS = hc.dist(2);
    delay(20);                   // DO NOT REMOVE


    // While loop to move forward while the front distance is greater than the threshold
    while (distanceFRT > thresholdDistanceFRT) {
      if (!movedForward){
       servoRight.write(0);   // Right Servo rotates clockwise
       servoLeft.write(180);    // Left Servo rotates anti-clockwise
       
      }
      distanceFRT = hc.dist(0);
      delay(20);
      //Serial.print("DistanceFRT: ");
      //Serial.println(distanceFRT);

      // -----------------PD Ctrl START-----------------
      // Fetch Side sensor vals
      distanceLHS = hc.dist(1);
      filteredDistanceLHS = EWMA_Filter(distanceLHS, filteredDistanceLHS, default_smoothingFactor);      // Filtering out noise
      delay(20);
      distanceRHS = hc.dist(2);
      filteredDistanceRHS = EWMA_Filter(distanceRHS, filteredDistanceRHS, default_smoothingFactor);
      delay(20);
      
      // PID calculations Start
      correction = PID_correction(distanceLHS, distanceRHS);

      // Impose new servo velocities
      leftservoVal = constrain(180 - correction, 0, 180);
      rightservoVal = constrain(0 - correction, 0, 180);

      servoLeft.write(leftservoVal);
      servoRight.write(rightservoVal);
      
      // Debug statement, comment it to hide
      //Serial.print(drift); Serial.print(", "); Serial.print(leftservoVal); Serial.print(", "); Serial.println(rightservoVal);
      // -----------------PD Ctrl END-----------------


      movedForward = true;
    }
    
    while (distanceFRT > secondthresholdDistanceFRT ){
        distanceFRT = hc.dist(0);
        delay(20);
        servoLeft.write(180);
        servoRight.write(0);
      }
        stopMove();
        Serial.println("EXITED WHILE LOOP ");
        delay(1000);                               // DEBUG: Delay just to see visually 
        
    // If the while loop was not executed, handle the alternative logic
    if (!movedForward) {
        Serial.println("Entered Alternative Logic ");
        stopMove();
        distanceLHS = hc.dist(1);
        delay(10);                   // DO NOT REMOVE
        distanceRHS = hc.dist(2);
        delay(10);                   // DO NOT REMOVE

        // Logic to determine the clear path
        if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS <= thresholdDistanceRHS)) { // works well after threshold change
            Serial.print("LOGIC : Turning Left");
            yawControl(-90);           // Turn Left if it's clear for battery(600) and usb(800)
        } else if ((distanceRHS > thresholdDistanceRHS) && (distanceLHS <= thresholdDistanceLHS)) {
            Serial.print("LOGIC : Turning Right");
            yawControl(90);
        } else if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS > thresholdDistanceRHS)) {
            if (turnBias == 0) {
              Serial.print("Junction Detected, turning left");
              yawControl(-90);
            } else if (turnBias == 1) {
              Serial.print("Junction Detected, turning right");
              yawControl(90);
            }
            Serial.print(" LHS: ");
            Serial.print(distanceLHS);
            Serial.print(", RHS: ");
            Serial.println(distanceRHS);
        } else {                                                                 // DEBUG: Print sensor values if none of the conditions to see problem (DEAD END CODE GO HERE)
            // Print sensor values if none of the conditions are met
            Serial.print("No clear path detected. Sensor values - FRT: ");
            Serial.print(distanceFRT);
            Serial.print(", LHS: ");
            Serial.print(distanceLHS);
            Serial.print(", RHS: ");
            Serial.println(distanceRHS);
            yawControl(-90);
            yawControl(-90);  // for some reason cant do 180 at once.
        }
    }
}


void MPU6050Initialise() {// The Serial prints can be commented out later...
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }
  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  /* Supply your gyro offsets here, scaled for min sensitivity */ // INPUT THE OFFSETS HERE
  mpu.setXAccelOffset(618);
  mpu.setYAccelOffset(2898);
  mpu.setZAccelOffset(537);
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(-20);
  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    mpu.GetActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
}

double readMPU6050loop() {
  if (!DMPReady) {
    Serial.println("Shit got stuck here 1");
    return;
  }
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    //Serial.println("Shit got stuck here 2");
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    //Serial.println("Shit got stuck here 3");

    mpu.dmpGetGravity(&gravity, &q);
    //Serial.println("Shit got stuck here 4");
    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    /* Display Euler angles in degrees */
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yawAngle = (ypr[0] * RAD_TO_DEG); //+ 180.0;
    // yaw Angle is the angle that will be used to accurate turning of the robot
    // Currently the data outputting is from the range of -179.99... degrees to +179.99... degrees.
    // It would be better suited for a bearing system from the range of 0 to 359.99.. degrees.
    // There may be issues when the angle reads as close to +-180 as this is not explicitly within the range.

    //Serial.print("yawAngle: ");
    //Serial.println(yawAngle);
    delay(20);
    return (yawAngle);       // Done so just to match the derived dynamic's +ve and -ve agreements.
    
    
  }
}

double yawControl(double desired_angle) {
  bool reached = false;
  readMPU6050loop(); 
  Serial.print("Yaw Angle: "); Serial.println(yawAngle);
  Serial.print("Target Angle: "); Serial.println(desired_angle);
  readMPU6050loop();
  double currentAngle = yawAngle;

  double yawIntegral = 0.0;

  // TODO add a breakout statement based on time so that it does not execute forever.
  while (reached == false) {
    double currentTime = millis();
    double dt = (currentTime - previousTime) / 1000.0;

    readMPU6050loop();
    delay(20); // ADDED THIS

    double yawError = (currentAngle + desired_angle) - yawAngle;
    
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    
    double yawProportional = KpYaw * yawError;
    integral_Turning += yawError * dt; // Accumulate integral
    double yawIntegral = KiYaw * integral_Turning;

    // anti-windup
    /*
    if (yawIntegral>=y_i_Max) yawIntegral = y_i_Max;
    else if (yawIntegral<=y_i_Min) yawIntegral = y_i_Min;
    */
    double yawDerivative = KdYaw * (yawError - prevYawError) / dt;
    // suggestion: Base derivative on yawAngle itself to avoid kick

    // Calculate control signal
    double controlSignal = yawProportional + yawIntegral + yawDerivative;

    prevYawError = yawError;
    previousTime = currentTime;

    // Apply control to servos for turning
    double leftSpeed = constrain(90 + controlSignal, 0, 180);
    double rightSpeed = constrain(90 + controlSignal, 0, 180);

    servoLeft.write(leftSpeed);
    servoRight.write(rightSpeed);

    Serial.print("yaw Angle: "); Serial.print(yawAngle);
    Serial.print(" yaw Error: "); Serial.println(yawError); //Serial.print(","); Serial.print(" | Integral: "); Serial.println(yawIntegral);
    /*
    Serial.print(yawError);
    Serial.print(" | Integral: ");
    */
    //Serial.println(controlSignal);
    //Serial.print(" | Control Signal: "); Serial.println(controlSignal);

    // TODO: Check if necessary
    // Exit condition if within desired range
    if (abs(yawError) < 0.7) {
      reached = true;
      Serial.println("Reached == True");
      servoLeft.write(90);  // Stop servos, precautionary measure
      servoRight.write(90);
    }
  }

  Serial.print("Executed turn");
}

void attachServos() {
  servoLeft.attach(12);         // Left signal connects to pin 12
  servoRight.attach(13);        // Right signal connects to pin 13
}
void detachServos() {           // Stops signals to Servos
  servoLeft.detach();         // Left signal disconnects to pin 12 
  servoRight.detach();        // Right signal disconnects to pin 13
}

void forwardMove(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    servoLeft.writeMicroseconds(1700);    // Left Servo rotates anti-clockwise
    servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
  }
  stopMove();
}

void backwardMove(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    servoLeft.writeMicroseconds(1300);    // Left Servo rotates clockwise
    servoRight.writeMicroseconds(1700);   // Right Servo rotates anti-clockwise
  }
  stopMove();
}

void turnLeft(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    servoLeft.writeMicroseconds(1300);    // Left Servo rotates clockwise
    servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
  }
  stopMove();
}

void turnRight(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    servoLeft.writeMicroseconds(1700);    // Left Servo rotates anti-clockwise
    servoRight.writeMicroseconds(1700);   // Right Servo rotates anti-clockwise
  }
  stopMove();
}


void stopMove() {
  servoLeft.write(90);
  servoRight.write(90);
}

double EWMA_Filter(double newVal, double prevFilteredVal, double smoothingFactor){
  return (smoothingFactor * newVal) + ((1-smoothingFactor) * prevFilteredVal);
}
double PID_correction(double distanceLHS, double distanceRHS) {
  currentTime = millis();
  dt = (double)(currentTime - previousTime) / 1000;
  
  // Drift is the error signal
  if ((distanceLHS < thresholdDistanceLHS) && (distanceRHS < thresholdDistanceRHS)) { // REQUIRES ADJUSTMENTS IF NEEDED
    // Both walls are at expected distances, use both for localisation
    Serial.println("Both walls are detectable");
      drift = distanceLHS - distanceRHS;
      kp_internal = Kp;
      kd_internal = Kd;
    
  } else if ((distanceRHS > thresholdDistanceRHS) && (distanceLHS <= thresholdDistanceRHS)) { 
    // Left wall following:
    Serial.println("Left wall is detectable");
      drift = distanceLHS - wallRef;
      //prev_error = 0.0;
      kp_internal = Kp*1.5;
      kd_internal = Kd*1.5;

  } else if ((distanceLHS > thresholdDistanceRHS) && (distanceRHS <= thresholdDistanceRHS)) {
    // Right Wall following:
    Serial.println("Right wall is detectable");
      drift = wallRef - distanceRHS;
      //prev_error = 0.0;
      kp_internal = Kp*1.5;
      kd_internal = Kd*1.5;

  } else {
    Serial.println("distanceLHS & RHS are not accepted... >:( ");
  }

  //kp_internal = Kp;
  //kd_internal = Kd;
  //drift = distanceLHS - distanceRHS;
  error = drift;
  deriv = (error - prev_error) / dt;
  if (abs(error) < 0.7) {
    integ = 0;
  } else {
    integ += error * dt;
  }
  correction =  (kd_internal * deriv) + (kp_internal * error) + (Ki * integ);   // This will determine the servo velocities

  // Store past values
  prev_error = error;
  previousTime = currentTime;

  // Debug statement, comment it to hide
  //Serial.print(drift); Serial.print(", "); Serial.print(correction); Serial.print(", "); Serial.print(deriv); Serial.print(", "); Serial.println(integ); 
  //Serial.print(drift); Serial.print(", "); Serial.print(leftservoVal); Serial.print(", "); Serial.println(rightservoVal);
  Serial.print(drift); Serial.print(", "); Serial.print(distanceLHS); Serial.print(", "); Serial.println(distanceRHS);
  return correction * 0.7;
}






// Legacy movement
  void pivotForwardLeft(int microsecondTime) {
    servoLeft.writeMicroseconds(1500);    // Left Servo stationary
    servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
    delay(microsecondTime);               // Move time in microseconds
  }

  void pivotForwardRight(int microsecondTime) {
    servoLeft.writeMicroseconds(1700);    // Left Servo anti-clockwise
    servoRight.writeMicroseconds(1500);   // Right Servo stationary
    delay(microsecondTime);               // Move time in microseconds
  }

  void pivotBackwardLeft(int microsecondTime) {
    servoLeft.writeMicroseconds(1500);    // Left Servo stationary
    servoRight.writeMicroseconds(1700);   // Right Servo anti-clockwise
    delay(microsecondTime);
  }

  void pivotBackwardRight(int microsecondTime) {
    servoLeft.writeMicroseconds(1300);    // Left Servo clockwise
    servoRight.writeMicroseconds(1500);   // Right Servo stationary
    delay(microsecondTime);
  }

  void forwardMoveRamp(int microsecondTime) {
    for (int speed = 0; speed <= 100; speed += 5) {   // Increase speed to full speed
      servoLeft.writeMicroseconds(1500 + speed);
      servoRight.writeMicroseconds(1500 - speed);
      delay(microsecondTime);                         // Incremental time
    }
    delay(1000); // full speed for 1 second 
  }

  void backwardMoveRamp(int microsecondTime) {
    for (int speed = 0; speed <= 100; speed += 5) {   // Increase speed to full speed
      servoLeft.writeMicroseconds(1500 + speed);
      servoRight.writeMicroseconds(1500 - speed);
      delay(microsecondTime);                         // Incremental time
    }
    delay(1000); // full speed for 1 second
  }
