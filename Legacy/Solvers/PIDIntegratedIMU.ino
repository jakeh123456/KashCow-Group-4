// File integrates with IMU for accurate turns 
// and PID control system for forward movements

// Import Libraries
#include <Servo.h>                          // Servo Library
#include <HCSR04.h>                         // UltraSonic library for 1 trigger
#include "I2Cdev.h"                         // Library allows for I2C communication
#include "MPU6050_6Axis_MotionApps20.h"      // Library from Electronic Cats that will do most of the IMU processing

// Definitions / Numerical Constant Values
#define GRAVITY_EARTH_MS2 9.80665 
#define RAD_TO_DEG 57.295779513082320876798154814105

// Servo declarations
Servo servoLeft;                                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;                               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// HCSR-04 Ultrasonic Declarations
HCSR04 hc(3, new int[3]{9,10, 11}, 3);          //initialisation class HCSR04 (trig pin , echo pin, number of sensor) (Front,Left,Right)

double distanceFRT, distanceLHS, distanceRHS;
const double thresholdDistanceFRT = 7.5;        // The left and right are pretty good now could make the front a bit bigger
const double thresholdDistanceLHS = 13.0;
const double thresholdDistanceRHS = 13.0;

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

// Turning Control System Declarations
float Kp_Turning = 1.5; // Proportional constant  | Recommended value =  {Needs adjusting and tested}
float Ki_Turning = 0.05; // Integral constant | Recommended value = {Needs adjusting and tested}
float Kd_Turning = 0.2; // Derivative constant  | Recommended value = {Needs adjusting and tested}

float previousError_Turning = 0.0; // To store previous error for derivative term
float integral_Turning = 0.0;      // To store accumulated error for integral term

// Forward Moving Control System Declarations (Alex)
double drift = 0.0;
double error = 0.0;
double prev_error = 0.0;
double deriv = 0.0;
double correction = 0.0;
double Kp = 20;           // Over-exaggerated motion
double Kd = 18;           // A lot of kick
double dt;
double leftservoVal;
double rightservoVal;
double currentTime;
double previousTime;

// Solving Bias
// 0 - Left Bias | 1 - Right Bias
int turnBias = 1;

// MAIN SETUP
void setup() {
  Serial.begin(115200);
  attachServos();
  MPU6050Initialise(); // Ensure that prior to Initialising the MPU6050 that the offsets are updated...
}

// MAIN LOOP
void loop() {
  bool movedForward = false;
  Serial.println("LOOP START");
  // Read initial sensor values
  distanceFRT = hc.dist(0);
  delay(10);                   // DO NOT REMOVE
  distanceLHS = hc.dist(1);
  delay(10);                   // DO NOT REMOVE
  distanceRHS = hc.dist(2);
  delay(10);                   // DO NOT REMOVE

  // While loop to move forward while the front distance is greater than the threshold
  while (distanceFRT > thresholdDistanceFRT) {
    if (!movedForward){
      servoRight.write(0);   // Right Servo rotates clockwise
      delay(20);
      servoLeft.write(180);    // Left Servo rotates anti-clockwise
      delay(20);
    }
    distanceFRT = hc.dist(0);
    delay(20);
    //Serial.print("DistanceFRT: ");
    //Serial.println(distanceFRT);

    
    // -----------------PD Ctrl START-----------------
    // Fetch Side sensor vals
    distanceLHS = hc.dist(1);
    delay(20);
    distanceRHS = hc.dist(2);
    delay(20);
    
    // PID calculations Start
    correction = PD_correction(distanceLHS, distanceRHS);

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
  stopMove();
  Serial.println("EXITED WHILE LOOP ");
  delay(1000);                               // DEBUG: Delay just to see visually 

  if (!movedForward) {
    Serial.print("Entered Alternative Logic ");
    stopMove();
    distanceLHS = hc.dist(1);
    delay(10);                   // DO NOT REMOVE
    distanceRHS = hc.dist(2);
    delay(10);                   // DO NOT REMOVE

    // Logic to determine the clear path
    if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS <= thresholdDistanceRHS)) { // works well after threshold change
      Serial.print("LOGIC : Turning Left");
      turnAccurateControl(false, 90.0); // Turning Left (false) by 90 deg
    }else if ((distanceRHS > thresholdDistanceRHS) && (distanceLHS <= thresholdDistanceLHS)) {
      Serial.print("LOGIC : Turning Right");
      turnAccurateControl(true, 90.0); // Turning Right (true) by 90 deg
    }else if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS > thresholdDistanceRHS)) {
      if (turnBias == 0) {
        Serial.print("Junction Detected, turning left");
        turnAccurateControl(false, 90.0); // Turning Left (false) by 90 deg
      } else if (turnBias == 1) {
        Serial.print("Junction Detected, turning right");
        turnAccurateControl(true,90.0); // Turning Right (true) by 90 deg
      }
      Serial.print(" LHS: ");
      Serial.print(distanceLHS);
      Serial.print(", RHS: ");
      Serial.println(distanceRHS);
    }else {                                                                 // DEBUG: Print sensor values if none of the conditions to see problem (DEAD END CODE GO HERE)
        // Print sensor values if none of the conditions are met
        turnAccurateControl(true,180.0);
        Serial.print("No clear path detected. Sensor values - FRT: ");
        Serial.print(distanceFRT);
        Serial.print(", LHS: ");
        Serial.print(distanceLHS);
        Serial.print(", RHS: ");
        Serial.println(distanceRHS);
    }
  }      
}

double PD_correction(double distanceLHS, double distanceRHS) {
  currentTime = millis();
  dt = (double)(currentTime - previousTime);

  // Drift is the error signal
  drift = distanceLHS - distanceRHS;
  error = drift;
  deriv = (error - prev_error) / dt;
  correction =  (Kd * deriv) + (Kp * error);   // This will determine the servo velocities

  // Store past values
  prev_error = error;
  previousTime = currentTime;

  // Debug statement, comment it to hide
  //Serial.print(drift); Serial.print(", "); Serial.print(leftservoVal); Serial.print(", "); Serial.println(rightservoVal);
  return correction;
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
  mpu.setXAccelOffset(633);
  mpu.setYAccelOffset(2998);
  mpu.setZAccelOffset(551);
  mpu.setXGyroOffset(37);
  mpu.setYGyroOffset(10);
  mpu.setZGyroOffset(-21);
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

float readMPU6050loop() {
  if (!DMPReady) return;
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    /* Display Euler angles in degrees */
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yawAngle = (ypr[0] * RAD_TO_DEG) + 180.0;
    // yaw Angle is the angle that will be used to accurate turning of the robot
    // Currently the data outputting is from the range of -179.99... degrees to +179.99... degrees.
    // It would be better suited for a bearing system from the range of 0 to 359.99.. degrees.
    // There may be issues when the angle reads as close to +-180 as this is not explicitly within the range.

    Serial.print("yawAngle: ");
    Serial.println(yawAngle);
    delay(100);
    return yawAngle;
    
  }
}

// This function should allow you to accurately turn to desired angle and in the direction that you choose.
void turnAccurate(bool direction, float desiredAngle) {    // Left is an Anti-clockwise movement
  // desired angle is how much you want to turn by, in degrees
  // False is for a left turn. True is for a right turn.
  readMPU6050loop();
  delay(50);
  readMPU6050loop();
  const float angleBeforeTurn = yawAngle;
  if (direction == false){
    float targetAngle = angleBeforeTurn - desiredAngle;
    if (targetAngle < 0) {
      targetAngle += 360.0; // Ensure the angle stays within 0-360 degrees
    }
    float currentAngleState = yawAngle;
    while (abs(currentAngleState - targetAngle) > 1.0) { // Allow a small margin of error
      turnLeft(100); // The robot will turn turn for 1 microsecond until it has reached the desired angle turn
      readMPU6050loop();
      delay(50);
      currentAngleState = yawAngle; // Update the current angle
      Serial.print("Turning Left - ___deg");
      Serial.print("Target Angle: ");
      Serial.print(targetAngle);
    }
  }
  else {
    float targetAngle = angleBeforeTurn + desiredAngle;
    if (targetAngle > 360.0) {
      targetAngle -= 360.0; // Ensure that the angle stays within 0-360 degrees
    }
    float currentAngleState = yawAngle;
    while (abs(currentAngleState - targetAngle) > 1.0) {  // Allow for a small margin of error
      turnRight(100);
      readMPU6050loop();
      delay(50);
      currentAngleState = yawAngle; //Update the current angle
      Serial.print("Turing Right - ___deg");
      Serial.print("Target Angle: ");
      Serial.print(targetAngle);
    }
  }
}

void turnAccurateControl(bool direction, float desiredAngle){
  // left turn is false, right turn is true.
  readMPU6050loop();
  delay(50);
  readMPU6050loop();
  float angleBeforeTurn = yawAngle;
  float targetAngle;

  if (direction == false) {
    targetAngle = angleBeforeTurn - desiredAngle;
    if (targetAngle < 0.0) {
      targetAngle += 360.0;
    }
  }
  else {
    targetAngle = angleBeforeTurn + desiredAngle;
    if (targetAngle > 360.0) {
      targetAngle -= 360.0;
    }
  }

  float currentAngleState = yawAngle;
  unsigned long previousMillis = millis();
  unsigned long currentMillis;

  if (direction == false) {
    while (abs(currentAngleState - targetAngle) > 1.0) {
      currentMillis = millis();
      // Only update the IMU reading every 50ms
      if ((currentMillis - previousMillis) >= 50) {
        previousMillis = currentMillis;
        readMPU6050loop();
        currentAngleState = yawAngle;
      }

      float error = targetAngle - currentAngleState;
      if (error < -180.0) {
        error += 360.0;
      } else if (error > 180.0) {
        error -= 360.0;
      }
      // For left turn, invert the error sign
      error = -error;

      float proportional = Kp_Turning * error;
      integral_Turning += error;
      float integralTerm = Ki_Turning * integral_Turning;
      float derivative = error - previousError_Turning;
      float derivativeTerm = Kd_Turning * derivative;
      float controlSignal = proportional + integralTerm + derivativeTerm;
      controlSignal = constrain(controlSignal,-255,255);

      if (controlSignal > 0){
        turnLeft(abs(controlSignal));
      } else if (controlSignal < 0){
        turnRight(abs(controlSignal));
      } else {turnRight(0);}
      previousError_Turning = error;

      // Debugging output (optional)
      Serial.print("Current Angle: ");
      Serial.print(currentAngleState);
      Serial.print(" Target Angle: ");
      Serial.print(targetAngle);
      Serial.print(" Error: ");
      Serial.print(error);
      Serial.print(" Control Signal: ");
      Serial.println(controlSignal);
    }
    turnRight(0);
    Serial.println("Turn complete...");
  }
  else {
    while (abs(currentAngleState - targetAngle) > 1.0) {  // Allow small margin of error
      currentMillis = millis();
      
      // Only update the sensor reading every 50ms
      if ((currentMillis - previousMillis) >= 50) {
        previousMillis = currentMillis;
        readMPU6050loop();
        currentAngleState = yawAngle;  // Update the current angle
      }

      // Calculate the error (difference between target and current angle)
      float error = targetAngle - currentAngleState;

      // Handle angle wrap-around (target angle is in [0, 360] range)
      if (error < -180) {
        error += 360;  // Correct for negative angles
      } else if (error > 180) {
        error -= 360;  // Correct for angles greater than 180
      }

      // Proportional term (P)
      float proportional = Kp_Turning * error;

      // Integral term (I)
      integral_Turning += error;
      float integralTerm = Ki_Turning * integral_Turning;

      // Derivative term (D)
      float derivative = error - previousError_Turning;
      float derivativeTerm = Kd_Turning * derivative;

      // Calculate the control signal (the turning speed)
      float controlSignal = proportional + integralTerm + derivativeTerm;

      // Ensure controlSignal is within a reasonable range (for motor control)
      controlSignal = constrain(controlSignal, -255, 255);

      // Apply the control signal to the motor (turn right)
      if (controlSignal > 0) {
        turnRight(abs(controlSignal)); // Turn right with positive control signal
      } else if (controlSignal < 0){
        turnLeft(abs(controlSignal)); // Turn left with negative control signal
      } else {turnRight(0);}// Stop turning if control signal is negative or zero

      // Save the error for the next iteration (for the derivative term)
      previousError_Turning = error;

      // Debugging output (optional)
      Serial.print("Current Angle: ");
      Serial.print(currentAngleState);
      Serial.print(" Target Angle: ");
      Serial.print(targetAngle);
      Serial.print(" Error: ");
      Serial.print(error);
      Serial.print(" Control Signal: ");
      Serial.println(controlSignal);
    }
    // Stop the robot after it has turned the desired angle
    turnRight(0);
    Serial.println("Turn complete!");
  }  
}

/*Basic Robot Maneuvers*/
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
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}