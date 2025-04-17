/*
  MPU6050 Digital Motion Processor
  Adapted from MPU6050 library 'MPU6050_DMP_ImuData_for_ROS'
   
  " Digital Motion Processor or DMP performs complex motion processing tasks.
    - Fuses the data from the accel, gyro, and external magnetometer if applied, 
    compensating individual sensor noise and errors.
    - Detect specific types of motion without the need to continuously monitor 
    raw sensor data with a microcontroller.
    - Reduce workload on the microprocessor.
    - Output processed data such as quaternions, Euler angles, and gravity vectors.
    The code includes auto-calibration and offsets generator tasks. Different 
    output formats available.

    Find the full MPU6050 library documentation here:
    https://github.com/ElectronicCats/mpu6050/wiki
  "
*/



// Import Libraries
#include "I2Cdev.h"                             // Library allows for I2C communication
#include "MPU6050_6Axis_MotionApps20.h"         // Example Library from Electronic Cats that will do most of the processing
#include <Servo.h>                              // Servo Library


Servo servoLeft;                                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;                               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// 
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

// Define Constant Values
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

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


// Variables for double integration of acceleration
const int ARRAY_SIZE= 2;
float xAccArray[ARRAY_SIZE];
float xVelArray[ARRAY_SIZE];
float yAccArray[ARRAY_SIZE];
float yVelArray[ARRAY_SIZE];
float zAccArray[ARRAY_SIZE];
float zVelArray[ARRAY_SIZE];
float currentDisplacement;
unsigned long currentTime = millis();
unsigned long previousTime;

float yawAngle;


void setup() {
  MPU6050setup();
  attachServos();
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  readMPU6050loop();
  
}

void MPU6050setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
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

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(40);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(-20);
  mpu.setXAccelOffset(578);
  mpu.setYAccelOffset(2826);
  mpu.setZAccelOffset(542);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
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
    
    float XAcceleration = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    float YAcceleration = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    float ZAcceleration = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

    /* Display Euler angles in degrees */
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yawAngle = (ypr[0] * RAD_TO_DEG) + 180.0;
    // yaw Angle is the angle that will be used to accurate turning of the robot
    // Currently the data outputting is from the range of -179.99... degrees to +179.99... degrees.
    // It would be better suited for a bearing system from the range of 0 to 359.99.. degrees.
    // There may be issues when the angle reads as close to +-180 as this is not explicitly within the range.

    // The pitch and roll are not essential to this project
    // float pitchAngle = ypr[1] * RAD_TO_DEG;
    // float rollAngle = ypr[2] * RAD_TO_DEG;

    Serial.print("XAcceleration: ");
    Serial.println(XAcceleration);
    Serial.print("YAcceleration: ");
    Serial.println(YAcceleration);
    Serial.print("ZAcceleration: ");
    Serial.println(ZAcceleration);
    Serial.print("yawAngle: ");
    Serial.println(yawAngle);
    // Serial.print("pitchAngle: ");
    // Serial.println(pitchAngle);
    // Serial.print("rollAngle: ");
    // Serial.println(rollAngle);
    
    //updateArray(XAcceleration, xAccArray);
    //doubleIntegrate(xAccArray, xVelArray);
    //Serial.print("Sample Time: ");
    //Serial.println(previousTime);
    //Serial.println();
    //delay(100);
    
  }
}

// This function calculates the average velocity of two consecutive acceleration values and 
// then calculate the average displacement of two consecutive velocity values. It then 
// changes the current displacement based on the new displacement value.
// This function is based on the Trapezoidal Rule 
float doubleIntegrate(float accArray[], float veloArray[]){
  unsigned long timePeriod =  0.1;//100ms        // NEED A MORE ACCURATE WAY OF MEASURING SAMPLE TIME
  float newVelocity = 0.5 * (accArray[0] + accArray[1]) * timePeriod;
  updateArray(newVelocity, veloArray);
  float newDisplacement = 0.5 * (veloArray[0] + veloArray[1]) * timePeriod;
  currentDisplacement += newDisplacement;
  Serial.print("Current Displacement: ");
  Serial.println(currentDisplacement);
}


// This function appends a new value to an array and delete the earliest value
// This function will be used to calculate the velocity and the acceleration of two acceleration (or velocity) values
// updateArray should be called before the doubleIntegrate function and should have inputs of the new Acceleration 
// value and the corresponding array for that value
float updateArray(float newValue, float arrayValues[]){
  // Shifting values to the left
  for (int i=0; i<(ARRAY_SIZE-1); i++){
    arrayValues[i] = arrayValues[i+1];
  }
  arrayValues[ARRAY_SIZE - 1] = newValue;
}

/// From "KashCow-Group-4/basic_maneuvers_boe_bot.ino"
void attachServos(){            // Signals can be sent to Servos
  servoLeft.attach(12);         // Left signal connects to pin 12
  servoRight.attach(13);        // Right signal connects to pin 13
}
void detachServos() {           // Stops signals to Servos
  servoLeft.detach();         // Left signal disconnects to pin 12 
  servoRight.detach();        // Right signal disconnects to pin 13
}
void turnLeft(int microsecondTime) {
  servoLeft.writeMicroseconds(1300);    // Left Servo rotates clockwise
  servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
  delay(microsecondTime);               // Move time in milliseconds
}
void turnRight(int microsecondTime) {
  servoLeft.writeMicroseconds(1700);    // Left Servo rotates anti-clockwise
  servoRight.writeMicroseconds(1700);   // Right Servo rotates anti-clockwise
  delay(microsecondTime);               // Move time in milliseconds
}


/// These functions should only be used when at a T-junction or left/right turn
/// Do not use for micro-adjustments of the robot
// This function should allow you to turn 90 degrees to the Left
void turnLeft90Accurate() {    // Left is an Anti-clockwise movement
  const float angleBeforeTurn = yawAngle;
  float targetAngle = angleBeforeTurn - 90.0;
  if (targetAngle < 0) {
    targetAngle += 360.0; // Ensure that the angle stays within 0-360 degrees
  }
  float currentAngleState = yawAngle;

  while (abs(currentAngleState - targetAngle) > 1.0) { // Allow for a small margin of error
    turnLeft(1);                                        // Can change for jittering
    currentAngleState = yawAngle; // Update the current angle
    // Serial.print("Turning Left - 90deg");
  }
}

// This function should allow you to turn 90 degrees to the Left
void turnRight90Accurate(){   //Right is a Clockwise movement
  const float angleBeforeTurn = yawAngle;
  float targetAngle = angleBeforeTurn + 90.0;
  if (targetAngle > 360.0) {
    targetAngle -= 360.0; // Ensure that the angle stays within 0-360 degrees
  }
  float currentAngleState = yawAngle;

  while (abs(currentAngleState - targetAngle) > 1.0) {  // Allow for a small margin of error
    turnRight(1);
    currentAngleState = yawAngle; //Update the current angle
    //Serial.print("Turing Right - 90deg");
  }
}

// This function should allow you to accurately turn to desired angle and in the direction that you choose.
void turnAccurate(bool direction, float desiredAngle) {    // Left is an Anti-clockwise movement
  // desired angle is how much you want to turn by, in degrees
  // False is for a left turn. True is for a right turn.
  const float angleBeforeTurn = yawAngle;
  if (direction == false){
    float targetAngle = angleBeforeTurn - desiredAngle;
    if (targetAngle < 0) {
      targetAngle += 360.0; // Ensure the angle stays within 0-360 degrees
    }
    float currentAngleState = yawAngle;
    while (abs(currentAngleState - targetAngle) > 1.0) { // Allow a small margin of error
      turnLeft(1); // The robot will turn left for 1 millisecond until it has reached the desired angle turn
      currentAngleState = yawAngle; // Update the current angle
      // Serial.print("Turning Left - 90deg");
    }
  }
  else {
    float targetAngle = angleBeforeTurn + desiredAngle;
    if (targetAngle > 360.0) {
      targetAngle -= 360.0; // Ensure that the angle stays within 0-360 degrees
    }
    float currentAngleState = yawAngle;
    while (abs(currentAngleState - targetAngle) > 1.0) {  // Allow for a small margin of error
      turnRight(1);
      currentAngleState = yawAngle; //Update the current angle
      //Serial.print("Turing Right - 90deg");
    }
  }
}

