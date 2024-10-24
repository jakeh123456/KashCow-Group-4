#include <Servo.h>              // Servo Library

Servo servoLeft;                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// Ultrasonic Initialisation
// -----------------CAUTION-----------------
// Current software is made for SEEED studio 
// ultrasonic sensor, the same digital pin 
// operates both emitted and echoed signal 
// when calculating distance. 
// -----------------------------------------
const int uSonicSIG1 = 2;       // Set the trigger to pin out 2
const int uSonicSIG2 = 3;       // Left sensor
const int uSonicSIG3 = 4;       // Right sensor

int distanceFRT, distanceLHS, distanceRHS;

const int thresholdDistanceFRT = 10;        // Provisionally all the same but can be tweaked for performance optimisation.
const int thresholdDistanceLHS = 10;
const int thresholdDistanceRHS = 10;

void setup() {
  // Not sure what it does but it makes it work
  attachServos();
  
  Serial.begin(9600);           // Sets baud rate.
}

void loop() {
  
  distanceFRT = measureDistance(uSonicSIG1);
  distanceLHS = measureDistance(uSonicSIG2);
  distanceRHS = measureDistance(uSonicSIG3);

  //Serial.print("Front Clearance: ");
  //Serial.print(distanceFRT);
  //Serial.println(" cm");

  //Serial.print("Left Clearance: ");
  //Serial.print(distanceLHS);
  //Serial.println(" cm");

  //Serial.print("Right Clearance: ");
  //Serial.print(distanceRHS);
  //Serial.println(" cm");

  delay(500);

  if (distanceFRT > thresholdDistanceFRT) {

    forwardMove(1000);
    Serial.print("Going Forward! \n");
  } else {
    stopMove();
    delay(500);

    // Logic to determine the clear path.
    if (distanceLHS > thresholdDistanceLHS && distanceRHS <= thresholdDistanceRHS) {
      turnLeft(1000);           // Turn Left if its clear
      Serial.print("TURNING LEFT \n");
    } else if (distanceRHS > thresholdDistanceRHS && distanceLHS <= thresholdDistanceLHS){
      turnRight(1000);          // Turn Right if its clear
      Serial.print("TURNING RIGHT \n");
    } else if (distanceLHS <= thresholdDistanceLHS && distanceRHS <= thresholdDistanceRHS) {
      turnRight(2000);         // Le spin
      Serial.print("Le Spin \n");
    }

    delay(500);
  }
}


// -----------------Movement-----------------
// This current version of the code has flipped
// the servo rotations.
// ------------------------------------------
void attachServos() {
  servoLeft.attach(12);         // Left signal connects to pin 12
  servoRight.attach(13);        // Right signal connects to pin 13
}

void detachServos() {           // Stops signals to Servos
  servoLeft.detach();         // Left signal disconnects to pin 12 
  servoRight.detach();        // Right signal disconnects to pin 13
}

void forwardMove(int microsecondTime) {
  servoLeft.writeMicroseconds(1300);    // Left Servo rotates anti-clockwise
  servoRight.writeMicroseconds(1700);   // Right Servo rotates clockwise
  delay(microsecondTime);               // Move time in microseconds
}

void backwardMove(int microsecondTime) {
  servoLeft.writeMicroseconds(1700);    // Left Servo rotates clockwise
  servoRight.writeMicroseconds(1300);   // Right Servo rotates anti-clockwise
  delay(microsecondTime);               // Move time in microseconds
}

void turnLeft(int microsecondTime) {
  servoLeft.writeMicroseconds(1300);    // Left Servo rotates clockwise
  servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
  delay(microsecondTime);               // Move time in microseconds
}

void turnRight(int microsecondTime) {
  servoLeft.writeMicroseconds(1700);    // Left Servo rotates anti-clockwise
  servoRight.writeMicroseconds(1700);   // Right Servo rotates anti-clockwise
  delay(microsecondTime);               // Move time in microseconds
}

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

void stopMove() {
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
}


// -----------------Ultrasonic-----------------
// Below lies the code to calculating the dist.
// relative to the SEEED studio ultrasonic 
// sensor. Make sure it is modified if HC-SR04
// is used.
// --------------------------------------------
int measureDistance(int sensorPin) {
  long duration;
  int distance;


  pinMode(sensorPin, OUTPUT);
  digitalWrite(sensorPin, LOW); // Force low signal, just to make sure
  delayMicroseconds(2);
  digitalWrite(sensorPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPin, LOW);

  pinMode(sensorPin, INPUT);
  duration = pulseIn(sensorPin, HIGH, 1000000); // 1 Second timeout

  if (duration > 0) {
    distance = duration * 0.034 / 2;
  } else {
    distance = -1;  // Value to indicate invalidity
  }
  
  return distance;
}