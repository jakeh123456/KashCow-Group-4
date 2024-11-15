#include <Servo.h>              // Servo Library
#include <HCSR04.h>             // UltraSonic libary for 1 trigger

Servo servoLeft;                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// -----------------CAUTION-----------------
// Current software is made for SEEED studio 
// ultrasonic sensor, the same digital pin 
// operates both emitted and echoed signal 
// when calculating distance. 
// -----------------------------------------

// Distance Parameters & Initialisation
HCSR04 hc(3, new int[3]{9,10, 11}, 3); //initialisation class HCSR04 (trig pin , echo pin, number of sensor) (Front,Left,Right)

double distanceFRT, distanceLHS, distanceRHS;

const double thresholdDistanceFRT = 5.0;        // Provisionally all the same but can be tweaked for performance optimisation.
const double thresholdDistanceLHS = 5.0;
const double thresholdDistanceRHS = 5.0;

// Robot State Memory
const int maxMoves = 45;        // Subject to fine tuning
int moveMemory[maxMoves];       // Reserve data in micro-controller - 32kb max data.
int moveIndex = 0;              // Iterable value for robot state.
int junctionIndex = -1;         // Backtrack to junction.

// Solving Bias
// 0 - Left Bias
// 1 - Right Bias
int turnBias = 1;



void setup() {
  // Not sure what it does but it makes it work
  attachServos();
  Serial.begin(9600);           // Sets baud rate.
}

void loop() {
  
  distanceFRT = hc.dist(0);
  delay(10);                   // DO NOT REMOVE
  distanceLHS = hc.dist(1);
  delay(10);                   // DO NOT REMOVE
  distanceRHS = hc.dist(2);
  delay(10);                   // DO NOT REMOVE


  /*
  Serial.print("Front Clearance: ");
  Serial.print(distanceFRT);
  Serial.println(" cm");

  Serial.print("Left Clearance: ");
  Serial.print(distanceLHS);
  Serial.println(" cm");

  Serial.print("Right Clearance: ");
  Serial.print(distanceRHS);
  Serial.println(" cm");
  */
  
  //delay(500);       // Sampling interval in ms

  if (distanceFRT > thresholdDistanceFRT) {

    backwardMove(1000);
    //Serial.print("Going Forward! \n");
    recordMove(1);
  } else {
    stopMove();
    delay(500); // Most likely junction detected, else a dead-end.

    // Logic to determine the clear path.
    if (distanceLHS > thresholdDistanceLHS && distanceRHS <= thresholdDistanceRHS) {
      turnLeft(1000);           // Turn Left if its clear
      //Serial.print("TURNING LEFT \n");
      recordMove(2);

    } else if (distanceRHS > thresholdDistanceRHS && distanceLHS <= thresholdDistanceLHS){
      turnRight(1000);          // Turn Right if its clear
      //Serial.print("TURNING RIGHT \n");
      recordMove(3);

    } else if (distanceLHS <= thresholdDistanceLHS && distanceRHS <= thresholdDistanceRHS) {
      backtrackTo(junctionIndex); // Dead-end detected.
      recordMove(4);
  
    } else if (distanceLHS > thresholdDistanceLHS && distanceRHS > thresholdDistanceRHS) {
      junctionIndex = moveIndex;
      if (turnBias == 0) {
        turnLeft(1000);
        recordMove(5);            // Junction Detected, turning left.
      } else if (turnBias == 1) {
        turnRight(1000);
        recordMove(6);            // Junction Detected, turning right.
      }
    }
  }
}

// -----------------Memory-------------------
// Ensures the robot can backtrack to previously
// visited junctions.
// 0 - Unused, free slot
// 1 - Moving forward
// 2 - Left Clear, Turning Left
// 3 - Right Clear, Turning Right
// 4 - Dead-end detected, backtracking
// 5 - Junction detected, Turning Left
// 6 - Junction detected, Turning Right
// ------------------------------------------
void recordMove(int move) {
  if (moveIndex < maxMoves) {
    moveMemory[moveIndex] = move;
    //Serial.print("Saved State: ");
    //Serial.print(move);
    //Serial.print("\n");
    moveIndex++;
  }
}


// ---------------Backtracking---------------
// This method allows the robot memory stack
// to be accessed and modified enabling the robot
// to go back to the junction that lead it to a 
// dead-end.
// 
// CAUTION:
// ENSURE SERVO SPIN VALUES IN BACKTRACKING METHOD
// ARE THE SAME AS IN THE GENERAL MAZE EXPLORATION.
// 
// ------------------------------------------
void backtrackTo(int junctionIndex) {
  while (moveIndex > junctionIndex) {
    int lastMove = moveMemory[moveIndex];

    // Majority of movements are reversed
    switch (lastMove) {
      case 1:
        forwardMove(1000);        // Kept to forward as robot is assumed to have performed a 180 at dead end.
        Serial.print("Backtracking: move back");
      break;

      case 2:
        turnRight(1000);
        Serial.print("Backtracking: turn right");
      break;

      case 3:
        turnLeft(1000);
        Serial.print("Backtracking: turn left");
      break;

      case 4:
        turnLeft(2000);
        Serial.print("Backtracking: leaving dead-end");
      break;

      case 5:
      break;

      case 6:
      break;
      // Compensates for existance of values 5 & 6 but no response required.
    }

    moveMemory[moveIndex] = 0;
    moveIndex--;

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
