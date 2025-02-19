#include <Servo.h>              // Servo Library
#include <HCSR04.h>             // UltraSonic library for 1 trigger

Servo servoLeft;                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo

// -----------------CAUTION-----------------
// TRIAL VERSION FOR PID STABILISER
// -----------------------------------------

// Distance Parameters & Initialisation
HCSR04 hc(3, new int[3]{9,10, 11}, 3); //initialisation class HCSR04 (trig pin , echo pin, number of sensor) (Front,Left,Right)

double distanceFRT, distanceLHS, distanceRHS;

const double thresholdDistanceFRT = 7.5;        // The left and right are pretty good now could make the front a bit bigger
const double thresholdDistanceLHS = 13.0;
const double thresholdDistanceRHS = 13.0;

// Solving Bias
// 0 - Left Bias
// 1 - Right Bias
int turnBias = 1;

// PID Params
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


void setup() {
  attachServos();
  Serial.begin(9600);           // Sets baud rate.
}

void loop() {
    // Reset the movedForward flag at the beginning of each cycle
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
        
    // If the while loop was not executed, handle the alternative logic
    if (!movedForward) {
        Serial.print("Entered Alternative Logic ");
        stopMove();
        distanceLHS = hc.dist(1);
        delay(10);                   // DO NOT REMOVE
        distanceRHS = hc.dist(2);
        delay(10);                   // DO NOT REMOVE

        // Logic to determine the clear path
        if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS <= thresholdDistanceRHS)) { // works well after threshold change
            turnLeft(600);           // Turn Left if it's clear for battery(600) and usb(800)
                Serial.print("LOGIC : Turning Left");
        } else if ((distanceRHS > thresholdDistanceRHS) && (distanceLHS <= thresholdDistanceLHS)) {
            turnRight(600);          // Turn Right if it's clear
                Serial.print("LOGIC : Turning Right");
        } else if ((distanceLHS > thresholdDistanceLHS) && (distanceRHS > thresholdDistanceRHS)) {
            if (turnBias == 0) {
                turnLeft(600);
                Serial.print("Junction Detected, turning left");
            } else if (turnBias == 1) {
                turnRight(600);
                Serial.print("Junction Detected, turning right");
            }
            Serial.print(" LHS: ");
            Serial.print(distanceLHS);
            Serial.print(", RHS: ");
            Serial.println(distanceRHS);
        } else {                                                                 // DEBUG: Print sensor values if none of the conditions to see problem (DEAD END CODE GO HERE)
            // Print sensor values if none of the conditions are met
            turnRight(1000);
            Serial.print("No clear path detected. Sensor values - FRT: ");
            Serial.print(distanceFRT);
            Serial.print(", LHS: ");
            Serial.print(distanceLHS);
            Serial.print(", RHS: ");
            Serial.println(distanceRHS);
        }
    }
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
  servoLeft.writeMicroseconds(1500);
  servoRight.writeMicroseconds(1500);
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