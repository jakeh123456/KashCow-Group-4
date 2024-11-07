#include <Servo.h>              // Servo Library

Servo servoLeft;                // Declare Left Servo - using servoLeft will determine the velocity of the Left Servo
Servo servoRight;               // Declare Right Servo - using servoRight will determine the velocity of the Right Servo


void setup() {                  // Set up function will run once 

  attachServos();
  delay(1000);

  forwardMove(2000);
  delay(1000);

  turnRight(550);    
  delay(500);
  
  forwardMove(2000);
  delay(1000);

  turnLeft(550);
  delay(500);

  detachServos();
  delay(500);
}

void loop() {                   // Loop function will run repeatedly
  

}


void attachServos() {
  servoLeft.attach(12);         // Left signal connects to pin 12
  servoRight.attach(13);        // Right signal connects to pin 13
}

void detachServos() {           // Stops signals to Servos
  servoLeft.detach();         // Left signal disconnects to pin 12 
  servoRight.detach();        // Right signal disconnects to pin 13
}

void forwardMove(int microsecondTime) {
  servoLeft.writeMicroseconds(1700);    // Left Servo rotates anti-clockwise
  servoRight.writeMicroseconds(1300);   // Right Servo rotates clockwise
  delay(microsecondTime);               // Move time in microseconds
}

void backwardMove(int microsecondTime) {
  servoLeft.writeMicroseconds(1300);    // Left Servo rotates clockwise
  servoRight.writeMicroseconds(1700);   // Right Servo rotates anti-clockwise
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



//  Need to change the value of microsecondTime to determine the angle of turns

