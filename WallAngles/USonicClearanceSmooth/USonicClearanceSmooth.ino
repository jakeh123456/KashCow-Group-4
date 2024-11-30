#include <Servo.h>              // Servo Library
#include <HCSR04.h>             // UltraSonic libary for 1 trigger
#include <movingAvg.h>

#define WINDOW_SIZE 6           // Protected window size constant
#define Pi 3.14159              // Pi value used for wall angle calculations

HCSR04 hc(3, new int[3]{9,10, 11}, 3); //initialisation class HCSR04 (trig pin , echo pin, number of sensor) (Front,Left,Right)

double distance1;               // Local distance from Usonic 1
double distance2;               // Local distance from Usonic 2
double smoothedClearance;       
double currentClearance;        // distance from 2 Usonic plane to surface.

// Wall angle variables
double wallDelta;
double wallTheta;
float UsonicSeperation = 10.5;  // Space in between sensors in cm
double smoothedWallTheta;


movingAvg clearanceFilter(WINDOW_SIZE);
movingAvg wallAngleFilter(WINDOW_SIZE);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  clearanceFilter.begin();      // Needed for the movingAvg Library
  wallAngleFilter.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  distance1 = hc.dist(0);
  delay(10);                    // Prevents sound interference
  distance2 = hc.dist(1);

  // Assumed flat surface obstacle angle relative to robot
  wallDelta = (distance1 - distance2);
  wallTheta = atan(wallDelta / UsonicSeperation) * (180/Pi);    // Returns angle in degrees.
  smoothedWallTheta = wallAngleFilter.reading(wallTheta);

  // Clearance Calculations
  currentClearance = linearClearance(distance1, distance2);
  smoothedClearance = clearanceFilter.reading(currentClearance);

  // Comma-separated for data automation in excel - performance optimisation
  Serial.print(currentClearance);
  Serial.print(",");
  Serial.print(smoothedClearance);
  Serial.print(",");
  Serial.print(wallTheta);
  Serial.print(",");
  Serial.println(smoothedWallTheta);
  delay(50);
}

double linearClearance(double d1, double d2){
  return min(d1, d2);
}