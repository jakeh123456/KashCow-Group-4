/*
  IMU Calibration

  Run this file before running the main MPU6050 file to find the offset values 
  to ensure that the values accurate and ammends the errors and deviations

  Most of the code is taken from the MPU6050 library example "IMU Zero"
  https://github.com/ElectronicCats/mpu6050/blob/master/examples/IMU_Zero/IMU_Zero.ino

  When executing the code on the MPU6050 sensor, make sure that the sensor is stationary 
  and on a flat, horizontal surface.

  "During the execution it will generate a dozen outputs, showing that for each of the 6 
  desired offsets, it is:
  - First, try to find two estimates, one too low and one too high.
  - Closing in until the bracket can't be made smaller.

  The line just above the "done" (it will take a few minutes to get there) describes the 
  optimum offsets for the X acceleration, Y acceleration, Z acceleration, X gyro, Y gyro, 
  and Z gyro, respectively."
*/

/*||||||||||||||||||||||||||||RUN THIS FILE BEFORE RUNNING 'calculateMPU6050.ino' TO FIND THE DESIRED OFFSETS|||||||||||||||||||||||||||||*/


#include "I2Cdev.h"               // Importing this library allows for interfacing with the I2C device
#include "MPU6050.h"              // MPU6050 library from Electronic Cats

MPU6050 mpu;                      // Class for MPU6050. Default I2C address is 0x68

// Constants
const int usDelay = 3150;         // Delay in ms to hold the sampling at 200Hz
const int NFast = 1000;           // Number of quick readings for averaging, the higher the better
const int NSlow = 10000;          // Number of slow readings for averaging, the higher the better
const int LinesBetweenHeaders = 5;

// Index values for acceleration and gyro values
const int iAx = 0, iAy = 1, iAz = 2, iGx = 3, iGy = 4, iGz = 5;

int LowValue[6], HighValue[6], Smoothed[6], LowOffset[6], HighOffset[6], Target[6];
int LinesOut, N, i;


void setup(void) {
  Initialise();                   // Initialisation function
  for (i = iAx; i <= iGz; i++) {
    Target[i] = 0;                // Fix for ZAccel
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  }
  Target[iAz] = 16384;            // Set the target for Z axes
  SetAveraging(NFast);            // Fast averaging set
  PullBracketsOut();
  PullBracketsIn();
  Serial.println("----------------------------- D O N E ---------------------------------");
}

void loop(void) {
  // In the final edit of the code 
  // Add the loop code to return the values for the acceleration and the gyroscope
}

// INITIALISE function
void Initialise(){
  // This 'if' and 'elif' statements setup the I2C communication; either by Wire or FastWire from the MPU6050 library
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
  #endif

  Serial.begin(9600); // Connects with the serial 
  // Init the module
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("MPU initializated");
  // Check module connection
  Serial.println("Testing device connections...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
  while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  Serial.println("\nPID tuning Each Dot = 100 readings");
  /*
  PID tuning (actually PI) works like this: changing the offset in the MPU6050 gives instant results, 
  allowing us to use the Proportional and Integral parts of the PID to find the ideal offsets. 
  The Integral uses the error from the set point (which is zero) and adds a fraction of this error to 
  the integral value. Each reading reduces the error towards the desired offset. The greater 
  the error, the more we adjust the integral value. 
  
  The Proportional part helps by filtering out noise from the integral calculation. The Derivative part is 
  not used due to noise and the sensor being stationary. With the noise removed, the integral value stabilizes 
  after about 600 readings. At the end of each set of 100 readings, the integral value is used for the actual 
  offsets, and the last proportional reading is ignored because it reacts to any noise.
  */
  Serial.println("\nXAccel\t\tYAccel\t\tZAccel\t\tXGyro\t\tYGyro\t\tZGyro");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("\n600 Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("700 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("800 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("900 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("1000 Total Readings");
  mpu.PrintActiveOffsets();
  Serial.println("\nAny of the above offsets will work nicely \n\nProving the PID with other method:");
}

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("\nAveraging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

// This function expands the range of offsets based on sensor readings
void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  Serial.println("Expanding:");
  ForceHeader();

  // loops until the offsets are fully expanded
  while (!Done) {
    Done = true;
    // Set the low offsets and outputs smoothed values
    SetOffsets(LowOffset); //Set low offsets
    GetSmoothed();
    // Loops through each axis in the array to get the low values
    for (i = 0; i <= 5; i++) { 
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } 
      else {
        NextLowOffset[i] = LowOffset[i];
      }
    }
    // Set the high offsets and outputs smoothed values
    SetOffsets(HighOffset);
    GetSmoothed();
    // Loops through each axis in the array to get the high values
    for (i = 0; i <= 5; i++) { // Get high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } 
      else {
        NextHighOffset[i] = HighOffset[i];
      }
    } 
    ShowProgress();
    // Changes the values of the offsets for the next iteration
    for (int i = 0; i <= 5; i++) {
      LowOffset[i] = NextLowOffset[i]; 
      HighOffset[i] = NextHighOffset[i];
    }
  }
}
// This function narrows the range of offsets based on sensor readings
void PullBracketsIn() {
  boolean AllBracketsNarrow;
  boolean StillWorking;
  int NewOffset[6];

  Serial.println("\nClosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  // Loops until everything is done
  while (StillWorking) {
    StillWorking = false;
    // Check if all brackets are narrow and adjusts the averaging
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } 
    else {
      AllBracketsNarrow = true; // Assume all brackets are narrow
    }
    // loops through each axis to calculate the new offsets
    for (int i = 0; i <= 5; i++) {
      // If the high offset is less than or equal to the low offset + 1, keep the low offset
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } 
      else { // Binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        // Check if the high offset is significantly greater than the low offset
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } 
    }
    // Set the new offsets based on the calculated values
    SetOffsets(NewOffset);
    GetSmoothed();
    // Loop through each axis to determine if we need to adjust the low or high offsets
    for (i = 0; i <= 5; i++) { // Closing in
      if (Smoothed[i] > Target[i]) { // Use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } 
      else { // Use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } 
    }
    ShowProgress();
  } 
} 

void ForceHeader() {
  LinesOut = 99;
}

/*Function to smooth the read values*/
void GetSmoothed() {
  int16_t RawValue[6];
  long Sums[6];
  // Sets all the values in  Sums array to 0
  for (i = 0; i <= 5; i++) {
    Sums[i] = 0;
  }
  
/* Get Sums*/
  for (i = 1; i <= N; i++) { 
    // Reads the raw data from MPU6050 on all 6 axes
    mpu.getMotion6( & RawValue[iAx], & RawValue[iAy], & RawValue[iAz], & RawValue[iGx], & RawValue[iGy], & RawValue[iGz]);
    delayMicroseconds(usDelay);
    // Adds the current value to the corresponding Sums index
    for (int j = 0; j <= 5; j++){
      Sums[j] = Sums[j] + RawValue[j];
    }
  } 
  // Calculate the smoothed values by averaging the Sums
  for (i = 0; i <= 5; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} 

/*Function for configure the obtained offsets*/
void SetOffsets(int TheOffsets[6]) {
  mpu.setXAccelOffset(TheOffsets[iAx]);
  mpu.setYAccelOffset(TheOffsets[iAy]);
  mpu.setZAccelOffset(TheOffsets[iAz]);
  mpu.setXGyroOffset(TheOffsets[iGx]);
  mpu.setYGyroOffset(TheOffsets[iGy]);
  mpu.setZGyroOffset(TheOffsets[iGz]);
}

/*Print the progress of the reading averages, add formatting for better visualization*/
void ShowProgress() {
 /*Header*/
  if (LinesOut >= LinesBetweenHeaders) { 
    Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  } 
  Serial.print(' ');
  for (i = 0; i <= 5; i++) {
    Serial.print('[');
    Serial.print(LowOffset[i]),
    Serial.print(',');
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(',');
    Serial.print(HighValue[i]);
    if (i == 5) {
      Serial.println("]");
    } 
    else {
      Serial.print("]\t");
    }
  }
  LinesOut++;
} 

/*
  At the end of the calibration, the Serial Monitor should display below message (with different values, depending on the offset.)


Averaging 10000 readings each time
  XAccel			             YAccel				         ZAccel			                 XGyro			         YGyro			        ZGyro
 [551,552] --> [-7,17]	[2739,2741] --> [-8,17]	[535,536] --> [16374,16392]	[27,28] --> [-4,2]	[10,10] --> [0,1]	[-18,-17] --> [-2,1]
 [551,552] --> [-9,17]	[2739,2740] --> [-8,13]	[535,536] --> [16378,16392]	[27,28] --> [-7,2]	[10,10] --> [0,1]	[-18,-17] --> [-2,1]
 [551,552] --> [-15,17]	[2739,2740] --> [-4,13]	[535,536] --> [16378,16392]	[27,28] --> [-9,2]	[10,10] --> [0,1]	[-18,-17] --> [-1,1]
----------------------------- D O N E ---------------------------------

for each axis measurement, the following is given:
[LowOffset,HighOffset] --> [LowValue,HighValue]

*/



