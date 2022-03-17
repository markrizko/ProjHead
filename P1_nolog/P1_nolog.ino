/*
  Project Headshot Prototype 1 Arduino Code
  This code is the same as the logger, first revision that DOES NOT LOG DATA
*/

#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
#include <SPI.h>
#include <math.h>

//QwiicKX132 kxAccel;
QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
//if unsure.
outputData myData; // This will hold the accelerometer's output.
double force; // force is constantly updated 
double mTh = 3.0; // mTh is min threshold value for subconcussive impact
double MTh = 6.0; // MTh is threshold for concussive impact
int per100 = 5; // output data rate is 50Hz, so 5 measures per 100ms

uint8_t ID = 125; // UNIQUE IDENTIFIER FOR DEVICE
double maximum = 0;
double temp;


void setup() {
  while (!Serial) {
    delay(50);
  }

  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();
  if ( !kxAccel.begin()){
    Serial.println("Could not communicate with the the KX13X. Freezing.");
    while (1);
  }
  else
    Serial.println("Ready.");



  if ( !kxAccel.initialize(DEFAULT_SETTINGS)) { // Loading default settings.
    Serial.println("Could not initialize the chip.");
    while (1);
  }
  else
    Serial.println("Initialized...");

  // kxAccel.setRange(KX132_RANGE16G);
  // kxAccel.setRange(KX134_RANGE32G); // For a larger range uncomment

}

void loop() {

  myData = kxAccel.getAccelData();

  force = sqrt((pow(myData.xData, 2) + pow(myData.yData, 2) + pow(myData.zData, 2)));

  
// basic threshold logging
//  if (force > 3.0){
//    Serial.println("Force above threshold: ");
//    Serial.print(force);
//    Serial.print("g ");
//    Serial.println();
//  }


// advanced threshold logging 
  if (force > mTh){
    maximum = force;
    
    for (int i = 0; i < per100-1; ++i){
      temp = sqrt((pow(myData.xData, 2) + pow(myData.yData, 2) + pow(myData.zData, 2)));
      if (maximum < temp){
       maximum = temp;
      }
      delay(20);
    }

    if (maximum < MTh){
      Serial.print("\nSubconcussive Impact Logged\n");
    }
    else{
      Serial.print("\nConcussive Impact Logged\n");
    }
    Serial.print("ID: ");
    Serial.print(ID);
    Serial.print("\tForce: ");
    Serial.print(maximum);
    // TODO DATA LOGGING
  }

  // advanced threshold logging pseudocode
    /* if (force > T){
   *   log for 100ms - loop and evaluate max
   *   after 100ms
   *   take max() of all forces over time
   *   log 1 packet of data with [ID, MAX] - 9B
   * }
   */

//  Serial.println("Total Force: ");
//  Serial.print(force);
//  Serial.print("g ");
  
//  Serial.print("X: ");
//  Serial.print(myData.xData, 4);
//  Serial.print("g ");
//  Serial.print(" Y: ");
//  Serial.print(myData.yData, 4);
//  Serial.print("g ");
//  Serial.print(" Z: ");
//  Serial.print(myData.zData, 4);
//  Serial.println("g ");

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz

}
