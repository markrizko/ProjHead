/*********************************************************
  Author: Elias Santistevan @ SparkFun Electronics
  Date: 5/2021
  Library repository can be found here:
  https://github.com/sparkfun/SparkFun_KX13X_Arduino_Library

  Basic example for reading back accelerometer values using I2C.

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).

  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.

  Distributed as-is; no warranty is given.
*******************************************************/

#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
#include <SPI.h>
#include <math.h>

//QwiicKX132 kxAccel;
QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
//if unsure.
outputData myData; // This will hold the accelerometer's output.
double force; // force is constantly updated 
double T = 5.0; // T is threshold value
int per100; // TODO find out how many times data is measured in 100ms

uint8_t ID = 125; // UNIQUE IDENTIFIER FOR DEVICE
double maximum = 0;
double temp;
 

unsigned long starttime, endtime;

void setup() {
    //Serial.begin(115200);
    //pinMode(10, OUTPUT);
    //SPI.begin();
    //digitalWrite(10,HIGH);
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
//  if (force > 3.0){
//    Serial.println("Force above threshold: ");
//    Serial.print(force);
//    Serial.print("g ");
//    Serial.println();
//  }

  // USE THIS LOOP TO COUNT # measures in 100ms
  starttime = millis();
    endtime = starttime;
    while((endtime - starttime) <= 100){
      

      endtime = millis();
    }

  if (force > T){
    for (int i = 0; i < per100; ++i){
      temp = sqrt((pow(myData.xData, 2) + pow(myData.yData, 2) + pow(myData.zData, 2)));
      if (maximum < temp){
       maximum = temp;
      }
    }
    // TODO DATA LOGGING
    
    maximum = 0.0;
  }

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
