/*
  Project Headshot Prototype 1 Arduino Code
  Operation code - to be used on helmet
  Logs data on EEPROM
*/

#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
#include <SPI.h>
#include <math.h>
#include "EEPROM.h"

//QwiicKX132 kxAccel;
QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
//if unsure.
outputData myData; // This will hold the accelerometer's output.
double force; // force is constantly updated
double mTh = 3.0; // mTh is min threshold value for subconcussive impact
double MTh = 6.0; // MTh is threshold for concussive impact
//int per100 = 5; // output data rate is 50Hz, so 5 measures per 100ms
int per100 = 40; // for ODR of 400Hz
uint8_t ID = 125; // UNIQUE IDENTIFIER FOR DEVICE
double maximum = 0; // variable to store maximum value calculated over 100ms interval
double temp;
int num_of_stored_dataset = 0;
int EEPROM_IDX = 0 + sizeof(num_of_stored_dataset);
uint8_t test_ID = 0;
double test_maximum = 0;


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

   EEPROM.init(); // initialize EEPROM

  if ( !kxAccel.initialize(DEFAULT_SETTINGS)) { // Loading default settings.
    Serial.println("Could not initialize the chip.");
    while (1);
  }
  else
    Serial.println("Initialized...");

  if (!kxAccel.setOutputDataRate(9)){ // 400 is maximum ODR for low power mode
    Serial.println("Could not set output data rate.");
    while(1);
  }
  else
    Serial.println("Output data rate set.");

}


void loop() {

  myData = kxAccel.getAccelData();

  force = sqrt((pow(myData.xData, 2) + pow(myData.yData, 2) + pow(myData.zData, 2)));

// advanced threshold logging
  if (force > mTh){
    maximum = force;
   
    for (int i = 0; i < per100-1; ++i){
      temp = sqrt((pow(myData.xData, 2) + pow(myData.yData, 2) + pow(myData.zData, 2)));
      if (maximum < temp){
       maximum = temp;
      }
      //delay(20); // turn on for 50Hz
      delay(2.5); // turn on for 400Hz
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
    Serial.print("\nWriting to EEPROM...\n");
    EEPROM.put(EEPROM_IDX , ID);
    EEPROM_IDX = EEPROM_IDX + sizeof(ID);
    EEPROM.put(EEPROM_IDX, maximum);
    EEPROM_IDX = EEPROM_IDX + sizeof(maximum);
    num_of_stored_dataset ++;
    EEPROM.put(0,num_of_stored_dataset);

  //delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
  delay(2.5); // delay for 400Hz
  }
}
