/* 
  Data reading test for Prototype1
  Reads all data stored into EEPROM during operation of the prototype
*/
#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
#include <SPI.h>
#include <math.h>
#include "EEPROM.h"

uint8_t ID;
int num_of_stored_dataset;
int EEPROM_IDX = 0 + sizeof(num_of_stored_dataset);
double force;

void setup() {
  Serial.begin(115200);
  Serial.print("EEPROM Data Reading...\n");
  Wire.begin();

  EEPROM.init();

  
   /* For DATA reading*/
    EEPROM.get(0,num_of_stored_dataset);
    for(int i = 0; i < num_of_stored_dataset; i++){
      EEPROM.get(EEPROM_IDX, ID);
      EEPROM_IDX = EEPROM_IDX + sizeof(ID);
      EEPROM.get(EEPROM_IDX, force);
      EEPROM_IDX = EEPROM_IDX + sizeof(force);
      Serial.print("ID: ");
      Serial.print(ID);
      Serial.print("\tForce: ");
      Serial.print(force);
      Serial.print("\n");
    }
    
  
}

void loop() {
}
