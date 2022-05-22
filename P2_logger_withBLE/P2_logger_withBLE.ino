/*
  Project Headshot Prototype 2 Arduino Code
  Operation code - to be used on helmet
  Logs data on EEPROM
*/
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_Qwiic_KX13X.h"
#include <SPI.h>
#include <math.h>
#include "EEPROM.h"
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

bool conn = false;
BLEDevice central;
BLEService impactService("f6a40646-f930-4075-b0f6-3da69ffce7aa");
BLEByteCharacteristic b_ID("f6a40646-f930-4075-b0f6-3da69ffce7-1d", BLERead);
BLEBoolCharacteristic b_conc("f6a40646-f930-4075-b0f6-3da69ffce7-c4c", BLERead);
BLEDoubleCharacteristic b_force("f6a40646-f930-4075-b0f6-3da69ffce7-4c", BLERead);
BLEIntCharacteristic b_rot_x("f6a40646-f930-4075-b0f6-3da69ffce7-701", BLERead);
BLEIntCharacteristic b_rot_y("f6a40646-f930-4075-b0f6-3da69ffce7-702", BLERead);
BLEIntCharacteristic b_rot_z("f6a40646-f930-4075-b0f6-3da69ffce7-703", BLERead);


//QwiicKX132 kxAccel;
QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
//if unsure.
outputData myData; // This will hold the accelerometer's output.
double force; // force is constantly updated
double gravity = 9.8; //gravity = 9.8 m/s^2
double mTh = 3 * gravity; // mTh is min threshold value for subconcussive impact
double MTh = 6 * gravity; // MTh is threshold for concussive impact
//int per100 = 5; // output data rate is 50Hz, so 5 measures per 100ms
int per100 = 40; // for ODR of 400Hz
uint8_t ID = 125; // UNIQUE IDENTIFIER FOR DEVICE
double maximum = 0; // variable to store maximum value calculated over 100ms interval
double temp;
int Gxmax;
int Gymax;
int Gzmax;
int Gxtemp;
int Gytemp;
int Gztemp;
int num_of_stored_dataset = 0;
int EEPROM_IDX = 0 + sizeof(num_of_stored_dataset);
uint8_t test_ID = 0;
double test_maximum = 0;

void Advertise(){
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("ProjHead");

  // start advertising
  BLE.advertise();

  Serial.println("BLE advertising as 'ProjHead'");
}

void serviceSetup(){
  impactService.addCharacteristic(b_ID);
  impactService.addCharacteristic(b_conc);
  impactService.addCharacteristic(b_force);
  impactService.addCharacteristic(b_rot_x);
  impactService.addCharacteristic(b_rot_y);
  impactService.addCharacteristic(b_rot_z);

  BLE.addService(impactService);

  b_ID.writeValue(ID);
  b_force.writeValue(0.0);
  b_rot_x.writeValue(0);
  b_rot_y.writeValue(0);
  b_rot_z.writeValue(0);
  b_conc.writeValue(false);

  Serial.println("Bluetooth service setup successful");
}

BLEDevice Connect(){

  BLEDevice _central = BLE.central();
  // if a central is connected to peripheral:
  if (_central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(_central.address());
    conn = true;
  }

  return _central;
}


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup() {
  while (!Serial) {
    delay(50);
  }
  
  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();
  if ( !accel.begin()){
    Serial.println("Could not communicate with the the ADXL375. Freezing.");
    while (1);
  }
  else
    Serial.println("Ready.");

   EEPROM.init(); // initialize EEPROM
/*
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
*/

  Advertise();
  serviceSetup();

}


void loop() {

  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  lsm.read();

if (!conn){
      // listen for BLE peripherals to connect:
      central = Connect();
    }

    

 /* myData = kxAccel.getAccelData();
*/
  force = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2)));
  if (!central.connected() && conn){
      conn = false;
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      
    }
// advanced threshold logging
  if (force > mTh){
    maximum = force;
    Gxmax = (int)lsm.gyroData.x;
    Gymax = (int)lsm.gyroData.y;
    Gzmax = (int)lsm.gyroData.z;
    for (int i = 0; i < per100-1; ++i){
      temp = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2)));
      Gxtemp = (int)lsm.gyroData.x;
      Gytemp = (int)lsm.gyroData.y;
      Gztemp = (int)lsm.gyroData.z;
      if (maximum < temp){
       maximum = temp;
      }
      if (Gxmax < Gxtemp){
       Gxmax = Gxtemp;
      }
      if (Gymax < Gytemp){
       Gymax = Gytemp;
      }
      if (Gzmax < Gztemp){
       Gzmax = Gztemp;
      }
      //delay(20); // turn on for 50Hz
      delay(2.5); // turn on for 400Hz
    }

    if (maximum < MTh){
      Serial.print("\nSubconcussive Impact Logged\n");
      b_conc.writeValue(false);
    }
    else{
      Serial.print("\nConcussive Impact Logged\n");
      b_conc.writeValue(true);
    }
    Serial.print("ID: ");
    Serial.print(ID);
    Serial.print("\tForce: ");
    Serial.print(maximum);
    Serial.print("\nGyro X:");
    Serial.print(Gxmax);
    Serial.print("\tGyro Y:");
    Serial.print(Gymax);
    Serial.print("\tGyro Z:");
    Serial.print(Gzmax);
    Serial.print("\nWriting to EEPROM...\n");
    Serial.print("\nData Transmitting Through BLE...\n");
    EEPROM.put(EEPROM_IDX , ID);
    EEPROM_IDX = EEPROM_IDX + sizeof(ID);
    EEPROM.put(EEPROM_IDX, maximum);
    EEPROM_IDX = EEPROM_IDX + sizeof(maximum);
    b_force.writeValue(maximum);
    EEPROM.put(EEPROM_IDX, Gxmax);
    EEPROM_IDX = EEPROM_IDX + sizeof(Gxmax);
    b_rot_x.writeValue(Gxmax);
    EEPROM.put(EEPROM_IDX, Gymax);
    EEPROM_IDX = EEPROM_IDX + sizeof(Gymax);
    b_rot_y.writeValue(Gymax);
    EEPROM.put(EEPROM_IDX, Gzmax);
    EEPROM_IDX = EEPROM_IDX + sizeof(Gzmax);
    b_rot_z.writeValue(Gzmax);
    num_of_stored_dataset ++;
    EEPROM.put(0,num_of_stored_dataset);



    

  //delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
  delay(2.5); // delay for 400Hz
  }
}
