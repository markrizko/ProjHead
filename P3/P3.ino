/*
  Project Headshot Prototype "3" Finalized Arduino Code
  Operation code - to be used on helmet
  Logs data on EEPROM
  Sends data to Bluetooth continuouosly for GUI
  Sends data to Bluetooth per impact for App

  Written by Mark Rizko and Tsung Hung Su
*/

// Including Necessary Libraries
#include <ArduinoBLE.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "EEPROM.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ISM330DHCX.h>

// instantiate accelerometer
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
// instantiate gyroscope
Adafruit_ISM330DHCX ism330dhcx;

// declare Bluetooth variables
bool conn = false; // debug variable for ensuring connection

BLEDevice central; // device connected to system

// Bluetooth service GATT table initialization, UUIDs are assigned arbitrarily
BLEService impactService("f6a40646-f930-4075-b0f6-3da69ffce7aa");
// Bluetooth Characteristic initialization
BLEStringCharacteristic b_ID("f6a40646-f930-4075-b0f6-3da69ffce7-1d", BLERead, 3); // ID of device
BLEStringCharacteristic b_conc("f6a40646-f930-4075-b0f6-3da69ffce7-c4c", BLERead, 5); // True = Concussive : False = Subconcussive
BLEStringCharacteristic b_force("f6a40646-f930-4075-b0f6-3da69ffce7-4c", BLERead, 8); // Acceleration of most recent impact in g
BLEStringCharacteristic b_cont_force("f6a40646-f930-4075-b0f6-3da69ffce7-cc55c", BLERead, 8); // Continuously updated acceleration for GUI
BLEStringCharacteristic b_rot_x("f6a40646-f930-4075-b0f6-3da69ffce7-701", BLERead, 8); // X rotation from gyroscope (dps)
BLEStringCharacteristic b_rot_y("f6a40646-f930-4075-b0f6-3da69ffce7-702", BLERead, 8); // Y rotation from gyroscope (dps)
BLEStringCharacteristic b_rot_z("f6a40646-f930-4075-b0f6-3da69ffce7-703", BLERead, 8); // Z rotation from gyroscope (dps)

#define MIN_THRESHOLD 15
#define CONC_THRESHOLD 30
#define DEVICE_ID 125

double force; // force is constantly updated
double gravity = 9.8; //gravity = 9.8 m/s^2
double mTh = MIN_THRESHOLD; // mTh is min threshold value for subconcussive impact
double MTh = CONC_THRESHOLD; // MTh is threshold for concussive impact

// parameter for logging window, readings per 100ms
//int per100 = 5; // for ODR of 50Hz
//int per100 = 40; // for ODR of 400Hz
int per100 = 320; // for ODR of 3200Hz

uint8_t ID = DEVICE_ID; // UNIQUE IDENTIFIER FOR DEVICE
double maximum = 0; // variable to store maximum value calculated over 100ms interval
double temp; // temporary accel value for evaluating max in logging window

// gyroscope data correlating to maximum accel value
double Gxmax; 
double Gymax;
double Gzmax;
// temporary gyroscope values
double Gxtemp;
double Gytemp;
double Gztemp;
double conv = 180 / PI;
// helper variables for EEPROM storage
int num_of_stored_dataset = 0;
int EEPROM_IDX = 0 + sizeof(num_of_stored_dataset);
uint8_t test_ID = 0;
double test_maximum = 0;

// helper function for converting double to string for bluetooth communication
String double2string(double n, int ndec) {
    String r = "";
    if (n < 0){
      n = n*-1.0;
      r+="-";
    }
    

    int v = n;
    r += v;     // whole number part
    r += '.';   // decimal point
    int i;
    for (i=0;i<ndec;i++) {
        // iterate through each decimal digit for 0..ndec 
        n -= v;
        n *= 10; 
        v = n;
        r += v;
    }

    return r;
}

// function to begin advertising device for Bluetooth
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

// setup bluetooth service and GATT table
void serviceSetup(){
  impactService.addCharacteristic(b_ID);
  impactService.addCharacteristic(b_conc);
  impactService.addCharacteristic(b_force);
  impactService.addCharacteristic(b_cont_force);
  impactService.addCharacteristic(b_rot_x);
  impactService.addCharacteristic(b_rot_y);
  impactService.addCharacteristic(b_rot_z);

  BLE.addService(impactService);

  // initialize values
  b_ID.writeValue(String(ID));
  b_force.writeValue("0.0g");
  b_cont_force.writeValue("0.0g");
  b_rot_x.writeValue("0dps");
  b_rot_y.writeValue("0dps");
  b_rot_z.writeValue("0dps");
  b_conc.writeValue("False");

  Serial.println("Bluetooth service setup successful");
}

// function runs when a device attempts to connect to our prototype
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

// initialize gyroscope
void initializeGyro(){
  // initialize gyroscope over qwiic (i2c)
  if (!ism330dhcx.begin_I2C()){
   Serial.println("Failed to find ISM330DHCX chip (gyro)");
  }
  else{
   Serial.println("Found ISM330DHCX!");
  }

 ism330dhcx.setGyroRange(ISM330DHCX_GYRO_RANGE_4000_DPS);
 Serial.print("Gyro range set to: ");
 switch (ism330dhcx.getGyroRange()) {
 case LSM6DS_GYRO_RANGE_125_DPS:
   Serial.println("125 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_250_DPS:
   Serial.println("250 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_500_DPS:
   Serial.println("500 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_1000_DPS:
   Serial.println("1000 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_2000_DPS:
   Serial.println("2000 degrees/s");
   break;
 case ISM330DHCX_GYRO_RANGE_4000_DPS:
   Serial.println("4000 degrees/s");
   break;
 }

  ism330dhcx.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);
 ism330dhcx.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
 Serial.print("Gyro data rate set to: ");
 switch (ism330dhcx.getGyroDataRate()) {
 case LSM6DS_RATE_SHUTDOWN:
   Serial.println("0 Hz");
   break;
 case LSM6DS_RATE_12_5_HZ:
   Serial.println("12.5 Hz");
   break;
 case LSM6DS_RATE_26_HZ:
   Serial.println("26 Hz");
   break;
 case LSM6DS_RATE_52_HZ:
   Serial.println("52 Hz");
   break;
 case LSM6DS_RATE_104_HZ:
   Serial.println("104 Hz");
   break;
 case LSM6DS_RATE_208_HZ:
   Serial.println("208 Hz");
   break;
 case LSM6DS_RATE_416_HZ:
   Serial.println("416 Hz");
   break;
 case LSM6DS_RATE_833_HZ:
   Serial.println("833 Hz");
   break;
 case LSM6DS_RATE_1_66K_HZ:
   Serial.println("1.66 KHz");
   break;
 case LSM6DS_RATE_3_33K_HZ:
   Serial.println("3.33 KHz");
   break;
 case LSM6DS_RATE_6_66K_HZ:
   Serial.println("6.66 KHz");
   break;
 }

 ism330dhcx.configInt1(false, false, true);
 ism330dhcx.configInt2(false, true, false);

}

// arduino setup function
void setup() {
  while (!Serial) {
    delay(50);
  }
  
  Serial.begin(115200);
  Serial.println("Welcome.");

  Wire.begin();

  // initialize accelerometer
  if ( !accel.begin()){
    Serial.println("Could not communicate with the the ADXL375. Freezing.");
    while (1);
  }
  else
    Serial.println("Ready.");

  // set ADXL data rate 
  accel.setDataRate(ADXL343_DATARATE_3200_HZ);

  EEPROM.init(); // initialize EEPROM
  
  initializeGyro();

  Advertise();
  serviceSetup();

}


// arduino loop function
void loop() {

  // Get a new sensor event for accelerometer 
  sensors_event_t event;
  accel.getEvent(&event);
  // get new sensor events for gyroscope
  sensors_event_t gyro;
  sensors_event_t accel_;
  sensors_event_t tempr;
  ism330dhcx.getEvent(&accel_, &gyro, &tempr);

  if (!conn){
      // listen for BLE peripherals to connect:
      central = Connect();
  }

  // detect if device has been disconnected
  if (!central.connected() && conn){
      conn = false;
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      
    }
    
  // get acceleration value from accelerometer and calculate magnitude
  // converted to g by dividing by gravity
  force = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) / gravity;

  // send value to GUI immediately
  String f = double2string(force, 2);
  b_cont_force.writeValue(f + "g");

// advanced threshold logging
  if (force > mTh){
    Serial.println("Impact detected");
    maximum = force;
    Gxtemp = gyro.gyro.x * conv;
    Gytemp = gyro.gyro.y * conv;
    Gztemp = gyro.gyro.z * conv;

    for (int i = 0; i < per100-1; ++i){
      temp = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) / gravity;

//      Serial.println(Gxtemp);
//      Serial.println(Gytemp);
//      Serial.println(Gztemp);
      if (maximum < temp){
        maximum = temp;
        Gxmax = Gxtemp;
        Gymax = Gytemp;
        Gzmax = Gztemp;
      }
      //delay(20); // turn on for 50Hz
      //delay(2.5); // turn on for 400Hz
      delayMicroseconds(312.5); // 3200Hz
    }
    
    // determine whether impact is subconcussive or concussive
    if (maximum < MTh){
      Serial.print("\nSubconcussive Impact Logged\n");
      b_conc.writeValue("False");
    }
    else{
      Serial.print("\nConcussive Impact Logged\n");
      b_conc.writeValue("True");
    }
    // serial prints for debugging
    // Serial.print("ID: ");
    // Serial.print(ID);
    // Serial.print("\tForce: ");
    // Serial.print(maximum);
    // Serial.print("\nGyro X:");
    // Serial.print((int)gyro.gyro.x);
    // Serial.print("\tGyro Y:");
    // Serial.print((int)gyro.gyro.y);
    // Serial.print("\tGyro Z:");
    // Serial.print((int)gyro.gyro.z);


    Serial.print("\nWriting to EEPROM...\n");
    EEPROM.put(EEPROM_IDX , ID);
    EEPROM_IDX = EEPROM_IDX + sizeof(ID);
    EEPROM.put(EEPROM_IDX, maximum);
    EEPROM_IDX = EEPROM_IDX + sizeof(maximum);
    EEPROM.put(EEPROM_IDX, Gxmax);
    EEPROM_IDX = EEPROM_IDX + sizeof(double);
    EEPROM.put(EEPROM_IDX, Gymax);
    EEPROM_IDX = EEPROM_IDX + sizeof(double);
    EEPROM.put(EEPROM_IDX, Gzmax);
    EEPROM_IDX = EEPROM_IDX + sizeof(double);
    num_of_stored_dataset ++;
    EEPROM.put(0,num_of_stored_dataset);

    Serial.print("\nData Transmitting Through BLE...\n");
    String maxx = double2string(maximum, 2);
    String Gx = double2string(Gxtemp, 2);
    String Gy = double2string(Gytemp, 2);
    String Gz = double2string(Gztemp, 2);
//    String Gx = String(Gxtemp, 2);
//    String Gy = String(Gytemp, 2);
//    String Gz = String(Gztemp, 2);
    Serial.println(Gx);
    b_force.writeValue(maxx + "g");
    b_rot_x.writeValue(Gx + "dps");
    b_rot_y.writeValue(Gy + "dps");
    b_rot_z.writeValue(Gz + "dps");
    



  //delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
  //delay(2.5); // delay for 400Hz
  delayMicroseconds(312.5); // 3200Hz
  }
}
