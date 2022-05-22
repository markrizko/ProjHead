/*
  Project Headshot Prototype 3 Arduino Code
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
#include <Adafruit_ISM330DHCX.h>
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_ISM330DHCX ism330dhcx;

bool conn = false;
BLEDevice central;
BLEService impactService("f6a40646-f930-4075-b0f6-3da69ffce7aa"); // UUID for BLE service
BLEStringCharacteristic b_ID("f6a40646-f930-4075-b0f6-3da69ffce7-1d", BLERead, 3); // inidiviual characteristic UUIDs
BLEStringCharacteristic b_conc("f6a40646-f930-4075-b0f6-3da69ffce7-c4c", BLERead, 5);
BLEStringCharacteristic b_force("f6a40646-f930-4075-b0f6-3da69ffce7-4c", BLERead, 8);
BLEStringCharacteristic b_rot_x("f6a40646-f930-4075-b0f6-3da69ffce7-701", BLERead, 8);
BLEStringCharacteristic b_rot_y("f6a40646-f930-4075-b0f6-3da69ffce7-702", BLERead, 8);
BLEStringCharacteristic b_rot_z("f6a40646-f930-4075-b0f6-3da69ffce7-703", BLERead, 8);


//QwiicKX132 kxAccel;
QwiicKX134 kxAccel; // Uncomment this if using the KX134 - check your board
//if unsure.
outputData myData; // This will hold the accelerometer's output.
double force; // force is constantly updated
double gravity = 9.8; //gravity = 9.8 m/s^2
double mTh = 6.5; // mTh is min threshold value for subconcussive impact
double MTh = 12; // MTh is threshold for concussive impact
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

String double2string(double n, int ndec) {
    String r = "";

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

  b_ID.writeValue(String(ID));
  b_force.writeValue("0.0");
  b_rot_x.writeValue("0");
  b_rot_y.writeValue("0");
  b_rot_z.writeValue("0");
  b_conc.writeValue("False");

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

   //EEPROM.init(); // initialize EEPROM
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

  Advertise();
  serviceSetup();

}


void loop() {

  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  sensors_event_t gyro;
  sensors_event_t accel;
  sensors_event_t tempr;
  ism330dhcx.getEvent(&accel, &gyro, &tempr);

if (!conn){
      // listen for BLE peripherals to connect:
      central = Connect();
    }

    

 /* myData = kxAccel.getAccelData();
*/
  force = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) / gravity;
  if (!central.connected() && conn){
      conn = false;
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      
    }
// advanced threshold logging
    maximum = force;
//    Gxmax = (int)lsm.gyroData.x;
//    Gymax = (int)lsm.gyroData.y;
//    Gzmax = (int)lsm.gyroData.z;
    for (int i = 0; i < per100-1; ++i){
      temp = sqrt((pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2))) / gravity;
//      Gxtemp = (int)lsm.gyroData.x;
//      Gytemp = (int)lsm.gyroData.y;
//      Gztemp = (int)lsm.gyroData.z;
      if (maximum < temp){
       maximum = temp;
      }
//      if (Gxmax < Gxtemp){
//       Gxmax = Gxtemp;
//      }
//      if (Gymax < Gytemp){
//       Gymax = Gytemp;
//      }
//      if (Gzmax < Gztemp){
//       Gzmax = Gztemp;
//      }
      //delay(20); // turn on for 50Hz
      delay(2.5); // turn on for 400Hz
    }
    
    if (maximum < MTh){
      Serial.print("\nSubconcussive Impact Logged\n");
      b_conc.writeValue("False");
    }
    else{
      Serial.print("\nConcussive Impact Logged\n");
      b_conc.writeValue("True");
    }
    Serial.print("ID: ");
    Serial.print(ID);
    Serial.print("\tForce: ");
    Serial.print(maximum);
    Serial.print("\nGyro X:");
    Serial.print((int)gyro.gyro.x);
    Serial.print("\tGyro Y:");
    Serial.print((int)gyro.gyro.y);
    Serial.print("\tGyro Z:");
    Serial.print((int)gyro.gyro.z);
    Serial.print("\nWriting to EEPROM...\n");
    Serial.print("\nData Transmitting Through BLE...\n");
    //EEPROM.put(EEPROM_IDX , ID);
    //EEPROM_IDX = EEPROM_IDX + sizeof(ID);
    //EEPROM.put(EEPROM_IDX, maximum);
    //EEPROM_IDX = EEPROM_IDX + sizeof(maximum);
    String maxx = double2string(maximum, 2);
    b_force.writeValue(maxx + "g");
    //EEPROM.put(EEPROM_IDX, (int)gyro.gyro.x);
    //EEPROM_IDX = EEPROM_IDX + sizeof(int);
    b_rot_x.writeValue(String((int)gyro.gyro.x) + "dps");
    //EEPROM.put(EEPROM_IDX, (int)gyro.gyro.y);
    //EEPROM_IDX = EEPROM_IDX + sizeof(int);
    b_rot_y.writeValue(String((int)gyro.gyro.y) + "dps");
    //EEPROM.put(EEPROM_IDX, (int)gyro.gyro.z);
    //EEPROM_IDX = EEPROM_IDX + sizeof(int);
    b_rot_z.writeValue(String((int)gyro.gyro.z) + "dps");
    //num_of_stored_dataset ++;
    //EEPROM.put(0,num_of_stored_dataset);



    

  //delay(20); // Delay should be 1/ODR (Output Data Rate), default is 50Hz
  delay(2.5); // delay for 400Hz

}
