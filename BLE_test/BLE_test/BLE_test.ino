#include <ArduinoBLE.h> //http://librarymanager/All#ArduinoBLE_IoT

int testinput;
bool conn = false;
BLEDevice central;

uint8_t ID = 125;

BLEService impactService("f6a40646-f930-4075-b0f6-3da69ffce7aa");
BLEByteCharacteristic b_ID("f6a40646-f930-4075-b0f6-3da69ffce7-1d", BLERead);
BLEBoolCharacteristic b_conc("f6a40646-f930-4075-b0f6-3da69ffce7-c4c", BLERead);
BLEDoubleCharacteristic b_force("f6a40646-f930-4075-b0f6-3da69ffce7-4c", BLERead);
BLEDoubleCharacteristic b_rot("f6a40646-f930-4075-b0f6-3da69ffce7-707", BLERead);


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
  impactService.addCharacteristic(b_rot);

  BLE.addService(impactService);

  b_ID.writeValue(ID);
  b_force.writeValue(0.0);
  b_rot.writeValue(0.0);
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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Advertise();

  serviceSetup();
}

void loop() {
  
    testinput = Serial.read();
//    if (testinput != -1){
//      Serial.println(testinput, DEC);
//    }
    if (testinput == 49){ // pass 1 to serial
      Serial.println("Value changed");
      b_force.writeValue(6.9);
      b_conc.writeValue(true);
    }
    else if (testinput == 50){ // pass 2 to serial
      Serial.println("Value changed");
      b_force.writeValue(3.1);
      b_conc.writeValue(false);
    }
    
    if (!conn){
      // listen for BLE peripherals to connect:
      central = Connect();
    }

    if (!central.connected() && conn){
      conn = false;
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      
    }
   
    
 }
