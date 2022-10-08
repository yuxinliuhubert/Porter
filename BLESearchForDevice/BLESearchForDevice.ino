#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

const int PIN = 13;
const int CUTOFF = -40;
int scanSeconds = 1;
BLEAdvertisedDevice myDevice;
  // return bluetooth scanning object
 BLEScan* pBLEScan;

  int scanTime = 5; //In seconds
  
String compareString = "f8f24fe2-4675-11ed-b878-0242ac120002";

  
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String scannedDeviceAddress = advertisedDevice.getServiceUUID().toString().c_str();
      Serial.println(advertisedDevice.toString().c_str());
      if (scannedDeviceAddress.equals(compareString)) {
        myDevice = advertisedDevice;
//        Serial.println(myDevice.getAddress().toString().c_str());
      
    }
    }
};

void setup() {
  pinMode(PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Scanning...");
  BLEDevice::init("");

  // set scan mode to active
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value

}

void loop() {
   BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(myDevice.toString().c_str());
  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  delay(2000);
}


  
  
