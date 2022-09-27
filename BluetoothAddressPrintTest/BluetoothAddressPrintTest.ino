#include "esp_bt_main.h"
#include "esp_bt_device.h"

void setup() {
  Serial.begin(115200);
  initBluetooth();
  printDeviceAddress();

  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
 
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
// Print code here

for (int i = 0; i < 6; i++) {
 
  char str[3];
 
  sprintf(str, "%02X", (int)point[i]);
  Serial.print(str);
 
  if (i < 5){
    Serial.print(":");
  }
 
}
}
