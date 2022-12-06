#include "HX711.h"

// HX711 circuit wiring
#define LC_RIGHT_DT 22
#define LC_RIGHT_SCK 23
#define LC_LEFT_DT 13
#define LC_LEFT_SCK 21

long leftR;
long rightR;
HX711 leftScale;
HX711 rightScale;

void setup() {
  Serial.begin(115200);
  leftScale.begin(LC_LEFT_DT, LC_LEFT_SCK);
  rightScale.begin(LC_RIGHT_DT, LC_RIGHT_SCK);
}

void loop() {

  if (leftScale.is_ready()) {
    leftR = leftScale.read();
    // Serial.print("HX711 reading: ");
    // Serial.println(reading);
  } 
  if (rightScale.is_ready()) {
    rightR = rightScale.read();
  }

  Serial.print(leftR);
  Serial.print(" ");
  Serial.println(rightR);

  delay(100);
  
}
