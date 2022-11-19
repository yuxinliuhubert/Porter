/************************************************
 * This sketch simulates a driving scenerio 
 * created 14 Nov 2018
 * by Ajala E Oladapo
 * @ InvenTech inc.
 * 
 * L298N takes a minimum of 12V and also gives out
 * 5V which can be used to power the InventOne brd.
 * We don't need an enable pin here because we assume
 * the two jumpers are plugged in.
************************************************/

#include <Drive.h>  //Include the Drive library
#include <analogWrite.h>
//Define L298N pin mappings
const int IN1 = 19;
const int IN2 = 21;
const int IN3 = 5;
const int IN4 = 18;
int D = 0;
int prevD = 0;
#define POT 14

const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 255;

Drive drive(IN1, IN2, IN3, IN4);  //Create an instance of the function

void setup() {
//Begin serial communication with monitor
  Serial.begin(115200);
  pinMode(POT, INPUT);
  
  Serial.println("Welcome to smart world");
}

void loop() {
  D = map(analogRead(POT), 0, 4095, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);
//D = 0;

Serial.println(D);


    //Ensure that you don't go past the maximum possible command
    if (D > MAX_PWM_VOLTAGE) {
      D = MAX_PWM_VOLTAGE;
    }
    else if (D < -MAX_PWM_VOLTAGE) {
      D = -MAX_PWM_VOLTAGE;
    }

     if (abs(prevD - D) > 5) {
//      motors.setSpeeds(-D, D);
      if (D >= 0) {
        drive.moveForward(D);
      } else {
        drive.moveBackward(-D);
      }
      prevD = D;
      
      
//  motors.stopIfFault(); 
    }
//      motors.setSpeeds(-D, D);
//      prevD = D;
  
//  drive.moveForward(255);
//  Serial.println("Forward");
//  delay(1000);
//  drive.moveForward(0);
//  delay(100000);
//  drive.moveBackward(255);
//  Serial.println("Backward");
//  delay(3000);
//  drive.turnRight(255);
//  Serial.println("Right");
//  delay(3000);
//  drive.turnLeft(255);
//  Serial.println("Left");
//  delay(3000);
}
