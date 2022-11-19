#include <A4990DualMotorDriverCarrier.h>
#include <Arduino.h>

/*
 * This example uses the A4990DualMotorDriverCarrier library to drive Motor 2 the
 * Pololu A4990 Dual Motor Driver Carrier. 
 */

#define LED_PIN 13
#define POT 14
#define LEFTMOTOR1 32
#define LEFTMOTOR2 33
#define LEFTMOTORITR 34
#define RIGHTMOTOR1 1
#define RIGHTMOTOR2 3
#define RIGHTMOTORITR 35
//#define LEFTMOTOR1 16
//#define LEFTMOTOR2 17
//#define LEFTMOTORITR 33
//#define RIGHTMOTOR1 32
//#define RIGHTMOTOR2 15
//#define RIGHTMOTORITR 27

int D = 0;
int prevD = 0;

//**************************************************
// Change this constructor to match your pinout!!!
A4990DualMotorDriverCarrier motors(LEFTMOTOR1, LEFTMOTOR2, RIGHTMOTOR1, RIGHTMOTOR2);
//**************************************************

const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(POT, INPUT);
  Serial.begin(115200);
  Serial.println("Pololu A4990 Dual Motor Driver Carrier Demo");
  
  // *************************************************
  // Uncomment one or both of the following lines if your motors' directions need to be flipped
  // motors.flipM1(true);
  // motors.flipM2(true);
  // *************************************************
  

  digitalWrite(LED_PIN, HIGH);   // Turn on LED for beautifulness
  

  //**************************************************
  // Change this constructor to match your pinout!!!
  motors.enableInterrupts(0, LEFTMOTORITR, 1, RIGHTMOTORITR); // Note that 0, 1 (the interrupt pins) stand for INT0/INT1 and NOT for PD0 or PD1
  //**************************************************


D = map(analogRead(POT), 0, 4095, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);
//D = 0;



    //Ensure that you don't go past the maximum possible command
    if (D > MAX_PWM_VOLTAGE) {
      D = MAX_PWM_VOLTAGE;
    }
    else if (D < -MAX_PWM_VOLTAGE) {
      D = -MAX_PWM_VOLTAGE;
    }

   
//      motors.setSpeeds(-D, D);
      prevD = D;
      

  motors.stopIfFault();    // It is good practice to stop if there are any errors, stopIfFault() is included for convenience
}

void loop()
{
  
//  motors.setSpeeds(0,0);
  //Stand-in mapping between the pot reading and motor command.
    D = map(analogRead(POT), 0, 4095, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);
   


    //Ensure that you don't go past the maximum possible command
    if (D > MAX_PWM_VOLTAGE) {
      D = MAX_PWM_VOLTAGE;
    }
    else if (D < -MAX_PWM_VOLTAGE) {
      D = -MAX_PWM_VOLTAGE;
    }

    if (abs(prevD - D) > 5) {
//      motors.setSpeeds(-D, D);
      prevD = D;
      Serial.println(D);
//  motors.stopIfFault(); 
    }

  
//    if (motors.changeFlagM1 == true) {
//      motors.changeFlagM1 = false;
////      Serial.println(motors.readM1Rotation());
//    }
//    if (motors.changeFlagM2 == true) {
//      motors.changeFlagM2 = false;
////      Serial.println(motors.readM2Rotation());
//    }
}
