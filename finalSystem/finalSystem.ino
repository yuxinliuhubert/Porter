
/***************************************************
  HUSKYLENS An Easy-to-use AI Machine Vision Sensor
  <https://www.dfrobot.com/product-1922.html>

***************************************************
  This example shows how to play with line tracking.

  Created 2020-03-13
  By [Angelo qiao](Angelo.qiao@dfrobot.com)

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
****************************************************/

/***********Notice and Trouble shooting***************
  1.Connection and Diagram can be found here
  <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
  2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

// Libraries
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "PIDConfig.h"
#include <string.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
#include <Drive.h>  //Include the Drive library
#include "analogWrite.h"
#include "LIDARLite_v4LED.h"
#include "HX711.h"
#include <PID_v1.h>;

// pin definitions
//Define L298N pin mappings
#define IN1 32
#define IN2 27
#define IN3 25
#define IN4 26


#define RX 15
#define TX 4

// #define rightEncoderY 34
// #define rightEncoderW 35

#define leftEncoderY 39
#define leftEncoderW 36

#define rightEncoderY 34
#define rightEncoderW 35
// NOTE: right encoder is defined opposite to the left encoder because they spin oppopsite to reach same effect

// #define leftEncoderY 36
// #define leftEncoderW 39
#define LED_LEFT 2
#define LC_RIGHT_DT 22
#define LC_RIGHT_SCK 23
#define LC_LEFT_DT 13
#define LC_LEFT_SCK 21

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"

#define ZUMO_FAST 255


HX711 leftScale;
HX711 rightScale;

#define WEIGHT_LIMIT 8

float leftWeight = 0.0;
float rightWeight = 0.0;
// state machine set up
// state 0 default machine follow mode; state 1 manual override
int state;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
int NOM_PWM_VOLTAGE = 150;

int xOrigin = SCREEN_X_CENTER;

int lD = 0;
int rD = 0;
int prevLD = 0;
int prevRD = 0;
boolean remoteButtonPressed = false;
boolean myPins[] = { 0, 0, 0, 0, 0 };

// double setPoint;
// double input;
//double output;
//PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);
// int iError = 0;
int prevResult = 160;
// Huskylens top left (0, 0), bottom right (320, 240), (W, H)



SoftwareSerial mySerial(RX, TX);  // RX, TX
//HUSKYLENS green line >> Pin 10 1st; blue line >> Pin 11 2nd

// Motor info
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;
int omegaSpeed = 0;
int omegaDes = 0;
int omegaMax = 18;  // CHANGE THIS VALUE TO YOUR MEASURED MAXIMUM SPEED
int dir = 1;
int potReading = 0;

//int Kp = 80;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
//int Ki = 2;

//Setup interrupt variables ----------------------------
volatile int leftCount = 0;              // encoder count
volatile int rightCount = 0;             // encoder count
volatile bool interruptCounter = false;  // check timer interrupt 1
volatile bool deltaT = false;            // check timer interrupt 2
int totalInterrupts = 0;                 // counts the number of triggering of the alarm
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

int initialWidth = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;
Drive drive(IN1, IN2, IN3, IN4);  //Create an instance of the function
// LIDARLite_v4LED myLIDAR;          //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  leftCount = leftEncoder.getCount();
  rightCount = rightEncoder.getCount();
  leftEncoder.clearCount();
  rightEncoder.clearCount();
  deltaT = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}


HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
int ID1 = 1;
void printResult(HUSKYLENSResult result);


// BLE set up
//boolean BLEConnected = false;
//boolean BLECurrentConnection = false;
String pressedOr = "";

uint8_t buttNum;

//SoftwareSerial mySerial(22,23);
//HUSKYLENS green line >> Pin 2 or SCL2; blue line >> Pin 23 SDA
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1)
    ;
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t *data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

void setup() {
  Serial.begin(115200);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  leftScale.begin(LC_LEFT_DT, LC_LEFT_SCK);
  rightScale.begin(LC_RIGHT_DT, LC_RIGHT_SCK);

  digitalWrite(LED_LEFT, HIGH);

  state = 0;
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(100);
  //
  //
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(100);
}



void Task1code(void *parameter) {
  // motor core setup function
  motorCoreSetup();

  // motor core loop function
  for (;;) {

    if (leftScale.is_ready()) {
      leftWeight = leftScale.read()*2.0964e-05+0.5336;
    }
    if (rightScale.is_ready()) {
      rightWeight = rightScale.read()*1.9881e-05+-0.1150;
    }
    if (leftWeight + rightWeight > WEIGHT_LIMIT) {
      digitalWrite(LED_LEFT, HIGH);
    } else {
      digitalWrite(LED_LEFT, LOW);
    }

    switch (state) {
      case 0:
        state0MotorCore();
        break;
      case 1:
        state1MotorCore();
        break;
    }
    // important so that the watchdog bug doesnt get triggered
    vTaskDelay(10);
  }
  vTaskDelete(NULL);
}


void Task2code(void *pvParameters) {

  // the setup function on command core
  commandCoreSetup();

  // the loop function on command core
  while (1) {
    switch (state) {
      case 0:
        state0CommandCore();
        break;
      case 1:
        state1CommandCore();
        break;
    }

    //    vTaskDelay(1);
  }
  vTaskDelete(NULL);
}

void loop() {
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}

void setSpeeds(int leftVDes, int rightVDes) {
  if (deltaT) {

    portENTER_CRITICAL(&timerMux1);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux1);

    int leftVDifference = leftVDes - leftCount;
    int rightVDifference = rightVDes - rightCount;
    int dLError = leftVDifference - prevLeftDifference;
    int dRError = rightVDifference - prevRightDifference;
    //    dLError = 0;
    //   dRError = 0;
    prevLeftDifference = leftVDifference;
    prevRightDifference = rightVDifference;
    //  Serial.println(String() + F("left difference is: ") + leftVDifference + F(", right difference is: ") + rightVDifference);
    //Serial.println(String() + F("left count is: ") + leftCount + F(", right count is: ") + rightCount);
    //Serial.println(String() + F("left Error is: ") + dLError + F(", right error is: ") + dRError);
    iLError += leftVDifference;
    iRError += rightVDifference;

    if (abs(iLError) >= sIMax) {
      if (iLError < 0) {
        iLError = -sIMax;
      } else {
        iLError = sIMax;
      }
    }

    if (abs(iRError) >= sIMax) {
      if (iRError < 0) {
        iRError = -sIMax;
      } else {
        iRError = sIMax;
      }
    }

    int leftD = calculateD(leftVDifference, iLError, dLError);
    int rightD = calculateD(rightVDifference, iRError, dRError);


    //      Serial.println(String() + F("iLError is: ") + iLError + F(", iRError is: ") + iRError);
// 
        //  Serial.println(String() + F("left input is: ") + leftD + F(", right input is: ") + rightD);
    Serial.println(String() + F("left speed is: ") + leftCount + F(", right speed is: ") + rightCount);


    if (leftD >= MAX_PWM_VOLTAGE) {
      leftD = MAX_PWM_VOLTAGE;
    }
    if (leftD <= -MAX_PWM_VOLTAGE) {
      leftD = -MAX_PWM_VOLTAGE;
    }

    if (rightD >= MAX_PWM_VOLTAGE) {
      rightD = MAX_PWM_VOLTAGE;
    }
    if (rightD <= -MAX_PWM_VOLTAGE) {
      rightD = -MAX_PWM_VOLTAGE;
    }
    if (leftD > 0) {
      //    analogWrite(leftPWM, rightD);
      // analogWrite(IN1, LOW);
      //    analogWrite(IN2, rightD);
      analogWrite(IN1, leftD);
      analogWrite(IN2, LOW);

    } else if (leftD < 0) {
      analogWrite(IN1, LOW);
      analogWrite(IN2, -leftD);
        //   analogWrite(IN1, -rightD);
        //  analogWrite(IN2, LOW);
    } else {
      analogWrite(IN1, LOW);
      analogWrite(IN2, LOW);
    }
    if (rightD > 0) {
      // analogWrite(IN3, LOW);
      // analogWrite(IN4, leftD);


      
         analogWrite(IN3, rightD);
         analogWrite(IN4, LOW);
    } else if (rightD < 0) {
         analogWrite(IN3, LOW);
         analogWrite(IN4, -rightD);


      // analogWrite(IN3, -leftD);
      // analogWrite(IN4, LOW);
    } else {
      analogWrite(IN3, LOW);
      analogWrite(IN4, LOW);
    }
  }
}

void stopMoving() {
  analogWrite(IN1, LOW);
  analogWrite(IN2, LOW);
  analogWrite(IN3, LOW);
  analogWrite(IN4, LOW);
}
//
int calculateD(int difference, int iError, int dError) {
  return Ksp * difference + Ksi * iError + Ksd * dError;
}


// Motor driving core (core 0) code

void motorCoreSetup() {
  mySerial.begin(9600);
  // PID variable set up
  // setPoint = 0;
  //  myPID.SetMode(AUTOMATIC);
  //  myPID.SetTunings(Kp, Ki, Kd);

  // sweepTimeComp = millis();
  surveyTimeComp = millis();

  ESP32Encoder::useInternalWeakPullResistors = UP;            // Enable the weak pull up resistors
  leftEncoder.attachHalfQuad(leftEncoderY, leftEncoderW);     // Attache pins for use as encoder pins
  leftEncoder.setCount(0);                                    // set starting count value after attaching

  // right motor must be attached the opposite way since it has to spin the other way 
  rightEncoder.attachHalfQuad(rightEncoderW, rightEncoderY);  // Attache pins for use as encoder pins
  rightEncoder.setCount(0);                                   // set starting count value after attaching


  timer1 = timerBegin(1, 80, true);              // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true);  // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true);          // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer1);                      // enable

  // Wire.begin(SDA, SCL);

  //check if LIDAR will acknowledge over I2C
  // if (myLIDAR.begin() == false) {
  //   Serial.println("Device did not acknowledge! Freezing.");
  //   //    while(1);
  // }

  if (!huskylens.begin(mySerial)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  Serial.println("Initializing HUSKYLENS");
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);  //Switch the algorithm to object tracking.
}



void state0MotorCore() {
  int32_t error;
  //   float newDistance;

  //getDistance() returns the distance reading in cm, slow operation
  // double newDistance = myLIDAR.getDistance();

  //  Serial.print("New distance: ");

  //  Serial.print(newDistance/100);
  //  Serial.println(" m");
  if (!huskylens.request(ID1)) {
    //    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    lD = 0;
    rD = 0;
    swept = false;
    timeReset = false;
    stopMoving();
  } else if (!huskylens.isLearned()) {
    //    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    initialWidth = 0;
    lD = 0;
    rD = 0;
    swept = false;
    timeReset = false;
    stopMoving();
  } else if (!huskylens.available()) {
    //    Serial.println(F("No block or arrow appears on the screen!"));
    if (!swept) {
      int currentTime = millis();
      if (!timeReset) {
        surveyTimeComp = currentTime;
        timeReset = true;
      }
      int initialLSpeed = 0;
      int initialRSpeed = 0;
      if (xOrigin < SCREEN_X_CENTER) {
        initialLSpeed = -SURVEY_SPEED;
        initialRSpeed = SURVEY_SPEED;
      } else if (xOrigin > SCREEN_X_CENTER) {
        initialLSpeed = SURVEY_SPEED;
        initialRSpeed = -SURVEY_SPEED;
      }
      lD = initialLSpeed;
      rD = initialRSpeed;
      // surveyTimeComp  = millis();

      if (currentTime - surveyTimeComp > SWEEP_INTERVAL / 4 && currentTime - surveyTimeComp < SWEEP_INTERVAL / 4 * 3) {
        lD = -initialLSpeed;
        rD = -initialRSpeed;
      } else if (currentTime - surveyTimeComp >= SWEEP_INTERVAL / 4 * 3 && currentTime - surveyTimeComp < SWEEP_INTERVAL) {
        lD = initialLSpeed;
        rD = initialRSpeed;
      } else if (currentTime - surveyTimeComp >= SWEEP_INTERVAL) {
        swept = true;
      }

      setSpeeds(lD, rD);
    } else {
      swept = true;
      stopMoving();
    }
    // setSpeeds(lD,rD);
  } else {
    lD = 0;
    rD = 0;
    swept = false;
    timeReset = false;
    HUSKYLENSResult result = huskylens.read();
           printResult(result);





    // horizontal tracking
    xOrigin = result.xCenter;
    int xDifference = SCREEN_X_CENTER - xOrigin;
    int xOutput = 0;
    if (abs(xDifference) > angleTolerance) {
      int dError = xDifference - prevXDifference;
      prevXDifference = xDifference;
      pErrorX += xDifference;
      if (abs(pErrorX) >= IMax) {
        if (pErrorX < 0) {
          pErrorX = -IMax;
        } else {
          pErrorX = IMax;
        }
      }
      // input = difference;


      xOutput = Kp * xDifference + Ki * pErrorX + Kd * dError;

      // Serial.println(xDifference);
      // Serial.println(xOutput);
      lD = -xOutput;
      rD = xOutput;
    }
    // else {
    //   stopMoving();
    // }




    // vertical tracking
    int yCenter = result.yCenter;
    int yDifference = SCREEN_Y_CENTER - yCenter;
    int yOutput = 0;
    if (abs(yDifference) > VERTICAL_TOLERANCE) {
      int dError = yDifference - prevYDifference;
      prevYDifference = yDifference;
      pErrorY += yDifference;
      if (abs(pErrorY) >= lIMax) {
        if (pErrorY < 0) {
          pErrorY = -lIMax;
        } else {
          pErrorY = lIMax;
        }
      }
      yOutput = Klp * yDifference + Kli * pErrorY + Kld * dError;

      lD += -yOutput;
      rD += -yOutput;
    }



    // depth perception
    if (initialWidth == 0) {
      initialWidth = result.width;
    }
    int currentWidth = result.width;
    int wDifference = initialWidth - currentWidth;
    int wOutput = 0;
    if (abs(wDifference) > WIDTH_TOLERANCE) {
      int dError = wDifference - prevWDifference;
      prevWDifference = wDifference;
      pErrorW += wDifference;
      if (abs(pErrorW) >= wIMax) {
        if (pErrorW < 0) {
          pErrorW = -wIMax;
        } else {
          pErrorW = wIMax;
        }
      }

      wOutput = Kwp * wDifference + Kwi * pErrorW + Kwd * dError;
      lD += wOutput;
      rD += wOutput;
    }




    Serial.println(String() + F("ld: ") + lD + F(", rD: ") + rD);


    // Data validation
    if (rD >= HUSKYSPEED) {
      rD = HUSKYSPEED;
    } else if (rD <= -HUSKYSPEED) {
      rD = -HUSKYSPEED;
    }
    if (lD <= -HUSKYSPEED) {
      lD = -HUSKYSPEED;
    } else if (lD >= HUSKYSPEED) {
      lD = HUSKYSPEED;
    }


    setSpeeds(lD, rD);
  }
}

void state1MotorCore() {
  if (myPins[4] == true) {
    myPins[0] = false;
    myPins[1] = false;
    myPins[2] = false;
    myPins[3] = false;
  }
  if (!myPins[0] && !myPins[1] && !myPins[2] && !myPins[3]) {
    lD = 0;
    rD = 0;
    prevLD = lD;
    prevRD = rD;
    iLError = 0;
    iRError = 0;
    stopMoving();
  } else {
    int forward = myPins[0] - myPins[1];  // 1 for forward motion, -1 backward, 0 hold position
    int left = myPins[2] - myPins[3];     // 1 for left motion, -1 right, 0 hold position

    // Serial.println(String() + forward + F(" ") + left);

    // if all four buttons held, hold position
    if (abs(forward) + abs(left) == 0) {
      setSpeeds(0, 0);
    } else {

      double leftTurningFraction = 1;
      double rightTurningFraction = 1;
      double leftLinearFraction = forward;
      double rightLinearFraction = forward;
      if (forward == 0) {
        if (left < 0) {
          leftLinearFraction = 1;
          rightLinearFraction = -1;

        } else if (left > 0) {
          leftLinearFraction = -1;
          rightLinearFraction = 1;
        }
      } else {
    

      if (left < 0) {// right turn
        leftTurningFraction = outerTurningSpeedFraction;
        rightTurningFraction = innerTurningSpeedFraction;

      } else if (left > 0) {
        leftTurningFraction = innerTurningSpeedFraction;
        rightTurningFraction = outerTurningSpeedFraction;
      }
      }



      int leftSpeed = leftTurningFraction * leftLinearFraction * vDes;
      int rightSpeed = rightTurningFraction * rightLinearFraction * vDes;
      
      //   Serial.print(leftSpeed);
      // Serial.print(" ");
      // Serial.println(rightSpeed);
      setSpeeds(leftSpeed, rightSpeed);
    }
  }
}


// command core code (core 1)
void commandCoreSetup() {
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
  ble.verbose(false);
  Serial.println(String() + F("state = ") + state);
}

void state0CommandCore() {
  // event checker
  if (ble.isConnected()) {
    state = 1;
    Serial.println(String() + F("state = ") + state);
    Serial.println(F("******************************"));

    // LED Activity command is only supported from 0.6.6
    if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
      // Change Mode LED Activity
      Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    }

    // Set Bluefruit to DATA mode
    Serial.println(F("Switching to DATA mode!"));
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("******************************"));
  }
}

void state1CommandCore() {

  // event checker
  if (ble.isConnected()) {

    // services
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len == 0) return;
    // Buttons
    if (packetbuffer[1] == 'B') {
      uint8_t buttnum = packetbuffer[2] - '0';
      buttNum = buttnum;  // 5 forward, 6 backward, 7 left, 8 right
      boolean pressed = packetbuffer[3] - '0';
      Serial.print("Button ");
      Serial.print(buttnum);
      if (pressed) {
        pressedOr = " pressed";
        switch (buttnum) {
          case 5:
            myPins[0] = true;
            break;
          case 6:
            myPins[1] = true;
            break;
          case 7:
            myPins[2] = true;
            break;
          case 8:
            myPins[3] = true;
            break;
          case 1:
            myPins[4] = true;
            break;
          case 2:
            vDes = MAX_VDES;
            break;
          case 3:
            vDes = NOM_VDES;
            break;
        }
        Serial.println(" pressed");
      } else {
        switch (buttNum) {
          case 1:
            myPins[4] = false;
            break;
          case 5:
            myPins[0] = false;
            break;
          case 6:
            myPins[1] = false;
            break;
          case 7:
            myPins[2] = false;
            break;
          case 8:
            myPins[3] = false;
            break;
        }
        pressedOr = " released";
        Serial.println(" released");
      }
    }
  } else {
    state = 0;
    Serial.println(String() + F("state = ") + state);
  }
}
