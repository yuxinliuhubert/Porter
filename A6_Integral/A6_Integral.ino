#include <ESP32Encoder.h>
#define BIN_1 26
#define BIN_2 25
#define LED_PIN 13
#define POT 14      // CHANGE POTENTIOMETER PIN HERE TO MATCH CIRCUIT IF NEEDED

ESP32Encoder encoder;

int omegaSpeed = 0;
int omegaDes = 0;
int omegaMax = 18;   // CHANGE THIS VALUE TO YOUR MEASURED MAXIMUM SPEED
int D = 0;
int dir = 1;
int potReading = 0;

int Kp = 80;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
int Ki = 2;
int pError = 0;

int IMax = 100;

//Setup interrupt variables ----------------------------
volatile int count = 0; // encoder count
volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  interruptCounter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(POT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // sets the initial state of LED as turned-off

  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachHalfQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

  // initilize timer
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 5000000, true); // 5000000 * 1 us = 5 s, autoreload true

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable
}

void loop() {

  if (interruptCounter) {
    portENTER_CRITICAL(&timerMux0);
    interruptCounter = false;
    portEXIT_CRITICAL(&timerMux0);
    
    totalInterrupts++;

    if (totalInterrupts%2 == 0) {
      dir = -dir;
    }
  }

  if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);

      omegaSpeed = count;
      potReading = analogRead(POT); 
      omegaDes = dir*map(potReading, 0, 4095, -omegaMax, omegaMax); // PLEASE SPECIFY OMEGAMAX VALUE ABOVE

      //A6 CONTROL SECTION
      //Stand-in mapping between the pot reading and motor command.
      //CHANGE THIS SECTION FOR P AND PI CONTROL

      pError += omegaDes-omegaSpeed;
      if (abs(pError) >= IMax) {
        if (pError < 0) {
          pError = -IMax;
        } else {
          pError = IMax;
        }
      }
//      D = map(omegaDes, -omegaMax, omegaMax, -NOM_PWM_VOLTAGE, NOM_PWM_VOLTAGE);  // REPLACE THIS LINE WITH P/PI CONTROLLER CODE
      D = Kp*(omegaDes-omegaSpeed)+Ki*pError;

      //END A6 CONTROL SECTION

      //Ensure that you don't go past the maximum possible command
      if (D > MAX_PWM_VOLTAGE) {
          D = MAX_PWM_VOLTAGE;
      }
      else if (D < -MAX_PWM_VOLTAGE) {
          D = -MAX_PWM_VOLTAGE;
      }

      //Map the D value to motor directionality
      //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
      if (D > 0) {
          ledcWrite(ledChannel_1, D);
          ledcWrite(ledChannel_2, D);
      }
      else if (D < 0) {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, -D);
      }
      else {
          ledcWrite(ledChannel_1, -D);
          ledcWrite(ledChannel_2, -D);
      }

      plotControlData();
  }

}


//Other functions

void plotControlData() {
  Serial.println("Speed, Desired_Speed, pError/10, PWM_Duty/10");
  Serial.print(omegaSpeed);
  Serial.print(" ");
  Serial.print(omegaDes);
  Serial.print(" ");
  Serial.print(pError/10);
  Serial.print(" ");
  Serial.println(D/10);//PWM is scaled by 1/10 to get more intelligible grap
}
