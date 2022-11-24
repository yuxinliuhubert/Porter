// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// k values
#define Kp 0.05
#define Ki 0.01
#define Kd 0.5

#define IMax 100

#define Ksp 8
#define Ksi 3
#define Ksd 0

#define sIMax 50

// 14 desireable speed when motor running
const int vDes = 14;

int prevLeftDifference = 0;
int prevRightDifference = 0;

int iLError = 0;
int iRError = 0;


// PWM channel 
// right motor motor 1, left motor motor 2
#define leftPWM 23
#define rightPWM 13
