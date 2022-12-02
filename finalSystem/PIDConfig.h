// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// k values for horizontal tracking
#define Kp 0.05
#define Ki 0.002
#define Kd 0.6

#define IMax 100

#define SCREEN_X_CENTER 160
#define angleTolerance 50
int prevXDifference = 0;
int pErrorX = 0;


// k value for vertical tracking
#define Klp 0.02
#define Kli 0.005
#define Kld 0.5

#define lIMax 30
#define SCREEN_Y_CENTER 120
#define VERTICAL_TOLERANCE 40
int prevYDifference = 0;
int pErrorY = 0;


// speeds
#define Ksp 8
#define Ksi 3
#define Ksd 0

#define sIMax 50

#define outerTurningSpeedFraction 2
#define innerTurningSpeedFraction 0.7

int iError = 0;

// 14 desireable speed when motor running

#define NOM_VDES 14
#define MAX_VDES 44
int vDes = NOM_VDES;


int prevLeftDifference = 0;
int prevRightDifference = 0;

int iLError = 0;
int iRError = 0;



// PWM channel 
// right motor motor 1, left motor motor 2
#define leftPWM 23
#define rightPWM 13
