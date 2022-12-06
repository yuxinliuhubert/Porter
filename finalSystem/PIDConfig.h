// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------

#define HUSKYSPEED 20

// k values for horizontal tracking
#define Kp 0.05
#define Ki 0.01
#define Kd 0.3

#define IMax 100

#define SCREEN_X_CENTER 160
#define angleTolerance 50
int prevXDifference = 0;
int pErrorX = 0;



// k value for vertical tracking
#define Klp 0.3
#define Kli 0.3
#define Kld 0.8

#define lIMax 30
#define SCREEN_Y_CENTER 120
#define VERTICAL_TOLERANCE 45
int prevYDifference = 0;
int pErrorY = 0;


// k value for width tracking
#define Kwp 0.2
#define Kwi 3
#define Kwd 0.1
#define wIMax 100
#define WIDTH_TOLERANCE 30
int prevWDifference = 0;
int pErrorW = 0;

// speeds
#define Ksp 8
#define Ksi 3
#define Ksd 0

#define sIMax 50

#define outerTurningSpeedFraction 2
#define innerTurningSpeedFraction 0.7

int iError = 0;

// 14 desireable speed when motor running

#define NOM_VDES 16
#define MAX_VDES 44
int vDes = NOM_VDES;


int prevLeftDifference = 0;
int prevRightDifference = 0;

int iLError = 0;
int iRError = 0;

#define SURVEY_SPEED 5
#define SWEEP_INTERVAL 4000
bool swept = false;
int surveyTimeComp;
bool timeReset = false;

// PWM channel 
// right motor motor 1, left motor motor 2
#define leftPWM 23
#define rightPWM 13
