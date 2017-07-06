bool firstTime = true;
static const bool UseOtherRobotPose = false;
static const bool UseLaser = false;
static const double followdist = 4;
static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const double stopdist = 0.3;
static const int avoidduration = 10;
static const int numberOfPosHist = 20;
static const int numUpdateReferenceFrame = 30;

static const float Q0 = 0.05;
static const float Q1 = 0.05;
static const float Q2 = 0.09;
static const float Q3 = 0.15;
static const float Q4 = 0.19;
static const float Q5 = 1;
static const float Q6 = 1;
static const float Q7 = 0.25;
static const float Q8 = 0.25;

static const float R0 = 0.05;
static const float R1 = 0.02;
static const float R2 = 0.1;
static const float R3 = 0.1;

static const float P0 = 0.01;
static const float P1 = 0.01;
static const float P2 = 0.01;
static const float P3 = 0.01;
static const float P4 = 0.01;
static const float P5 = 1;
static const float P6 = 1;
static const float P7 = 0.5;
static const float P8 = 0.5;


// #define ROBOT_X_IDX	0
// #define ROBOT_X_IDX	1
// #define ROBOT_X_IDX	2
// #define ROBOT_X_IDX	3
// #define ROBOT_X_IDX	4

// #define ROBOT_X_IDX	0
