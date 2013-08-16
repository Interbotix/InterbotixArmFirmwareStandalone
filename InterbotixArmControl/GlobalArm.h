
#ifndef GLOBALARM_H
#define GLOBALARM_H

// ArmStatus values for ID Packet
#define CART3D    0
#define CART3D90  1
#define CYL3D     2
#define CYL3D90   3
#define BCK       4


//=============================================================================
//=============================================================================
// Define some Min and Maxs for IK Movements...
//                y   Z
//                |  /
//                |/
//            ----+----X (X and Y are flat on ground, Z is up in air...
//                |
//                |
//=============================================================================
//=============================================================================

//=============================================================================
// Reactor Global Constraints & Work Area Definition
//=============================================================================
#ifdef PINCHER

#define ARMID       1
#define CNT_SERVOS  5 //(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0]))

/* Servo IDs */
enum {
  SID_BASE=1, SID_SHOULDER, SID_ELBOW, SID_WRIST, SID_GRIP};

// Normal Work Area

#define IK_MAX_X  200
#define IK_MIN_X  -200

#define IK_MAX_Y  240
#define IK_MIN_Y  50

#define IK_MAX_Z  250
#define IK_MIN_Z  20

#define IK_MAX_GA  90
#define IK_MIN_GA  -90

// 90 mode Work Area

#define IK_MAX_X_90  200
#define IK_MIN_X_90  -200

#define IK_MAX_Y_90  140
#define IK_MIN_Y_90  20

#define IK_MAX_Z_90  150
#define IK_MIN_Z_90  10

#define IK_MAX_GA_90  -45
#define IK_MIN_GA_90  -90

// Define Ranges for the different servos...
#define BASE_MIN    0
#define BASE_MAX    1023

#define SHOULDER_MIN  205 
#define SHOULDER_MAX  815

#define ELBOW_MIN    205
#define ELBOW_MAX    1023

#define WRIST_MIN    205
#define WRIST_MAX    815

#define WROT_MIN     0
#define WROT_MAX     1023

#define GRIP_MIN     0
#define GRIP_MAX     512

// Define some lengths and offsets used by the arm
#define BaseHeight          140L   // (L0)
#define ShoulderLength      105L   // (L1)
#define ElbowLength         105L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         100L   // (L3)Wrist length including Wrist rotate
#define G_OFFSET            0      // Offset for static side of gripper?

#endif


//=============================================================================
// Reactor Global Constraints & Work Area Definition
//=============================================================================
#ifdef REACTOR

#define ARMID       2
#define CNT_SERVOS  8 //(sizeof(pgm_axdIDs)/sizeof(pgm_axdIDs[0]))

/* Servo IDs */
enum {
  SID_BASE=1, SID_RSHOULDER, SID_LSHOULDER, SID_RELBOW, SID_LELBOW, SID_WRIST, SID_WRISTROT, SID_GRIP};

// Normal Work Area

#define IK_MAX_X  300
#define IK_MIN_X  -300

#define IK_MAX_Y  350
#define IK_MIN_Y  50

#define IK_MAX_Z  350
#define IK_MIN_Z  20

#define IK_MAX_GA  90
#define IK_MIN_GA   -90

// 90 mode Work Area

#define IK_MAX_X_90  300
#define IK_MIN_X_90  -300

#define IK_MAX_Y_90  225
#define IK_MIN_Y_90  20

#define IK_MAX_Z_90  225
#define IK_MIN_Z_90  10

#define IK_MAX_GA_90  -45
#define IK_MIN_GA_90  -90

// Define Ranges for the different servos...
#define BASE_MIN    0
#define BASE_MAX    1023

#define SHOULDER_MIN  205 
#define SHOULDER_MAX  810

#define ELBOW_MIN    210
#define ELBOW_MAX    900

#define WRIST_MIN    200
#define WRIST_MAX    830

#define WROT_MIN     0
#define WROT_MAX     1023

#define GRIP_MIN     0
#define GRIP_MAX     512

// Define some lengths and offsets used by the arm
#define BaseHeight          110L   // (L0)
#define ShoulderLength      150L   // (L1)
#define ElbowLength         147L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         137L   // (L3)Wrist length including Wrist rotate
#define G_OFFSET            0      // Offset for static side of gripper?

#endif



//=============================================================================
// WidowX Global Constraints & Work Area Definition
//=============================================================================
#ifdef WIDOWX

#define ARMID       3
#define CNT_SERVOS  6 //(sizeof(p)/sizeof(pgm_axdIDs[0]))

/* Servo IDs */
enum {
  SID_BASE=1, SID_SHOULDER, SID_ELBOW, SID_WRIST, SID_WRISTROT, SID_GRIP};

// Normal Work Area

#define IK_MAX_X  300
#define IK_MIN_X  -300

#define IK_MAX_Y  400
#define IK_MIN_Y  50

#define IK_MAX_Z  350
#define IK_MIN_Z  20

#define IK_MAX_GA  90
#define IK_MIN_GA   -90

// 90 mode Work Area

#define IK_MAX_X_90  300
#define IK_MIN_X_90  -300

#define IK_MAX_Y_90  250
#define IK_MIN_Y_90  20

#define IK_MAX_Z_90  200
#define IK_MIN_Z_90  10

#define IK_MAX_GA_90  -45
#define IK_MIN_GA_90  -90

// Define Ranges for the different servos...
#define BASE_MIN    0
#define BASE_MAX    4095

#define SHOULDER_MIN  1024 
#define SHOULDER_MAX  3072

#define ELBOW_MIN    1024
#define ELBOW_MAX    3072

#define WRIST_MIN    1024
#define WRIST_MAX    3072

#define WROT_MIN     0
#define WROT_MAX     1023

#define GRIP_MIN     0
#define GRIP_MAX     512

// Define some lengths and offsets used by the arm
#define BaseHeight          125L   // (L0)
#define ShoulderLength      150L   // (L1)
#define ElbowLength         142L   //(L2)Length of the Arm from Elbow Joint to Wrist Joint
#define WristLength         155L   // (L3)Wrist length including Wrist rotate
#define G_OFFSET            0      // Offset for static side of gripper?

#endif



//=============================================================================
//=============================================================================
#endif