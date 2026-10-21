/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization

 This file provides the headers for the EV3 localization starter code. Please have a look to
see what is provided by the starter. The corresponding .c file contains the implementation
as well as clearly defined *** TO DO *** sections that correspond to the parts of the
code that you have to complete in order to implement the localization algorithms.

 In a nutshell, the starter code provides:

 * Reading a map from an input image (in .ppm format). The map is bordered with red,
   must have black streets with yellow intersections, and buildings must be either
   blue, green, or be left white (no building).

 * Setting up an array with map information which contains, for each intersection,
   the colours of the buildings around it in ** CLOCKWISE ** order from the top-left.

 * Initialization of the EV3 robot (opening a socket and setting up the communication
   between your laptop and your bot)

 What you must implement:

 * All aspects of robot control:
   - Finding and then following a street
   - Recognizing intersections
   - Scanning building colours around intersections
   - Detecting the map boundary and turning around or going back - the robot must not
     wander outside the map (though of course it's possible parts of the robot will
     leave the map while turning at the boundary)

 * The histogram-based localization algorithm that the robot will use to determine its
   location in the map - this is as discussed in lecture.

 * Basic robot exploration strategy so the robot can scan different intersections in
   a sequence that allows it to achieve reliable localization

 * Basic path planning - once the robot has found its location, it must drive toward a
   user-specified position somewhere in the map.

 What you need to understand thoroughly in order to complete this project:

 * The histogram localization method as discussed in lecture. The general steps of
   probabilistic robot localization.

 * Sensors and signal management - your colour readings will be noisy and unreliable,
   you have to handle this smartly

 * Robot control with feedback - your robot does not perform exact motions, you can
   assume there will be error and drift, your code has to handle this.

 * The robot control API you will use to get your robot to move, and to acquire
   sensor data. Please see the API directory and read through the header files and
   attached documentation

 Starter code:
 F. Estrada, 2018 - for CSC C85

*/

#ifndef __localization_header
#define __localization_header

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include "./EV3_RobotControl/btcomm.h"

#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D2:17"
#endif

#define COLOUR_PORT PORT_1
#define GYRO_PORT PORT_2
#define TOUCH_PORT PORT_3
#define LEFT_MOTOR MOTOR_B
#define RIGHT_MOTOR MOTOR_C
#define SLIDE_MOTOR MOTOR_D

#define CALIBRATION_SAMPLE_COUNT 20
#define CALIBRATION_OUTPUT_FILE "colour_calibration.csv"
#define NUM_COLOURS 6
#define AMBIENT_WEIGHT 0.2

// We have localized when a belief value exceeds this threshold
#define BELIEF_THRESHOLD 0.4

#define BLACK 1
#define BLUE 2
#define GREEN 3
#define YELLOW 4
#define RED 5
#define WHITE 6

typedef struct
{
  char name[16];
  double red;
  double green;
  double blue;
  double ambient;
} Calibration;

int parse_map(unsigned char *map_img, int rx, int ry);
int robot_localization(int *robot_x, int *robot_y, int *direction);
int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y);
int find_street(void);
int drive_along_street(void);
int scan_intersection(int *tl, int *tr, int *br, int *bl);
int turn_at_intersection(int turn_direction);
void calibrate_sensor(void);
unsigned char *readPPMimage(const char *filename, int *rx, int *ry);

/**
 * Custom Helper Functions
 **/

// Movement
void drive(int max_ms, int direction);
int drive_straight_pid(int max_ms, int stop_at_yellow, int direction);
int driveStraightPID(int max_ms, int direction);
int turn_by_gyro(int target_deg);
void turnLeft(void);
void turnRight(void);
int turnToDirection(int *direction, int target_direction);
void centerRobotOnIntersection();

// Colour and Calibration
void printColourName(int colour);
int load_calibration(const char *filename, Calibration calibration[]);
int read_calibrated_colour(Calibration calibration[]);
void print_colour_readings(void);

// Localization
void initializeBeliefs(double beliefArray[][4]);
void normalize_beliefs(int numIntersections);
double get_highest_belief_coords(int numIntersections, int *robot_x, int *robot_y, int *direction);
void rotate_beliefs(int numIntersections, int angle);
void shiftBeliefs(int numIntersections);
void update_beliefs_based_on_intersection(int numIntersections, int topLeft, int topRight, int backRight, int backLeft);
void printBeliefs(void);

#endif
