/*

  CSC C85 - Embedded Systems - Project # 1 - EV3 Robot Localization

 This file provides the implementation of all the functionality required for the EV3
 robot localization project. Please read through this file carefully, and note the
 sections where you must implement functionality for your bot.
 You are allowed to change *any part of this file*, not only the sections marked
 ** TO DO **. You are also allowed to add functions as needed (which must also
 be added to the header file). However, *you must clearly document* where you
 made changes so your work can be properly evaluated by the TA.

 NOTES on your implementation:

 * It should be free of unreasonable compiler warnings - if you choose to ignore
   a compiler warning, you must have a good reason for doing so and be ready to
   defend your rationale with your TA.
 * It must be free of memory management errors and memory leaks - you are expected
   to develop high wuality, clean code. Test your code extensively with valgrind,
   and make sure its memory management is clean.

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

 --- OPTIONALLY but strongly recommended ---

  The starter code provides a skeleton for implementing a sensor calibration routine,
 it is called when the code receives -1  -1 as target coordinates. The goal of this
 function should be to gather informatin about what the sensor reads for different
 colours under the particular map/room illumination/battery level conditions you are
 working on - it's entirely up to you how you want to do this, but note that careful
 calibration would make your work much easier, by allowing your robot to more
 robustly (and with fewer mistakes) interpret the sensor data into colours.

   --> The code will exit after calibration without running localization (no target!)
       SO - your calibration code must *save* the calibration information into a
            file, and you have to add code to main() to read and use this
            calibration data yourselves.

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

#include "EV3_Localization.h"

#define BLACK_SUM_MAX 580
#define YELLOW_MIN 300
#define RED_MIN 300

#define BASE_POWER 25
#define KP 2.0
#define KI 0.0
#define KD 0.0
// #define LOOP_MS 50
#define I_LIMIT 20.0
#define GYRO_SIGN 1

// Turning
#define BACKUP_MS 600
#define OVERSHOOT 10

// Slider and touch sensor configuration

// Color Sensor denoising
#define MAX_SAMPLES 15
#define STILL_SAMPLES 9

int map[400][4];        // This holds the representation of the map, up to 20x20
                        // intersections, raster ordered, 4 building colours per
                        // intersection.
int sx, sy;             // Size of the map (number of intersections along x and y)
double beliefs[400][4]; // Beliefs for each location and motion direction

Calibration calibration[NUM_COLOURS];

int main(int argc, char *argv[])
{
    char mapname[1024];
    int dest_x, dest_y, rx, ry;
    unsigned char *map_image;

    memset(&map[0][0], 0, 400 * 4 * sizeof(int));
    sx = 0;
    sy = 0;

    if (argc < 4)
    {
        fprintf(stderr, "Usage: EV3_Localization map_name dest_x dest_y\n");
        fprintf(stderr, "    map_name - should correspond to a properly formatted .ppm map image\n");
        fprintf(stderr, "    dest_x, dest_y - target location for the bot within the map, -1 -1 calls calibration routine\n");
        exit(1);
    }
    strcpy(&mapname[0], argv[1]);
    dest_x = atoi(argv[2]);
    dest_y = atoi(argv[3]);

    if (dest_x == -1 && dest_y == -1)
    {
        calibrate_sensor();
        exit(1);
    }

    // Read calibrated colour values for comparison during runtime
    int calibration_count = load_calibration(CALIBRATION_OUTPUT_FILE, calibration);
    if (calibration_count == 0)
    {
        fprintf(stderr, "Unable to load colour calibration\n");
        exit(0);
    }
    else if (calibration_count == -1)
    {
        exit(0);
        // Optional, make some sort of fallback to not use read_calibrated_colour
    }

    // Your code for reading any calibration information should not go below this line //

    map_image = readPPMimage(&mapname[0], &rx, &ry);
    if (parse_map(map_image, rx, ry) == 0)
    {
        fprintf(stderr, "Unable to parse input image map. Make sure the image is properly formatted\n");
        free(map_image);
        exit(1);
    }

    if (map_image == NULL)
    {
        fprintf(stderr, "Unable to open specified map image\n");
        exit(1);
    }

    if (dest_x < 0 || dest_x >= sx || dest_y < 0 || dest_y >= sy)
    {
        fprintf(stderr, "Destination location is outside of the map\n");
        free(map_image);
        exit(1);
    }

    // Initialize beliefs - uniform probability for each location and direction
    initializeBeliefs(beliefs);

    // Open a socket to the EV3 for remote controlling the bot.
    if (BT_open(HEXKEY) != 0)
    {
        fprintf(stderr, "Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the\n");
        fprintf(stderr, " hex key for the EV3 matches the one in EV3_Localization.h\n");
        free(map_image);
        exit(1);
    }

    fprintf(stderr, "Map size: %d, %d \n", sx, sy);
    fprintf(stderr, "All set, ready to go! \n");

    /*******************************************************************************************************************************
     *
     *  TO DO - Implement the main localization loop, this loop will have the robot explore the map, scanning intersections and
     *          updating beliefs in the beliefs array until a single location/direction is determined to be the correct one.
     *
     *          The beliefs array contains one row per intersection (recall that the number of intersections in the map_image
     *          is given by sx, sy, and that the map[][] array contains the colour indices of buildings around each intersection.
     *          Indexing into the map[][] and beliefs[][] arrays is by raster order, so for an intersection at i,j (with 0<=i<=sx-1
     *          and 0<=j<=sy-1), index=i+(j*sx)
     *
     *          In the beliefs[][] array, you need to keep track of 4 values per intersection, these correspond to the belief the
     *          robot is at that specific intersection, moving in one of the 4 possible directions as follows:
     *
     *          beliefs[i][0] <---- belief the robot is at intersection with index i, facing UP
     *          beliefs[i][1] <---- belief the robot is at intersection with index i, facing RIGHT
     *          beliefs[i][2] <---- belief the robot is at intersection with index i, facing DOWN
     *          beliefs[i][3] <---- belief the robot is at intersection with index i, facing LEFT
     *
     *          Initially, all of these beliefs have uniform, equal probability. Your robot must scan intersections and update
     *          belief values based on agreement between what the robot sensed, and the colours in the map.
     *
     *          You have two main tasks these are organized into two major functions:
     *
     *          robot_localization()    <---- Runs the localization loop until the robot's location is found
     *          go_to_target()          <---- After localization is achieved, takes the bot to the specified map location
     *
     *          The target location, read from the command line, is left in dest_x, dest_y
     *
     *          Here in main(), you have to call these two functions as appropriate. But keep in mind that it is always possible
     *          that even if your bot managed to find its location, it can become lost again while driving to the target
     *          location, or it may be the initial localization was wrong and the robot ends up in an unexpected place -
     *          a very solid implementation should give your robot the ability to determine it's lost and needs to
     *          run localization again.
     *
     *******************************************************************************************************************************/

    // driveStraightOnLinePID(YELLOW, 1, 1);
    // centerRobotOnRoad();
    // turnRight();

    // printBeliefs();
    // exit(0);
    // printf("%d\n", map[0][0]);
    // printf("%d\n", map[0][1]);
    // printf("%d\n", map[0][2]);
    // printf("%d\n", map[0][3]);
    // exit(0);

    // while (1)
    // {
    //     int res = read_colour_still();
    //     printColourName(res);
    // }
    // int tr, tl, br, bl;
    // scan_intersection(&tl, &tr, &br, &bl);
    // while (1)
    // {

    //     fprintf(stderr, "Press Enter to drive to next intersection...\n");
    //     getchar();
    //     fprintf(stderr, "Driving to next intersection ... \n");
    //     driveStraightOnLinePID(YELLOW, 1, 1);

    //     fprintf(stderr, "Press Enter to center robot on intersection...\n");
    //     getchar();
    //     fprintf(stderr, "Centering robot on intersection ... \n");
    //     centerRobotOnRoad();

    //     fprintf(stderr, "Press Enter to turn right on intersection...\n");
    //     getchar();
    //     fprintf(stderr, "Turning right on intersection ... \n");
    //     turnRight();
    // }

    // exit(0);

    int robot_x = 0;
    int robot_y = 0;
    int direction = 0;

    robot_localization(&robot_x, &robot_y, &direction);

    fprintf(stderr, "We have localized! Navigating to target ...\n");

    go_to_target(robot_x, robot_y, direction, dest_x, dest_y);

    // Cleanup and exit - DO NOT WRITE ANY CODE BELOW THIS LINE
    BT_close();
    free(map_image);
    exit(0);
}

// ###################################  COLOR SENSOR ###################################//

// Denoising for color sensor readings

static int median(const int *v, int n)
{
    int tmp[MAX_SAMPLES];
    for (int i = 0; i < n; i++)
        tmp[i] = v[i]; // copy so the original order is kept

    for (int i = 0; i < n - 1; i++) // simple bubble sort
        for (int j = 0; j < n - 1 - i; j++)
            if (tmp[j] > tmp[j + 1]) // regular int comparison
            {
                int t = tmp[j];
                tmp[j] = tmp[j + 1];
                tmp[j + 1] = t;
            }

    return tmp[n / 2]; // middle value
}

// Helper function to get color from sensor reading
// int classify_colour(int R, int G, int B, int A)
// {
//     if (A < 5 && (R + G + B) < BLACK_SUM_MAX)
//         return (1); // black
//     if (R > YELLOW_MIN && G > YELLOW_MIN && B < 0.6 * (R < G ? R : G))
//         return (4); // yellow
//     if (R > RED_MIN && G < 0.5 * R && B < 0.5 * R)
//         return (5); // red: R high
//     if (G > R && G > B && A < 5)
//         return (3); // green
//     if (B > R && B > G && A < 5)
//         return (2); // blue
//     return (6);     // anything else: unknown
// }

int classify_colour(int R, int G, int B, int A)
{
    int rgb[3] = {R, G, B};
    double sensorSum = R + G + B;
    double minDistance = -1;
    int closestColour = 6;

    if (sensorSum == 0)
        return 0;

    for (int j = 0; j < NUM_COLOURS; j++)
    {
        double calibratedSum = calibration[j].red + calibration[j].green + calibration[j].blue;

        if (calibratedSum == 0)
            continue;

        double dr = (double)rgb[0] / sensorSum - calibration[j].red / calibratedSum;
        double dg = (double)rgb[1] / sensorSum - calibration[j].green / calibratedSum;
        double db = (double)rgb[2] / sensorSum - calibration[j].blue / calibratedSum;

        double ambientDifference = (double)A - calibration[j].ambient;

        double distance = dr * dr + dg * dg + db * db + AMBIENT_WEIGHT * ambientDifference * ambientDifference;

        int colour;
        if (strcmp(calibration[j].name, "Black") == 0)
            colour = BLACK;
        else if (strcmp(calibration[j].name, "Blue") == 0)
            colour = BLUE;
        else if (strcmp(calibration[j].name, "Green") == 0)
            colour = GREEN;
        else if (strcmp(calibration[j].name, "Yellow") == 0)
            colour = YELLOW;
        else if (strcmp(calibration[j].name, "Red") == 0)
            colour = RED;
        else if (strcmp(calibration[j].name, "White") == 0)
            colour = WHITE;
        else
            continue;

        if (minDistance < 0 || distance < minDistance)
        {
            minDistance = distance;
            closestColour = colour;
        }
    }

    return minDistance < 0 ? 0 : closestColour;
}

// Denoise reading
int read_colour_still(void)
{
    int R[MAX_SAMPLES], G[MAX_SAMPLES], B[MAX_SAMPLES], A[MAX_SAMPLES], cls[MAX_SAMPLES];
    int r = 0, g = 0, b = 0, a = 0, k = 0;

    BT_read_colour_RGBraw_NXT(COLOUR_PORT, &r, &g, &b, &a); // throw away the first read (stale after stopping)

    for (int tries = 0; tries < 2 * STILL_SAMPLES && k < STILL_SAMPLES; tries++)
    {
        if (BT_read_colour_RGBraw_NXT(COLOUR_PORT, &r, &g, &b, &a) == 1) // only keep successful reads
        {
            R[k] = r;
            G[k] = g;
            B[k] = b;
            A[k] = a;
            cls[k] = classify_colour(r, g, b, a);
            k++;
        }
    }
    if (k < 3)
        return (-1);
    int c = classify_colour(median(R, k), median(G, k), median(B, k), median(A, k));
    return (c);
}

void printColourName(int colour)
{
    switch (colour)
    {
    case 1:
        fprintf(stderr, "Black\n");
        break;
    case 2:
        fprintf(stderr, "Blue\n");
        break;
    case 3:
        fprintf(stderr, "Green\n");
        break;
    case 4:
        fprintf(stderr, "Yellow\n");
        break;
    case 5:
        fprintf(stderr, "Red\n");
        break;
    case 6:
        fprintf(stderr, "White\n");
        break;
    default:
        fprintf(stderr, "Unknown colour (%d)\n", colour);
        break;
    }
}

const char *getColourName(int colour)
{
    switch (colour)
    {
    case 1:
        return "Black";
    case 2:
        return "Blue";
    case 3:
        return "Green";
    case 4:
        return "Yellow";
    case 5:
        return "Red";
    case 6:
        return "White";
    default:
        return "Unknown";
    }
}

//****************** Movement *******************//

// Turning to target degree helper
int turn_by_gyro(int target_deg)
{
    int angle = 0, rate = 0;
    int dir = (target_deg > 0) ? 1 : -1;
    BT_all_stop(0);
    usleep(500000);
    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
        return (-1);
    for (int i = 0; i < 200; i++)
    {
        if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        {
            BT_all_stop(1);
            return (-1);
        }
        // printf("Current angle: %d\n, rate: %d\n", angle, rate);
        if (dir * GYRO_SIGN * angle >= abs(target_deg) - OVERSHOOT)
            break;
        BT_turn(LEFT_MOTOR, dir * 25, RIGHT_MOTOR, -dir * 25);
    }
    BT_all_stop(1);
    return (0);
}

void turnLeft(void)
{
    turn_by_gyro(-90);
}

void turnRight(void)
{
    turn_by_gyro(90);
}

// Turn to specified direction (0 is North, increasing clockwise)
// Updates pointer direction with new direction after turning
// Use this once we have localized to navigate the map
int turnToDirection(int *direction, int target_direction)
{
    int angle = (target_direction - *direction) * 90;

    if (angle > 180)
    {
        angle -= 360;
    }
    else if (angle < -180)
    {
        angle += 360;
    }

    turn_by_gyro(angle);

    *direction = target_direction;
    return 1;
}

void drive(int max_ms, int direction)
{
    int power = direction * BASE_POWER;

    BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, power);
    usleep(max_ms * 1000);
    BT_all_stop(1);
}

// Assume the robot's sensor is currently on the desired section of road/intersection (yellow/black)
// Drive such that the robot is centered, allowing for turning
void centerRobotOnRoad()
{
    drive_straight_pid(1000, 1);
}

// Assume the robot's sensor is currently on the wall (red)
void backUpRobotToIntersection()
{
    driveStraightOnLinePID(YELLOW, -1, 1);
}

// Return 1 if the sensor detects black or yellow (on a street)
int isOnStreet(void)
{
    int c = read_colour_still();
    return (c == BLACK || c == YELLOW);
}

// Attempt to locate the street by turning x degrees to the left/right
int locateAndTurnToStreet(void)
{
    int baseSearchAngle = 12;

    BT_all_stop(1);

    // Turn slightly to the left to find the street
    printf("Locate road attempt 1 \n");
    turn_by_gyro(-baseSearchAngle);
    usleep(500000);
    if (isOnStreet())
    {
        return 1;
    }

    printf("Locate road attempt 2 \n");
    // Turn slightly to the right to find the street
    turn_by_gyro(2 * baseSearchAngle);
    usleep(500000);
    if (isOnStreet())
    {
        return 1;
    }

    printf("Locate road attempt 3 \n");
    // Turn more to the left to find the street
    turn_by_gyro(-3 * baseSearchAngle);
    usleep(500000);
    if (isOnStreet())
    {
        return 1;
    }

    printf("Locate road attempt 4 \n");
    // Turn more to the right to find the street
    turn_by_gyro(4 * baseSearchAngle);
    usleep(500000);
    if (isOnStreet())
    {
        return 1;
    }

    printf("Cant't locate roate, going back to starting orientation ... \n");
    turn_by_gyro(-2 * baseSearchAngle);
    usleep(500000);

    fprintf(stderr, "Could not locate street within +/- %d degrees of the initial heading\n", 2 * baseSearchAngle);
    return 0;
}

//****************** PIDs *******************//

/*
 * PID Controller
 *
 * - Use this controller for driving in a straight line for x amount of time (good for small shifts in alignment)
 * - direction = 1 to drive fowards, direction = -1 to drive backwards
 * - No line following or colour detection logic
 *
 */
int drive_straight_pid(int max_ms, int direction)
{
    int angle = 0, rate = 0;
    double integral = 0, u;
    int lp, rp;
    int base = direction * BASE_POWER;

    struct timespec start;
    BT_all_stop(0);
    usleep(500000);

    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
    {
        BT_all_stop(1);
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &start);

    while (elapsed_ms_since(start) < max_ms)
    {
        if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        {
            BT_all_stop(1);
            return -1;
        }

        double err = GYRO_SIGN * angle;

        integral += err;
        if (integral > I_LIMIT)
            integral = I_LIMIT;
        if (integral < -I_LIMIT)
            integral = -I_LIMIT;

        u = KP * err + KI * integral + KD * GYRO_SIGN * rate;

        lp = (int)(base - u);
        rp = (int)(base + u);

        if (lp > 100)
            lp = 100;
        if (lp < -100)
            lp = -100;
        if (rp > 100)
            rp = 100;
        if (rp < -100)
            rp = -100;

        BT_turn(LEFT_MOTOR, lp, RIGHT_MOTOR, rp);
        usleep(10000);
    }

    BT_all_stop(1);
    return 0;
}

/*
 * PID Controller
 *
 * - Use this controller for driving along lines (drive until we reach a certain point)
 * - If stop_at_colour is enabled, stops at the first instance of the colour (See mapping) and return 4 (only after two consecutive readings)
 * - If stayOnStreet is enabled, attempt to relocate the street *yellow/black) if we leave it
 * - direction = 1 to drive fowards, direction = -1 to drive backwards
 * - Drives infefinitely until we reach x colour or a wall (red)
 *
 * - Stops and returns 5 if red is detected
 * - Stops and returns 0 if the time limit is reached
 */
int driveStraightOnLinePID(int stopAtColour, int stayOnStreet, int direction)
{
    int angle = 0, rate = 0;
    int R, G, B, A;
    int colour_hits = 0;
    int search_dir = -1;
    double integral = 0, u;
    int lp, rp;
    int base = direction * BASE_POWER;

    BT_all_stop(0);
    usleep(500000);

    if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
    {
        BT_all_stop(1);
        return -1;
    }

    while (1)
    {
        if (BT_read_gyro(GYRO_PORT, 0, &angle, &rate) != 1)
        {
            BT_all_stop(1);
            return -1;
        }

        double err = GYRO_SIGN * angle;

        integral += err;
        if (integral > I_LIMIT)
            integral = I_LIMIT;
        if (integral < -I_LIMIT)
            integral = -I_LIMIT;

        u = KP * err + KI * integral + KD * GYRO_SIGN * rate;

        lp = (int)(base - u);
        rp = (int)(base + u);

        if (lp > 100)
            lp = 100;
        if (lp < -100)
            lp = -100;
        if (rp > 100)
            rp = 100;
        if (rp < -100)
            rp = -100;

        if (BT_read_colour_RGBraw_NXT(COLOUR_PORT, &R, &G, &B, &A) == 1)
        {
            int c = classify_colour(R, G, B, A);

            if (c == RED)
            {
                BT_all_stop(1);
                return RED;
            }

            if (stopAtColour != 0)
            {
                if (c == stopAtColour)
                {
                    BT_all_stop(1);
                    return stopAtColour;
                }
                else
                    colour_hits = 0;
            }
        }
        else
        {
            colour_hits = 0;
        }

        if (stayOnStreet)
        {
            int street = isOnStreet();

            if (street == 0)
            {
                fprintf(stderr, "Off the street, recovering...\n");
                BT_all_stop(1);

                // Search for the street, and rotate such that we face more towards it
                if (!locateAndTurnToStreet())
                    continue;

                fprintf(stderr, "We found the street again ...\n");

                // Reset gyro and accumulated integral
                if (BT_read_gyro(GYRO_PORT, 1, &angle, &rate) != 1)
                {
                    BT_all_stop(1);
                    return -1;
                }

                integral = 0;

                // // Drive a bit forward to get back onto the street
                // drive_straight_pid(100, direction);
                continue;
            }
        }

        BT_turn(LEFT_MOTOR, lp, RIGHT_MOTOR, rp);
        usleep(10000);
    }
}

//******************   Sliding sensor mechanism *******************//

// Amount is time in ms
int scan_left(int us)
{
    BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, -90);
    usleep(us);
    BT_all_stop(1);
    int color = read_colour_still();
    return color;
}

int scan_right(int us)
{
    BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, 90);
    usleep(us);
    BT_all_stop(1);
    int color = read_colour_still();
    return color;
}
// Positions the color sensor in the middle again
int reset()
{
    while (BT_read_touch_sensor(TOUCH_PORT) == 0)
    {
        BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, -80);
    }
    BT_all_stop(1);
    scan_right(2000000);
    BT_all_stop(1);
    usleep(1000000);
    return 0;
}

//****************** Main Movement Functions *******************//

/*

    // Drive straight until we reach a road
    // Upon reaching a road, turn 90 degrees (to the right) to face onto the road
    // Upon reaching a wall, back up, turn 90 degrees, and try again
    // Assume the robot is perpendicular to the road and starts off the road

*/
int find_street(void)
{
    /*
     * This function gets your robot onto a street, wherever it is placed on the map. You can do this in many ways, but think
     * about what is the most effective and reliable way to detect a street and stop your robot once it's on it.
     *
     * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
     * bot after calling this function
     */

    int c = read_colour_still();
    usleep(1000000);
    if (c == BLACK || c == YELLOW)
    {
        fprintf(stderr, "We are already on the road, skipping find street process ...\n");
        BT_all_stop(1);
        return 1;
    }

    // Drive till we find a street
    int status = driveStraightOnLinePID(BLACK, 0, 1);

    // Found a road, turn onto the road
    if (status == BLACK)
    {
        fprintf(stderr, "We found a road, turning right onto the road ...\n");
        centerRobotOnRoad();
        turnRight();
        return 1;
    }

    // Hit a wall, back up a bit and turn right
    if (status == RED)
    {

        fprintf(stderr, "We hit a wall while trying to find a street, backing up and turning right ...\n");
        backUpRobotToIntersection();
        centerRobotOnRoad();
        turnRight();

        status = driveStraightOnLinePID(BLACK, 1, 1);

        if (status == BLACK)
        {
            fprintf(stderr, "We found a road, turning right onto the road ...\n");
            centerRobotOnRoad();
            turnRight();
            return 1;
        }
    }

    return 0;
}

/*

    // Pick a direction and drive till we reach an intersection. We must only end up at adjacent intersections
    // Return the *final* angle turned by the robot from the starting position (degrees). This value should be a multiple of 90 (+/-)

    // Loop prevention logic: Back up and turn 90 degrees left if we hit a wall, repeat till we reach a new intersection

*/
int drive_along_street(void)
{
    /*
     * This function drives your bot along a street, making sure it stays on the street without straying to other pars of
     * the map. It stops at an intersection.
     *
     * You can implement this in many ways, including a controlled (PID for example), a neural network trained to track and
     * follow streets, or a carefully coded process of scanning and moving. It's up to you, feel free to consult your TA
     * or the course instructor for help carrying out your plan.
     *
     * You can use the return value to indicate success or failure, or to inform the rest of your code of the state of your
     * bot after calling this function.
     */

    int angleTurned = 0;

    while (1)
    {
        int status = driveStraightOnLinePID(YELLOW, 1, 1);

        // We found the next intersection, return
        if (status == 4)
        {
            fprintf(stderr, "We found the next intersection ...\n");
            return angleTurned;
        }

        // We hit a wall, try again next loop
        if (status == 5)
        {
            fprintf(stderr, "We hit a wall, returning back to intersection and turning left to try again ...\n");

            // Back up to the starting intersection
            backUpRobotToIntersection();

            // Turn left 90 degrees and try another street
            centerRobotOnRoad();
            turnLeft();
            angleTurned -= 90;
        }
        else
        {
            // Something went wrong!
            BT_all_stop(1);
            return -1;
        }
    }

    return angleTurned;
}

/*

    // Scan all 4 buildings at an intersection and update respective pointers with colour mapping
    // Assume we are alraedy at the intersection when running this method
    // The robot should return to the same starting location + direction after running

*/
int scan_intersection(int *tl, int *tr, int *br, int *bl)
{
    /*
     * This function carries out the intersection scan - the bot should (obviously) be placed at an intersection for this,
     * and the specific set of actions will depend on how you designed your bot and its sensor. Whatever the process, you
     * should make sure the intersection scan is reliable - i.e. the positioning of the sensor is reliably over the buildings
     * it needs to read, repeatably, and as the robot moves over the map.
     *
     * Use the APIs sensor reading calls to poll the sensors. You need to remember that sensor readings are noisy and
     * unreliable so * YOU HAVE TO IMPLEMENT SOME KIND OF SENSOR / SIGNAL MANAGEMENT * to obtain reliable measurements.
     *
     * Recall your lectures on sensor and noise management, and implement a strategy that makes sense. Document your process
     * in the code below so your TA can quickly understand how it works.
     *
     * Once your bot has read the colours at the intersection, it must return them using the provided pointers to 4 integer
     * variables:
     *
     * tl - top left building colour
     * tr - top right building colour
     * br - bottom right building colour
     * bl - bottom left building colour
     *
     * The function's return value can be used to indicate success or failure, or to notify your code of the bot's state
     * after this call.
     */

    fprintf(stderr, "Starting intersection scan ...\n");

    // Return invalid colour values, and a zero to indicate failure (you will replace this with your code)
    int color = read_colour_still();
    if (color == 4)
    {
        // drive forward
        BT_drive(RIGHT_MOTOR, LEFT_MOTOR, 30);
        usleep(450000);
        BT_all_stop(1);
        // Scan left and right
        *(tr) = scan_right(2900000);
        usleep(1000000); // sleep for 1 second
        scan_left(2900000);
        usleep(1000000); // sleep for 1 second

        turn_by_gyro(-15);

        *(tl) = scan_left(2200000);
        // reset();
        scan_right(2200000);
        usleep(1000000); // sleep for 1 second

        turn_by_gyro(15);

        // Back to intersection
        BT_drive(RIGHT_MOTOR, LEFT_MOTOR, -30);
        usleep(450000);
        BT_all_stop(1);

        // drive back
        BT_drive(RIGHT_MOTOR, LEFT_MOTOR, -30);
        usleep(350000);
        BT_all_stop(1);
        usleep(1000000); // sleep for 1 second

        // scan back right
        *(br) = scan_right(2900000);
        usleep(1000000); // sleep for 1 second
        scan_left(2900000);

        usleep(1000000); // sleep for 1 second

        // scan  back left
        turn_by_gyro(-15);
        *(bl) = scan_left(2200000);
        scan_right(2200000);
        usleep(1000000);
        turn_by_gyro(15);

        // drive back to intersection
        BT_drive(RIGHT_MOTOR, LEFT_MOTOR, 30);
        usleep(350000);
        BT_all_stop(1);

        fprintf(stderr,
                "Intersection scanned: TL=%s, TR=%s, BR=%s, BL=%s\n",
                getColourName(*tl),
                getColourName(*tr),
                getColourName(*br),
                getColourName(*bl));
        return 1;
    }
    else
    {
        fprintf(stderr, "Robot is not on a yellow intersection, scan failed ...\n");
    }
}

int turn_at_intersection(int turn_direction)
{
    // Not going to use this
    // Assume we stop at all intersections
    // Use turnLeft() and turnRight() instead for turning
    return (0);
}

int robot_localization(int *robot_x, int *robot_y, int *direction)
{
    /*  This function implements the main robot localization process. You have to write all code that will control the robot
     *  and get it to carry out the actions required to achieve localization.
     *
     *  Localization process:
     *
     *  - Find the street, and drive along the street toward an intersection
     *  - Scan the colours of buildings around the intersection
     *  - Update the beliefs in the beliefs[][] array according to the sensor measurements and the map data
     *  - Repeat the process until a single intersection/facing direction is distintly more likely than all the rest
     *
     *  * We have provided headers for the following functions:
     *
     *  find_street()
     *  drive_along_street()
     *  scan_intersection()
     *  turn_at_intersection()
     *
     *  You *do not* have to use them, and can write your own to organize your robot's work as you like, they are
     *  provided as a suggestion.
     *
     *  Note that *your bot must explore* the map to achieve reliable localization, this means your intersection
     *  scanning strategy should not rely exclusively on moving forward, but should include turning and exploring
     *  other streets than the one your bot was initially placed on.
     *
     *  For each of the control functions, however, you will need to use the EV3 API, so be sure to become familiar with
     *  it.
     *
     *  In terms of sensor management - the API allows you to read colours either as indexed values or RGB, it's up to
     *  you which one to use, and how to interpret the noisy, unreliable data you're likely to get from the sensor
     *  in order to update beliefs.
     *
     *  HOWEVER: *** YOU must document clearly both in comments within this function, and in your report, how the
     *               sensor is used to read colour data, and how the beliefs are updated based on the sensor readings.
     *
     *  DO NOT FORGET - Beliefs should always remain normalized to be a probability distribution, that means the
     *                  sum of beliefs over all intersections and facing directions must be 1 at all times.
     *
     *  The function receives as input pointers to three integer values, these will be used to store the estimated
     *   robot's location and facing direction. The direction is specified as:
     *   0 - UP
     *   1 - RIGHT
     *   2 - BOTTOM
     *   3 - LEFT
     *
     *  The function's return value is 1 if localization was successful, and 0 otherwise.
     */

    // Could just move this into the fns themselves to avoid having this paassed in as a param so often, idk
    int numIntersections = sx * sy;

    // Drive till we reach a street
    find_street();

    while (1)
    {
        int topLeft, topRight, backRight, backLeft;

        // Drive to the next intersection available
        int angleRotated = drive_along_street();

        if (angleRotated)
        {
            // Rotate belief values by angleRotated if the robot has changed direction while driving
            rotate_beliefs(numIntersections, angleRotated);
        }

        shiftBeliefs(numIntersections);

        // Scan all 4 colours on the intersection
        int c = scan_intersection(&topLeft, &topRight, &backRight, &backLeft);
        fprintf(stderr, "we got to here %d \n", c);

        // Update beliefs based on values from scanned intersections
        update_beliefs_based_on_intersection(numIntersections, topLeft, topRight, backRight, backLeft);

        printf("we got to here 2\n");
        normalize_beliefs(numIntersections);
        printBeliefs();

        double highestBelief = get_highest_belief_coords(numIntersections, robot_x, robot_y, direction);
        // Return when we obtain a belief higher than BELIEF_THRESHOLD. This will be our localized coordinates/direction
        if (highestBelief > BELIEF_THRESHOLD)
        {
            // Pointer values robot_x, robot_y, and direction are already updated by highestBelief
            return 1;
        }

        // Drive forwards a bit to not scan the same intersection again
        centerRobotOnRoad();
        // Repeat
    }

    // We should not end up here
    return (0);
}

int go_to_target(int robot_x, int robot_y, int direction, int target_x, int target_y)
{
    /*
     * This function is called once localization has been successful, it performs the actions required to take the robot
     * from its current location to the specified target location.
     *
     * You have to write the code required to carry out this task - once again, you can use the function headers provided, or
     * write your own code to control the bot, but document your process carefully in the comments below so your TA can easily
     * understand how everything works.
     *
     * Your code should be able to determine if the robot has gotten lost (or if localization was incorrect), and your bot
     * should be able to recover.
     *
     * Inputs - The robot's current location x,y (the intersection coordinates, not image pixel coordinates)
     *          The target's intersection location
     *
     * Return values: 1 if successful (the bot reached its target destination), 0 otherwise
     */

    fprintf(stderr, "Navigating from (%d, %d) to (%d, %d)\n", robot_x, robot_y, target_x, target_y);

    // Align x coord
    while (robot_x != target_x)
    {
        int target_direction;

        // Turn left/right
        if (target_x > robot_x)
        {
            fprintf(stderr, "Turning to face Left ...\n");
            target_direction = 1;
        }
        else
        {
            fprintf(stderr, "Turning to face Right ...\n");
            target_direction = 3;
        }

        // Turn and drive to the next intersection
        centerRobotOnRoad();
        turnToDirection(&direction, target_direction);

        fprintf(stderr, "Moving horizontally toward x=%d\n", target_x);
        drive_along_street();

        // Update current x coord
        robot_x += (target_x > robot_x) ? 1 : -1;

        fprintf(stderr, "Current position: (%d, %d)\n", robot_x, robot_y);
    }

    // Align y coord
    while (robot_y != target_y)
    {
        int target_direction;

        // Turn up/down
        if (target_y > robot_y)
        {
            fprintf(stderr, "Turning to face Down ...\n");
            target_direction = 2;
        }
        else
        {
            fprintf(stderr, "Turning to face Up ...\n");
            target_direction = 0;
        }

        // Turn and drive to the next intersection
        centerRobotOnRoad();
        turnToDirection(&direction, target_direction);

        fprintf(stderr, "Moving vertically toward y=%d\n", target_y);
        drive_along_street();

        // Update current y coord
        robot_y += (target_y > robot_y) ? 1 : -1;

        fprintf(stderr, "Current position: (%d, %d)\n", robot_x, robot_y);
    }

    fprintf(stderr, "Arrived at target (%d, %d)\n", robot_x, robot_y);
    return 1;
}

void calibrate_sensor(void)
{
    fprintf(stderr, "Calibration logic has been moved, please use ./calibrate_colours instead \n");
}

int parse_map(unsigned char *map_img, int rx, int ry)
{
    /*
      This function takes an input image map array, and two integers that specify the image size.
      It attempts to parse this image into a representation of the map in the image. The size
      and resolution of the map image should not affect the parsing (i.e. you can make your own
      maps without worrying about the exact position of intersections, roads, buildings, etc.).

      However, this function requires:

      * White background for the image  [255 255 255]
      * Red borders around the map  [255 0 0]
      * Black roads  [0 0 0]
      * Yellow intersections  [255 255 0]
      * Buildings that are pure green [0 255 0], pure blue [0 0 255], or white [255 255 255]
      (any other colour values are ignored - so you can add markings if you like, those
       will not affect parsing)

      The image must be a properly formated .ppm image, see readPPMimage below for details of
      the format. The GIMP image editor saves properly formatted .ppm images, as does the
      imagemagick image processing suite.

      The map representation is read into the map array, with each row in the array corrsponding
      to one intersection, in raster order, that is, for a map with k intersections along its width:

       (row index for the intersection)

       0     1     2    3 ......   k-1

       k    k+1   k+2  ........

       Each row will then contain the colour values for buildings around the intersection
       clockwise from top-left, that is


       top-left               top-right

               intersection

       bottom-left           bottom-right

       So, for the first intersection (at row 0 in the map array)
       map[0][0] <---- colour for the top-left building
       map[0][1] <---- colour for the top-right building
       map[0][2] <---- colour for the bottom-right building
       map[0][3] <---- colour for the bottom-left building

       Color values for map locations are defined as follows (this agrees with what the
       EV3 sensor returns in indexed-colour-reading mode):

       1 -  Black
       2 -  Blue
       3 -  Green
       4 -  Yellow
       5 -  Red
       6 -  White

       If you find a 0, that means you're trying to access an intersection that is not on the
       map! Also note that in practice, because of how the map is defined, you should find
       only Green, Blue, or White around a given intersection.

       The map size (the number of intersections along the horizontal and vertical directions) is
       updated and left in the global variables sx and sy.

          Feel free to create your own maps for testing (you'll have to print them to a reasonable
          size to use with your bot).

       */

    int last3[3];
    int x, y;
    unsigned char R, G, B;
    int ix, iy;
    int bx, by, dx, dy, wx, wy; // Intersection geometry parameters
    int tgl;
    int idx;

    ix = iy = 0; // Index to identify the current intersection

    // Determine the spacing and size of intersections in the map
    tgl = 0;
    for (int i = 0; i < rx; i++)
    {
        for (int j = 0; j < ry; j++)
        {
            R = *(map_img + ((i + (j * rx)) * 3));
            G = *(map_img + ((i + (j * rx)) * 3) + 1);
            B = *(map_img + ((i + (j * rx)) * 3) + 2);
            if (R == 255 && G == 255 && B == 0)
            {
                // First intersection, top-left pixel. Scan right to find width and spacing
                bx = i; // Anchor for intersection locations
                by = j;
                for (int k = i; k < rx; k++) // Find width and horizontal distance to next intersection
                {
                    R = *(map_img + ((k + (by * rx)) * 3));
                    G = *(map_img + ((k + (by * rx)) * 3) + 1);
                    B = *(map_img + ((k + (by * rx)) * 3) + 2);
                    if (tgl == 0 && (R != 255 || G != 255 || B != 0))
                    {
                        tgl = 1;
                        wx = k - i;
                    }
                    if (tgl == 1 && R == 255 && G == 255 && B == 0)
                    {
                        tgl = 2;
                        dx = k - i;
                    }
                }
                for (int k = j; k < ry; k++) // Find height and vertical distance to next intersection
                {
                    R = *(map_img + ((bx + (k * rx)) * 3));
                    G = *(map_img + ((bx + (k * rx)) * 3) + 1);
                    B = *(map_img + ((bx + (k * rx)) * 3) + 2);
                    if (tgl == 2 && (R != 255 || G != 255 || B != 0))
                    {
                        tgl = 3;
                        wy = k - j;
                    }
                    if (tgl == 3 && R == 255 && G == 255 && B == 0)
                    {
                        tgl = 4;
                        dy = k - j;
                    }
                }

                if (tgl != 4)
                {
                    fprintf(stderr, "Unable to determine intersection geometry!\n");
                    return (0);
                }
                else
                    break;
            }
        }
        if (tgl == 4)
            break;
    }
    fprintf(stderr, "Intersection parameters: base_x=%d, base_y=%d, width=%d, height=%d, horiz_distance=%d, vertical_distance=%d\n", bx, by, wx, wy, dx, dy);

    sx = 0;
    for (int i = bx + (wx / 2); i < rx; i += dx)
    {
        R = *(map_img + ((i + (by * rx)) * 3));
        G = *(map_img + ((i + (by * rx)) * 3) + 1);
        B = *(map_img + ((i + (by * rx)) * 3) + 2);
        if (R == 255 && G == 255 && B == 0)
            sx++;
    }

    sy = 0;
    for (int j = by + (wy / 2); j < ry; j += dy)
    {
        R = *(map_img + ((bx + (j * rx)) * 3));
        G = *(map_img + ((bx + (j * rx)) * 3) + 1);
        B = *(map_img + ((bx + (j * rx)) * 3) + 2);
        if (R == 255 && G == 255 && B == 0)
            sy++;
    }

    fprintf(stderr, "Map size: Number of horizontal intersections=%d, number of vertical intersections=%d\n", sx, sy);

    // Scan for building colours around each intersection
    idx = 0;
    for (int j = 0; j < sy; j++)
        for (int i = 0; i < sx; i++)
        {
            x = bx + (i * dx) + (wx / 2);
            y = by + (j * dy) + (wy / 2);

            fprintf(stderr, "Intersection location: %d, %d\n", x, y);
            // Top-left
            x -= wx;
            y -= wy;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0)
                map[idx][0] = 3;
            else if (R == 0 && G == 0 && B == 255)
                map[idx][0] = 2;
            else if (R == 255 && G == 255 && B == 255)
                map[idx][0] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Left RGB=%d,%d,%d\n", i, j, R, G, B);

            // Top-right
            x += 2 * wx;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0)
                map[idx][1] = 3;
            else if (R == 0 && G == 0 && B == 255)
                map[idx][1] = 2;
            else if (R == 255 && G == 255 && B == 255)
                map[idx][1] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Top-Right RGB=%d,%d,%d\n", i, j, R, G, B);

            // Bottom-right
            y += 2 * wy;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0)
                map[idx][2] = 3;
            else if (R == 0 && G == 0 && B == 255)
                map[idx][2] = 2;
            else if (R == 255 && G == 255 && B == 255)
                map[idx][2] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Right RGB=%d,%d,%d\n", i, j, R, G, B);

            // Bottom-left
            x -= 2 * wx;
            R = *(map_img + ((x + (y * rx)) * 3));
            G = *(map_img + ((x + (y * rx)) * 3) + 1);
            B = *(map_img + ((x + (y * rx)) * 3) + 2);
            if (R == 0 && G == 255 && B == 0)
                map[idx][3] = 3;
            else if (R == 0 && G == 0 && B == 255)
                map[idx][3] = 2;
            else if (R == 255 && G == 255 && B == 255)
                map[idx][3] = 6;
            else
                fprintf(stderr, "Colour is not valid for intersection %d,%d, Bottom-Left RGB=%d,%d,%d\n", i, j, R, G, B);

            fprintf(stderr, "Colours for this intersection: %d, %d, %d, %d\n", map[idx][0], map[idx][1], map[idx][2], map[idx][3]);

            idx++;
        }

    return (1);
}

unsigned char *readPPMimage(const char *filename, int *rx, int *ry)
{
    // Reads an image from a .ppm file. A .ppm file is a very simple image representation
    // format with a text header followed by the binary RGB data at 24bits per pixel.
    // The header has the following form:
    //
    // P6
    // # One or more comment lines preceded by '#'
    // 340 200
    // 255
    //
    // The first line 'P6' is the .ppm format identifier, this is followed by one or more
    // lines with comments, typically used to inidicate which program generated the
    // .ppm file.
    // After the comments, a line with two integer values specifies the image resolution
    // as number of pixels in x and number of pixels in y.
    // The final line of the header stores the maximum value for pixels in the image,
    // usually 255.
    // After this last header line, binary data stores the RGB values for each pixel
    // in row-major order. Each pixel requires 3 bytes ordered R, G, and B.
    //
    // NOTE: Windows file handling is rather crotchetty. You may have to change the
    //       way this file is accessed if the images are being corrupted on read
    //       on Windows.
    //

    FILE *f;
    unsigned char *im;
    char line[1024];
    int i;
    unsigned char *tmp;
    double *fRGB;

    im = NULL;
    f = fopen(filename, "rb+");
    if (f == NULL)
    {
        fprintf(stderr, "Unable to open file %s for reading, please check name and path\n", filename);
        return (NULL);
    }
    fgets(&line[0], 1000, f);
    if (strcmp(&line[0], "P6\n") != 0)
    {
        fprintf(stderr, "Wrong file format, not a .ppm file or header end-of-line characters missing\n");
        fclose(f);
        return (NULL);
    }
    fprintf(stderr, "%s\n", line);
    // Skip over comments
    fgets(&line[0], 511, f);
    while (line[0] == '#')
    {
        fprintf(stderr, "%s", line);
        fgets(&line[0], 511, f);
    }
    sscanf(&line[0], "%d %d\n", rx, ry); // Read image size
    fprintf(stderr, "nx=%d, ny=%d\n\n", *rx, *ry);

    fgets(&line[0], 9, f); // Read the remaining header line
    fprintf(stderr, "%s\n", line);
    im = (unsigned char *)calloc((*rx) * (*ry) * 3, sizeof(unsigned char));
    if (im == NULL)
    {
        fprintf(stderr, "Out of memory allocating space for image\n");
        fclose(f);
        return (NULL);
    }
    fread(im, (*rx) * (*ry) * 3 * sizeof(unsigned char), 1, f);
    fclose(f);

    return (im);
}

int load_calibration(const char *filename, Calibration calibration[])
{
    FILE *file = fopen(filename, "r");
    if (file == NULL)
    {
        fprintf(stderr, "Cannot find calibration file %s \n", filename);
        return -1;
    }

    char line[128];
    int count = 0;

    fgets(line, sizeof(line), file);

    while (count < NUM_COLOURS &&
           fgets(line, sizeof(line), file) != NULL)
    {
        if (sscanf(line, "%15[^,],%lf,%lf,%lf,%lf",
                   calibration[count].name,
                   &calibration[count].red,
                   &calibration[count].green,
                   &calibration[count].blue,
                   &calibration[count].ambient) == 5)
        {
            count++;
        }
    }

    fclose(file);
    return count;
}

// Helper function to determine colour with calibrated values
// Return Value Colour:
//  0    No colour read
//  1    Black
//  2    Blue
//  3    Green
//  4    Yellow
//  5    Red
//  6    White
int read_calibrated_colour(Calibration calibration[])
{
    int R[STILL_SAMPLES], G[STILL_SAMPLES], B[STILL_SAMPLES], A[STILL_SAMPLES];
    int counts[7] = {0};
    int r, g, b, a;
    int k = 0;

    for (int tries = 0; tries < 2 * STILL_SAMPLES && k < STILL_SAMPLES; tries++)
    {
        if (BT_read_colour_RGBraw_NXT(COLOUR_PORT, &r, &g, &b, &a) == 1)
        {
            R[k] = r;
            G[k] = g;
            B[k] = b;
            A[k] = a;
            k++;
        }
    }

    if (k < 3)
    {
        return 0;
    }

    for (int i = 0; i < k; i++)
    {
        int rgb[3] = {R[i], G[i], B[i]};
        double minDistance = -1;
        int closestColour = 0;

        for (int j = 0; j < NUM_COLOURS; j++)
        {
            // Calculate Normalized Eucliduan distance between points to compare colours and find best match
            double sensorSum = rgb[0] + rgb[1] + rgb[2];
            double calibratedSum = calibration[j].red + calibration[j].green + calibration[j].blue;

            if (sensorSum == 0 || calibratedSum == 0)
            {
                continue;
            }

            double dr = (double)rgb[0] / sensorSum - calibration[j].red / calibratedSum;
            double dg = (double)rgb[1] / sensorSum - calibration[j].green / calibratedSum;
            double db = (double)rgb[2] / sensorSum - calibration[j].blue / calibratedSum;

            double ambientDifference = (double)A[i] - calibration[j].ambient;

            double distance = dr * dr + dg * dg + db * db + AMBIENT_WEIGHT * ambientDifference * ambientDifference;

            int colour = 0;
            if (strcmp(calibration[j].name, "Black") == 0)
                colour = BLACK;
            else if (strcmp(calibration[j].name, "Blue") == 0)
                colour = BLUE;
            else if (strcmp(calibration[j].name, "Green") == 0)
                colour = GREEN;
            else if (strcmp(calibration[j].name, "Yellow") == 0)
                colour = YELLOW;
            else if (strcmp(calibration[j].name, "Red") == 0)
                colour = RED;
            else if (strcmp(calibration[j].name, "White") == 0)
                colour = WHITE;
            else
                continue;

            if (minDistance < 0 || distance < minDistance)
            {
                minDistance = distance;
                closestColour = colour;
            }
        }

        if (closestColour != 0)
        {
            counts[closestColour]++;
        }
    }

    int bestColour = 0;
    for (int i = 1; i <= 6; i++)
    {
        if (counts[i] > counts[bestColour])
        {
            bestColour = i;
        }
    }

    // printColourName(bestColour);
    return bestColour;
}

// Normalize values in beliefs[400][4]. numIntersections represents sx * sy
void normalize_beliefs(int numIntersections)
{
    double sum = 0;

    for (int i = 0; i < numIntersections; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            sum += beliefs[i][j];
        }
    }

    for (int i = 0; i < numIntersections; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            beliefs[i][j] /= sum;
        }
    }
}

// Get coordinates (x, y and direction) of the highest belief value in beliefs[400][4]. Also return highest found belief value. numIntersections represents sx * sy
double get_highest_belief_coords(int numIntersections, int *robot_x, int *robot_y, int *direction)
{
    double highest = beliefs[0][0];
    *robot_x = 0;
    *robot_y = 0;
    *direction = 0;

    for (int i = 0; i < numIntersections; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (beliefs[i][j] > highest)
            {
                highest = beliefs[i][j];

                *robot_x = i % sx;
                *robot_y = i / sx;
                *direction = j;
            }
        }
    }

    return highest;
}

// Rotate belief values by x degrees. Parameter angle must be the angle in degrees to rotate (+/-)
void rotate_beliefs(int numIntersections, int angle)
{
    // Convert angle into positive number
    int indexesToRotate = (angle / 90 + 4) % 4;

    if (!indexesToRotate)
    {
        return;
    }

    for (int i = 0; i < numIntersections; i++)
    {
        double tempValues[4];

        for (int d = 0; d < 4; d++)
        {
            // Keep track of temp values, but rotated by indexesToRotate
            tempValues[(d + indexesToRotate) % 4] = beliefs[i][d];
        }

        for (int d = 0; d < 4; d++)
        {
            // Copy over temp values to beliefs array
            beliefs[i][d] = tempValues[d];
        }
    }
}

// Shifts beliefs values 1 unit in every respective direction to account for robot movement
void shiftBeliefs(int numIntersections)
{
    // Initialize temp belief map as small number
    double tempBeliefs[numIntersections][4];
    for (int i = 0; i < numIntersections; i++)
    {
        for (int d = 0; d < 4; d++)
        {
            tempBeliefs[i][d] = 1e-10;
        }
    }

    for (int i = 0; i < numIntersections; i++)
    {

        int x = i % sx;
        int y = i / sx;

        // Shift up if possible
        if (y > 0)
        {
            tempBeliefs[i - sx][0] += beliefs[i][0];
        }

        // Shift right if possible
        if (x < sx - 1)
        {
            tempBeliefs[i + 1][1] += beliefs[i][1];
        }

        // Shift down if possible
        if (y < sy - 1)
        {
            tempBeliefs[i + sx][2] += beliefs[i][2];
        }

        // Shift left if possible
        if (x > 0)
        {
            tempBeliefs[i - 1][3] += beliefs[i][3];
        }
    }

    for (int i = 0; i < numIntersections; i++)
    {
        for (int d = 0; d < 4; d++)
        {
            // Copy over to new array
            beliefs[i][d] = tempBeliefs[i][d];
        }
    }
}

// Update beliefs based on data from scanned intersections
void update_beliefs_based_on_intersection(int numIntersections, int topLeft, int topRight, int backRight, int backLeft)
{
    for (int i = 0; i < numIntersections; i++)
    {
        // Check all 4 possible rotations of the intersection
        for (int d = 0; d < 4; d++)
        {
            int matches = 0;

            if (map[i][(4 - d) % 4] == topLeft)
            {
                matches++;
            }

            if (map[i][(5 - d) % 4] == topRight)
            {
                matches++;
            }

            if (map[i][(6 - d) % 4] == backRight)
            {
                matches++;
            }

            if (map[i][(7 - d) % 4] == backLeft)
            {
                matches++;
            }

            // TODO: Configure these factors such that localization works in a limiited number of loops
            if (matches == 4)
            {
                // If this intersection is exists and the robot is facing the right direction, increase the belief
                beliefs[i][d] *= 3;
            }
            else if (matches == 3)
            {
                // Small adjustment in case of sensor error, close enough match
                beliefs[i][d] *= 1.2;
            }
            else
            {
                // Decrease if not matching at all
                beliefs[i][d] /= 2;
            }
        }
    }
}

// Initialize beliefs - Uniform probability for each location and direction
void initializeBeliefs(double beliefArray[][4])
{
    for (int j = 0; j < sy; j++)
    {
        for (int i = 0; i < sx; i++)
        {
            beliefArray[i + (j * sx)][0] = 1.0 / (double)(sx * sy * 4);
            beliefArray[i + (j * sx)][1] = 1.0 / (double)(sx * sy * 4);
            beliefArray[i + (j * sx)][2] = 1.0 / (double)(sx * sy * 4);
            beliefArray[i + (j * sx)][3] = 1.0 / (double)(sx * sy * 4);
        }
    }
}

// Logging method for belief map visualization
// Top left is Up
// Top right is Right
// Bottom right is Bottom
// Bottom left is Left
void printBeliefs(void)
{
    for (int y = 0; y < sy; y++)
    {
        for (int x = 0; x < sx; x++)
        {
            int i = y * sx + x;
            printf("%8.6f %8.6f", beliefs[i][0], beliefs[i][1]);
            if (x < sx - 1)
                printf(" | ");
        }
        printf("\n");

        // Print aligned cell borders
        for (int x = 0; x < sx; x++)
        {
            printf("-----------------");
            if (x < sx - 1)
                printf("+");
        }
        printf("\n");

        for (int x = 0; x < sx; x++)
        {
            int i = y * sx + x;
            printf("%8.6f %8.6f", beliefs[i][3], beliefs[i][2]);
            if (x < sx - 1)
                printf(" | ");
        }
        printf("\n\n");
    }
}

// Helper fn for timer
long elapsed_ms_since(struct timespec start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    return (now.tv_sec - start.tv_sec) * 1000L + (now.tv_nsec - start.tv_nsec) / 1000000L;
}