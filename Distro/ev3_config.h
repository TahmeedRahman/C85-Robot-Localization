/*
 * ev3_config.h -- one place for your robot's settings and test-safety helpers.
 *
 * NOTE: test tooling added by the team; not part of the starter code.
 *
 * Include it from every test file (and, if you like, from EV3_Localization.h):
 *      #include "ev3_config.h"
 *
 * It provides:
 *   1. HEXKEY and the motor / sensor port names       (edit them below, once)
 *   2. robot_connect() / robot_shutdown()             (use them in your test main)
 *      -> Ctrl+C, kill, closing the terminal, or a crash (segfault) all make
 *         the robot stop every motor and close the connection before exiting.
 *   3. A section where you list prototypes of functions you want to reuse
 *      from the main program.
 */

#ifndef EV3_CONFIG_H
#define EV3_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "EV3_RobotControl/btcomm.h"

/* ------------------------------------------------------------------------ */
/*  1. EDIT ONCE: your robot                                                */
/* ------------------------------------------------------------------------ */

#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D2:17"     /* <-- your EV3's Bluetooth hex id */
#endif

/* Motors (MOTOR_A .. MOTOR_D) -- change to match how you wired the robot */
#define LEFT_MOTOR       MOTOR_B
#define RIGHT_MOTOR      MOTOR_C
#define SLIDE_MOTOR      MOTOR_D

/* Sensors (PORT_1 .. PORT_4) -- change to match how you wired the robot */
#define COLOUR_PORT      PORT_1
#define GYRO_PORT        PORT_2
#define ULTRASONIC_PORT  PORT_4
#define TOUCH_PORT       PORT_3

/* ------------------------------------------------------------------------ */
/*  2. Connection + safety (no need to edit)                                */
/* ------------------------------------------------------------------------ */

static volatile sig_atomic_t robot_connected = 0;
static volatile sig_atomic_t robot_stopping  = 0;

/* Runs on Ctrl+C (SIGINT), kill (SIGTERM), closed terminal (SIGHUP) and
 * crashes (SIGSEGV). It stops ALL motors, closes the socket and exits, even
 * if your test is stuck in a loop that never checks anything.
 * A second Ctrl+C exits immediately (in case the EV3 isn't answering).     */
static void robot_emergency_stop(int sig)
{
 static const char msg[] = "\n[ev3_config] stopping: all motors stopped, connection closed\n";

 if (robot_stopping) _exit(128 + sig);
 robot_stopping = 1;
 if (robot_connected)
 {
  BT_all_stop(1);                       /* 1 = active brake */
  BT_close();
 }
 if (write(STDERR_FILENO, msg, sizeof(msg) - 1) < 0) { /* nothing to do */ }
 _exit(128 + sig);
}

/* Call first thing in main(). Returns 0 on success, -1 if the EV3 can't be reached. */
static inline int robot_connect(void)
{
 signal(SIGINT,  robot_emergency_stop);
 signal(SIGTERM, robot_emergency_stop);
 signal(SIGHUP,  robot_emergency_stop);
 signal(SIGSEGV, robot_emergency_stop);

 if (BT_open(HEXKEY) != 0)
 {
  fprintf(stderr, "Could not connect to the EV3 (%s). Is it on and paired?\n", HEXKEY);
  return -1;
 }
 robot_connected = 1;
 return 0;
}

/* Call at the end of main(): stops all motors and closes the connection. */
static inline void robot_shutdown(void)
{
 if (!robot_connected) return;
 robot_connected = 0;
 BT_all_stop(1);
 BT_close();
}

/* Sleep helper (milliseconds) */
static inline void msleep(int ms) { usleep((useconds_t)ms * 1000); }

/* ------------------------------------------------------------------------ */
/*  3. Prototypes of functions you want to reuse in the main program        */
/*     (add one line per function once it works in its test file)           */
/* ------------------------------------------------------------------------ */

/* Example:
 * int  read_colour_clean(void);
 * void turn_left_90(void);
 */

#endif
