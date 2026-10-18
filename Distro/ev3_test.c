/*
 * ev3_test.c - Hardware bring-up tool for the EV3 localization project.
 * Menu-driven: test motors, drive, turn, and every sensor, one at a time.
 *
 * Build (from project folder that contains EV3_RobotControl/):
 *   make ev3_test
 * Run:
 *   ./ev3_test            (or ./ev3_test 00:16:53:AA:BB:CC to override the EV3 address)
 *
 * Press Ctrl-C during any live read loop to stop it (motors are stopped for you).
 */
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "EV3_RobotControl/btcomm.h"

#ifndef HEXKEY
#define HEXKEY "00:16:53:55:D2:17"      /* <-- your EV3's Bluetooth address */
#endif

/* ---- EDIT THESE to match how YOU plugged things in ---- */
#define LEFT_MOTOR    MOTOR_B
#define RIGHT_MOTOR   MOTOR_C
#define COLOUR_PORT   PORT_1             /* EV3 or NXT colour sensor */
#define GYRO_PORT     PORT_2
#define US_PORT       PORT_3
#define TOUCH_PORT    PORT_4
/* ------------------------------------------------------- */

extern int *socket_id;           /* defined in btcomm.c, used to clean up failed connects */

static volatile sig_atomic_t stop_flag = 0;
static void on_sigint(int s) { (void)s; stop_flag = 1; }
static void msleep(int ms) { usleep(ms * 1000); }

static const char *colour_name[8] =
  {"none","black","blue","green","yellow","red","white","brown"};

/* ---------- Sound / LED ---------- */
static void test_beep(void)
{
 int tones[50][3] = {{440,250,30},{660,250,30},{880,400,30},{-1,-1,-1}};
 BT_play_tone_sequence(tones);       /* {frequency Hz, duration ms, volume 0-63} */
}

static void test_led(void)
{
 BT_set_LED_colour(LED_GREEN);  msleep(600);
 BT_set_LED_colour(LED_RED);    msleep(600);
 BT_set_LED_colour(LED_ORANGE); msleep(600);
 BT_set_LED_colour(LED_GREEN);  /* back to normal */
}

/* ---------- Motors ---------- */
static void test_each_motor(void)
{
 char m[4] = {MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D};
 const char *n = "ABCD";
 for (int i = 0; i < 4; i++) {
   printf("Motor %c forward 0.7s ...\n", n[i]);
   BT_motor_port_start(m[i], 40);
   msleep(700);
   BT_motor_port_stop(m[i], 1);
   msleep(400);
 }
 printf("Done. Note which motor moved which wheel, and which direction was 'forward'.\n");
}

static void test_drive(int power, int ms)
{
 printf("Driving at power %d for %d ms\n", power, ms);
 BT_drive(LEFT_MOTOR, RIGHT_MOTOR, power);
 msleep(ms);
 BT_all_stop(1);                     /* 1 = active brake */
}

static void test_spin(int power, int ms)
{
 printf("Spinning in place (left=+%d, right=-%d) for %d ms\n", power, power, ms);
 BT_turn(LEFT_MOTOR, power, RIGHT_MOTOR, -power);
 msleep(ms);
 BT_all_stop(1);
}

/* ---------- Sensors (live loops) ---------- */
static void loop_colour_indexed(void)
{
 printf("Indexed colour (Ctrl-C to stop)\n");
 while (!stop_flag) {
   int c = BT_read_colour_sensor(COLOUR_PORT);
   printf("\rcolour=%d (%s)      ", c, (c >= 0 && c < 8) ? colour_name[c] : "?");
   fflush(stdout);
   msleep(100);
 }
 printf("\n");
}

static void loop_colour_rgb_ev3(void)
{
 int rgb[3];
 printf("EV3 RGB raw, range ~0-1020 (Ctrl-C to stop)\n");
 while (!stop_flag) {
   if (BT_read_colour_sensor_RGB(COLOUR_PORT, rgb) == 0)
     printf("\rR=%4d G=%4d B=%4d      ", rgb[0], rgb[1], rgb[2]);
   fflush(stdout);
   msleep(100);
 }
 printf("\n");
}

static void loop_colour_rgb_nxt(void)
{
 int R, G, B, A;
 printf("NXT RGB (ambient already subtracted) (Ctrl-C to stop)\n");
 while (!stop_flag) {
   if (BT_read_colour_RGBraw_NXT(COLOUR_PORT, &R, &G, &B, &A) == 1)
     printf("\rR=%5d G=%5d B=%5d  ambient=%5d      ", R, G, B, A);
   fflush(stdout);
   msleep(100);
 }
 printf("\n");
}

static void loop_gyro(void)
{
 int ang, rate;
 printf("Gyro. Keep the robot STILL for the first call (it zeroes the angle). Rotate it by hand.\n");
 BT_read_gyro(GYRO_PORT, 1, &ang, &rate);     /* reset=1 -> current angle becomes 0 */
 while (!stop_flag) {
   if (BT_read_gyro(GYRO_PORT, 0, &ang, &rate) == 1)
     printf("\rangle=%5d deg   rate=%5d deg/s      ", ang, rate);
   fflush(stdout);
   msleep(50);
 }
 printf("\n");
}

static void loop_ultrasonic_touch(void)
{
 printf("Ultrasonic + touch (Ctrl-C to stop)\n");
 while (!stop_flag) {
   printf("\rdistance=%4d   touch=%d      ",
          BT_read_ultrasonic_sensor(US_PORT), BT_read_touch_sensor(TOUCH_PORT));
   fflush(stdout);
   msleep(100);
 }
 printf("\n");
}

/* Take n indexed readings and return the most common one (simple noise rejection). */
static int mode_colour(int n)
{
 int counts[8] = {0};
 for (int i = 0; i < n; i++) {
   int c = BT_read_colour_sensor(COLOUR_PORT);
   if (c >= 0 && c < 8) counts[c]++;
   msleep(20);
 }
 int best = 0;
 for (int i = 1; i < 8; i++) if (counts[i] > counts[best]) best = i;
 return best;
}

static void test_colour_vote(void)
{
 printf("Place sensor over a colour and press Enter (type q to quit): ");
 char buf[16];
 getchar();
 while (fgets(buf, sizeof buf, stdin) && buf[0] != 'q') {
   int c = mode_colour(15);
   printf("  15-sample vote -> %d (%s)\nNext (Enter/q): ", c, colour_name[c]);
 }
}

/* ---------- Gyro-controlled turn ---------- */
/* Turns by 'deg' degrees (sign sets direction - check on your robot which sign is
 * clockwise!). Fast at first, slows when close, brakes at the end. Returns final angle. */
static int gyro_turn(int deg, int power)
{
 int ang = 0, rate = 0, s = (deg > 0) ? 1 : -1, target = abs(deg);
 BT_read_gyro(GYRO_PORT, 1, &ang, &rate);                  /* zero reference */
 BT_turn(LEFT_MOTOR, s * power, RIGHT_MOTOR, -s * power);
 for (int i = 0; i < 2000 && !stop_flag; i++) {            /* safety limit */
   if (BT_read_gyro(GYRO_PORT, 0, &ang, &rate) != 1) break;
   int remaining = target - abs(ang);
   if (remaining <= 2) break;
   if (remaining < 25)                                     /* slow down near the end */
     BT_turn(LEFT_MOTOR, s * 20, RIGHT_MOTOR, -s * 20);
 }
 BT_all_stop(1);
 msleep(400);                                              /* let it settle, then re-read */
 BT_read_gyro(GYRO_PORT, 0, &ang, &rate);
 return ang;
}

/* ---------- Menu ---------- */
static void menu(void)
{
 printf("\n==== EV3 test menu ====\n"
        " 1 beep           2 LED colours\n"
        " 3 each motor     4 drive forward 1s (power 30)\n"
        " 5 drive back 1s  6 spin 1s\n"
        " 7 colour indexed 8 colour RGB (EV3)   9 colour RGB (NXT)\n"
        "10 gyro live     11 gyro turn 90      12 gyro turn -90\n"
        "13 ultrasonic+touch                   14 colour 15-sample vote\n"
        "15 sensor type/mode of colour port    0 quit\n> ");
}

int main(int argc, char *argv[])
{
 const char *addr = (argc > 1) ? argv[1] : HEXKEY;
 signal(SIGINT, on_sigint);
 /* Retry a few times: the first attempt often collides with the OS bringing the link up */
 int connected = 0;
 for (int attempt = 1; attempt <= 6 && !connected; attempt++) {
   fprintf(stderr, "Connection attempt %d of 6...\n", attempt);
   if (BT_open(addr) == 0) connected = 1;
   else {
     /* BT_open() leaves its socket open on failure, which blocks every retry (EBUSY).
      * Close it ourselves before trying again. socket_id is a global in btcomm.c. */
     if (socket_id) { close(*socket_id); free(socket_id); socket_id = NULL; }
     sleep(3);
   }
 }
 if (!connected) {
   fprintf(stderr, "Could not connect. Is the EV3 on, Bluetooth visible, paired, and the address right?\n");
   return 1;
 }
 int choice = -1;
 while (choice != 0) {
   stop_flag = 0;
   menu();
   if (scanf("%d", &choice) != 1) break;
   switch (choice) {
     case 1:  test_beep(); break;
     case 2:  test_led(); break;
     case 3:  test_each_motor(); break;
     case 4:  test_drive(30, 1000); break;
     case 5:  test_drive(-30, 1000); break;
     case 6:  test_spin(30, 1000); break;
     case 7:  loop_colour_indexed(); break;
     case 8:  loop_colour_rgb_ev3(); break;
     case 9:  loop_colour_rgb_nxt(); break;
     case 10: loop_gyro(); break;
     case 11: printf("final gyro angle: %d\n", gyro_turn(90, 30)); break;
     case 12: printf("final gyro angle: %d\n", gyro_turn(-90, 30)); break;
     case 13: loop_ultrasonic_touch(); break;
     case 14: test_colour_vote(); break;
     case 15: BT_get_type_mode(COLOUR_PORT); break;
     default: break;
   }
   BT_all_stop(1);   /* always leave motors stopped */
 }
 BT_all_stop(0);
 BT_close();
 return 0;
}