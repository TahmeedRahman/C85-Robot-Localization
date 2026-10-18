/*
 * template.c -- empty scaffold for a quick robot test.
 *
 * NOTE: test tooling added by the team; not part of the starter code.
 *
 *   1. Copy it:      cp tests/template.c tests/my_test.c      (or: make new T=my_test)
 *   2. Write your function(s) in the middle section, call them from main().
 *   3. Build + run:  make my_test   then   ./my_test
 *   4. Ctrl+C at any time stops all motors and exits (see ev3_config.h).
 *
 * When a function works and you want it in the main program: make it
 * non-static, add its prototype to ev3_config.h, and list this file in
 * MODULES in the Makefile. The #ifndef below removes this file's main()
 * when it is built into the main program.
 */

#include "ev3_config.h"
#include <time.h>

/* ---------------------- functions under test ---------------------- */

static double now_s(void)
{
 struct timespec t;
 clock_gettime(CLOCK_MONOTONIC, &t);
 return t.tv_sec + t.tv_nsec / 1e9;
}

int slide_right(int ms){
       BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, 90);
       msleep(ms);
       BT_all_stop(1);
       printf("Scanned right side\n");
      
       return 0;

}

int slide_left(int ms){
       BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, -90);
       msleep(ms);
       BT_all_stop(1);
       return 0;
}

int reset(){
       while(BT_read_touch_sensor(TOUCH_PORT)==0){
              BT_drive(SLIDE_MOTOR, SLIDE_MOTOR, -80);
       }
       BT_all_stop(1);
       slide_right(2500);
       msleep(1000);
       return 0;
}
/* --------------------------- test main ---------------------------- */
#ifndef NO_TEST_MAIN
int main(void)
{
if (robot_connect() != 0) return 1;

 slide_right(3000);
 reset();
 //slide_right(2500);
 robot_shutdown();
}
#endif
// ms should be 3500