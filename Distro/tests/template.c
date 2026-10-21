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

/* ---------------------- functions under test ---------------------- */



/* --------------------------- test main ---------------------------- */
#ifndef NO_TEST_MAIN
int main(void)
{
 if (robot_connect() != 0) return 1;

 /* call your functions here */

 robot_shutdown();
 return 0;
}
#endif
