/**
 * @file flight_mode_test.c
 * @brief Test of flight mode module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_flight_mode.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "flight mode"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  const rrosace_mode_t mode_in = RROSACE_ALTITUDE_HOLD;
  rrosace_mode_t mode_out;
  rrosace_flight_mode_t *p_flight_mode = rrosace_flight_mode_new();

  if (!p_flight_mode) {
    goto out;
  }

  mode_out = rrosace_flight_mode_get_mode(p_flight_mode);
  if (mode_out != RROSACE_UNDEFINED) {
    goto out;
  }

  rrosace_flight_mode_set_mode(p_flight_mode, mode_in);
  mode_out = rrosace_flight_mode_get_mode(p_flight_mode);
  if (mode_in == mode_out) {
    ret = EXIT_SUCCESS;
  }

out:
  rrosace_flight_mode_del(p_flight_mode);

  return (ret);
}

int main() {
  int ret;

  const test_t test_step = {"step", test_step_func};
  const test_t *p_tests[2];

  p_tests[0] = &test_step;
  p_tests[1] = NULL;

  ret = exec_tests(MODULE, p_tests);

  return (ret);
}

#undef MODULE
