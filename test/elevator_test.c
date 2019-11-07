/**
 * @file elevator_test.c
 * @brief Test of elevator module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_elevator.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "elevator"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  const double omega = 0.25;
  const double xi = 0.85;
  const double delta_e_c = 0.0;
  const double freq = RROSACE_ELEVATOR_DEFAULT_FREQ;
  const double dt = 1 / freq;
  double delta_e;

  rrosace_elevator_t *p_elevator = rrosace_elevator_new(omega, xi);

  if (!p_elevator) {
    goto out;
  }

  ret = rrosace_elevator_step(p_elevator, delta_e_c, &delta_e, dt);

out:
  rrosace_elevator_del(p_elevator);

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
