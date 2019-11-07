/**
 * @file flight_dynamics_test.c
 * @brief Test of flight dynamics module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_flight_dynamics.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "flight dynamics"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  rrosace_flight_dynamics_t *p_flight_dynamics = rrosace_flight_dynamics_new();
  const double delta_e = 0.0;
  const double t = 0.0;
  const double freq = RROSACE_FLIGHT_DYNAMICS_DEFAULT_FREQ;
  const double dt = 1.0 / freq;
  double h;
  double vz;
  double va;
  double q;
  double az;

  if (!p_flight_dynamics) {
    goto out;
  }

  ret = rrosace_flight_dynamics_step(p_flight_dynamics, delta_e, t, &h, &vz,
                                     &va, &q, &az, dt);

out:
  rrosace_flight_dynamics_del(p_flight_dynamics);

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
