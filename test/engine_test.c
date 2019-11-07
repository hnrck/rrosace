/**
 * @file engine_test.c
 * @brief Test of engine module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_engine.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "engine"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  const double tau = 0.75;
  const double delta_th_c = 0.0;
  const double freq = RROSACE_ENGINE_DEFAULT_FREQ;
  const double dt = 1.0 / freq;
  double t;

  rrosace_engine_t *p_engine = rrosace_engine_new(tau);

  if (!p_engine) {
    goto out;
  }

  ret = rrosace_engine_step(p_engine, delta_th_c, &t, dt);

out:
  rrosace_engine_del(p_engine);

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
