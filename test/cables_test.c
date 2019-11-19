/**
 * @file cables_test.c
 * @brief Test of cables module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_cables.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "cables"

static int test_step_func(void);

static int test_step_func(void) {
  int ret = EXIT_FAILURE;
  size_t it;
  const double deltas_e_c[] = {1.0, 2.0};
  const double deltas_th_c[] = {1.0, 2.0};
  rrosace_cables_input_t inputs[] = {
      {0.0, 0.0, RROSACE_RELAY_CLOSED, RROSACE_RELAY_CLOSED},
      {0.0, 0.0, RROSACE_RELAY_OPENED, RROSACE_RELAY_OPENED},
  };
  rrosace_cables_output_t output;

  for (it = 0; it < sizeof(inputs) / sizeof *(inputs); ++it) {
    inputs[it].delta_e_c = deltas_e_c[it];
    inputs[it].delta_th_c = deltas_th_c[it];
  }

  rrosace_cables_step(inputs, sizeof(inputs), &output);

  ret = ((output.delta_e_c == deltas_e_c[0]) &&
         (output.delta_th_c == deltas_th_c[0]))
            ? EXIT_SUCCESS
            : EXIT_FAILURE;

  return (ret);
}

int main(void) {
  int ret;

  const test_t test_step = {"step", test_step_func};
  const test_t *p_tests[2];

  p_tests[0] = &test_step;
  p_tests[1] = NULL;

  ret = exec_tests(MODULE, p_tests);

  return (ret);
}

#undef MODULE
