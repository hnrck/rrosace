/**
 * @file filters_test.c
 * @brief Test of filters module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_filters.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "filters"

static int test_one_filter(rrosace_filter_type_t type,
                           rrosace_filter_frequency_t frequency);
static int test_step_func();

static int test_one_filter(rrosace_filter_type_t type,
                           rrosace_filter_frequency_t frequency) {
  int ret = EXIT_FAILURE;
  const double h = 0;
  double h_f;
  rrosace_filter_t *p_filter = rrosace_filter_new(type, frequency);

  if (!p_filter) {
    goto out;
  }

  ret = rrosace_filter_step(p_filter, h, &h_f);

out:
  rrosace_filter_del(p_filter);

  return (ret);
}

static int test_step_func() {
  int ret = EXIT_SUCCESS;
  rrosace_filter_type_t type;

  for (type = RROSACE_ALTITUDE_FILTER;
       type <= RROSACE_VERTICAL_ACCELERATION_FILTER; ++type) {
    rrosace_filter_frequency_t frequency;
    for (frequency = RROSACE_FILTER_FREQ_100HZ;
         frequency <= RROSACE_FILTER_FREQ_25HZ; ++frequency) {
      if (test_one_filter(type, frequency) == EXIT_FAILURE) {
        ret = EXIT_FAILURE;
      }
    }
  }

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
