/**
 * @file fcu_test.c
 * @brief Test of FCU module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_fcu.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "FCU"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  const double val_in = 1.0;
  double val_out;
  rrosace_fcu_t *p_fcu = rrosace_fcu_new();

  if (!p_fcu) {
    goto out;
  }

  rrosace_fcu_set_h_c(p_fcu, val_in);
  rrosace_fcu_set_vz_c(p_fcu, val_in);
  rrosace_fcu_set_va_c(p_fcu, val_in);

  val_out = rrosace_fcu_get_h_c(p_fcu);
  if (val_out != val_in) {
    goto out;
  }

  val_out = rrosace_fcu_get_vz_c(p_fcu);
  if (val_out != val_in) {
    goto out;
  }

  val_out = rrosace_fcu_get_va_c(p_fcu);
  if (val_out != val_in) {
    goto out;
  }

  ret = EXIT_SUCCESS;

out:
  rrosace_fcu_del(p_fcu);

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
