/**
 * @file fcc_test.c
 * @brief Test of FCC module.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <rrosace_fcc.h>
#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

#define MODULE "FCC"

static int test_step_func();

static int test_step_func() {
  int ret = EXIT_FAILURE;
  rrosace_fcc_t *p_fcc_com = rrosace_fcc_new();
  rrosace_fcc_t *p_fcc_mon = rrosace_fcc_new();
  rrosace_mode_t mode = RROSACE_ALTITUDE_HOLD;
  double h_f = 0.0;
  double vz_f = 0.0;
  double va_f = 0.0;
  double q_f = 0.0;
  double az_f = 0.0;
  double h_c = 0.0;
  double vz_c = 0.0;
  double va_c = 0.0;
  double delta_e_c;
  double delta_th_c;
  int other_master_in_law = 0;
  const double freq = RROSACE_FCC_DEFAULT_FREQ;
  unsigned int relay_delta_e_c;
  unsigned int relay_delta_th_c;
  unsigned int master_in_law;

  if (!p_fcc_com || !p_fcc_mon) {
    goto out;
  }

  ret = rrosace_fcc_com_step(p_fcc_com, mode, h_f, vz_f, va_f, q_f, az_f, h_c,
                             vz_c, va_c, &delta_e_c, &delta_th_c, 1. / freq);
  if (ret == EXIT_FAILURE) {
    goto out;
  }
  ret = rrosace_fcc_mon_step(p_fcc_mon, mode, h_f, vz_f, va_f, q_f, az_f, h_c,
                             vz_c, va_c, delta_e_c, delta_th_c,
                             other_master_in_law, &relay_delta_e_c,
                             &relay_delta_th_c, &master_in_law, 1. / freq);
  if (ret == EXIT_FAILURE) {
    goto out;
  }

out:
  rrosace_fcc_del(p_fcc_com);
  rrosace_fcc_del(p_fcc_mon);

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
