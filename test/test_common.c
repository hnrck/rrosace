/**
 * @file test_common.c
 * @brief Common functions for testing modules body.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <stdio.h>
#include <stdlib.h>

#include "test_common.h"

static int exec_test(const test_t *p_test);

static int exec_test(const test_t *p_test) {
  int ret = EXIT_FAILURE;

  if (!p_test) {
    printf("ERROR: Test does not exist.");
    goto out;
  }

  printf("TEST: %s\n", p_test->name);

  ret = p_test->function();

  printf("\tTest %s\n", ret == EXIT_FAILURE ? "failed" : "succeeded");

out:
  return (ret);
}

int exec_tests(const char test_name[], const test_t *const p_tests[]) {
  int ret = EXIT_FAILURE;
  const test_t *const *p_t;

  printf("TESTS %s\n", test_name);

  if (!p_tests) {
    goto out;
  }

  ret = 0;

  for (p_t = p_tests; *p_t != NULL; ++p_t) {
    if (exec_test(*p_t) == EXIT_FAILURE) {
      ret = EXIT_FAILURE;
    }
  }

  printf("\tTests %s", ret == EXIT_FAILURE ? "failed" : "succeeded");

out:
  return (ret);
}
