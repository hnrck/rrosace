/**
 * @file test_common.h
 * @brief Common functions for testing modules header.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

typedef int (*test_function_t)();

struct test {
  const char *name;
  test_function_t function;
};

typedef struct test test_t;

int exec_tests(const char /* test_name */[],
               const test_t *const /* p_tests */[]);

#endif /* TESTS_TEST_COMMON_H */
