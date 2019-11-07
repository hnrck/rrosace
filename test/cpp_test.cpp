/**
 * @file cpp_test.cc
 * @brief Test of C++ port of the RROSACE SOCPSS library.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 */

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <rrosace.h>

#if __cplusplus <= 199711L
#ifndef nullptr
#define nullptr NULL
#endif /* nullptr */
#endif /* __cplusplus <= 199711L */

int main() {
  std::cout << "Checking function reachability" << std::endl;
  std::cout << "This test is considered as a success when the executable "
               "succesfully link with the librrosace..."
            << std::endl;
  bool ret = true;

  // Elevator
  ret &= ((&rrosace_elevator_new) != nullptr);
  ret &= ((&rrosace_elevator_del) != nullptr);
  ret &= ((&rrosace_elevator_del) != nullptr);

  // Engine
  ret &= ((&rrosace_engine_new) != nullptr);
  ret &= ((&rrosace_engine_del) != nullptr);
  ret &= ((&rrosace_engine_step) != nullptr);

  // FCCs
  ret &= ((&rrosace_fcc_new) != nullptr);
  ret &= ((&rrosace_fcc_del) != nullptr);
  ret &= ((&rrosace_fcc_com_step) != nullptr);
  ret &= ((&rrosace_fcc_mon_step) != nullptr);

  // Filters
  ret &= ((&rrosace_filter_new) != nullptr);
  ret &= ((&rrosace_filter_del) != nullptr);
  ret &= ((&rrosace_filter_step) != nullptr);

  // Flight dynamics
  ret &= ((&rrosace_flight_dynamics_new) != nullptr);
  ret &= ((&rrosace_flight_dynamics_del) != nullptr);
  ret &= ((&rrosace_flight_dynamics_step) != nullptr);

  // Flight mode
  ret &= ((&rrosace_flight_mode_new) != nullptr);
  ret &= ((&rrosace_flight_mode_del) != nullptr);
  ret &= ((&rrosace_flight_mode_set_mode) != nullptr);
  ret &= ((&rrosace_flight_mode_get_mode) != nullptr);

  std::cout << "...OK" << std::endl;

  return (ret ? EXIT_SUCCESS : EXIT_FAILURE);
}
