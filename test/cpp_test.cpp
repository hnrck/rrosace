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

#if __cplusplus <= 199711L
int main() {
#else
auto main() -> int {
#endif /* __cplusplus <= 199711L */
  std::cout << "Checking function reachability" << std::endl;
  std::cout << "This test is considered as a success when the executable "
               "succesfully link with the librrosace..."
            << std::endl;
  bool ret = true;

  // Elevator
  {
    double delta_e_c = RROSACE::DELTA_E_C_EQ;
    double delta_e;

    std::cout << "Elevator test" << std::endl;

    RROSACE::Elevator elevator =
        RROSACE::Elevator(RROSACE::OMEGA, RROSACE::XI, delta_e_c, delta_e);
    (void)elevator.step();

    std::cout << "delta_e: " << RROSACE::DELTA_E_EQ << " -> " << delta_e
              << std::endl;
  }

  // Engine
  {
    double delta_th_c = RROSACE_DELTA_TH_C_EQ;
    double t;

    std::cout << "Engine test" << std::endl;

    RROSACE::Engine engine = RROSACE::Engine(RROSACE::TAU, delta_th_c, t);

    (void)engine.step();

    std::cout << "t: " << RROSACE::T_EQ << " -> " << t << std::endl;
  }

  // Flight dynmaics
  {
    double delta_e = RROSACE::DELTA_E_EQ;
    double t = RROSACE::T_EQ;

    double h;
    double vz;
    double va;
    double q;
    double az;

    std::cout << "Flight dynamics test" << std::endl;

    RROSACE::FlightDynamics flight_dynamics =
        RROSACE::FlightDynamics(delta_e, t, h, vz, va, q, az, t);

    (void)flight_dynamics.step();

    std::cout << "h: " << RROSACE::H_EQ << " -> " << h << std::endl;
    std::cout << "vz: " << RROSACE::VZ_EQ << " -> " << vz << std::endl;
    std::cout << "va: " << RROSACE::VA_EQ << " -> " << va << std::endl;
    std::cout << "q: " << RROSACE::Q_EQ << " -> " << q << std::endl;
    std::cout << "az: " << RROSACE::AZ_EQ << " -> " << az << std::endl;
  }

  // FCCs
  ret &= ((&rrosace_fcc_new) != nullptr);
  ret &= ((&rrosace_fcc_del) != nullptr);
  ret &= ((&rrosace_fcc_com_step) != nullptr);
  ret &= ((&rrosace_fcc_mon_step) != nullptr);

  // Filters
  ret &= ((&rrosace_filter_new) != nullptr);
  ret &= ((&rrosace_filter_del) != nullptr);
  ret &= ((&rrosace_filter_step) != nullptr);

  // Flight mode
  ret &= ((&rrosace_flight_mode_new) != nullptr);
  ret &= ((&rrosace_flight_mode_del) != nullptr);
  ret &= ((&rrosace_flight_mode_set_mode) != nullptr);
  ret &= ((&rrosace_flight_mode_get_mode) != nullptr);

  std::cout << "...OK" << std::endl;

  return (ret ? EXIT_SUCCESS : EXIT_FAILURE);
}
