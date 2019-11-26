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
    const double delta_e_c = RROSACE::DELTA_E_C_EQ;
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
    const double delta_th_c = RROSACE_DELTA_TH_C_EQ;
    double t;

    std::cout << "Engine test" << std::endl;

    RROSACE::Engine engine = RROSACE::Engine(RROSACE::TAU, delta_th_c, t);

    (void)engine.step();

    std::cout << "t: " << RROSACE::T_EQ << " -> " << t << std::endl;
  }

  // Flight dynmaics
  {
    const double delta_e = RROSACE::DELTA_E_EQ;
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

  // Filters
  {

    const double h = RROSACE::H_EQ;
    const double vz = RROSACE::VZ_EQ;
    const double va = RROSACE::VA_EQ;
    const double q = RROSACE::Q_EQ;
    const double az = RROSACE::AZ_EQ;

    double h_f;
    double vz_f;
    double va_f;
    double q_f;
    double az_f;

    RROSACE::AltitudeFilter altitudeFilter = RROSACE::AltitudeFilter(h, h_f);
    RROSACE::VerticalAirspeedFilter verticalAirspeedFilter =
        RROSACE::VerticalAirspeedFilter(vz, vz_f);
    RROSACE::TrueAirspeedFilter trueAirspeedFilter =
        RROSACE::TrueAirspeedFilter(va, va_f);
    RROSACE::PitchRateFilter pitchRateFilter = RROSACE::PitchRateFilter(q, q_f);
    RROSACE::VerticalAccelerationFilter verticalAccelerationFilter =
        RROSACE::VerticalAccelerationFilter(az, az_f);

    (void)altitudeFilter.step();
    (void)verticalAirspeedFilter.step();
    (void)trueAirspeedFilter.step();
    (void)pitchRateFilter.step();
    (void)verticalAccelerationFilter.step();

    std::cout << "h_f: " << RROSACE::H_F_EQ << " -> " << h_f << std::endl;
    std::cout << "vz_f: " << RROSACE::VZ_F_EQ << " -> " << vz_f << std::endl;
    std::cout << "va_f: " << RROSACE::VA_F_EQ << " -> " << va_f << std::endl;
    std::cout << "q_f: " << RROSACE::Q_F_EQ << " -> " << q_f << std::endl;
    std::cout << "az_f: " << RROSACE::AZ_F_EQ << " -> " << az_f << std::endl;
  }

  // Flight mode
  {
    const RROSACE::FlightMode::Mode mode_in = RROSACE::FlightMode::Mode::RROSACE_COMMANDED;
    RROSACE::FlightMode::Mode mode_out;

    std::cout << "Flight mode test" << std::endl;

    RROSACE::FlightMode flight_mode = RROSACE::FlightMode(mode_in, mode_out);

    (void)flight_mode.step();

    std::cout << "mode: " << RROSACE::FlightMode::Mode::RROSACE_COMMANDED << " -> " << mode_out << std::endl;
  }

  // FCU
  {
    const double h_c_in = RROSACE::H_EQ;
    const double vz_c_in = RROSACE::VZ_EQ;
    const double va_c_in = RROSACE::VA_EQ;
    double h_c_out;
    double vz_c_out;
    double va_c_out;

    std::cout << "FCU test" << std::endl;

    RROSACE::FlightControlUnit fcu = RROSACE::FlightControlUnit(h_c_in, vz_c_in, va_c_in, h_c_out, vz_c_out, va_c_out);

    (void)fcu.step();

    std::cout << "h_c: " << RROSACE::H_EQ << " -> " << h_c_out << std::endl;
    std::cout << "vz_c: " << RROSACE::VZ_EQ << " -> " << vz_c_out << std::endl;
    std::cout << "va_c: " << RROSACE::VA_EQ << " -> " << va_c_out << std::endl;
  }

  // FCCs
  ret &= ((&rrosace_fcc_new) != nullptr);
  ret &= ((&rrosace_fcc_del) != nullptr);
  ret &= ((&rrosace_fcc_com_step) != nullptr);
  ret &= ((&rrosace_fcc_mon_step) != nullptr);

  std::cout << "...OK" << std::endl;

  return (ret ? EXIT_SUCCESS : EXIT_FAILURE);
}
