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

    std::cout << "Filters test" << std::endl;

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
    const RROSACE::FlightMode::Mode mode_in = RROSACE_COMMANDED;
    RROSACE::FlightMode::Mode mode_out;

    std::cout << "Flight mode test" << std::endl;

    RROSACE::FlightMode flight_mode = RROSACE::FlightMode(mode_in, mode_out);

    (void)flight_mode.step();

    std::cout << "mode: " << RROSACE_COMMANDED << " -> " << mode_out
              << std::endl;
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

    RROSACE::FlightControlUnit fcu = RROSACE::FlightControlUnit(
        h_c_in, vz_c_in, va_c_in, h_c_out, vz_c_out, va_c_out);

    (void)fcu.step();

    std::cout << "h_c: " << RROSACE::H_EQ << " -> " << h_c_out << std::endl;
    std::cout << "vz_c: " << RROSACE::VZ_EQ << " -> " << vz_c_out << std::endl;
    std::cout << "va_c: " << RROSACE::VA_EQ << " -> " << va_c_out << std::endl;
  }

  // FCCs
  {
    const RROSACE::FlightMode::Mode mode = RROSACE_COMMANDED;

    const double h = RROSACE::H_EQ;
    const double vz = RROSACE::VZ_EQ;
    const double va = RROSACE::VA_EQ;
    const double q = RROSACE::Q_EQ;
    const double az = RROSACE::AZ_EQ;

    const double h_c = RROSACE::H_EQ;
    const double vz_c = RROSACE::VZ_EQ;
    const double va_c = RROSACE::VA_EQ;

    double delta_e_c;
    double delta_th_c;

    const RROSACE::MasterInLaw &other_master_in_law = RROSACE_NOT_MASTER_IN_LAW;

    RROSACE::RelayState relay_delta_e_c;
    RROSACE::RelayState relay_delta_th_c;
    RROSACE::MasterInLaw master_in_law;

    std::cout << "FCC test" << std::endl;

    RROSACE::FlightControlComputer flightControlComputerCommand =
        RROSACE::FlightControlComputer(mode, h, vz, va, q, az, h_c, vz_c, va_c,
                                       delta_e_c, delta_th_c);
    RROSACE::FlightControlComputer flightControlComputerMonitor =
        RROSACE::FlightControlComputer(mode, h, vz, va, q, az, h_c, vz_c, va_c,
                                       delta_e_c, delta_th_c,
                                       other_master_in_law, relay_delta_e_c,
                                       relay_delta_th_c, master_in_law);

    (void)flightControlComputerCommand.step();
    (void)flightControlComputerMonitor.step();

    std::cout << "delta_e_c: " << RROSACE::DELTA_E_C_EQ << " -> " << delta_e_c
              << std::endl;
    std::cout << "delta_th_c: " << RROSACE::DELTA_TH_C_EQ << " -> "
              << delta_th_c << std::endl;
    std::cout << "relay_delta_e_c: " << RROSACE_RELAY_CLOSED << " ->"
              << relay_delta_e_c << std::endl;
    std::cout << "relay_delta_th_c: " << RROSACE_RELAY_CLOSED << " -> "
              << relay_delta_th_c << std::endl;
    std::cout << "master_in_law: " << RROSACE_MASTER_IN_LAW << " -> "
              << master_in_law << std::endl;
  }

  // Cables
  {
    const RROSACE::Cables::Input input_1 = {
        RROSACE::DELTA_E_C_EQ, RROSACE::DELTA_TH_C_EQ, RROSACE_RELAY_CLOSED,
        RROSACE_RELAY_CLOSED};

    const RROSACE::Cables::Input input_2 = {
        RROSACE::DELTA_E_C_EQ, RROSACE::DELTA_TH_C_EQ, RROSACE_RELAY_CLOSED,
        RROSACE_RELAY_CLOSED};

    RROSACE::Cables::Output output;

    std::cout << "Cables test" << std::endl;

    RROSACE::Cables cables = RROSACE::Cables(
        input_1.delta_e_c, input_1.delta_th_c, input_1.relay_delta_e_c,
        input_1.relay_delta_th_c, input_2.delta_e_c, input_2.delta_th_c,
        input_2.relay_delta_e_c, input_2.relay_delta_th_c, output.delta_e_c,
        output.delta_th_c);
    (void)cables.step();

    std::cout << "delta_e_c: " << RROSACE::DELTA_E_C_EQ << " -> "
              << output.delta_e_c << std::endl;
    std::cout << "delta_th_c: " << RROSACE::DELTA_TH_C_EQ << " -> "
              << output.delta_th_c << std::endl;
  }

  std::cout << "...OK" << std::endl;

  return EXIT_SUCCESS;
}
