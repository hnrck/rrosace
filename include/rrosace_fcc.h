/**
 * @file rrosace_fcc.h
 * @brief RROSACE Scheduling of cyber-physical system library FCC header.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 *
 * Based on the Open Source ROSACE (Research Open-Source Avionics and Control
 * Engineering) case study.
 * Implementations of ROSACE available at:
 * https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/
 * Publication of ROSACE available at:
 * https://oatao.univ-toulouse.fr/11522/1/Siron_11522.pdf
 * Publication of RROSACE available at:
 * https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/redundant/report_redundant_rosace_matlab.pdf
 */

#ifndef RROSACE_FCC_H
#define RROSACE_FCC_H

#include <rrosace_constants.h>
#include <rrosace_flight_mode.h>

#define RROSACE_FCC_DEFAULT_FREQ (RROSACE_DEFAULT_CYBER_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct FCC model structure */
struct rrosace_fcc;

/** @typedef FCC model */
typedef struct rrosace_fcc rrosace_fcc_t;

/**
 * @brief Create and initialize an FCC
 * @return A new FCC
 */
rrosace_fcc_t *rrosace_fcc_new();

/**
 * @brief Copy a flight control computer in a new one
 * @param[in] p_other the flight control computer to copy
 * @return A new flight control computer
 */
rrosace_fcc_t *rrosace_fcc_copy(const rrosace_fcc_t *p_other);
/**
 * @brief Destroy an FCC
 * @param[in,out] p_fcc An FCC to destroy
 */
void rrosace_fcc_del(rrosace_fcc_t *p_fcc);

/**
 * @brief Execute an instance of an FCC model in command mode
 * @param[in,out] p_fcc The FCC model to execute
 * @param[in] mode The FCC flight mode
 * @param[in] h_f The FCC altitude
 * @param[in] vz_f The vertical speed
 * @param[in] va_f The airspeed
 * @param[in] q_f The pitch rate
 * @param[in] az_f The vertical acceleration
 * @param[in] h_c The altitude command
 * @param[in] vz_c The vertical speed command
 * @param[in] va_c The airspeed command
 * @param[out] p_delta_e_c A computed delta elevator deflection
 * @param[out] p_delta_th_c A computed delta throttle
 * @param[in] dt The execution period of the model
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_fcc_com_step(rrosace_fcc_t *p_fcc, rrosace_mode_t mode, double h_f,
                         double vz_f, double va_f, double q_f, double az_f,
                         double h_c, double vz_c, double va_c,
                         double *p_delta_e_c, double *p_delta_th_c, double dt);
/**
 * @brief Execute an instance of an FCC model in command mode
 * @param[in,out] p_fcc The FCC model to execute
 * @param[in] mode The FCC flight mode
 * @param[in] h_f The FCC altitude
 * @param[in] vz_f The vertical speed
 * @param[in] va_f The airspeed
 * @param[in] q_f The pitch rate
 * @param[in] az_f The vertical acceleration
 * @param[in] h_c The altitude command
 * @param[in] vz_c The vertical speed command
 * @param[in] va_c The airspeed command
 * @param[in] delta_e_c_monitored The delta elevator deflection to monitor
 * @param[in] delta_th_c_monitored The delta throttle to monitor
 * @param[in] other in law True (!0) if other is master in law, else False (0)
 * @param[out] p_relay_delta_e_c Command for delta elevator deflection relay
 * @param[out]  p_relay_delta_th_c Command for delta throttle relay
 * @param[out]  p_master_in_law True (!0) if master in law, else False (0)
 * @param[in] dt The execution period of the model
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 * @return
 */
int rrosace_fcc_mon_step(rrosace_fcc_t *p_fcc, rrosace_mode_t mode, double h_f,
                         double vz_f, double va_f, double q_f, double az_f,
                         double h_c, double vz_c, double va_c,
                         double delta_e_c_monitored,
                         double delta_th_c_monitored,
                         rrosace_master_in_law_t other_master_in_law,
                         rrosace_relay_state_t *p_relay_delta_e_c,
                         rrosace_relay_state_t *p_relay_delta_th_c,
                         rrosace_master_in_law_t *p_master_in_law, double dt);

#ifdef __cplusplus
}
namespace RROSACE {

class IFlightControlComputer : public Model {};

/** @class Flight control computer
 *  @brief C++ wrapper for C-based flight control computer, based on Model
 * interface.
 */
class FlightControlComputerCommand : public IFlightControlComputer {
private:
  /** Wrapped C-based flight control computer */
  rrosace_fcc_t *p_fcc; /**< C-struct fcc wrapped */

  const FlightMode::Mode &r_mode;

  const double &r_h;
  const double &r_vz;
  const double &r_va;
  const double &r_q;
  const double &r_az;

  const double &r_h_c;
  const double &r_vz_c;
  const double &r_va_c;

  double &r_delta_e_c;
  double &r_delta_th_c;

  const double &r_dt;

public:
  /**
   * @brief Flight control computer constructor
   * @param[in] dt The execution period of the flight control computer model
   * instance, default 1 / DEFAULT_FREQ
   */
  FlightControlComputerCommand(const FlightMode::Mode &mode, const double &h,
                               const double &vz, const double &va,
                               const double &q, const double &az,
                               const double &h_c, const double &vz_c,
                               const double &va_c, double &delta_e_c,
                               double &delta_th_c, const double &dt)
      : p_fcc(rrosace_fcc_new()), r_mode(mode), r_h(h), r_vz(vz), r_va(va),
        r_q(q), r_az(az), r_h_c(h_c), r_vz_c(vz_c), r_va_c(va_c),
        r_delta_e_c(delta_e_c), r_delta_th_c(delta_th_c), r_dt(dt) {}

  /**
   * @brief Flight control computer copy constructor
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputerCommand(const FlightControlComputerCommand &other)
      : p_fcc(rrosace_fcc_copy(other.p_fcc)), r_mode(other.r_mode),
        r_h(other.r_h), r_vz(other.r_vz), r_va(other.r_va), r_q(other.r_q),
        r_az(other.r_az), r_h_c(other.r_h_c), r_vz_c(other.r_vz_c),
        r_va_c(other.r_va_c), r_delta_e_c(other.r_delta_e_c),
        r_delta_th_c(other.r_delta_th_c), r_dt(other.r_dt) {}

  /**
   * @brief Flight control computer copy assignement
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputerCommand &
  operator=(const FlightControlComputerCommand &other) {
    p_fcc = rrosace_fcc_copy(other.p_fcc);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight control computer move constructor
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputerCommand(FlightControlComputerCommand &&) = default;

  /**
   * @brief Flight control computer move assignement
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputerCommand &
  operator=(FlightControlComputerCommand &&) = default;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight control computer destructor
   */
  ~FlightControlComputerCommand() { rrosace_fcc_del(p_fcc); }

/**
 * @brief  execute a flight control computer model instance
 * @return exit_success if ok, else exit_failure
 */
#if __cplusplus >= 201703l
  [[nodiscard]]
#endif
  int step() {
    return rrosace_fcc_com_step(p_fcc, r_mode, r_h, r_vz, r_va, r_q, r_az,
                                r_h_c, r_vz_c, r_va_c, &r_delta_e_c,
                                &r_delta_th_c, r_dt);
  }
};

/** @class Flight control computer
 *  @brief C++ wrapper for C-based flight control computer, based on Model
 * interface.
 */
class FlightControlComputerMonitor : public IFlightControlComputer {
private:
  /** Wrapped C-based flight control computer */
  rrosace_fcc_t *p_fcc; /**< C-struct fcc wrapped */

  const FlightMode::Mode &r_mode;

  const double &r_h;
  const double &r_vz;
  const double &r_va;
  const double &r_q;
  const double &r_az;

  const double &r_h_c;
  const double &r_vz_c;
  const double &r_va_c;

  const double &r_delta_e_c;
  const double &r_delta_th_c;

  const MasterInLaw &r_other_master_in_law;

  RelayState &r_relay_delta_e_c;
  RelayState &r_relay_delta_th_c;
  MasterInLaw &r_master_in_law;

  const double r_dt;

public:
  /**
   * @brief Flight control computer constructor
   * @param[in] dt The execution period of the flight control computer model
   * instance, default 1 / DEFAULT_FREQ
   */
  FlightControlComputerMonitor(
      const FlightMode::Mode &mode, const double &h, const double &vz,
      const double &va, const double &q, const double &az, const double &h_c,
      const double &vz_c, const double &va_c, const double &delta_e_c,
      const double &delta_th_c, const MasterInLaw &other_master_in_law,
      RelayState &relay_delta_e_c, RelayState &relay_delta_th_c,
      MasterInLaw &master_in_law, const double &dt)
      : p_fcc(rrosace_fcc_new()), r_mode(mode), r_h(h), r_vz(vz), r_va(va),
        r_q(q), r_az(az), r_h_c(h_c), r_vz_c(vz_c), r_va_c(va_c),
        r_delta_e_c(delta_e_c), r_delta_th_c(delta_th_c),
        r_other_master_in_law(other_master_in_law),
        r_relay_delta_e_c(relay_delta_e_c),
        r_relay_delta_th_c(relay_delta_th_c), r_master_in_law(master_in_law),
        r_dt(dt) {}

  /**
   * @brief Flight control computer copy constructor
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputerMonitor(const FlightControlComputerMonitor &other)
      : p_fcc(rrosace_fcc_copy(other.p_fcc)), r_mode(other.r_mode),
        r_h(other.r_h), r_vz(other.r_vz), r_va(other.r_va), r_q(other.r_q),
        r_az(other.r_az), r_h_c(other.r_h_c), r_vz_c(other.r_vz_c),
        r_va_c(other.r_va_c), r_delta_e_c(other.r_delta_e_c),
        r_delta_th_c(other.r_delta_th_c),
        r_other_master_in_law(other.r_other_master_in_law),
        r_relay_delta_e_c(other.r_relay_delta_e_c),
        r_relay_delta_th_c(other.r_relay_delta_th_c),
        r_master_in_law(other.r_master_in_law), r_dt(other.r_dt) {}

  /**
   * @brief Flight control computer copy assignement
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputerMonitor &
  operator=(const FlightControlComputerMonitor &other) {
    p_fcc = rrosace_fcc_copy(other.p_fcc);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight control computer move constructor
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputerMonitor(FlightControlComputerMonitor &&) = default;

  /**
   * @brief Flight control computer move assignement
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputerMonitor &
  operator=(FlightControlComputerMonitor &&) = default;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight control computer destructor
   */
  ~FlightControlComputerMonitor() { rrosace_fcc_del(p_fcc); }

/**
 * @brief  execute a flight control computer model instance
 * @return exit_success if ok, else exit_failure
 */
#if __cplusplus >= 201703l
  [[nodiscard]]
#endif
  int step() {
    return rrosace_fcc_mon_step(
        p_fcc, r_mode, r_h, r_vz, r_va, r_q, r_az, r_h_c, r_vz_c, r_va_c,
        r_delta_e_c, r_delta_th_c, r_other_master_in_law, &r_relay_delta_e_c,
        &r_relay_delta_th_c, &r_master_in_law, r_dt);
  }
};

/** @class Flight control computer
 *  @brief C++ wrapper for C-based flight control computer, based on Model
 * interface.
 */
class FlightControlComputer
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFlightControlComputer {
private:
  IFlightControlComputer *p_flight_control_computer;

  double m_dt;

public:
  /** Default flight control computer frequency */
  static const int DEFAULT_FREQ = RROSACE_FCC_DEFAULT_FREQ;

  /**
   * @brief Flight control computer constructor
   * @param[in] dt The execution period of the flight control computer model
   * instance, default 1 / DEFAULT_FREQ
   */
  FlightControlComputer(const FlightMode::Mode &mode, const double &h,
                        const double &vz, const double &va, const double &q,
                        const double &az, const double &h_c, const double &vz_c,
                        const double &va_c, double &delta_e_c,
                        double &delta_th_c, double dt = 1. / DEFAULT_FREQ)
      : p_flight_control_computer(new FlightControlComputerCommand(
            mode, h, vz, va, q, az, h_c, vz_c, va_c, delta_e_c, delta_th_c,
            m_dt)),
        m_dt(dt) {}

  /**
   * @brief Flight control computer constructor
   * @param[in] dt The execution period of the flight control computer model
   * instance, default 1 / DEFAULT_FREQ
   */
  FlightControlComputer(
      const FlightMode::Mode &mode, const double &h, const double &vz,
      const double &va, const double &q, const double &az, const double &h_c,
      const double &vz_c, const double &va_c, const double &delta_e_c,
      const double &delta_th_c, const MasterInLaw &other_master_in_law,
      RelayState &relay_delta_e_c, RelayState &relay_delta_th_c,
      MasterInLaw &master_in_law, double dt = 1. / DEFAULT_FREQ)
      : p_flight_control_computer(new FlightControlComputerMonitor(
            mode, h, vz, va, q, az, h_c, vz_c, va_c, delta_e_c, delta_th_c,
            other_master_in_law, relay_delta_e_c, relay_delta_th_c,
            master_in_law, m_dt)),
        m_dt(dt) {}

  /**
   * @brief Flight control computer copy constructor
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputer(const FlightControlComputer &other)
      : p_flight_control_computer(other.p_flight_control_computer),
        m_dt(other.m_dt) {}

  /**
   * @brief Flight control computer copy assignement
   * @param[in] other another flight control computer to construct
   */
  FlightControlComputer &operator=(const FlightControlComputer &other) {
    p_flight_control_computer =
        FlightControlComputer(other).p_flight_control_computer;
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight control computer move constructor
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputer(FlightControlComputer &&) = default;

  /**
   * @brief Flight control computer move assignement
   * @param[in] ' ' an flight control computer to move
   */
  FlightControlComputer &operator=(FlightControlComputer &&) = default;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight control computer destructor
   */
  ~FlightControlComputer() { delete (p_flight_control_computer); }

/**
 * @brief  execute a flight control computer model instance
 * @return exit_success if ok, else exit_failure
 */
#if __cplusplus >= 201703l
  [[nodiscard]]
#endif
  int step() {
    return p_flight_control_computer->step();
  }
};
} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_FCC_H */
