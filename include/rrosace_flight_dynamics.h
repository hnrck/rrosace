/**
 * @file rrosace_flight_dynamics.h
 * @brief RROSACE Scheduling of cyber-physical system library flight dynamics
 * header.
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

#ifndef RROSACE_FLIGHT_DYNAMICS_H
#define RROSACE_FLIGHT_DYNAMICS_H

#include <rrosace_constants.h>

/** Flight dynamics default freq */
#define RROSACE_FLIGHT_DYNAMICS_DEFAULT_FREQ (RROSACE_DEFAULT_PHYSICAL_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct Flight dynamics model structure */
struct rrosace_flight_dynamics;

/** @typedef Flight dynamics model */
typedef struct rrosace_flight_dynamics rrosace_flight_dynamics_t;

/**
 * @brief Create and initialize a new flight dynamics model
 * @return The new flight dynamics model
 */
rrosace_flight_dynamics_t *rrosace_flight_dynamics_new();

/**
 * @brief Copy a flight dynamics in a new one
 * @param[in] p_other the flight dynamics to copy
 * @return A new flight dynamics
 */
rrosace_flight_dynamics_t *
rrosace_flight_dynamics_copy(const rrosace_flight_dynamics_t *p_other);

/**
 * @brief Destroy a flight dynamics model
 * @param[in,out] p_flight_dynamics The flight dynamics model to destroy
 */
void rrosace_flight_dynamics_del(rrosace_flight_dynamics_t *p_flight_dynamics);

/**
 * @brief Execute an model instance of a given duration
 * @param[in,out] p_flight_dynamics The flight dynamics model to execute
 * @param[in] delta_e The elevator deflection
 * @param[in] t The thrust
 * @param[out] p_h A simulated altitude
 * @param[out] p_vz A simulated vertical speed
 * @param[out] p_va A simulated true airspeed
 * @param[out] p_q A simulated pitch rate
 * @param[out] p_az A simulated vertical acceleration
 * @param[in] dt The model instance execution period
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_flight_dynamics_step(rrosace_flight_dynamics_t *p_flight_dynamics,
                                 double delta_e, double t, double *p_h,
                                 double *p_vz, double *p_va, double *p_q,
                                 double *p_az, double dt);

#ifdef __cplusplus
}
namespace RROSACE {

/** @class Flight dynamics
 *  @brief C++ wrapper for C-based flight dynamics, based on Model interface.
 */
class FlightDynamics
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
private:
  /** Wrapped C-based flight dynamics */
  rrosace_flight_dynamics_t
      *p_flight_dynamics; /**< C-struct flight_dynamics wrapped */

  const double &r_delta_e;
  const double &r_t;

  double &r_h;
  double &r_vz;
  double &r_va;
  double &r_q;
  double &r_az;

  double m_dt;

public:
  /** Default flight dynamics frequency */
  static const int DEFAULT_FREQ = RROSACE_FLIGHT_DYNAMICS_DEFAULT_FREQ;

  /**
   * @brief Flight dynamics constructor
   * @param[in] dt The execution period of the flight dynamics model instance,
   * default 1 / DEFAULT_FREQ
   */
  FlightDynamics(const double &delta_e, const double &t, double &h, double &vz,
                 double &va, double &q, double &az,
                 double dt = 1. / DEFAULT_FREQ)
      : p_flight_dynamics(rrosace_flight_dynamics_new()), r_delta_e(delta_e),
        r_t(t), r_h(h), r_vz(vz), r_va(va), r_q(q), r_az(az), m_dt(dt) {}

  /**
   * @brief Flight dynamics copy constructor
   * @param[in] other another flight dynamics to construct
   */
  FlightDynamics(const FlightDynamics &other)
      : p_flight_dynamics(
            rrosace_flight_dynamics_copy(other.p_flight_dynamics)),
        r_delta_e(other.r_delta_e), r_t(other.r_t), r_h(other.r_h),
        r_vz(other.r_vz), r_va(other.r_va), r_q(other.r_q), r_az(other.r_az),
        m_dt(other.m_dt) {}

  /**
   * @brief Flight dynamics copy assignement
   * @param[in] other another flight dynamics to construct
   */
  FlightDynamics &operator=(const FlightDynamics &other) {
    p_flight_dynamics = rrosace_flight_dynamics_copy(other.p_flight_dynamics);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight dynamics move constructor
   * @param[in] ' ' an flight dynamics to move
   */
  FlightDynamics(FlightDynamics &&) = default;

  /**
   * @brief Flight dynamics move assignement
   * @param[in] ' ' an flight dynamics to move
   */
  FlightDynamics &operator=(FlightDynamics &&) = delete;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight dynamics destructor
   */
  ~FlightDynamics() { rrosace_flight_dynamics_del(p_flight_dynamics); }

/**
 * @brief  Execute a flight dynamics model instance
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  int step() {
    return rrosace_flight_dynamics_step(p_flight_dynamics, r_delta_e, r_t, &r_h,
                                        &r_vz, &r_va, &r_q, &r_az, m_dt);
  }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return m_dt;
  }
};
} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_FLIGHT_DYNAMIC_H */
