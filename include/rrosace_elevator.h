/**
 * @file rrosace_elevator.h
 * @brief RROSACE Scheduling of cyber-physical system library elevator
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

#ifndef RROSACE_ELEVATOR_H
#define RROSACE_ELEVATOR_H

#include <rrosace_common.h>
#include <rrosace_constants.h>

/** Elevator parameter omega */
#define RROSACE_OMEGA (25)
/** Elevator parameter xi */
#define RROSACE_XI (0.85)

/** Elevator default freq */
#define RROSACE_ELEVATOR_DEFAULT_FREQ (RROSACE_DEFAULT_PHYSICAL_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct rrosace_elevator model for rrosace elevator */
struct rrosace_elevator;

/** @typedef alias for struct rrosace_elevator */
typedef struct rrosace_elevator rrosace_elevator_t;

/**
 * @brief Create and initialize a new elevator
 * @param[in] omega The elevator omega parameter
 * @param[in] xi The elevator xi parameter
 * @return A new elevator
 */
rrosace_elevator_t *rrosace_elevator_new(double omega, double xi);

/**
 * @brief Copy an elevator in a new one
 * @param[in] p_other the elevator to copy
 * @return A new elevator
 */
rrosace_elevator_t *rrosace_elevator_copy(const rrosace_elevator_t *p_other);

/**
 * @brief Destroy an elevator
 * @param[in,out] p_elevator The elevator to destroy
 */
void rrosace_elevator_del(rrosace_elevator_t *p_elevator);

/**
 * @brief Execute an elevator model instance
 * @param[in,out] p_elevator The model to execute
 * @param[in] delta_e_c The elevator deflection commanded
 * @param[out] p_delta_e The simulated elevator deflection
 * @param[in] dt The model instance execution period
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_elevator_step(rrosace_elevator_t *p_elevator, double delta_e_c,
                          double *p_delta_e, double dt);

#ifdef __cplusplus
}
namespace RROSACE {

/** Elevator default parameter omega */
static const double OMEGA = RROSACE_OMEGA;

/** Elevator default parameter xi */
static const double XI = RROSACE_XI;

/** @class Elevator
 *  @brief C++ wrapper for C-based elevator, based on Model interface.
 */
class Elevator
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
private:
  /** Wrapped C-based elevator */
  rrosace_elevator_t *p_elevator;

  const double &r_delta_e_c;

  double &r_delta_e;

  double m_dt;

public:
  /** Default elevator frequency */
  static const int DEFAULT_FREQ = RROSACE_ELEVATOR_DEFAULT_FREQ;

  /**
   * @brief Elevator constructor
   * @param[in] omega The elevator omega parameter
   * @param[in] xi The elevator xi parameter
   * @param[in] delta_e_c The elevator deflection commanded
   * @param[out] delta_e The simulated elevator deflection
   * @param[in] dt The model instance execution period, 1 / DEFAULT_FREQ by
   * default
   */
  Elevator(double omega, double xi, const double &delta_e_c, double &delta_e,
           double dt = 1 / DEFAULT_FREQ)
      : p_elevator(rrosace_elevator_new(omega, xi)), r_delta_e_c(delta_e_c),
        r_delta_e(delta_e), m_dt(dt) {}

  /**
   * @brief Elevator copy constructor
   * @param[in] other another elevator to construct
   */
  Elevator(const Elevator &other)
      : p_elevator(rrosace_elevator_copy(other.p_elevator)),
        r_delta_e_c(other.r_delta_e_c), r_delta_e(other.r_delta_e),
        m_dt(other.m_dt) {}

  /**
   * @brief Elevator copy assignement
   * @param[in] other another elevator to construct
   */
  Elevator &operator=(const Elevator &other) {
    p_elevator = rrosace_elevator_copy(other.p_elevator);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Elevator move constructor
   * @param[in] ' ' an elevator to move
   */
  Elevator(Elevator &&) = default;

  /**
   * @brief Elevator move assignement
   * @param[in] ' ' an elevator to move
   */
  Elevator &operator=(Elevator &&) = delete;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Elevator destructor
   */
  ~Elevator() { rrosace_elevator_del(p_elevator); }

  /**
   * @brief Execute an elevator model instance
   * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
   */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  int step() {
    return rrosace_elevator_step(p_elevator, r_delta_e_c, &r_delta_e, m_dt);
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

#endif /* RROSACE_ELEVATOR_H */
