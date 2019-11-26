/**
 * @file rrosace_fcu.h
 * @brief RROSACE Scheduling of cyber-physical system library FCU header.
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
 *
 */

#ifndef RROSACE_FCU_H
#define RROSACE_FCU_H

#include <rrosace_constants.h>

/** FCU default freq */
#define RROSACE_FCU_DEFAULT_FREQ (RROSACE_DEFAULT_CYBER_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct FCU model structure */
struct rrosace_fcu;

/** @typedef FCU model */
typedef struct rrosace_fcu rrosace_fcu_t;

/**
 * @brief Create and initialize a new FCU
 * @return An new FCU
 */
rrosace_fcu_t *rrosace_fcu_new();

/**
 * @brief Copy a flight control unit in a new one
 * @param[in] p_other the flight control unit to copy
 * @return A new flight control unit
 */
rrosace_fcu_t *rrosace_fcu_copy(const rrosace_fcu_t *p_other);

/**
 * @brief Destroy an FCU
 * @param[in,out] p_fcu The FCU to destroy
 */
void rrosace_fcu_del(rrosace_fcu_t *p_fcu);

/**
 * @brief Set altitude command to FCU
 * @param[in,out] p_fcu The FCU to set
 * @param[in] h_c The altitude command
 */
void rrosace_fcu_set_h_c(rrosace_fcu_t *p_fcu, double h_c);

/**
 * @brief Set vertical speed command to FCU
 * @param[in,out] p_fcu The FCU to set
 * @param[in] vz_c The vertical speed command
 */
void rrosace_fcu_set_vz_c(rrosace_fcu_t *p_fcu, double vz_c);

/**
 * @brief Set airspeed command to FCU
 * @param[in,out] p_fcu The FCU to set
 * @param[in] va_c The airspeed command
 */
void rrosace_fcu_set_va_c(rrosace_fcu_t *p_fcu, double va_c);

/**
 * @brief Get altitude command from an FCU
 * @param[in] p_fcu The FCU with the command
 * @return The altitude command
 */
double rrosace_fcu_get_h_c(const rrosace_fcu_t *p_fcu);

/**
 * @brief Get vertical speed command from an FCU
 * @param[in] p_fcu The FCU with the command
 * @return The vertical speed command
 */
double rrosace_fcu_get_vz_c(const rrosace_fcu_t *p_fcu);

/**
 * @brief Get airspeed command from an FCU
 * @param[in] p_fcu The FCU with the command
 * @return The airspeed command
 */
double rrosace_fcu_get_va_c(const rrosace_fcu_t *p_fcu);

#ifdef __cplusplus
}
namespace RROSACE {

/** @class Flight control unit
 *  @brief C++ wrapper for C-based flight control unit, based on Model
 * interface.
 */
class FlightControlUnit
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
private:
  /** Wrapped C-based flight control unit */
  rrosace_fcu_t *p_fcu; /**< C-struct fcu wrapped */

  const double &r_h_c_in;
  const double &r_vz_c_in;
  const double &r_va_c_in;

  double &r_h_c_out;
  double &r_vz_c_out;
  double &r_va_c_out;

  double m_dt;

public:
  /** Default flight control unit frequency */
  static const int DEFAULT_FREQ = RROSACE_FCU_DEFAULT_FREQ;

  /**
   * @brief Flight control unit constructor
   * @param[in] dt The execution period of the flight control unit model
   * instance, default 1 / DEFAULT_FREQ
   */
  FlightControlUnit(const double &h_c_in, const double &vz_c_in,
                    const double &va_c_in, double &h_c_out, double &vz_c_out,
                    double &va_c_out, double dt = 1. / DEFAULT_FREQ)
      : p_fcu(rrosace_fcu_new()), r_h_c_in(h_c_in), r_vz_c_in(vz_c_in),
        r_va_c_in(va_c_in), r_h_c_out(h_c_out), r_vz_c_out(vz_c_out),
        r_va_c_out(va_c_out), m_dt(dt) {}

  /**
   * @brief Flight control unit copy constructor
   * @param[in] other another flight control unit to construct
   */
  FlightControlUnit(const FlightControlUnit &other)
      : p_fcu(rrosace_fcu_copy(other.p_fcu)), r_h_c_in(other.r_h_c_in),
        r_vz_c_in(other.r_vz_c_in), r_va_c_in(other.r_va_c_in),
        r_h_c_out(other.r_h_c_out), r_vz_c_out(other.r_vz_c_out),
        r_va_c_out(other.r_va_c_out), m_dt(other.m_dt) {}

  /**
   * @brief Flight control unit copy assignement
   * @param[in] other another flight control unit to construct
   */
  FlightControlUnit &operator=(const FlightControlUnit &other) {
    p_fcu = rrosace_fcu_copy(other.p_fcu);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight control unit move constructor
   * @param[in] ' ' an flight control unit to move
   */
  FlightControlUnit(FlightControlUnit &&) = default;

  /**
   * @brief Flight control unit move assignement
   * @param[in] ' ' an flight control unit to move
   */
  FlightControlUnit &operator=(FlightControlUnit &&) = default;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight control unit destructor
   */
  ~FlightControlUnit() { rrosace_fcu_del(p_fcu); }

/**
 * @brief  Execute a flight control unit model instance
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  int step() {
    int ret_code = EXIT_FAILURE;

    rrosace_fcu_set_h_c(p_fcu, r_h_c_in);
    rrosace_fcu_set_vz_c(p_fcu, r_vz_c_in);
    rrosace_fcu_set_va_c(p_fcu, r_va_c_in);

    r_h_c_out = rrosace_fcu_get_h_c(p_fcu);
    r_vz_c_out = rrosace_fcu_get_vz_c(p_fcu);
    r_va_c_out = rrosace_fcu_get_va_c(p_fcu);

    ret_code = (r_h_c_in == r_h_c_out) && (r_vz_c_in == r_vz_c_out) &&
                       (r_va_c_in == r_va_c_out)
                   ? EXIT_SUCCESS
                   : EXIT_FAILURE;

    return ret_code;
  }
};
} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_FCU_H */
