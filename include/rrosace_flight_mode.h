/**
 * @file rrosace_flight_mode.h
 * @brief RROSACE Scheduling of cyber-physical system library flight mode
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
 * Publication of RROSACE * available at:
 * https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/redundant/report_redundant_rosace_matlab.pdf
 */

#ifndef RROSACE_FLIGHT_MODE_H
#define RROSACE_FLIGHT_MODE_H

#include <rrosace_constants.h>

/** Flight mode default freq */
#define RROSACE_FLIGHT_MODE_DEFAULT_FREQ (RROSACE_DEFAULT_CYBER_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @enum Auto-pilot modes */
enum rrosace_mode {
  RROSACE_UNDEFINED,     /**< Undefined mode, leads to errors*/
  RROSACE_ALTITUDE_HOLD, /**< Altitude hold auto-pilot */
  RROSACE_COMMANDED      /**< Commanded auto-pilot */
};

/** @typedef Alias for mode */
typedef enum rrosace_mode rrosace_mode_t;

/** @struct Flight mode model structure */
struct rrosace_flight_mode;

/** @typedef Flight mode model */
typedef struct rrosace_flight_mode rrosace_flight_mode_t;

/**
 * @brief Create and initialize a flight mode model
 * @return New flight mode
 */
rrosace_flight_mode_t *rrosace_flight_mode_new();

/**
 * @brief Copy a flight mode in a new one
 * @param[in] p_other the flight mode to copy
 * @return A new flight mode
 */
rrosace_flight_mode_t *
rrosace_flight_mode_copy(const rrosace_flight_mode_t *p_other);

/**
 * @brief Destroy a flight mode model
 * @param[in,out] p_flight_mode The model to destroy
 */
void rrosace_flight_mode_del(rrosace_flight_mode_t *p_flight_mode);

/**
 * @brief Set flight mode
 * @param[in,out] p_flight_mode The flight mode model to set
 * @param[in] mode The flight mode to set
 */
void rrosace_flight_mode_set_mode(rrosace_flight_mode_t *p_flight_mode,
                                  rrosace_mode_t mode);

/**
 * @brief Get flight mode
 * @param[in] p_flight_mode A model with a fligh mode
 * @return The mode flight mode
 */
rrosace_mode_t
rrosace_flight_mode_get_mode(const rrosace_flight_mode_t *p_flight_mode);

#ifdef __cplusplus
}
namespace RROSACE {

/** @class Flight mode
 *  @brief C++ wrapper for C-based flight mode, based on Model interface.
 */
class FlightMode
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
public:
  /** @typedef Alias for mode */
  typedef rrosace_mode_t Mode;

private:
  /** Wrapped C-based flight mode */
  rrosace_flight_mode_t *p_flight_mode; /**< C-struct flight_mode wrapped */

  const Mode &r_mode_in;

  Mode &r_mode_out;

  double m_dt;

public:
  /** Default flight mode frequency */
  static const int DEFAULT_FREQ = RROSACE_FLIGHT_MODE_DEFAULT_FREQ;

  /**
   * @brief Flight mode constructor
   * @param[in] dt The execution period of the flight mode model instance,
   * default 1 / DEFAULT_FREQ
   */
  FlightMode(const Mode &mode_in, Mode &mode_out, double dt = 1. / DEFAULT_FREQ)
      : p_flight_mode(rrosace_flight_mode_new()), r_mode_in(mode_in),
        r_mode_out(mode_out), m_dt(dt) {}

  /**
   * @brief Flight mode copy constructor
   * @param[in] other another flight mode to construct
   */
  FlightMode(const FlightMode &other)
      : p_flight_mode(rrosace_flight_mode_copy(other.p_flight_mode)),
        r_mode_in(other.r_mode_in), r_mode_out(other.r_mode_out),
        m_dt(other.m_dt) {}

  /**
   * @brief Flight mode copy assignement
   * @param[in] other another flight mode to construct
   */
  FlightMode &operator=(const FlightMode &other) {
    p_flight_mode = rrosace_flight_mode_copy(other.p_flight_mode);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Flight mode move constructor
   * @param[in] ' ' an flight mode to move
   */
  FlightMode(FlightMode &&) = default;

  /**
   * @brief Flight mode move assignement
   * @param[in] ' ' an flight mode to move
   */
  FlightMode &operator=(FlightMode &&) = delete;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Flight mode destructor
   */
  ~FlightMode() { rrosace_flight_mode_del(p_flight_mode); }

/**
 * @brief  Execute a flight mode model instance
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  int step() {
    rrosace_flight_mode_set_mode(p_flight_mode, r_mode_in);
    r_mode_out = rrosace_flight_mode_get_mode(p_flight_mode);
    return (r_mode_in == r_mode_out ? EXIT_SUCCESS : EXIT_FAILURE);
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

#endif /* RROSACE_FLIGHT_MODE_H */
