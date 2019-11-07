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
#endif /* __cplusplus */

#endif /* RROSACE_FLIGHT_MODE_H */
