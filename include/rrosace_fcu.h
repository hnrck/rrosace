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
#endif /* __cplusplus */

#endif /* RROSACE_FCU_H */
