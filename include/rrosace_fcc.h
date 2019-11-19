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
#endif /* __cplusplus */

#endif /* RROSACE_FCC_H */
