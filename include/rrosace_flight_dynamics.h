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
#endif /* __cplusplus */

#endif /* RROSACE_FLIGHT_DYNAMIC_H */
