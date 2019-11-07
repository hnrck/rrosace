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

#include <rrosace_constants.h>

/** Elevator parameter omega */
#define RROSACE_OMEGA (25.)
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
#endif /* __cplusplus */

#endif /* RROSACE_ELEVATOR_H */
