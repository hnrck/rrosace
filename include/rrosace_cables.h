/**
 * @file rrosace_cables.h
 * @brief RROSACE Scheduling of cyber-physical system library cables header.
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

#ifndef RROSACE_CABLES_H
#define RROSACE_CABLES_H

#include <rrosace_constants.h>

/** Cables default freq */
#define RROSACE_CABLES_DEFAULT_FREQ (RROSACE_DEFAULT_CYBER_FREQ)

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct rrosace_cables_input RROSACE cables input data from FCCs */
struct rrosace_cables_input {
  double delta_e_c;     /**< Elevator deflection command from FCCs */
  double delta_th_c;    /**< Throttle command  from FCCs */
  int relay_delta_e_c;  /**< Relay value for elevator deflection command */
  int relay_delta_th_c; /**< Relay value for delta throttle command */
};

/** @typedef Alias for cables input */
typedef struct rrosace_cables_input rrosace_cables_input_t;

/** @struct rrosace_cables_input RROSACE cables output data for engine and
 * elevator */
struct rrosace_cables_output {
  double delta_e_c;  /**< Elevator deflection command for elevator */
  double delta_th_c; /**< Delta throttle command for engine */
};

/** @typedef Alias for cables output */
typedef struct rrosace_cables_output rrosace_cables_output_t;

/**
 * @brief cables step, consume inputs and produce output
 * @param[in] inputs the data from FCCs to consume
 * @param[in] nb_input the number of FCCs data to consume
 * @param[out] p_output a pointer to the data to produce to elevator and
 * engine
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_cables_step(const rrosace_cables_input_t inputs[], size_t nb_input,
                        rrosace_cables_output_t *p_output);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RROSACE_CABLES_H */
