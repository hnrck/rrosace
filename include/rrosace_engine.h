/**
 * @file rrosace_engine.h
 * @brief RROSACE Scheduling of cyber-physical system library engine header.
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

#ifndef RROSACE_ENGINE_H
#define RROSACE_ENGINE_H

#include <rrosace_constants.h>

/** Engine parameter tau */
#define RROSACE_TAU (0.75)

/** Engine default freq */
#define RROSACE_ENGINE_DEFAULT_FREQ (RROSACE_DEFAULT_PHYSICAL_FREQ)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @struct Engine model structure */
struct rrosace_engine;

/** @typedef Engine model */
typedef struct rrosace_engine rrosace_engine_t;

/**
 * @brief Create and initialize a new engine model
 * @param[in] tau The engine tau parameter
 * @return A new engine
 */
rrosace_engine_t *rrosace_engine_new(double tau);

/**
 * @brief Destroy an engine
 * @param[in,out] p_engine The engine to destroy
 */
void rrosace_engine_del(rrosace_engine_t *p_engine);

/**
 * @brief  Execute an engine model instance
 * @param[in,out] p_engine The engine model to execute
 * @param[in] delta_th_c The commanded delta throttle
 * @param[out] p_t The simulated thrust
 * @param[in] dt The execution period of the engine model instance
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_engine_step(rrosace_engine_t *p_engine, double delta_th_c,
                        double *p_t, double dt);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* RROSACE_ENGINE_H */
