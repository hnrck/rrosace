/**
 * @file engine.c
 * @brief RROSACE Scheduling of cyber-physical system library engine body.
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

#include <stdlib.h>

#include <rrosace_constants.h>
#include <rrosace_engine.h>

#define K (26350.0)

struct rrosace_engine {
  double tau;
  double x;
};

rrosace_engine_t *rrosace_engine_new(double tau) {
  rrosace_engine_t *p_engine =
      (rrosace_engine_t *)calloc(1, sizeof(rrosace_engine_t));

  if (!p_engine) {
    goto out;
  }

  p_engine->tau = tau;
  p_engine->x = RROSACE_DELTA_TH_C_EQ;

out:
  return (p_engine);
}

void rrosace_engine_del(rrosace_engine_t *p_engine) {
  if (p_engine) {
    free(p_engine);
  }
}

int rrosace_engine_step(rrosace_engine_t *p_engine, double delta_th_c,
                        double *p_t, double dt) {
  int ret = EXIT_FAILURE;
  double x_dot;

  if (!p_engine) {
    goto out;
  }

  if (!p_t) {
    goto out;
  }

  *p_t = K * p_engine->x;

  x_dot = -p_engine->tau * p_engine->x + p_engine->tau * delta_th_c;

  p_engine->x += dt * x_dot;

  ret = EXIT_SUCCESS;

out:
  return (ret);
}
