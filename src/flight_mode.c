/**
 * @file flight_mode.c
 * @brief RROSACE Scheduling of cyber-physical system library flight mode
 * body.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-06-10
 *
 * Based on the Open Source ROSACE (Research Open-Source Avionics and Control
 * Engineering) case study.
 * Implementations of ROSACE available at:
 * https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/
 * Publication of ROSACE available at:
 * https://oatao.univ-toulouse.fr/11522/1/Siron_11522.pdf Publication of RROSACE
 * available at:
 * https://svn.onera.fr/schedmcore/branches/ROSACE_CaseStudy/redundant/report_redundant_rosace_matlab.pdf
 *
 */

#include <stdlib.h>

#include <rrosace_flight_mode.h>

struct rrosace_flight_mode {
  double mode;
};

rrosace_flight_mode_t *rrosace_flight_mode_new() {
  rrosace_flight_mode_t *p_flight_mode =
      (rrosace_flight_mode_t *)calloc(1, sizeof(rrosace_flight_mode_t));

  if (!p_flight_mode) {
    goto out;
  }

  p_flight_mode->mode = RROSACE_UNDEFINED;

out:
  return (p_flight_mode);
}

void rrosace_flight_mode_del(rrosace_flight_mode_t *p_flight_mode) {
  if (p_flight_mode) {
    free(p_flight_mode);
  }
}

void rrosace_flight_mode_set_mode(rrosace_flight_mode_t *p_flight_mode,
                                  rrosace_mode_t mode) {
  if (!p_flight_mode) {
    goto out;
  }

  p_flight_mode->mode = mode;

out:
  return;
}

rrosace_mode_t
rrosace_flight_mode_get_mode(const rrosace_flight_mode_t *p_flight_mode) {
  rrosace_mode_t mode = RROSACE_UNDEFINED;

  if (!p_flight_mode) {
    goto out;
  }

  mode = p_flight_mode->mode;

out:
  return (mode);
}
