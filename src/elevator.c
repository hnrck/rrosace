/**
 * @file elevator.c
 * @brief RROSACE Scheduling of cyber-physical system library elevator body.
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

#include <rrosace_constants.h>
#include <rrosace_elevator.h>

#define K (2.0)

struct rrosace_elevator {
  double omega;
  double xi;
  double x[2];
};

rrosace_elevator_t *rrosace_elevator_new(double omega, double xi) {
  rrosace_elevator_t *p_elevator =
      (rrosace_elevator_t *)calloc(1, sizeof(rrosace_elevator_t));

  if (!p_elevator) {
    goto out;
  }

  p_elevator->omega = omega;
  p_elevator->xi = xi;
  p_elevator->x[0] = RROSACE_DELTA_E_EQ;
  p_elevator->x[1] = 0.0;

out:
  return (p_elevator);
}

void rrosace_elevator_del(rrosace_elevator_t *p_elevator) {
  if (p_elevator) {
    free(p_elevator);
  }
}

int rrosace_elevator_step(rrosace_elevator_t *p_elevator, double delta_e_c,
                          double *p_delta_e, double dt) {
  int ret = EXIT_FAILURE;
  double x_dot[2];

  if (!p_elevator) {
    goto out;
  }

  if (!p_delta_e) {
    goto out;
  }

  *p_delta_e = p_elevator->x[0];

  x_dot[0] = p_elevator->x[1];
  x_dot[1] = -p_elevator->omega * p_elevator->omega * p_elevator->x[0] -
             K * p_elevator->xi * p_elevator->omega * p_elevator->x[1] +
             p_elevator->omega * p_elevator->omega * delta_e_c;

  p_elevator->x[0] += dt * x_dot[0];
  p_elevator->x[1] += dt * x_dot[1];

  ret = EXIT_SUCCESS;

out:
  return (ret);
}
