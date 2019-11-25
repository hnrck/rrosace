/**
 * @file p_flight_dynamics.c
 * @brief RROSACE Scheduling of cyber-physical system library flight dynamics
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

#include <math.h>
#include <stdlib.h>

#include <rrosace_constants.h>
#include <rrosace_flight_dynamics.h>

/* Trimming parameters */
#define THETA_EQ (0.026485847681737)

/* Atmosphere parameters */
#define RHO_0 (1.225)
#define G_0 (9.80665)
#define T0_0 (288.15)
#define T0_H (-0.0065)
#define RS (287.05)

/* Aircraft parameters */
#define MASSE (57837.5)
#define I_Y (3781272.0)
#define S (122.6)
#define C_BAR (4.29)
#define CD_0 (0.016)
#define CD_ALPHA (2.5)
#define CD_DELTA_E (0.05)
#define CL_ALPHA (5.5)
#define CL_DELTA_E (0.193)
#define ALPHA_0 (-0.05)
#define CM_0 (0.04)
#define CM_ALPHA (-0.83)
#define CM_DELTA_E (-1.5)
#define CM_Q (-30)

#define K (0.5)

struct rrosace_flight_dynamics {
  double u;
  double w;
  double q;
  double theta;
  double h;
};

rrosace_flight_dynamics_t *rrosace_flight_dynamics_new() {
  rrosace_flight_dynamics_t *p_flight_dynamics =
      (rrosace_flight_dynamics_t *)calloc(1, sizeof(rrosace_flight_dynamics_t));

  if (!p_flight_dynamics) {
    goto out;
  }

  p_flight_dynamics->u = RROSACE_VA_EQ * cos(THETA_EQ);
  p_flight_dynamics->w = RROSACE_VA_EQ * sin(THETA_EQ);
  p_flight_dynamics->q = RROSACE_Q_EQ;
  p_flight_dynamics->theta = THETA_EQ;
  p_flight_dynamics->h = RROSACE_H_EQ;

out:
  return (p_flight_dynamics);
}

rrosace_flight_dynamics_t *
rrosace_flight_dynamics_copy(const rrosace_flight_dynamics_t *p_other) {
  rrosace_flight_dynamics_t *p_flight_dynamics = rrosace_flight_dynamics_new();

  if (!p_flight_dynamics) {
    goto out;
  }

  p_flight_dynamics->u = p_other->u;
  p_flight_dynamics->w = p_other->w;
  p_flight_dynamics->q = p_other->q;
  p_flight_dynamics->theta = p_other->theta;
  p_flight_dynamics->h = p_other->h;

out:
  return (p_flight_dynamics);
}

void rrosace_flight_dynamics_del(rrosace_flight_dynamics_t *p_flight_dynamics) {
  if (p_flight_dynamics) {
    free(p_flight_dynamics);
  }
}

int rrosace_flight_dynamics_step(rrosace_flight_dynamics_t *p_flight_dynamics,
                                 double delta_e, double t, double *p_h,
                                 double *p_vz, double *p_va, double *p_q,
                                 double *p_az, double dt) {
  int ret = EXIT_FAILURE;

  double u_dot;
  double w_dot;
  double q_dot;
  double theta_dot;
  double h_dot;

  double cd;
  double cl;
  double cm;

  double xa;
  double za;
  double ma;

  double alpha;
  double qbar;
  double v;
  double rho;

  if (!p_flight_dynamics) {
    goto out;
  }

  if (!p_h || !p_vz || !p_va || !p_q || !p_az) {
    goto out;
  }

  rho = RHO_0 *
        pow(1.0 + T0_H / T0_0 * p_flight_dynamics->h, -G_0 / (RS * T0_H) - 1.0);
  alpha = atan(p_flight_dynamics->w / p_flight_dynamics->u);
  v = sqrt(p_flight_dynamics->u * p_flight_dynamics->u +
           p_flight_dynamics->w * p_flight_dynamics->w);
  qbar = K * rho * v * v;
  cl = CL_DELTA_E * delta_e + CL_ALPHA * (alpha - ALPHA_0);
  cd = CD_0 + CD_DELTA_E * delta_e +
       CD_ALPHA * (alpha - ALPHA_0) * (alpha - ALPHA_0);
  cm = CM_0 + CM_DELTA_E * delta_e + CM_ALPHA * alpha +
       K * CM_Q * p_flight_dynamics->q * C_BAR / v;
  xa = -qbar * S * (cd * cos(alpha) - cl * sin(alpha));
  za = -qbar * S * (cd * sin(alpha) + cl * cos(alpha));
  ma = qbar * C_BAR * S * cm;

  *p_va = v;
  *p_vz = p_flight_dynamics->w * cos(p_flight_dynamics->theta) -
          p_flight_dynamics->u * sin(p_flight_dynamics->theta);
  *p_q = p_flight_dynamics->q;
  *p_az = G_0 * cos(p_flight_dynamics->theta) + za / MASSE;
  *p_h = p_flight_dynamics->h;

  u_dot = -G_0 * sin(p_flight_dynamics->theta) -
          p_flight_dynamics->q * p_flight_dynamics->w + (xa + t) / MASSE;
  w_dot = G_0 * cos(p_flight_dynamics->theta) +
          p_flight_dynamics->q * p_flight_dynamics->u + za / MASSE;
  q_dot = ma / I_Y;
  theta_dot = p_flight_dynamics->q;
  h_dot = p_flight_dynamics->u * sin(p_flight_dynamics->theta) -
          p_flight_dynamics->w * cos(p_flight_dynamics->theta);

  p_flight_dynamics->u += dt * u_dot;
  p_flight_dynamics->w += dt * w_dot;
  p_flight_dynamics->q += dt * q_dot;
  p_flight_dynamics->theta += dt * theta_dot;
  p_flight_dynamics->h += dt * h_dot;

  ret = EXIT_SUCCESS;

out:
  return (ret);
}
