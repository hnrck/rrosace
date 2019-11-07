/**
 * @file fcc.c
 * @brief RROSACE Scheduling of cyber-physical system library FCC body.
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
#include <rrosace_fcc.h>

/* Espsilons */
#define EPSILON_DELTA_E_C (0.001)
#define EPSILON_DELTA_TH_C (0.001)

/* Controller parameters */
#define H_SWITCH (50.0)

/* Altitude hold */
#define KP_H (0.1014048)
#define KI_H (0.0048288)

/* Setpoint commands */
/* #define Cl_alt_hold (2.57024) */

/* Va Speed controller */
#define K1_INT_VA (0.049802610664357)
#define K1_VA (-0.486813084356079)
#define K1_VZ (-0.077603095495388)
#define K1_Q (21.692383376322041)

/* Vz Speed controller */
#define K2_INT_VZ (0.000627342822264)
#define K2_VZ (-0.003252836726554)
#define K2_Q (0.376071446897134)
#define K2_AZ (-0.001566907423747)

struct controller {
  double integrator;
};

struct altitude_hold_controller {
  struct controller controller;
  int need_reinit;
  double old_vz_c;
};

typedef struct controller controller_t;
typedef struct altitude_hold_controller altitude_hold_controller_t;

struct rrosace_fcc {
  altitude_hold_controller_t altitude_hold;
  controller_t vz_control;
  controller_t va_control;
};

static int init_altitude_hold_integrator(
    altitude_hold_controller_t * /* p_altitude_hold */, double /* diff_h */);

static int
altitude_hold_model(altitude_hold_controller_t * /* p_altitude_hold */,
                    double /* vz_c */, double /* h_f */, double /* h_c */,
                    double /* dt */, double * /* p_vz */);

static int va_control_model(controller_t * /* p_va_control */,
                            double /* va_f */, double /* vz_f */,
                            double /* q_f */, double /* va_c */,
                            double /* dt */, double * /* p_delta_th_c */);

static int vz_control_model(controller_t * /* p_vz_control */,
                            double /* vz_f */, double /* vz_c */,
                            double /* q_f */, double /* az_f */,
                            double /* dt */, double * /* p_delta_e_c */);

static int fcc_step(rrosace_fcc_t * /* p_fcc */, rrosace_mode_t /* mode */,
                    double /* h_f */, double /* vz_f */, double /* va_f */,
                    double /* q_f */, double /* az_f */, double /* h_c */,
                    double /* vz_c */, double /* va_c */,
                    const double * /* p_delta_e_c_monitored */,
                    const double * /* p_delta_th_c_monitored */,
                    const unsigned int * /* p_other_master_in_law */,
                    double * /* p_delta_e_c */, double * /* p_delta_th_c */,
                    unsigned int * /* p_relay_delta_e_c */,
                    unsigned int * /* p_relay_delta_th_c */,
                    unsigned int * /* p_master_in_law */, double /* dt */);

static int
init_altitude_hold_integrator(altitude_hold_controller_t *p_altitude_hold,
                              double diff_h) {
  int ret = EXIT_FAILURE;

  if (!p_altitude_hold) {
    goto out;
  }

  if (p_altitude_hold->need_reinit) {
    p_altitude_hold->controller.integrator =
        p_altitude_hold->old_vz_c - diff_h * KP_H;
    p_altitude_hold->need_reinit = 0;
  }

  ret = EXIT_SUCCESS;

out:
  return (ret);
}

static int altitude_hold_model(altitude_hold_controller_t *p_altitude_hold,
                               double vz_c, double h_f, double h_c, double dt,
                               double *p_vz) {
  int ret = EXIT_FAILURE;
  const double diff_h = h_f - h_c;

  if (!p_altitude_hold) {
    goto out;
  }

  if (diff_h < -H_SWITCH) {
    *p_vz = vz_c;
    p_altitude_hold->need_reinit = 1;
    p_altitude_hold->old_vz_c = *p_vz;
  } else if (diff_h > H_SWITCH) {
    *p_vz = -vz_c;
    p_altitude_hold->need_reinit = 1;
    p_altitude_hold->old_vz_c = *p_vz;
  } else {
    init_altitude_hold_integrator(p_altitude_hold, diff_h);
    *p_vz = KP_H * diff_h + p_altitude_hold->controller.integrator;
    p_altitude_hold->controller.integrator += dt * KI_H * diff_h;
  }

  ret = EXIT_SUCCESS;

out:
  return (ret);
}

static int va_control_model(controller_t *p_va_control, double va_f,
                            double vz_f, double q_f, double va_c, double dt,
                            double *p_delta_th_c) {
  int ret = EXIT_FAILURE;

  if (!p_va_control) {
    goto out;
  }

  if (!p_delta_th_c) {
    goto out;
  }

  *p_delta_th_c = p_va_control->integrator + K1_VA * (va_f - RROSACE_VA_EQ) +
                  K1_VZ * vz_f + K1_Q * q_f;
  p_va_control->integrator += dt * K1_INT_VA * (va_c - va_f);

  ret = EXIT_SUCCESS;

out:
  return (ret);
}

static int vz_control_model(controller_t *p_vz_control, double vz_f,
                            double vz_c, double q_f, double az_f, double dt,
                            double *p_delta_e_c) {
  int ret = EXIT_FAILURE;

  if (!p_vz_control) {
    goto out;
  }

  if (!p_delta_e_c) {
    goto out;
  }

  *p_delta_e_c =
      p_vz_control->integrator + K2_VZ * vz_f + K2_Q * q_f + K2_AZ * az_f;
  p_vz_control->integrator += dt * K2_INT_VZ * (vz_c - vz_f);

  ret = EXIT_SUCCESS;

out:
  return (ret);
}

static int fcc_step(rrosace_fcc_t *p_fcc, rrosace_mode_t flight_mode,
                    double h_f, double vz_f, double va_f, double q_f,
                    double az_f, double h_c, double vz_c, double va_c,
                    const double *p_delta_e_c_monitored,
                    const double *p_delta_th_c_monitored,
                    const unsigned int *p_other_master_in_law,
                    double *p_delta_e_c, double *p_delta_th_c,
                    unsigned int *p_relay_delta_e_c,
                    unsigned int *p_relay_delta_th_c,
                    unsigned int *p_master_in_law, double dt) {

  int ret = EXIT_FAILURE;
  double computed_vz_c;
  double delta_e_c;
  double delta_th_c;
  const int is_com = (p_delta_e_c_monitored == NULL) ||
                     (p_delta_th_c_monitored == NULL) ||
                     (p_other_master_in_law == NULL);

  if (!p_fcc) {
    goto out;
  }

  if (flight_mode == RROSACE_ALTITUDE_HOLD) {
    ret = altitude_hold_model(&p_fcc->altitude_hold, vz_c, h_f, h_c, dt,
                              &computed_vz_c);
    if (ret == EXIT_FAILURE) {
      goto out;
    }
  } else if (flight_mode == RROSACE_COMMANDED) {
    computed_vz_c = vz_c;
  } else {
    goto out;
  }

  ret = va_control_model(&p_fcc->va_control, va_f, vz_f, q_f, va_c, dt,
                         &delta_th_c);
  if (ret == EXIT_FAILURE) {
    goto out;
  }
  ret = vz_control_model(&p_fcc->vz_control, vz_f, computed_vz_c, q_f, az_f, dt,
                         &delta_e_c);
  if (ret == EXIT_FAILURE) {
    goto out;
  }

  if (is_com) {
    *p_delta_e_c = delta_e_c;
    *p_delta_th_c = delta_th_c;
  } else {
    unsigned int relay_delta_e_c;
    unsigned int relay_delta_th_c;
    unsigned int master_in_law;

    relay_delta_e_c =
        fabs(delta_e_c - *p_delta_e_c_monitored) >= EPSILON_DELTA_E_C;
    relay_delta_th_c =
        fabs(delta_th_c - *p_delta_th_c_monitored) >= EPSILON_DELTA_TH_C;
    master_in_law = relay_delta_e_c && relay_delta_th_c;

    relay_delta_e_c |= *p_other_master_in_law;
    relay_delta_th_c |= *p_other_master_in_law;

    master_in_law &= (unsigned int)!*p_other_master_in_law;

    *p_relay_delta_e_c = relay_delta_e_c;
    *p_relay_delta_th_c = relay_delta_th_c;
    *p_master_in_law = master_in_law;
  }

out:
  return (ret);
}

rrosace_fcc_t *rrosace_fcc_new() {
  rrosace_fcc_t *p_fcc = calloc(1, sizeof(rrosace_fcc_t));

  if (!p_fcc) {
    goto out;
  }

  p_fcc->altitude_hold.controller.integrator = 0.;
  p_fcc->altitude_hold.need_reinit = 1;
  p_fcc->altitude_hold.old_vz_c = 0.;
  p_fcc->vz_control.integrator = RROSACE_DELTA_E_C_EQ;
  p_fcc->va_control.integrator = RROSACE_DELTA_TH_C_EQ;

out:
  return (p_fcc);
}

void rrosace_fcc_del(rrosace_fcc_t *p_fcc) {
  if (p_fcc) {
    free(p_fcc);
  }
}

int rrosace_fcc_com_step(rrosace_fcc_t *p_fcc, rrosace_mode_t mode, double h_f,
                         double vz_f, double va_f, double q_f, double az_f,
                         double h_c, double vz_c, double va_c,
                         double *p_delta_e_c, double *p_delta_th_c, double dt) {
  return (fcc_step(p_fcc, mode, h_f, vz_f, va_f, q_f, az_f, h_c, vz_c, va_c,
                   NULL, NULL, NULL, p_delta_e_c, p_delta_th_c, NULL, NULL,
                   NULL, dt));
}

int rrosace_fcc_mon_step(rrosace_fcc_t *p_fcc, rrosace_mode_t mode, double h_f,
                         double vz_f, double va_f, double q_f, double az_f,
                         double h_c, double vz_c, double va_c,
                         double delta_e_c_monitored,
                         double delta_th_c_monitored,
                         unsigned int other_master_in_law,
                         unsigned int *p_relay_delta_e_c,
                         unsigned int *p_relay_delta_th_c,
                         unsigned int *p_master_in_law, double dt) {
  return (fcc_step(p_fcc, mode, h_f, vz_f, va_f, q_f, az_f, h_c, vz_c, va_c,
                   &delta_e_c_monitored, &delta_th_c_monitored,
                   &other_master_in_law, NULL, NULL, p_relay_delta_e_c,
                   p_relay_delta_th_c, p_master_in_law, dt));
}
