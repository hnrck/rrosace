/**
 * @file first_experiment.c
 * @Synopsis RROSACE simple loop with models from the RROSACE library (See
 * simoulink ROSACE for reference).
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2019-11-17
 */

#include <stdio.h>
#include <stdlib.h>

#include <rrosace.h>

#define SIMPLE_LOOP_VZ_C (2.5)

#define NB_FCCS_COUPLES (2)
#define NB_FCCS (NB_FCCS_COUPLES * 2)

struct models {
  rrosace_engine_t *p_engine;
  rrosace_elevator_t *p_elevator;
  rrosace_flight_dynamics_t *p_flight_dynamics;
  rrosace_filter_t *p_h_filter;
  rrosace_filter_t *p_vz_filter;
  rrosace_filter_t *p_va_filter;
  rrosace_filter_t *p_q_filter;
  rrosace_filter_t *p_az_filter;
  rrosace_flight_mode_t *p_flight_mode;
  rrosace_fcu_t *p_fcu;
  rrosace_fcc_t *p_fccs[NB_FCCS];
};
typedef struct models models_t;

struct values {
  rrosace_mode_t mode;
  double delta_e;
  double delta_e_c_partial[NB_FCCS_COUPLES];
  double delta_th_c_partial[NB_FCCS_COUPLES];
  rrosace_relay_state_t relay_delta_e_c[NB_FCCS_COUPLES];
  rrosace_relay_state_t relay_delta_th_c[NB_FCCS_COUPLES];
  double delta_e_c;
  double delta_th_c;
  double t;
  double h;
  double vz;
  double va;
  double q;
  double az;
  double h_f;
  double vz_f;
  double va_f;
  double q_f;
  double az_f;
  rrosace_master_in_law_t master_in_laws[NB_FCCS_COUPLES];
  rrosace_master_in_law_t other_master_in_laws[NB_FCCS_COUPLES];
  double h_c;
  double vz_c;
  double va_c;
};
typedef struct values values_t;

static int create_models(models_t * /* p_models */);

static void delete_models(models_t * /* p_models */);

static int loop(double /* time_max */);

static int simulation_step(models_t * /* p_models */, values_t * /* p_values */,
                           double * /* p_time */);

static void print_values(double /* time */, const values_t * /* p_values */);

static int create_models(models_t *p_models) {
  int ret = EXIT_FAILURE;
  size_t i;

  if (!p_models) {
    goto out;
  }

  p_models->p_engine = rrosace_engine_new(RROSACE_TAU);
  p_models->p_elevator = rrosace_elevator_new(RROSACE_OMEGA, RROSACE_XI);
  p_models->p_flight_dynamics = rrosace_flight_dynamics_new();
  p_models->p_h_filter =
      rrosace_filter_new(RROSACE_ALTITUDE_FILTER, RROSACE_FILTER_FREQ_50HZ);
  p_models->p_vz_filter = rrosace_filter_new(RROSACE_VERTICAL_AIRSPEED_FILTER,
                                             RROSACE_FILTER_FREQ_100HZ);
  p_models->p_va_filter = rrosace_filter_new(RROSACE_TRUE_AIRSPEED_FILTER,
                                             RROSACE_FILTER_FREQ_100HZ);
  p_models->p_q_filter =
      rrosace_filter_new(RROSACE_PITCH_RATE_FILTER, RROSACE_FILTER_FREQ_100HZ);
  p_models->p_az_filter = rrosace_filter_new(
      RROSACE_VERTICAL_ACCELERATION_FILTER, RROSACE_FILTER_FREQ_100HZ);
  p_models->p_flight_mode = rrosace_flight_mode_new();
  p_models->p_fcu = rrosace_fcu_new();

  for (i = 0; i < NB_FCCS; ++i) {
    p_models->p_fccs[i] = rrosace_fcc_new();
  }

  if (!p_models->p_engine || !p_models->p_elevator ||
      !p_models->p_flight_dynamics || !p_models->p_h_filter ||
      !p_models->p_vz_filter || !p_models->p_va_filter ||
      !p_models->p_q_filter || !p_models->p_az_filter ||
      !p_models->p_flight_mode || !p_models->p_fcu) {
    goto out;
  }

  for (i = 0; i < NB_FCCS; ++i) {
    if (!p_models->p_fccs[i]) {
      goto out;
    }
  }

  ret = EXIT_SUCCESS;

out:
  return (ret);
}

static void delete_models(models_t *p_models) {
  size_t i;

  if (!p_models) {
    goto out;
  }

  rrosace_engine_del(p_models->p_engine);
  rrosace_elevator_del(p_models->p_elevator);
  rrosace_flight_dynamics_del(p_models->p_flight_dynamics);
  rrosace_filter_del(p_models->p_h_filter);
  rrosace_filter_del(p_models->p_vz_filter);
  rrosace_filter_del(p_models->p_va_filter);
  rrosace_filter_del(p_models->p_q_filter);
  rrosace_filter_del(p_models->p_az_filter);
  rrosace_flight_mode_del(p_models->p_flight_mode);
  rrosace_fcu_del(p_models->p_fcu);

  for (i = 0; i < NB_FCCS; ++i) {
    rrosace_fcc_del(p_models->p_fccs[i]);
  }

out:
  return;
}

static int loop(double time_max) {
  int ret;
  models_t models;
  values_t values = {
      RROSACE_COMMANDED,
      RROSACE_DELTA_E_EQ,
      {RROSACE_DELTA_E_EQ, RROSACE_DELTA_E_EQ},
      {RROSACE_DELTA_TH_EQ, RROSACE_DELTA_TH_EQ},
      {RROSACE_RELAY_CLOSED, RROSACE_RELAY_OPENED},
      {RROSACE_RELAY_CLOSED, RROSACE_RELAY_OPENED},
      RROSACE_DELTA_E_C_EQ,
      RROSACE_DELTA_TH_C_EQ,
      RROSACE_T_EQ,
      RROSACE_H_EQ,
      RROSACE_VZ_EQ,
      RROSACE_VA_EQ,
      RROSACE_Q_EQ,
      RROSACE_AZ_EQ,
      RROSACE_H_F_EQ,
      RROSACE_VZ_F_EQ,
      RROSACE_VA_F_EQ,
      RROSACE_Q_F_EQ,
      RROSACE_AZ_F_EQ,
      {RROSACE_MASTER_IN_LAW, RROSACE_NOT_MASTER_IN_LAW},
      {RROSACE_NOT_MASTER_IN_LAW, RROSACE_MASTER_IN_LAW},
      RROSACE_H_EQ,
      SIMPLE_LOOP_VZ_C,
      RROSACE_VA_EQ,
  };
  double time;

  ret = create_models(&models);

  if (ret == EXIT_FAILURE) {
    goto out;
  }

  /* Init FCU */
  rrosace_flight_mode_set_mode(models.p_flight_mode, values.mode);
  rrosace_fcu_set_va_c(models.p_fcu, values.va_c);
  rrosace_fcu_set_vz_c(models.p_fcu, values.vz_c);

  for (time = 0., ret = EXIT_SUCCESS;
       (time < time_max) && (ret == EXIT_SUCCESS);) {
    ret = simulation_step(&models, &values, &time);
    print_values(time, &values);
  }

out:
  delete_models(&models);

  return (ret);
}

static int simulation_step(models_t *p_models, values_t *p_values,
                           double *p_time) {
  int ret = EXIT_SUCCESS;
  size_t i;
  double dt;

  static size_t logical_time = 0;
  static const size_t elevator_logical_period = 1;
  static const size_t engine_logical_period = 1;
  static const size_t flight_dynamics_logical_period = 1;
  static const size_t altitude_filter_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FREQ_50_HZ);
  static const size_t airspeed_filter_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FREQ_100_HZ);
  static const size_t vertical_speed_filter_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FREQ_100_HZ);
  static const size_t pitch_rate_filter_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FREQ_100_HZ);
  static const size_t vertical_acceleration_filter_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FREQ_100_HZ);
  static const size_t fcu_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FCU_DEFAULT_FREQ);
  static const size_t flight_mode_logical_period = (size_t)(
      RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FLIGHT_MODE_DEFAULT_FREQ);
  static const size_t fcc_logical_period =
      (size_t)(RROSACE_DEFAULT_PHYSICAL_FREQ / RROSACE_FCC_DEFAULT_FREQ);
  static const size_t cables_logical_period = 1;

  if (!p_models || !p_values || !p_time) {
    goto out;
  }

  if (logical_time % elevator_logical_period == 0) {
    dt = 1. / RROSACE_ELEVATOR_DEFAULT_FREQ;
    rrosace_elevator_step(p_models->p_elevator, p_values->delta_e_c,
                          &p_values->delta_e, dt);
  }

  if (logical_time % engine_logical_period == 0) {
    dt = 1. / RROSACE_ENGINE_DEFAULT_FREQ;
    rrosace_engine_step(p_models->p_engine, p_values->delta_th_c, &p_values->t,
                        dt);
  }

  if (logical_time % flight_dynamics_logical_period == 0) {
    dt = 1. / RROSACE_FLIGHT_DYNAMICS_DEFAULT_FREQ;
    rrosace_flight_dynamics_step(p_models->p_flight_dynamics, p_values->delta_e,
                                 p_values->t, &p_values->h, &p_values->vz,
                                 &p_values->va, &p_values->q, &p_values->az,
                                 dt);
  }

  if (logical_time % altitude_filter_logical_period == 0) {
    rrosace_filter_step(p_models->p_h_filter, p_values->h, &p_values->h_f);
  }

  if (logical_time % vertical_speed_filter_logical_period == 0) {
    rrosace_filter_step(p_models->p_vz_filter, p_values->vz, &p_values->vz_f);
  }

  if (logical_time % airspeed_filter_logical_period == 0) {
    rrosace_filter_step(p_models->p_va_filter, p_values->va, &p_values->va_f);
  }

  if (logical_time % pitch_rate_filter_logical_period == 0) {
    rrosace_filter_step(p_models->p_q_filter, p_values->q, &p_values->q_f);
  }

  if (logical_time % vertical_acceleration_filter_logical_period == 0) {
    rrosace_filter_step(p_models->p_az_filter, p_values->az, &p_values->az_f);
  }

  if (logical_time % flight_mode_logical_period == 0) {
    p_values->mode = rrosace_flight_mode_get_mode(p_models->p_flight_mode);
  }

  if (logical_time % fcu_logical_period == 0) {
    p_values->h_c = rrosace_fcu_get_h_c(p_models->p_fcu);
    p_values->vz_c = rrosace_fcu_get_vz_c(p_models->p_fcu);
    p_values->va_c = rrosace_fcu_get_va_c(p_models->p_fcu);
  }

  if (logical_time % fcc_logical_period == 0) {
    dt = 1. / RROSACE_FCC_DEFAULT_FREQ;
    for (i = 0; i < NB_FCCS_COUPLES; ++i) {
      rrosace_fcc_com_step(p_models->p_fccs[i], p_values->mode, p_values->h_f,
                           p_values->vz_f, p_values->va_f, p_values->q_f,
                           p_values->az_f, p_values->h_c, p_values->vz_c,
                           p_values->va_c, &p_values->delta_e_c_partial[i],
                           &p_values->delta_th_c_partial[i], dt);
    }

    for (i = 0; i < NB_FCCS_COUPLES; ++i) {
      rrosace_fcc_mon_step(
          p_models->p_fccs[i + NB_FCCS_COUPLES], p_values->mode, p_values->h_f,
          p_values->vz_f, p_values->va_f, p_values->q_f, p_values->az_f,
          p_values->h_c, p_values->vz_c, p_values->va_c,
          p_values->delta_e_c_partial[i], p_values->delta_th_c_partial[i],
          p_values->other_master_in_laws[i], &p_values->relay_delta_e_c[i],
          &p_values->relay_delta_th_c[i], &p_values->master_in_laws[i], dt);
    }
  }

  if (logical_time % cables_logical_period == 0) {
    rrosace_cables_input_t cables_input[NB_FCCS_COUPLES];
    rrosace_cables_output_t cables_output;

    for (i = 0; i < NB_FCCS_COUPLES; ++i) {
      cables_input[i].delta_e_c = p_values->delta_e_c_partial[i];
      cables_input[i].delta_th_c = p_values->delta_th_c_partial[i];
      cables_input[i].relay_delta_e_c = p_values->relay_delta_e_c[i];
      cables_input[i].relay_delta_th_c = p_values->relay_delta_th_c[i];
    }

    rrosace_cables_step(cables_input, sizeof(cables_input), &cables_output);

    p_values->delta_e_c = cables_output.delta_e_c;
    p_values->delta_th_c = cables_output.delta_th_c;
  }

  dt = 1. / RROSACE_DEFAULT_PHYSICAL_FREQ;

  *p_time = (++logical_time) * dt;

  ret = EXIT_SUCCESS;
out:
  return (ret);
}

static void print_values(double time, const values_t *p_values) {
  fprintf(stdout, "%5.3f,%5.6f,%5.6f,%5.6f\n", time, p_values->h, p_values->vz,
          p_values->va);
}

int main() {
  int ret;
  const double time_max = 50.0;

  printf("time (s),altitude (m),vertical speed (m/s),airspeed (m/s)\n");

  ret = loop(time_max);

  return (ret);
}
