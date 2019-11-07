/**
 * @file filters.c
 * @brief RROSACE Scheduling of cyber-physical system library filters body.
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
 */

#include <stddef.h>
#include <stdlib.h>

#include <rrosace_constants.h>
#include <rrosace_filters.h>

static double val_eq[] = {RROSACE_H_EQ, RROSACE_VZ_EQ, RROSACE_VA_EQ,
                          RROSACE_Q_EQ, RROSACE_AZ_EQ};

static const double second_order_coeff_altitude_100_as[] = {0.766000101841272,
                                                            -1.734903205885821};
static const double second_order_coeff_altitude_100_bs[] = {0.014857648981438,
                                                            0.016239246974013};
static const double second_order_coeff_altitude_50_as[] = {0.586756156020839,
                                                           -1.477888930110354};
static const double second_order_coeff_altitude_50_bs[] = {0.049596808318647,
                                                           0.059270417591839};
static const double second_order_coeff_altitude_33_as[] = {0.445839214374383,
                                                           -1.227970132817902};
static const double second_order_coeff_altitude_33_bs[] = {0.094268996251840,
                                                           0.123600085304640};
static const double second_order_coeff_altitude_25_as[] = {0.344282786628352,
                                                           -1.010643377701049};
static const double second_order_coeff_altitude_25_bs[] = {0.137177088974822,
                                                           0.196462319952482};

static const double second_order_coeff_vertical_airspeed_100_as[] = {
    0.956543675476034, -1.955578398054313};
static const double second_order_coeff_vertical_airspeed_100_bs[] = {
    0.000479064865372430, 0.000486212556348925};
static const double second_order_coeff_vertical_airspeed_50_as[] = {
    0.914975803093201, -1.911199519984605};
static const double second_order_coeff_vertical_airspeed_50_bs[] = {
    0.001860178914816, 0.001916104193780};
static const double second_order_coeff_vertical_airspeed_33_as[] = {
    0.874036784828483, -1.865563793814790};
static const double second_order_coeff_vertical_airspeed_33_bs[] = {
    0.004141433623051, 0.004331557390642};
static const double second_order_coeff_vertical_airspeed_25_as[] = {
    0.837180720246048, -1.822731999002980};
static const double second_order_coeff_vertical_airspeed_25_bs[] = {
    0.007010380719078, 0.007438340523990};

static const double second_order_coeff_true_airspeed_100_as[] = {
    0.956543675476034, -1.955578398054313};
static const double second_order_coeff_true_airspeed_100_bs[] = {
    0.000479064865372430, 0.000486212556348925};
static const double second_order_coeff_true_airspeed_50_as[] = {
    0.914975803093201, -1.911199519984605};
static const double second_order_coeff_true_airspeed_50_bs[] = {
    0.001860178914816, 0.001916104193780};
static const double second_order_coeff_true_airspeed_33_as[] = {
    0.874036784828483, -1.865563793814790};
static const double second_order_coeff_true_airspeed_33_bs[] = {
    0.004141433623051, 0.004331557390642};
static const double second_order_coeff_true_airspeed_25_as[] = {
    0.837180720246048, -1.822731999002980};
static const double second_order_coeff_true_airspeed_25_bs[] = {
    0.007010380719078, 0.007438340523990};

static const double second_order_coeff_pitch_rate_100_as[] = {
    0.766000101841272, -1.734903205885821};
static const double second_order_coeff_pitch_rate_100_bs[] = {
    0.014857648981438, 0.016239246974013};
static const double second_order_coeff_pitch_rate_50_as[] = {
    0.586756156020839, -1.477888930110354};
static const double second_order_coeff_pitch_rate_50_bs[] = {0.049596808318647,
                                                             0.059270417591839};
static const double second_order_coeff_pitch_rate_33_as[] = {
    0.445839214374383, -1.227970132817902};
static const double second_order_coeff_pitch_rate_33_bs[] = {0.094268996251840,
                                                             0.123600085304640};
static const double second_order_coeff_pitch_rate_25_as[] = {
    0.344282786628352, -1.010643377701049};
static const double second_order_coeff_pitch_rate_25_bs[] = {0.137177088974822,
                                                             0.196462319952482};

static const double second_order_coeff_vertical_acceleration_100_as[] = {
    0.411240701442774, -1.158045899830964};
static const double second_order_coeff_vertical_acceleration_100_bs[] = {
    0.107849979167580, 0.145344822444230};
static const double second_order_coeff_vertical_acceleration_50_as[] = {
    0.169118914523145, -0.518588903229759};
static const double second_order_coeff_vertical_acceleration_50_bs[] = {
    0.229019233988375, 0.421510777305010};
static const double second_order_coeff_vertical_acceleration_33_as[] = {
    0.067700864731348, -0.115832026705568};
static const double second_order_coeff_vertical_acceleration_33_bs[] = {
    0.263451167882487, 0.688417670143293};
static const double second_order_coeff_vertical_acceleration_25_as[] = {
    0.028601207249487, 0.069303378493245};
static const double second_order_coeff_vertical_acceleration_25_bs[] = {
    0.228783762747218, 0.869120822995514};

/* Second order filter. */
struct second_order_filter {
  double x[2]; /**< States */
};

/* Anti-aliasing filter. */
struct rrosace_filter {
  const double *as;
  const double *bs;
  union {
    struct second_order_filter second_order_filter;
  } selected_filter;
  double (*filtering)(rrosace_filter_t *, double);
};

static double second_order_filtering(rrosace_filter_t * /* p_filter */,
                                     double /* to_filter */);

static int set_filter_coeffs(rrosace_filter_t * /* p_filter */,
                             enum rrosace_filter_type /* type */,
                             enum rrosace_filter_frequency /* frequency */);

static int set_filter_type(rrosace_filter_t * /* p_filter */,
                           enum rrosace_filter_type /* type */);

static double second_order_filtering(rrosace_filter_t *p_filter,
                                     double to_filter) {
  /* Output */
  double y = 0;

  if (p_filter) {
    /* State and next state */
    double x_next[] = {0, 0};

    /* Coeffs */
    const double *as = p_filter->as;
    const double *bs = p_filter->bs;

    /* Iterator */
    size_t i;

    x_next[1] = p_filter->selected_filter.second_order_filter.x[0];
    /* Output */
    y = p_filter->selected_filter.second_order_filter.x[1];

    /* Next state calculation */
    for (i = 0; i < 2; ++i) {
      x_next[i] += -as[i] * p_filter->selected_filter.second_order_filter.x[1] +
                   bs[i] * to_filter;
    }

    /* Updating current state for next iteration */
    for (i = 0; i < 2; ++i) {
      p_filter->selected_filter.second_order_filter.x[i] = x_next[i];
    }
  }

  return (y);
}

static int set_filter_coeffs(rrosace_filter_t *p_filter,
                             enum rrosace_filter_type type,
                             enum rrosace_filter_frequency frequency) {
  int output = 1;

  if (!p_filter) {
    goto out;
  }

  switch (type) {
  case RROSACE_ALTITUDE_FILTER:
    switch (frequency) {
    case RROSACE_FILTER_FREQ_100HZ:
      p_filter->as = second_order_coeff_altitude_100_as;
      p_filter->bs = second_order_coeff_altitude_100_bs;
      break;
    case RROSACE_FILTER_FREQ_50HZ:
      p_filter->as = second_order_coeff_altitude_50_as;
      p_filter->bs = second_order_coeff_altitude_50_bs;
      break;
    case RROSACE_FILTER_FREQ_33HZ:
      p_filter->as = second_order_coeff_altitude_33_as;
      p_filter->bs = second_order_coeff_altitude_33_bs;
      break;
    case RROSACE_FILTER_FREQ_25HZ:
      p_filter->as = second_order_coeff_altitude_25_as;
      p_filter->bs = second_order_coeff_altitude_25_bs;
      break;
    default:
      goto out;
    }
    break;
  case RROSACE_VERTICAL_AIRSPEED_FILTER:
    switch (frequency) {
    case RROSACE_FILTER_FREQ_100HZ:
      p_filter->as = second_order_coeff_vertical_airspeed_100_as;
      p_filter->bs = second_order_coeff_vertical_airspeed_100_bs;
      break;
    case RROSACE_FILTER_FREQ_50HZ:
      p_filter->as = second_order_coeff_vertical_airspeed_50_as;
      p_filter->bs = second_order_coeff_vertical_airspeed_50_bs;
      break;
    case RROSACE_FILTER_FREQ_33HZ:
      p_filter->as = second_order_coeff_vertical_airspeed_33_as;
      p_filter->bs = second_order_coeff_vertical_airspeed_33_bs;
      break;
    case RROSACE_FILTER_FREQ_25HZ:
      p_filter->as = second_order_coeff_vertical_airspeed_25_as;
      p_filter->bs = second_order_coeff_vertical_airspeed_25_bs;
      break;
    default:
      goto out;
    }
    break;
  case RROSACE_TRUE_AIRSPEED_FILTER:
    switch (frequency) {
    case RROSACE_FILTER_FREQ_100HZ:
      p_filter->as = second_order_coeff_true_airspeed_100_as;
      p_filter->bs = second_order_coeff_true_airspeed_100_bs;
      break;
    case RROSACE_FILTER_FREQ_50HZ:
      p_filter->as = second_order_coeff_true_airspeed_50_as;
      p_filter->bs = second_order_coeff_true_airspeed_50_bs;
      break;
    case RROSACE_FILTER_FREQ_33HZ:
      p_filter->as = second_order_coeff_true_airspeed_33_as;
      p_filter->bs = second_order_coeff_true_airspeed_33_bs;
      break;
    case RROSACE_FILTER_FREQ_25HZ:
      p_filter->as = second_order_coeff_true_airspeed_25_as;
      p_filter->bs = second_order_coeff_true_airspeed_25_bs;
      break;
    default:
      goto out;
    }
    break;
  case RROSACE_PITCH_RATE_FILTER:
    switch (frequency) {
    case RROSACE_FILTER_FREQ_100HZ:
      p_filter->as = second_order_coeff_pitch_rate_100_as;
      p_filter->bs = second_order_coeff_pitch_rate_100_bs;
      break;
    case RROSACE_FILTER_FREQ_50HZ:
      p_filter->as = second_order_coeff_pitch_rate_50_as;
      p_filter->bs = second_order_coeff_pitch_rate_50_bs;
      break;
    case RROSACE_FILTER_FREQ_33HZ:
      p_filter->as = second_order_coeff_pitch_rate_33_as;
      p_filter->bs = second_order_coeff_pitch_rate_33_bs;
      break;
    case RROSACE_FILTER_FREQ_25HZ:
      p_filter->as = second_order_coeff_pitch_rate_25_as;
      p_filter->bs = second_order_coeff_pitch_rate_25_bs;
      break;
    default:
      goto out;
    }
    break;
  case RROSACE_VERTICAL_ACCELERATION_FILTER:
    switch (frequency) {
    case RROSACE_FILTER_FREQ_100HZ:
      p_filter->as = second_order_coeff_vertical_acceleration_100_as;
      p_filter->bs = second_order_coeff_vertical_acceleration_100_bs;
      break;
    case RROSACE_FILTER_FREQ_50HZ:
      p_filter->as = second_order_coeff_vertical_acceleration_50_as;
      p_filter->bs = second_order_coeff_vertical_acceleration_50_bs;
      break;
    case RROSACE_FILTER_FREQ_33HZ:
      p_filter->as = second_order_coeff_vertical_acceleration_33_as;
      p_filter->bs = second_order_coeff_vertical_acceleration_33_bs;
      break;
    case RROSACE_FILTER_FREQ_25HZ:
      p_filter->as = second_order_coeff_vertical_acceleration_25_as;
      p_filter->bs = second_order_coeff_vertical_acceleration_25_bs;
      break;
    default:
      goto out;
    }
    break;
  default:
    goto out;
  }

  output = 0;

out:
  return (output);
}

static int set_filter_type(rrosace_filter_t *p_filter,
                           enum rrosace_filter_type type) {
  int output = 1;

  if (!p_filter) {
    goto out;
  }

  p_filter->selected_filter.second_order_filter.x[0] =
      val_eq[type] * (1.0 + p_filter->as[1] - p_filter->bs[1]);
  p_filter->selected_filter.second_order_filter.x[1] = val_eq[type];

  output = 0;

out:
  return (output);
}

rrosace_filter_t *rrosace_filter_new(rrosace_filter_type_t filter_type,
                                     rrosace_filter_frequency_t frequency) {
  rrosace_filter_t *p_filter =
      (rrosace_filter_t *)calloc(1, sizeof(rrosace_filter_t));
  int init_result;

  if (!p_filter) {
    goto out;
  }

  init_result = set_filter_coeffs(p_filter, filter_type, frequency);
  init_result += set_filter_type(p_filter, filter_type);

  if (init_result) {
    rrosace_filter_del(p_filter);
    goto out;
  }

  p_filter->filtering = second_order_filtering;

out:
  return (p_filter);
}

void rrosace_filter_del(rrosace_filter_t *p_filter) {
  if (p_filter) {
    free(p_filter);
  }
}

int rrosace_filter_step(rrosace_filter_t *p_filter, double to_filter,
                        double *p_filtered) {
  int ret = EXIT_FAILURE;

  if (!p_filter) {
    goto out;
  }

  if (!p_filtered) {
    goto out;
  }

  *p_filtered = p_filter->filtering(p_filter, to_filter);
  ret = EXIT_SUCCESS;

out:
  return (ret);
}
