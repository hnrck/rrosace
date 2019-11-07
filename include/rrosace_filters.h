/**
 * @file rrosace_filters.h
 * @brief RROSACE Scheduling of cyber-physical system library filters header.
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

#ifndef RROSACE_FILTERS_H
#define RROSACE_FILTERS_H

#include <rrosace_constants.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @enum Types of anti-aliasing filters */
enum rrosace_filter_type {
  RROSACE_ALTITUDE_FILTER,             /**< Altitude */
  RROSACE_VERTICAL_AIRSPEED_FILTER,    /**< Vertical speed */
  RROSACE_TRUE_AIRSPEED_FILTER,        /**< Airspeed */
  RROSACE_PITCH_RATE_FILTER,           /**< Pitch rate */
  RROSACE_VERTICAL_ACCELERATION_FILTER /**< Vertical acceleration */
};

/** @typedef Alias for the types of anti-aliasing filters */
typedef enum rrosace_filter_type rrosace_filter_type_t;

/** Frequencies of anti-aliasing filter. */
enum rrosace_filter_frequency {
  RROSACE_FILTER_FREQ_100HZ, /**< 100 Hz */
  RROSACE_FILTER_FREQ_50HZ,  /**< 50 Hz */
  RROSACE_FILTER_FREQ_33HZ,  /**< 35 Hz */
  RROSACE_FILTER_FREQ_25HZ   /**< 25 Hz */
};

/** @typedef Alias for the frequencies of anti-aliasing filters */
typedef enum rrosace_filter_frequency rrosace_filter_frequency_t;

/** Anti-aliasing filter model structure */
struct rrosace_filter;

/** @typedef Anti-aliasing filter model */
typedef struct rrosace_filter rrosace_filter_t;

/**
 * @brief Anti-aliasing filter constructor
 * @param[in] filter_type The type of filter to create
 * @param[in] frequency The frequency of the filter
 * @return A new filter
 */
rrosace_filter_t *rrosace_filter_new(rrosace_filter_type_t filter_type,
                                     rrosace_filter_frequency_t frequency);

/**
 * @brief Anti-aliasing filter destructor
 * @param[in,out] p_filter The filter to destroy
 */
void rrosace_filter_del(rrosace_filter_t *p_filter);

/**
 * @brief Anti-aliasing filter next state
 * @param[in,out] p_filter The filter to execute
 * @param[in] to_filter Data to filter
 * @param[out] p_filtered Filtered data
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_filter_step(rrosace_filter_t *p_filter, double to_filter,
                        double *p_filtered);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RROSACE_FILTERS_H */
