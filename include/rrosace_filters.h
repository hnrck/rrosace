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
 * @brief Anti-aliasing filter copy constructor
 * @param[in] p_other a filter to copy
 * @return A new filter
 */
rrosace_filter_t *rrosace_filter_copy(const rrosace_filter_t *p_other);

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
namespace RROSACE {

/** @class Filter interface
 *  @brief C++ wrapper for C-based filter, based on Model interface.
 */
class IFilter : public Model {};

/** @class Filter
 *  @brief Implementation of IFilter containing common logic
 */
class Filter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {
private:
  /** Wrapped C-based filter */
  rrosace_filter_t *p_filter; /**< C-struct filter wrapped */

  /** The value to filter */
  const double &r_value;
  /** The value filtered */
  double &r_filtered_value;

  /** The execution period of the filter model instance */
  double m_dt;

public:
  /**
   * @brief Filter constructor
   * @param[in,out] p_filter a filter
   * @param[in] value the value to filter
   * @param[out] filtered_value the value filtered
   * @param[in] f the filter frequency
   */
  Filter(rrosace_filter_t *p_filter, const double &value,
         double &filtered_value, double f)
      : p_filter(p_filter), r_value(value), r_filtered_value(filtered_value),
        m_dt(1. / f) {}

  /**
   * @brief Filter copy constructor
   * @param[in] other another filter to construct
   */
  Filter(const Filter &other)
      : p_filter(rrosace_filter_copy(other.p_filter)), r_value(other.r_value),
        r_filtered_value(other.r_filtered_value), m_dt(other.m_dt) {}

  /**
   * @brief Filter copy assignement
   * @param[in] other another filter to construct
   */
  Filter &operator=(const Filter &other) {
    p_filter = rrosace_filter_copy(other.p_filter);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Filter move constructor
   * @param[in] ' ' an filter to move
   */
  Filter(Filter &&) = default;

  /**
   * @brief Filter move assignement
   * @param[in] ' ' an filter to move
   */
  Filter &operator=(Filter &&) = delete;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Filter destructor
   */
  ~Filter() { rrosace_filter_del(p_filter); }

  /**
   * @brief  Execute a filter model instance
   */
  void step() {
    const int ret = rrosace_filter_step(p_filter, r_value, &r_filtered_value);
    if (ret == EXIT_FAILURE) {
      throw(std::runtime_error("Filter step failed."));
    }
  }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return m_dt;
  }
};

/** @class Altitude filter
 *  @brief Implementation of IFilter containing altitude particularities
 */
class AltitudeFilter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {

private:
  Filter filter;

public:
  /**
   * @brief Filter constructor
   * @param[in] h the altitude to filter
   * @param[out] h_f the altitude filtered
   * @param[in] f the filter frequency
   */
  AltitudeFilter(const double &h, double &h_f,
                 rrosace_filter_frequency f = RROSACE_FILTER_FREQ_50HZ)
      : filter(rrosace_filter_new(RROSACE_ALTITUDE_FILTER, f), h, h_f,
               FREQ_50_HZ) {}
  /**
   * @brief  execute a filter model instance
   */
  void step() { filter.step(); }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return filter.get_dt();
  }
};

/** @class Vertical airspeed filter
 *  @brief Implementation of IFilter containing vertical airspeed
 * particularities
 */
class VerticalAirspeedFilter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {
private:
  Filter filter;

public:
  /**
   * @brief Filter constructor
   * @param[in] vz the vertical airspeed to filter
   * @param[out] vz_f the vertical airspeed filtered
   * @param[in] f the filter frequency
   */
  VerticalAirspeedFilter(const double &vz, double &vz_f,
                         rrosace_filter_frequency f = RROSACE_FILTER_FREQ_100HZ)
      : filter(rrosace_filter_new(RROSACE_VERTICAL_AIRSPEED_FILTER, f), vz,
               vz_f, FREQ_100_HZ) {}

  /**
   * @brief  execute a filter model instance
   */
  void step() { filter.step(); }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return filter.get_dt();
  }
};

/** @class True airspeed filter
 *  @brief Implementation of IFilter containing true airspeed particularities
 */
class TrueAirspeedFilter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {
private:
  Filter filter;

public:
  /**
   * @brief Filter constructor
   * @param[in] va the airspeed to filter
   * @param[out] va_f the airspeed filtered
   * @param[in] f the filter frequency
   */
  TrueAirspeedFilter(const double &va, double &va_f,
                     rrosace_filter_frequency f = RROSACE_FILTER_FREQ_100HZ)
      : filter(rrosace_filter_new(RROSACE_TRUE_AIRSPEED_FILTER, f), va, va_f,
               FREQ_100_HZ) {}

  /**
   * @brief  execute a filter model instance
   */
  void step() { filter.step(); }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return filter.get_dt();
  }
};

/** @class Pitch rate filter
 *  @brief Implementation of IFilter containing pitch rate particularities
 */
class PitchRateFilter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {
private:
  Filter filter;

public:
  /**
   * @brief Filter constructor
   * @param[in] q the pitch rate to filter
   * @param[out] q_f the pitch rate filtered
   * @param[in] f the filter frequency
   */
  PitchRateFilter(const double &q, double &q_f,
                  rrosace_filter_frequency f = RROSACE_FILTER_FREQ_100HZ)
      : filter(rrosace_filter_new(RROSACE_PITCH_RATE_FILTER, f), q, q_f,
               FREQ_100_HZ) {}

  /**
   * @brief  execute a filter model instance
   */
  void step() { filter.step(); }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return filter.get_dt();
  }
};

/** @class Vertical acceleration filter
 *  @brief Implementation of IFilter containing vertical acceleration
 * particularities
 */
class VerticalAccelerationFilter
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public IFilter {
private:
  Filter filter;

public:
  /**
   * @brief Filter constructor
   * @param[in] az the vertical acceleration to filter
   * @param[out] az_f the vertical acceleration filtered
   * @param[in] f the filter frequency
   */
  VerticalAccelerationFilter(
      const double &az, double &az_f,
      rrosace_filter_frequency f = RROSACE_FILTER_FREQ_100HZ)
      : filter(rrosace_filter_new(RROSACE_VERTICAL_ACCELERATION_FILTER, f), az,
               az_f, FREQ_100_HZ) {}

  /**
   * @brief  execute a filter model instance
   */
  void step() { filter.step(); }

/**
 * @brief Get period set in model
 * @return period, in s
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  double
  get_dt() const {
    return filter.get_dt();
  }
};

} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_FILTERS_H */
