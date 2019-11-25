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

#include <rrosace_common.h>
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
#if __cplusplus <= 199711L
typedef struct rrosace_engine rrosace_engine_t;
#else
using rrosace_engine_t = struct rrosace_engine;
#endif

/**
 * @brief Create and initialize a new engine model
 * @param[in] tau The engine tau parameter
 * @return A new engine
 */
rrosace_engine_t *rrosace_engine_new(double tau);

/**
 * @brief Copy an engine in a new one
 * @param[in] p_other the engine to copy
 * @return A new engine
 */
rrosace_engine_t *rrosace_engine_copy(const rrosace_engine_t *p_other);

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
namespace RROSACE {

/** Engine default parameter tau */
static const double TAU = RROSACE_TAU;

/** @class Elevator
 *  @brief C++ wrapper for C-based elevator, based on Model interface.
 */
class Engine
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
private:
  /** Wrapped C-based elevator */
  rrosace_engine_t *p_engine; /**< C-struct engine wrapped */

  const double &r_delta_th_c;
  double &r_t;
  double m_dt;

public:
  /** Default elevator frequency */
  static const int DEFAULT_FREQ = RROSACE_ENGINE_DEFAULT_FREQ;

  /**
   * @brief Elevator constructor
   * @param[in] tau The engine tau parameter
   * @param[in] delta_th_c The commanded delta throttle
   * @param[out] t The simulated thrust
   * @param[in] dt The execution period of the engine model instance, default 1
   * / DEFAULT_FREQ
   */
  Engine(double tau, const double &delta_th_c, double &t,
         double dt = 1. / DEFAULT_FREQ)
      : p_engine(rrosace_engine_new(tau)), r_delta_th_c(delta_th_c), r_t(t),
        m_dt(dt) {}

  /**
   * @brief Elevator copy constructor
   * @param[in] other another elevator to construct
   */
  Engine(const Engine &other)
      : p_engine(rrosace_engine_copy(other.p_engine)),
        r_delta_th_c(other.r_delta_th_c), r_t(other.r_t), m_dt(other.m_dt) {}

  /**
   * @brief Elevator copy assignement
   * @param[in] other another elevator to construct
   */
  Engine &operator=(const Engine &other) {
    p_engine = rrosace_engine_copy(other.p_engine);
    return *this;
  }

#if __cplusplus > 199711L

  /**
   * @brief Elevator move constructor
   * @param[in] ' ' an elevator to move
   */
  Engine(Engine &&) = default;

  /**
   * @brief Elevator move assignement
   * @param[in] ' ' an elevator to move
   */
  Engine &operator=(Engine &&) = default;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Elevator destructor
   */
  ~Engine() { rrosace_engine_del(p_engine); }

/**
 * @brief  Execute an engine model instance
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
#if __cplusplus >= 201703L
  [[nodiscard]]
#endif
  int step() {
    return rrosace_engine_step(p_engine, r_delta_th_c, &r_t, m_dt);
  }
};
} /* namespace RROSACE */
#endif /* __cplusplus */
#endif /* RROSACE_ENGINE_H */
