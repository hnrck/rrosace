/**
 * @file rrosace_cables.h
 * @brief RROSACE Scheduling of cyber-physical system library cables header.
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

#ifndef RROSACE_CABLES_H
#define RROSACE_CABLES_H

#include <rrosace_common.h>

/** Cables default freq */
#define RROSACE_CABLES_DEFAULT_FREQ (RROSACE_DEFAULT_PHYSICAL_FREQ)

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** RROSACE relays states */
enum rrosace_relay_state {
  RROSACE_RELAY_CLOSED, /**< relay is closed, equivalent to True */
  RROSACE_RELAY_OPENED  /**< relay is opened, equivalent to False */
};
typedef enum rrosace_relay_state rrosace_relay_state_t;

/** @struct rrosace_cables_input RROSACE cables input data from FCCs */
struct rrosace_cables_input {
  double delta_e_c;  /**< Elevator deflection command from FCCs */
  double delta_th_c; /**< Throttle command  from FCCs */
  rrosace_relay_state_t
      relay_delta_e_c; /**< Relay value for elevator deflection command */
  rrosace_relay_state_t
      relay_delta_th_c; /**< Relay value for delta throttle command */
};

/** @typedef Alias for cables input */
typedef struct rrosace_cables_input rrosace_cables_input_t;

/** @struct rrosace_cables_input RROSACE cables output data for engine and
 * elevator */
struct rrosace_cables_output {
  double delta_e_c;  /**< Elevator deflection command for elevator */
  double delta_th_c; /**< Delta throttle command for engine */
};

/** @typedef Alias for cables output */
typedef struct rrosace_cables_output rrosace_cables_output_t;

/**
 * @brief cables step, consume inputs and produce output
 * @param[in] inputs the data from FCCs to consume
 * @param[in] nb_input the number of FCCs data to consume
 * @param[out] p_output a pointer to the data to produce to elevator and
 * engine
 * @return EXIT_SUCCESS if OK, else EXIT_FAILURE
 */
int rrosace_cables_step(const rrosace_cables_input_t inputs[], size_t nb_input,
                        rrosace_cables_output_t *p_output);

#ifdef __cplusplus
}
namespace RROSACE {

/** @class Cables
 *  @brief C++ wrapper for C-based cables, based on Model interface.
 */
class Cables
#if __cplusplus > 199711L
    final
#endif /* __cplusplus > 199711L */
    : public Model {
public:
  typedef struct rrosace_cables_input Input;
  typedef struct rrosace_cables_output Output;

  /** Relay is closed, equivalent to True */
  static const int RELAY_CLOSED = RROSACE_RELAY_CLOSED;
  /** Relay is opened, equivalent to False */
  static const int RELAY_OPENED = RROSACE_RELAY_OPENED;

  typedef enum rrosace_relay_state RelayState;

private:
  const double &r_delta_e_c_partial_1;
  const double &r_delta_th_c_partial_1;
  const RelayState &r_relay_delta_e_c_1;
  const RelayState &r_relay_delta_th_c_1;

  const double &r_delta_e_c_partial_2;
  const double &r_delta_th_c_partial_2;
  const RelayState &r_relay_delta_e_c_2;
  const RelayState &r_relay_delta_th_c_2;

  double &r_delta_e_c;
  double &r_delta_th_c;

  double m_dt;

public:
  /** Default cables frequency */
  static const int DEFAULT_FREQ = RROSACE_CABLES_DEFAULT_FREQ;

  /**
   * @brief Cables constructor
   * @param[in] dt The execution period of the cables model instance, default 1
   * / DEFAULT_FREQ
   */
  Cables(const double &delta_e_c_partial_1, const double &delta_th_c_partial_1,
         const RelayState &relay_delta_e_c_1,
         const RelayState &relay_delta_th_c_1,
         const double &delta_e_c_partial_2, const double &delta_th_c_partial_2,
         const RelayState &relay_delta_e_c_2,
         const RelayState &relay_delta_th_c_2, double &delta_e_c,
         double &delta_th_c, double dt = 1. / DEFAULT_FREQ)
      : r_delta_e_c_partial_1(delta_e_c_partial_1),
        r_delta_th_c_partial_1(delta_th_c_partial_1),
        r_relay_delta_e_c_1(relay_delta_e_c_1),
        r_relay_delta_th_c_1(relay_delta_th_c_1),
        r_delta_e_c_partial_2(delta_e_c_partial_2),
        r_delta_th_c_partial_2(delta_th_c_partial_2),
        r_relay_delta_e_c_2(relay_delta_e_c_2),
        r_relay_delta_th_c_2(relay_delta_th_c_2), r_delta_e_c(delta_e_c),
        r_delta_th_c(delta_th_c), m_dt(dt) {}

  /**
   * @brief Cables copy constructor
   * @param[in] other another cables to construct
   */
  Cables(const Cables &other)
      : r_delta_e_c_partial_1(other.r_delta_e_c_partial_1),
        r_delta_th_c_partial_1(other.r_delta_th_c_partial_1),
        r_relay_delta_e_c_1(other.r_relay_delta_e_c_1),
        r_relay_delta_th_c_1(other.r_relay_delta_th_c_1),
        r_delta_e_c_partial_2(other.r_delta_e_c_partial_2),
        r_delta_th_c_partial_2(other.r_delta_th_c_partial_2),
        r_relay_delta_e_c_2(other.r_relay_delta_e_c_2),
        r_relay_delta_th_c_2(other.r_relay_delta_th_c_2),
        r_delta_e_c(other.r_delta_e_c), r_delta_th_c(other.r_delta_th_c),
        m_dt(other.m_dt) {}

  /**
   * @brief Cables copy assignement
   * @param[in] other another cables to construct
   */
  Cables &operator=(const Cables &) { return *this; }

#if __cplusplus > 199711L

  /**
   * @brief Cables move constructor
   * @param[in] ' ' an cables to move
   */
  Cables(Cables &&) = default;

  /**
   * @brief Cables move assignement
   * @param[in] ' ' an cables to move
   */
  Cables &operator=(Cables &&) = delete;

#endif /* __cplusplus > 199711L */

  /**
   * @brief Cables destructor
   */
  ~Cables() {}

  /**
   * @brief  Execute an cables model instance
   */
  void step() {
    int ret;
    const Input inputs[2] = {{r_delta_e_c_partial_1, r_delta_th_c_partial_1,
                              r_relay_delta_e_c_1, r_relay_delta_th_c_1},
                             {r_delta_e_c_partial_2, r_delta_th_c_partial_2,
                              r_relay_delta_e_c_2, r_relay_delta_th_c_2}};
    Output output;
    ret = rrosace_cables_step(inputs, 2, &output);
    r_delta_e_c = output.delta_e_c;
    r_delta_th_c = output.delta_th_c;
    if (ret == EXIT_FAILURE) {
      throw(std::runtime_error("Cables step failed."));
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
} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_CABLES_H */
