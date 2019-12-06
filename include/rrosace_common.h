/**
 * @file rrosace_common.h
 * @brief RROSACE common header.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2019-11-17
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

#ifndef RROSACE_COMMON_H
#define RROSACE_COMMON_H

#include <rrosace_constants.h>

#ifdef __cplusplus

#include <cstdlib>
#include <exception>

#if __cplusplus <= 199711L
#ifndef nullptr
#define nullptr NULL
#endif /* nullptr */
#endif /* __cplusplus <= 199711L */

namespace RROSACE {
/// Model abstract class
class Model {
public:
  virtual ~Model()
#if __cplusplus <= 199711L
      {};
#else
      = default;
#endif /* __cplusplus <= 199711L */
  /// Execute a model iteration
  virtual void step() = 0;
  /// Get model period
  virtual double get_dt() const = 0;
};

} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_COMMON_H */
