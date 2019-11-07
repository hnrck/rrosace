/**
 * @file fcu.c
 * @brief RROSACE Scheduling of cyber-physical system library FCU body.
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

#include <rrosace_fcu.h>

struct rrosace_fcu {
  double h_c;
  double vz_c;
  double va_c;
};

rrosace_fcu_t *rrosace_fcu_new() {
  rrosace_fcu_t *p_fcu = (rrosace_fcu_t *)calloc(1, sizeof(rrosace_fcu_t));

  if (!p_fcu) {
    goto out;
  }

  p_fcu->h_c = 0.;
  p_fcu->vz_c = 0.;
  p_fcu->va_c = 0.;

out:
  return (p_fcu);
}

void rrosace_fcu_del(rrosace_fcu_t *p_fcu) {
  if (p_fcu) {
    free(p_fcu);
  }
}

void rrosace_fcu_set_h_c(rrosace_fcu_t *p_fcu, double h_c) { p_fcu->h_c = h_c; }

void rrosace_fcu_set_vz_c(rrosace_fcu_t *p_fcu, double vz_c) {
  p_fcu->vz_c = vz_c;
}

void rrosace_fcu_set_va_c(rrosace_fcu_t *p_fcu, double va_c) {
  p_fcu->va_c = va_c;
}

double rrosace_fcu_get_h_c(const rrosace_fcu_t *p_fcu) { return (p_fcu->h_c); }

double rrosace_fcu_get_vz_c(const rrosace_fcu_t *p_fcu) {
  return (p_fcu->vz_c);
}

double rrosace_fcu_get_va_c(const rrosace_fcu_t *p_fcu) {
  return (p_fcu->va_c);
}
