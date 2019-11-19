/**
 * @file cables.c
 * @brief RROSACE Scheduling of cyber-physical system library cables body.
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

#include <stdlib.h>

#include <rrosace_cables.h>

enum switch_state { NOT_SWITCHED = 0, SWITCHED = 1 };
typedef enum switch_state switch_state_t;

static void reset_output(rrosace_cables_output_t * /* p_output */);

static switch_state_t switch_input(const rrosace_cables_input_t * /* p_input */,
                                   rrosace_cables_output_t * /* p_output */);
/**
 * @brief Reset cable output
 * @param[in,out] p_output a pointer to the cable output to reset
 * @return Nothing
 */
static void reset_output(rrosace_cables_output_t *p_output) {
  p_output->delta_e_c = 0.0;
  p_output->delta_th_c = 0.0;
}

/**
 * @brief Compute output of cables from input if switches are closed
 * @param[in] input a pointer to the cables input
 * @param[out] output a pointer to the cables output if switch closed
 * @return The switch state, SWITCHED if switched, else NOT_SWITCHED
 */
static switch_state_t switch_input(const rrosace_cables_input_t *p_input,
                                   rrosace_cables_output_t *p_output) {
  switch_state_t switched = NOT_SWITCHED;

  if ((p_input->relay_delta_e_c == RROSACE_RELAY_CLOSED) &&
      (p_input->relay_delta_th_c == RROSACE_RELAY_CLOSED)) {
    switched = SWITCHED;
    p_output->delta_e_c = p_input->delta_e_c;
    p_output->delta_th_c = p_input->delta_th_c;
  }

  return (switched);
}

int rrosace_cables_step(const rrosace_cables_input_t inputs[], size_t nb_input,
                        rrosace_cables_output_t *p_output) {
  int ret = EXIT_FAILURE;
  size_t it;
  switch_state_t output_selected = NOT_SWITCHED;

  if (!p_output) {
    goto out;
  }

  reset_output(p_output);

  for (it = 0, output_selected = 0;
       (it < nb_input) && (output_selected != SWITCHED); ++it) {
    output_selected = switch_input(&inputs[it], p_output);
  }

  ret = EXIT_SUCCESS;

out:
  return (ret);
}
