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

static void reset_output(rrosace_cables_output_t * /* p_output */);

static int switch_delta_c(int /* relay_delta_c */);

static int switch_input(const rrosace_cables_input_t * /* p_input */,
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
 * @brief Determine the state of a switch from its relay command
 * @param[in] relay_delta_c the relay command for differential command
 * @return State of the switch, True (!0) if closed, else False (0)
 */
static int switch_delta_c(int relay_delta_c) { return (relay_delta_c == 0); }

/**
 * @brief Compute output of cables from input if switches are closed
 * @param[in] input a pointer to the cables input
 * @param[out] output a pointer to the cables output if switch closed
 * @return The output state, True (!0) if switched, else False (0)
 */
static int switch_input(const rrosace_cables_input_t *p_input,
                        rrosace_cables_output_t *p_output) {
  unsigned int switched = 0;

  if (switch_delta_c(p_input->relay_delta_e_c) &&
      switch_delta_c(p_input->relay_delta_th_c)) {
    switched = 1;
    p_output->delta_e_c = p_input->delta_e_c;
    p_output->delta_th_c = p_input->delta_th_c;
  }

  return (switched);
}

int rrosace_cables_step(const rrosace_cables_input_t inputs[], size_t nb_input,
                        rrosace_cables_output_t *p_output) {
  int ret = EXIT_FAILURE;
  size_t it;
  int output_selected;

  if (!p_output) {
    goto out;
  }

  reset_output(p_output);

  for (it = 0, output_selected = 0; (it < nb_input) && !output_selected; ++it) {
    output_selected = switch_input(&inputs[it], p_output);
  }

  ret = EXIT_SUCCESS;

out:
  return (ret);
}
