/**
 * @file rrosace_constants.h
 * @brief RROSACE Scheduling of cyber-physical system library constants
 * header.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-08-01
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

#ifndef RROSACE_CONSTANTS_H
#define RROSACE_CONSTANTS_H

/** Initial elevator deflection */
#define RROSACE_DELTA_E_EQ (0.012009615652468)
/** Initial throttle */
#define RROSACE_DELTA_TH_EQ (1.5868660794926)
/** Initial thrust */
#define RROSACE_T_EQ (41813.9211946300056297)

/** Initial elevator deflection command */
#define RROSACE_DELTA_E_C_EQ (RROSACE_DELTA_E_EQ)
/** Initial delta throttle command */
#define RROSACE_DELTA_TH_C_EQ (RROSACE_DELTA_TH_EQ)

/** Initial Altitude */
#define RROSACE_H_EQ (10000.)
/** Iintial true airspeed */
#define RROSACE_VA_EQ (230.)
/** Initial vertical speed */
#define RROSACE_VZ_EQ (0.)
/** Initial pitch rate */
#define RROSACE_Q_EQ (0.)
/** Initial vertical acceleration */
#define RROSACE_AZ_EQ (0.)

/** Initial filtered altitude */
#define RROSACE_H_F_EQ (RROSACE_H_EQ)
/** Initial filtered airspeed */
#define RROSACE_VA_F_EQ (RROSACE_VA_EQ)
/** Initial filtered vertical speed */
#define RROSACE_VZ_F_EQ (RROSACE_VZ_EQ)
/** Initial filtered pitch rate */
#define RROSACE_Q_F_EQ (RROSACE_Q_EQ)
/** Initial filtered vertical acceleration */
#define RROSACE_AZ_F_EQ (RROSACE_AZ_EQ)

/** 50Hz frequency */
#define RROSACE_FREQ_50_HZ (50.)
/** 100Hz frequency */
#define RROSACE_FREQ_100_HZ (100.)
/** 200Hz frequency */
#define RROSACE_FREQ_200_HZ (200.)

/** Default frequency for cyber components */
#define RROSACE_DEFAULT_CYBER_FREQ (RROSACE_FREQ_50_HZ)
/** Default frequency for physical components */
#define RROSACE_DEFAULT_PHYSICAL_FREQ (RROSACE_FREQ_200_HZ)

/** RROSACE relays states */
enum rrosace_relay_state {
  RROSACE_RELAY_CLOSED, /**< relay is closed, equivalent to True */
  RROSACE_RELAY_OPENED  /**< relay is opened, equivalent to False */
};
typedef enum rrosace_relay_state rrosace_relay_state_t;

#endif /* RROSACE_CONSTANTS_H */
