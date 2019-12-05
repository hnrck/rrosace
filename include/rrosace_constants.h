/**
 * @file rrosace_constants.h
 * @brief RROSACE Scheduling of cyber-physical system library constants
 * header.
 * @author Henrick Deschamps
 * @version 1.0.0
 * @date 2016-08-01
 *
 * Based on the Open Source ROSACE (Research Open-Source Avionics and Control
 * Engineering) case study. Implementations of ROSACE available at:
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

/** time resolution */
#define RROSACE_TIME_RESOLUTION (1e-3)

enum rrosace_default_freq {
  RROSACE_FREQ_50_HZ = 50,   /**< 50Hz frequency */
  RROSACE_FREQ_100_HZ = 100, /**< 100Hz frequency */
  RROSACE_FREQ_200_HZ = 200  /**< 200Hz frequency */
};
typedef enum rrosace_default_freq rrosace_default_freq_t;

/** Default frequency for cyber components */
#define RROSACE_DEFAULT_CYBER_FREQ (RROSACE_FREQ_50_HZ)

/** Default frequency for physical components */
#define RROSACE_DEFAULT_PHYSICAL_FREQ (RROSACE_FREQ_200_HZ)

#ifdef __cplusplus
namespace RROSACE {

/** Initial elevator deflection command */
static const double DELTA_E_EQ = RROSACE_DELTA_E_EQ;
/** Initial throttle */
static const double DELTA_TH_EQ = RROSACE_DELTA_TH_EQ;
/** Initial thrust */
static const double T_EQ = RROSACE_T_EQ;
/** Initial elevator deflection command */
static const double DELTA_E_C_EQ = RROSACE_DELTA_E_C_EQ;
/** Initial delta throttle command */
static const double DELTA_TH_C_EQ = RROSACE_DELTA_TH_C_EQ;

/** Initial Altitude */
static const double H_EQ = RROSACE_H_EQ;
/** Iintial true airspeed */
static const double VA_EQ = RROSACE_VA_EQ;
/** Initial vertical speed */
static const double VZ_EQ = RROSACE_VZ_EQ;
/** Initial pitch rate */
static const double Q_EQ = RROSACE_Q_EQ;
/** Initial vertical acceleration */
static const double AZ_EQ = RROSACE_AZ_EQ;

/** Initial filtered altitude */
static const double H_F_EQ = RROSACE_H_F_EQ;
/** Initial filtered airspeed */
static const double VA_F_EQ = RROSACE_VA_F_EQ;
/** Initial filtered vertical speed */
static const double VZ_F_EQ = RROSACE_VZ_F_EQ;
/** Initial filtered pitch rate */
static const double Q_F_EQ = RROSACE_Q_F_EQ;
/** Initial filtered vertical acceleration */
static const double AZ_F_EQ = RROSACE_AZ_F_EQ;

/** time resolution */
static const double TIME_RESOLUTION = RROSACE_TIME_RESOLUTION;

/** 50Hz frequency */
static const int FREQ_50_HZ = RROSACE_FREQ_50_HZ;
/** 100Hz frequency */
static const int FREQ_100_HZ = RROSACE_FREQ_100_HZ;
/** 200Hz frequency */
static const int FREQ_200_HZ = RROSACE_FREQ_200_HZ;

/** Default frequency for cyber components */
static const int DEFAULT_CYBER_FREQ = RROSACE_DEFAULT_CYBER_FREQ;
/** Default frequency for physical components */
static const int DEFAULT_PHYSICAL_FREQ = RROSACE_DEFAULT_PHYSICAL_FREQ;
} /* namespace RROSACE */
#endif /* __cplusplus */

#endif /* RROSACE_CONSTANTS_H */
