/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AdaptiveController.h
 *
 * Code generated for Simulink model 'AdaptiveController'.
 *
 * Model version                  : 3.28
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Jul 22 11:48:07 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef AdaptiveController_h_
#define AdaptiveController_h_
#ifndef AdaptiveController_COMMON_INCLUDES_
#define AdaptiveController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* AdaptiveController_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator2_DSTATE;/* '<Root>/Discrete-Time Integrator2' */
  real_T DiscreteTimeIntegrator3_DSTATE;/* '<Root>/Discrete-Time Integrator3' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<Root>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<Root>/Discrete-Time Integrator1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T theta;                        /* '<Root>/theta' */
  real_T thetaDot;                     /* '<Root>/thetaDot' */
  real_T thetaDoubleDot;               /* '<Root>/thetaDoubleDot' */
  real_T referenceThetaDoubleDot;      /* '<Root>/referenceThetaDoubleDot' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T control;                      /* '<Root>/control' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void AdaptiveController_initialize(void);
extern void AdaptiveController_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'AdaptiveController'
 * '<S1>'   : 'AdaptiveController/MATLAB Function'
 */
#endif                                 /* AdaptiveController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
