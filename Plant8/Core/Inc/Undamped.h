/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Undamped.h
 *
 * Code generated for Simulink model 'Undamped'.
 *
 * Model version                  : 3.22
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Jun 13 16:24:22 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef Undamped_h_
#define Undamped_h_
#ifndef Undamped_COMMON_INCLUDES_
#define Undamped_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* Undamped_COMMON_INCLUDES_ */

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
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S1>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator1_DSTATE;/* '<S1>/Discrete-Time Integrator1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T controlTorque;                /* '<Root>/controlTorque' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T theta;                        /* '<Root>/theta' */
  real_T thetaDot;                     /* '<Root>/thetaDot' */
  real_T thetaDoubleDot;               /* '<Root>/thetaDoubleDot' */
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
extern void Undamped_initialize(void);
extern void Undamped_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('AdaptivelyControlledPendulum1c/Undamped Pendulum Model')    - opens subsystem AdaptivelyControlledPendulum1c/Undamped Pendulum Model
 * hilite_system('AdaptivelyControlledPendulum1c/Undamped Pendulum Model/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'AdaptivelyControlledPendulum1c'
 * '<S1>'   : 'AdaptivelyControlledPendulum1c/Undamped Pendulum Model'
 */
#endif                                 /* Undamped_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
