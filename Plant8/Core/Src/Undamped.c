/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Undamped.c
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

#include "Undamped.h"
#include <math.h>
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void Undamped_step(void)
{
  real_T rtb_DiscreteTimeIntegrator;
  real_T rtb_uI;

  /* DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Outport: '<Root>/theta' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   */
  rtY.theta = rtDW.DiscreteTimeIntegrator1_DSTATE;

  /* Gain: '<S1>/1//I' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
   *  Gain: '<S1>/mgl'
   *  Inport: '<Root>/controlTorque'
   *  Sin: '<S1>/Sine Wave Function'
   *  Sum: '<S1>/Add'
   */
  rtb_uI = (rtU.controlTorque - 10.0 * sin(rtDW.DiscreteTimeIntegrator1_DSTATE))
    * 0.1;

  /* Outport: '<Root>/thetaDoubleDot' */
  rtY.thetaDoubleDot = rtb_uI;

  /* Outport: '<Root>/thetaDot' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   */
  rtY.thetaDot = rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.001 * rtb_uI;

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' */
  rtDW.DiscreteTimeIntegrator1_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator;
}

/* Model initialize function */
void Undamped_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
