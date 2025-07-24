/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: AdaptiveController.c
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

#include "AdaptiveController.h"
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
void AdaptiveController_step(void)
{
  real_T y[4];
  real_T p_dot[2];
  real_T a;
  real_T a_idx_0;
  real_T rtb_Add1;
  real_T rtb_Add2;
  real_T rtb_DiscreteTimeIntegrator2;
  real_T rtb_SineWaveFunction;
  int32_T i;

  /* DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' */
  rtb_DiscreteTimeIntegrator2 = rtDW.DiscreteTimeIntegrator2_DSTATE;

  /* Sum: '<Root>/Add2' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator2'
   *  Inport: '<Root>/thetaDot'
   */
  rtb_Add2 = rtU.thetaDot - rtDW.DiscreteTimeIntegrator2_DSTATE;

  /* Sum: '<Root>/Add1' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator3'
   *  Inport: '<Root>/theta'
   */
  rtb_Add1 = rtU.theta - rtDW.DiscreteTimeIntegrator3_DSTATE;

  /* Sin: '<Root>/Sine Wave Function' incorporates:
   *  Inport: '<Root>/theta'
   */
  rtb_SineWaveFunction = sin(rtU.theta);

  /* Outport: '<Root>/control' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator1'
   *  Gain: '<Root>/K0'
   *  Gain: '<Root>/K1'
   *  Inport: '<Root>/referenceThetaDoubleDot'
   *  Product: '<Root>/Product'
   *  Product: '<Root>/Product1'
   *  Sum: '<Root>/Add'
   *  Sum: '<Root>/Add3'
   */
  rtY.control = ((rtU.referenceThetaDoubleDot - 8.0 * rtb_Add2) - 16.0 *
                 rtb_Add1) * rtDW.DiscreteTimeIntegrator_DSTATE +
    rtDW.DiscreteTimeIntegrator1_DSTATE * rtb_SineWaveFunction;

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  Inport: '<Root>/thetaDoubleDot'
   */
  a = 1.0 / rtDW.DiscreteTimeIntegrator_DSTATE;
  a_idx_0 = a * rtU.thetaDoubleDot;
  a *= rtb_SineWaveFunction;
  rtb_SineWaveFunction = -90000.0 * a_idx_0 + 0.0 * a;
  a_idx_0 = 0.0 * a_idx_0 + -90000.0 * a;
  y[0] = rtb_SineWaveFunction * 0.0;
  y[1] = a_idx_0 * 0.0;
  y[2] = rtb_SineWaveFunction;
  y[3] = a_idx_0;
  for (i = 0; i < 2; i++) {
    rtb_SineWaveFunction = y[i + 2];
    a_idx_0 = y[i];
    p_dot[i] = (rtb_SineWaveFunction * 0.0664 + a_idx_0 * 0.0312) * rtb_Add2 +
      (rtb_SineWaveFunction * 0.0312 + a_idx_0 * 1.3125) * rtb_Add1;
  }

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator2' incorporates:
   *  Inport: '<Root>/referenceThetaDoubleDot'
   */
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.001 * rtU.referenceThetaDoubleDot;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator3' */
  rtDW.DiscreteTimeIntegrator3_DSTATE += 0.001 * rtb_DiscreteTimeIntegrator2;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.001 * p_dot[0];

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtDW.DiscreteTimeIntegrator1_DSTATE += 0.001 * p_dot[1];
}

/* Model initialize function */
void AdaptiveController_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_DSTATE = 8.0;

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator1' */
  rtDW.DiscreteTimeIntegrator1_DSTATE = 5.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
