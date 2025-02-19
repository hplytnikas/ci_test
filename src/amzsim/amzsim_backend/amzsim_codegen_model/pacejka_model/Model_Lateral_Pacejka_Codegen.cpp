//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Model_Lateral_Pacejka_Codegen.cpp
//
// Code generated for Simulink model 'Model_Lateral_Pacejka_Codegen'.
//
// Model version                  : 1.86
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Mon Mar 11 08:54:07 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "Model_Lateral_Pacejka_Codegen.h"
#include "rtwtypes.h"
#include <cmath>

// Model step function
void Model_Lateral_Pacejka_Codegen::step() {
  real_T Delay3_DSTATE_tmp;
  real_T Outputs_ax_tmp;
  real_T rtb_Product;
  real_T rtb_Product_c;
  real_T rtb_Product_iu;
  real_T rtb_Product_lk;
  real_T rtb_Product_lu;
  real_T rtb_Product_p;
  real_T rtb_Subtract;
  real_T rtb_Subtract_a;
  real_T rtb_Subtract_c;
  real_T rtb_Subtract_f;
  real_T rtb_Switch;
  real_T tmp;

  // Math: '<S5>/Square' incorporates:
  //   Delay: '<Root>/Delay'
  //   Math: '<S6>/Square'

  rtb_Subtract = Model_Lateral_Pacejka_Codegen_Y.Outputs_vx * Model_Lateral_Pacejka_Codegen_Y.Outputs_vx;

  // Product: '<S5>/Product' incorporates:
  //   Constant: '<S5>/Constant'
  //   Constant: '<S5>/Constant1'
  //   Constant: '<S5>/Constant2'
  //   Constant: '<S5>/Constant3'
  //   Math: '<S5>/Square'

  rtb_Product_c = rtb_Subtract * 0.960288515703119;

  // Product: '<S6>/Product' incorporates:
  //   Constant: '<S6>/Constant'
  //   Constant: '<S6>/Constant1'
  //   Constant: '<S6>/Constant2'
  //   Constant: '<S6>/Constant3'

  rtb_Product = rtb_Subtract * 2.4147605950429307;

  // Product: '<S17>/Divide' incorporates:
  //   Constant: '<S17>/car.L_Wheelbase'
  //   Delay: '<Root>/Delay3'
  //   Gain: '<S17>/car.Z_CG'
  //   Gain: '<S17>/car.m_Vehicle_total'
  //   Product: '<S15>/Divide'
  //   Product: '<S16>/Divide'
  //   Product: '<S18>/Divide'

  rtb_Product_p = 176.0 * Model_Lateral_Pacejka_Codegen_Y.Outputs_ax * 0.258 / 1.53;

  // Product: '<S17>/Divide2' incorporates:
  //   Constant: '<S17>/car.L_Wheelbase2'
  //   Gain: '<S17>/car.Z_CG1'
  //   Product: '<S15>/Divide2'
  //   Product: '<S16>/Divide2'
  //   Product: '<S18>/Divide2'

  rtb_Subtract_c = 0.258 * rtb_Product_c / 1.53;

  // Product: '<S17>/Divide3' incorporates:
  //   Constant: '<S17>/car.L_Trackwidth_R'
  //   Delay: '<Root>/Delay4'
  //   Gain: '<S17>/car.Z_CG2'
  //   Gain: '<S17>/car.m_Vehicle_total2'
  //   Product: '<S15>/Divide3'
  //   Product: '<S16>/Divide3'
  //   Product: '<S18>/Divide3'

  rtb_Switch = 176.0 * Model_Lateral_Pacejka_Codegen_Y.Outputs_ay * 0.258 / 1.22;

  // Gain: '<S17>/Gain' incorporates:
  //   Constant: '<S17>/car.L_Wheelbase1'
  //   Gain: '<S17>/car.L_F'
  //   Product: '<S17>/Divide'
  //   Product: '<S17>/Divide1'
  //   Product: '<S17>/Divide2'
  //   Product: '<S17>/Divide3'
  //   Sum: '<S17>/Add'
  //   Sum: '<S17>/Add1'

  rtb_Product_lu =
      ((((rtb_Product + Model_Lateral_Pacejka_Co_ConstB.carg_e) * 0.765 / 1.53 + rtb_Product_p) + rtb_Subtract_c) -
       rtb_Switch) *
      0.5;

  // Sum: '<S21>/Subtract' incorporates:
  //   Gain: '<S21>/car.mu_Roll'
  //   Inport generated from: '<Root>/In Bus Element3'

  rtb_Subtract = Model_Lateral_Pacejka_Codegen_U.Inputs_FRL - 0.01 * rtb_Product_lu;

  // Switch: '<S21>/Switch1' incorporates:
  //   Inport generated from: '<Root>/In Bus Element3'

  if (Model_Lateral_Pacejka_Codegen_U.Inputs_FRL >= 0.0) {
    // Switch: '<S21>/Switch' incorporates:
    //   Constant: '<S21>/Constant'

    if (!(rtb_Subtract > 0.0)) {
      rtb_Subtract = 0.0;
    }

    // End of Switch: '<S21>/Switch'
  }

  // End of Switch: '<S21>/Switch1'

  // Gain: '<S18>/Gain' incorporates:
  //   Constant: '<S18>/car.L_Wheelbase1'
  //   Gain: '<S18>/car.L_F'
  //   Product: '<S18>/Divide1'
  //   Sum: '<S18>/Add'
  //   Sum: '<S18>/Add1'

  rtb_Product_lk =
      ((((rtb_Product + Model_Lateral_Pacejka_Co_ConstB.carg_o) * 0.765 / 1.53 + rtb_Product_p) + rtb_Subtract_c) +
       rtb_Switch) *
      0.5;

  // Sum: '<S22>/Subtract' incorporates:
  //   Gain: '<S22>/car.mu_Roll'
  //   Inport generated from: '<Root>/In Bus Element4'

  rtb_Subtract_a = Model_Lateral_Pacejka_Codegen_U.Inputs_FRR - 0.01 * rtb_Product_lk;

  // Switch: '<S22>/Switch1' incorporates:
  //   Inport generated from: '<Root>/In Bus Element4'

  if (Model_Lateral_Pacejka_Codegen_U.Inputs_FRR >= 0.0) {
    // Switch: '<S22>/Switch' incorporates:
    //   Constant: '<S22>/Constant'

    if (!(rtb_Subtract_a > 0.0)) {
      rtb_Subtract_a = 0.0;
    }

    // End of Switch: '<S22>/Switch'
  }

  // End of Switch: '<S22>/Switch1'

  // Gain: '<S15>/Gain' incorporates:
  //   Constant: '<S15>/car.L_Wheelbase1'
  //   Gain: '<S15>/car.L_R'
  //   Product: '<S15>/Divide1'
  //   Sum: '<S15>/Add'
  //   Sum: '<S15>/Add1'

  rtb_Product_iu =
      ((((rtb_Product + Model_Lateral_Pacejka_Co_ConstB.carg) * 0.765 / 1.53 - rtb_Product_p) - rtb_Subtract_c) -
       rtb_Switch) *
      0.5;

  // Sum: '<S19>/Subtract' incorporates:
  //   Gain: '<S19>/car.mu_Roll'
  //   Inport generated from: '<Root>/In Bus Element1'

  rtb_Subtract_f = Model_Lateral_Pacejka_Codegen_U.Inputs_FFL - 0.01 * rtb_Product_iu;

  // Switch: '<S19>/Switch1' incorporates:
  //   Inport generated from: '<Root>/In Bus Element1'

  if (Model_Lateral_Pacejka_Codegen_U.Inputs_FFL >= 0.0) {
    // Switch: '<S19>/Switch' incorporates:
    //   Constant: '<S19>/Constant'

    if (!(rtb_Subtract_f > 0.0)) {
      rtb_Subtract_f = 0.0;
    }

    // End of Switch: '<S19>/Switch'
  }

  // End of Switch: '<S19>/Switch1'

  // Gain: '<S16>/Gain' incorporates:
  //   Constant: '<S16>/car.L_Wheelbase1'
  //   Gain: '<S16>/car.L_R'
  //   Product: '<S16>/Divide1'
  //   Sum: '<S16>/Add'
  //   Sum: '<S16>/Add1'

  rtb_Product_p =
      ((((rtb_Product + Model_Lateral_Pacejka_Co_ConstB.carg_n) * 0.765 / 1.53 - rtb_Product_p) - rtb_Subtract_c) +
       rtb_Switch) *
      0.5;

  // Sum: '<S20>/Subtract' incorporates:
  //   Gain: '<S20>/car.mu_Roll'
  //   Inport generated from: '<Root>/In Bus Element2'

  rtb_Subtract_c = Model_Lateral_Pacejka_Codegen_U.Inputs_FFR - 0.01 * rtb_Product_p;

  // Switch: '<S20>/Switch1' incorporates:
  //   Inport generated from: '<Root>/In Bus Element2'

  if (Model_Lateral_Pacejka_Codegen_U.Inputs_FFR >= 0.0) {
    // Switch: '<S20>/Switch' incorporates:
    //   Constant: '<S20>/Constant'

    if (!(rtb_Subtract_c > 0.0)) {
      rtb_Subtract_c = 0.0;
    }

    // End of Switch: '<S20>/Switch'
  }

  // End of Switch: '<S20>/Switch1'

  // Switch: '<S24>/Switch' incorporates:
  //   Constant: '<S24>/constant'
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Delay: '<Root>/Delay2'
  //   Gain: '<S24>/car.L_F'
  //   Inport generated from: '<Root>/In Bus Element'
  //   Product: '<S24>/Divide'
  //   Sum: '<S24>/Subtract'
  //   Sum: '<S24>/Sum'
  //   Trigonometry: '<S24>/Atan'

  if (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx != 0.0) {
    rtb_Switch = Model_Lateral_Pacejka_Codegen_U.Inputs_delta -
                 std::atan((0.765 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot +
                            Model_Lateral_Pacejka_Codegen_Y.Outputs_vy) /
                           Model_Lateral_Pacejka_Codegen_Y.Outputs_vx);
  } else {
    rtb_Switch = 0.0;
  }

  // End of Switch: '<S24>/Switch'

  // Switch: '<S23>/Switch' incorporates:
  //   Constant: '<S23>/Constant'
  //   Gain: '<S23>/tire.Bf'
  //   Gain: '<S23>/tire.Cf'
  //   Gain: '<S23>/tire.Df'
  //   Gain: '<S23>/tire.T_peak_front'
  //   Gain: '<S23>/tire.T_slope_front'
  //   Sum: '<S23>/Sum'
  //   Trigonometry: '<S23>/Atan'
  //   Trigonometry: '<S23>/Sin'

  if (rtb_Switch != 0.0) {
    tmp = std::sin(std::atan(60.3197 * rtb_Switch * 0.4) * 0.9767) * 3.3663 * 0.37;
  } else {
    tmp = 0.0;
  }

  // Product: '<S19>/Product' incorporates:
  //   Switch: '<S23>/Switch'

  rtb_Product_iu *= tmp;

  // Switch: '<S26>/Switch' incorporates:
  //   Constant: '<S26>/constant'
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Delay: '<Root>/Delay2'
  //   Gain: '<S26>/car.L_F'
  //   Inport generated from: '<Root>/In Bus Element'
  //   Product: '<S26>/Divide'
  //   Sum: '<S26>/Subtract'
  //   Sum: '<S26>/Sum'
  //   Trigonometry: '<S26>/Atan'

  if (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx != 0.0) {
    rtb_Switch = Model_Lateral_Pacejka_Codegen_U.Inputs_delta -
                 std::atan((0.765 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot +
                            Model_Lateral_Pacejka_Codegen_Y.Outputs_vy) /
                           Model_Lateral_Pacejka_Codegen_Y.Outputs_vx);
  } else {
    rtb_Switch = 0.0;
  }

  // End of Switch: '<S26>/Switch'

  // Switch: '<S25>/Switch' incorporates:
  //   Constant: '<S25>/Constant'
  //   Gain: '<S25>/tire.Bf'
  //   Gain: '<S25>/tire.Cf'
  //   Gain: '<S25>/tire.Df'
  //   Gain: '<S25>/tire.T_peak_front'
  //   Gain: '<S25>/tire.T_slope_front'
  //   Sum: '<S25>/Sum'
  //   Trigonometry: '<S25>/Atan'
  //   Trigonometry: '<S25>/Sin'

  if (rtb_Switch != 0.0) {
    tmp = std::sin(std::atan(60.3197 * rtb_Switch * 0.4) * 0.9767) * 3.3663 * 0.37;
  } else {
    tmp = 0.0;
  }

  // Product: '<S20>/Product' incorporates:
  //   Switch: '<S25>/Switch'

  rtb_Product_p *= tmp;

  // Trigonometry: '<S10>/Cos' incorporates:
  //   Inport generated from: '<Root>/In Bus Element'
  //   Trigonometry: '<S12>/Cos'
  //   Trigonometry: '<S14>/Cos'

  rtb_Switch = std::cos(Model_Lateral_Pacejka_Codegen_U.Inputs_delta);

  // Trigonometry: '<S10>/Cos1' incorporates:
  //   Inport generated from: '<Root>/In Bus Element'
  //   Trigonometry: '<S12>/Cos1'
  //   Trigonometry: '<S13>/Cos2'

  Delay3_DSTATE_tmp = std::sin(Model_Lateral_Pacejka_Codegen_U.Inputs_delta);

  // Switch: '<S7>/Switch' incorporates:
  //   Constant: '<S7>/Constant'
  //   Delay: '<Root>/Delay'
  //   Gain: '<S7>/Gain'
  //   Gain: '<S7>/car.mu_Roll'
  //   Sum: '<S7>/Sum'

  if (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx != 0.0) {
    tmp = (rtb_Product + Model_Lateral_Pacejka_Co_ConstB.Product) * 0.01 * 0.0;
  } else {
    tmp = 0.0;
  }

  // Sum: '<S10>/Sum2' incorporates:
  //   Sum: '<S12>/Sum1'

  Outputs_ax_tmp = rtb_Product_iu + rtb_Product_p;

  // Sum: '<S10>/Sum1' incorporates:
  //   Sum: '<S12>/Sum2'

  rtb_Product = rtb_Subtract_f + rtb_Subtract_c;

  // Product: '<S10>/Divide' incorporates:
  //   Constant: '<S10>/car.m_Vehicle_total'
  //   Delay: '<Root>/Delay3'
  //   Product: '<S10>/Product'
  //   Product: '<S10>/Product1'
  //   Sum: '<S10>/Sum'
  //   Sum: '<S10>/Sum1'
  //   Sum: '<S10>/Sum2'
  //   Sum: '<S10>/Sum3'
  //   Switch: '<S7>/Switch'
  //   Trigonometry: '<S10>/Cos'
  //   Trigonometry: '<S10>/Cos1'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_ax =
      ((((rtb_Product * rtb_Switch + (rtb_Subtract + rtb_Subtract_a)) - Outputs_ax_tmp * Delay3_DSTATE_tmp) -
        rtb_Product_c) -
       tmp) /
      176.0;

  // Switch: '<S28>/Switch' incorporates:
  //   Constant: '<S28>/constant'
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Delay: '<Root>/Delay2'
  //   Gain: '<S28>/Gain'
  //   Gain: '<S28>/car.L_R'
  //   Product: '<S28>/Divide'
  //   Sum: '<S28>/Sum'
  //   Trigonometry: '<S28>/Atan'

  if (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx != 0.0) {
    rtb_Product_c = -std::atan(
        (Model_Lateral_Pacejka_Codegen_Y.Outputs_vy - 0.765 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot) /
        Model_Lateral_Pacejka_Codegen_Y.Outputs_vx);
  } else {
    rtb_Product_c = 0.0;
  }

  // End of Switch: '<S28>/Switch'

  // Switch: '<S27>/Switch' incorporates:
  //   Constant: '<S27>/Constant'
  //   Gain: '<S27>/tire.Br'
  //   Gain: '<S27>/tire.Cr'
  //   Gain: '<S27>/tire.Dr'
  //   Gain: '<S27>/tire.T_peak_rear'
  //   Gain: '<S27>/tire.T_slope_rear'
  //   Sum: '<S27>/Sum'
  //   Trigonometry: '<S27>/Atan'
  //   Trigonometry: '<S27>/Sin'

  if (rtb_Product_c != 0.0) {
    tmp = std::sin(std::atan(60.3197 * rtb_Product_c * 0.4) * 0.9767) * 3.3663 * 0.37;
  } else {
    tmp = 0.0;
  }

  // Product: '<S21>/Product' incorporates:
  //   Switch: '<S27>/Switch'

  rtb_Product_lu *= tmp;

  // Switch: '<S30>/Switch' incorporates:
  //   Constant: '<S30>/constant'
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Delay: '<Root>/Delay2'
  //   Gain: '<S30>/Gain'
  //   Gain: '<S30>/car.L_R'
  //   Product: '<S30>/Divide'
  //   Sum: '<S30>/Sum'
  //   Trigonometry: '<S30>/Atan'

  if (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx != 0.0) {
    rtb_Product_c = -std::atan(
        (Model_Lateral_Pacejka_Codegen_Y.Outputs_vy - 0.765 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot) /
        Model_Lateral_Pacejka_Codegen_Y.Outputs_vx);
  } else {
    rtb_Product_c = 0.0;
  }

  // End of Switch: '<S30>/Switch'

  // Switch: '<S29>/Switch' incorporates:
  //   Constant: '<S29>/Constant'
  //   Gain: '<S29>/tire.Br'
  //   Gain: '<S29>/tire.Cr'
  //   Gain: '<S29>/tire.Dr'
  //   Gain: '<S29>/tire.T_peak_rear'
  //   Gain: '<S29>/tire.T_slope_rear'
  //   Sum: '<S29>/Sum'
  //   Trigonometry: '<S29>/Atan'
  //   Trigonometry: '<S29>/Sin'

  if (rtb_Product_c != 0.0) {
    tmp = std::sin(std::atan(60.3197 * rtb_Product_c * 0.4) * 0.9767) * 3.3663 * 0.37;
  } else {
    tmp = 0.0;
  }

  // Product: '<S12>/Product' incorporates:
  //   Product: '<S13>/Product'

  rtb_Product_c = Outputs_ax_tmp * rtb_Switch;

  // Product: '<S12>/Product1' incorporates:
  //   Product: '<S13>/Product1'

  rtb_Product *= Delay3_DSTATE_tmp;

  // Sum: '<S12>/Sum' incorporates:
  //   Product: '<S22>/Product'
  //   Sum: '<S13>/Add3'
  //   Switch: '<S29>/Switch'

  rtb_Product_lu += rtb_Product_lk * tmp;

  // Product: '<S12>/Divide' incorporates:
  //   Constant: '<S12>/car.m_Vehicle_total'
  //   Delay: '<Root>/Delay4'
  //   Product: '<S12>/Product'
  //   Product: '<S12>/Product1'
  //   Sum: '<S12>/Sum'
  //   Sum: '<S12>/Sum3'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_ay = ((rtb_Product_c + rtb_Product_lu) + rtb_Product) / 176.0;

  // Product: '<S13>/Divide' incorporates:
  //   Constant: '<S13>/car.I_Z_Vehicle'
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/car.L_F'
  //   Gain: '<S13>/car.L_F1'
  //   Gain: '<S13>/car.L_R'
  //   Gain: '<S13>/car.L_Trackwidth_F'
  //   Gain: '<S14>/Gain'
  //   Gain: '<S14>/car.L_Trackwidth_F'
  //   Gain: '<S14>/car.L_Trackwidth_R'
  //   Product: '<S13>/Product2'
  //   Product: '<S14>/Product'
  //   Sum: '<S13>/Add2'
  //   Sum: '<S13>/Add4'
  //   Sum: '<S14>/Add'
  //   Sum: '<S14>/Add1'
  //   Sum: '<S14>/Add2'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot_dot =
      ((((rtb_Product_iu - rtb_Product_p) * Delay3_DSTATE_tmp * 1.22 * 0.5 +
         (rtb_Product_c * 0.765 + rtb_Product * 0.765)) -
        rtb_Product_lu * 0.765) +
       ((rtb_Subtract_c - rtb_Subtract_f) * rtb_Switch * 1.22 + (rtb_Subtract_a - rtb_Subtract) * 1.22) * 0.5) /
      190.69;

  // Outport generated from: '<Root>/Out Bus Element2' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_psi = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE;

  // DiscreteIntegrator: '<S10>/Discrete-Time Integrator' incorporates:
  //   Delay: '<Root>/Delay'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_vx = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_m;

  // Trigonometry: '<S9>/Cos' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  //   Trigonometry: '<S11>/cos'

  rtb_Subtract = std::cos(Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE);

  // DiscreteIntegrator: '<S12>/Discrete-Time Integrator' incorporates:
  //   Delay: '<Root>/Delay1'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_vy = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_b;

  // Trigonometry: '<S9>/sin' incorporates:
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  //   Trigonometry: '<S11>/sin'

  rtb_Subtract_a = std::sin(Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE);

  // Outport generated from: '<Root>/Out Bus Element' incorporates:
  //   DiscreteIntegrator: '<S9>/Discrete-Time Integrator'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_px = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_p;

  // Outport generated from: '<Root>/Out Bus Element1' incorporates:
  //   DiscreteIntegrator: '<S11>/Discrete-Time Integrator'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_py = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTAT_pu;

  // DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Delay: '<Root>/Delay2'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot = Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_g;

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  // incorporates:
  //   Delay: '<Root>/Delay2'

  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE +=
      0.005 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot;

  // Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator'
  // incorporates:
  //   Delay: '<Root>/Delay3'

  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_m += 0.005 * Model_Lateral_Pacejka_Codegen_Y.Outputs_ax;

  // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator'
  // incorporates:
  //   Delay: '<Root>/Delay4'

  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_b += 0.005 * Model_Lateral_Pacejka_Codegen_Y.Outputs_ay;

  // Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
  // incorporates:
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Product: '<S9>/Product'
  //   Product: '<S9>/Product1'
  //   Sum: '<S9>/Subtract'
  //   Trigonometry: '<S9>/Cos'
  //   Trigonometry: '<S9>/sin'

  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_p +=
      (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx * rtb_Subtract -
       Model_Lateral_Pacejka_Codegen_Y.Outputs_vy * rtb_Subtract_a) *
      0.005;

  // Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator'
  // incorporates:
  //   Delay: '<Root>/Delay'
  //   Delay: '<Root>/Delay1'
  //   Product: '<S11>/Product'
  //   Product: '<S11>/Product1'
  //   Sum: '<S11>/Subtract'

  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTAT_pu +=
      (Model_Lateral_Pacejka_Codegen_Y.Outputs_vx * rtb_Subtract_a +
       Model_Lateral_Pacejka_Codegen_Y.Outputs_vy * rtb_Subtract) *
      0.005;

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_g +=
      0.005 * Model_Lateral_Pacejka_Codegen_Y.Outputs_psi_dot_dot;

  // Outport generated from: '<Root>/Out Bus Element9' incorporates:
  //   Inport generated from: '<Root>/In Bus Element'

  Model_Lateral_Pacejka_Codegen_Y.Outputs_delta = Model_Lateral_Pacejka_Codegen_U.Inputs_delta;
}

// Model initialize function
void Model_Lateral_Pacejka_Codegen::initialize() {
  // InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE = Model_Lateral_Pacejka_Co_ConstB.Constant;

  // InitializeConditions for DiscreteIntegrator: '<S10>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_m = Model_Lateral_Pacejka_Co_ConstB.carvx0;

  // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_b = Model_Lateral_Pacejka_Co_ConstB.carvy0;

  // InitializeConditions for DiscreteIntegrator: '<S9>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_p = Model_Lateral_Pacejka_Co_ConstB.Constant_l;

  // InitializeConditions for DiscreteIntegrator: '<S11>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTAT_pu = Model_Lateral_Pacejka_Co_ConstB.Constant_i;

  // InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time
  // Integrator'
  Model_Lateral_Pacejka_Codege_DW.DiscreteTimeIntegrator_DSTATE_g = Model_Lateral_Pacejka_Co_ConstB.Constant_c;
}

// Model terminate function
void Model_Lateral_Pacejka_Codegen::terminate() {
  // (no terminate code required)
}

// Constructor
Model_Lateral_Pacejka_Codegen::Model_Lateral_Pacejka_Codegen()
    : Model_Lateral_Pacejka_Codegen_U(), Model_Lateral_Pacejka_Codegen_Y(), Model_Lateral_Pacejka_Codege_DW(),
      Model_Lateral_Pacejka_Codege_M() {
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
Model_Lateral_Pacejka_Codegen::~Model_Lateral_Pacejka_Codegen() = default;

// Real-Time Model get method
Model_Lateral_Pacejka_Codegen::RT_MODEL_Model_Lateral_Pacejk_T *Model_Lateral_Pacejka_Codegen::getRTM() {
  return (&Model_Lateral_Pacejka_Codege_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
