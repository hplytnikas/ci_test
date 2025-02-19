//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Model_Lateral_Pacejka_Codegen.h
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
#ifndef RTW_HEADER_Model_Lateral_Pacejka_Codegen_h_
#define RTW_HEADER_Model_Lateral_Pacejka_Codegen_h_
#include "Model_Lateral_Pacejka_Codegen_types.h"
#include "rtwtypes.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
#endif

// Class declaration for model Model_Lateral_Pacejka_Codegen
class Model_Lateral_Pacejka_Codegen final {
  // public data and function members
public:
  // Block states (default storage) for system '<Root>'
  struct DW_Model_Lateral_Pacejka_Code_T {
    real_T DiscreteTimeIntegrator_DSTATE;   // '<S8>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator_DSTATE_m; // '<S10>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator_DSTATE_b; // '<S12>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator_DSTATE_p; // '<S9>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator_DSTAT_pu; // '<S11>/Discrete-Time Integrator'
    real_T DiscreteTimeIntegrator_DSTATE_g; // '<S13>/Discrete-Time Integrator'
  };

  // Invariant block signals (default storage)
  struct ConstB_Model_Lateral_Pacejka__T {
    real_T Product;    // '<S7>/Product'
    real_T Constant;   // '<S8>/Constant'
    real_T Constant_l; // '<S9>/Constant'
    real_T carvx0;     // '<S10>/car.vx0'
    real_T Constant_i; // '<S11>/Constant'
    real_T carvy0;     // '<S12>/car.vy0'
    real_T Constant_c; // '<S13>/Constant'
    real_T carg;       // '<S15>/car.g'
    real_T carg_n;     // '<S16>/car.g'
    real_T carg_e;     // '<S17>/car.g'
    real_T carg_o;     // '<S18>/car.g'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_Model_Lateral_Pacejka_Co_T {
    real_T Inputs_delta; // '<Root>/Inputs_delta'
    real_T Inputs_FFL;   // '<Root>/Inputs_FFL'
    real_T Inputs_FFR;   // '<Root>/Inputs_FFR'
    real_T Inputs_FRL;   // '<Root>/Inputs_FRL'
    real_T Inputs_FRR;   // '<Root>/Inputs_FRR'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_Model_Lateral_Pacejka_Co_T {
    real_T Outputs_px;          // '<Root>/Outputs_px'
    real_T Outputs_py;          // '<Root>/Outputs_py'
    real_T Outputs_psi;         // '<Root>/Outputs_psi'
    real_T Outputs_vx;          // '<Root>/Outputs_vx'
    real_T Outputs_vy;          // '<Root>/Outputs_vy'
    real_T Outputs_psi_dot;     // '<Root>/Outputs_psi_dot'
    real_T Outputs_ax;          // '<Root>/Outputs_ax'
    real_T Outputs_ay;          // '<Root>/Outputs_ay'
    real_T Outputs_psi_dot_dot; // '<Root>/Outputs_psi_dot_dot'
    real_T Outputs_delta;       // '<Root>/Outputs_delta'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_Model_Lateral_Pacejk_T {
    const char_T *volatile errorStatus;
  };

  // Copy Constructor
  Model_Lateral_Pacejka_Codegen(Model_Lateral_Pacejka_Codegen const &) = delete;

  // Assignment Operator
  Model_Lateral_Pacejka_Codegen &operator=(Model_Lateral_Pacejka_Codegen const &) & = delete;

  // Move Constructor
  Model_Lateral_Pacejka_Codegen(Model_Lateral_Pacejka_Codegen &&) = delete;

  // Move Assignment Operator
  Model_Lateral_Pacejka_Codegen &operator=(Model_Lateral_Pacejka_Codegen &&) = delete;

  // Real-Time Model get method
  Model_Lateral_Pacejka_Codegen::RT_MODEL_Model_Lateral_Pacejk_T *getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_Model_Lateral_Pacejka_Co_T *pExtU_Model_Lateral_Pacejka_Co_T) {
    Model_Lateral_Pacejka_Codegen_U = *pExtU_Model_Lateral_Pacejka_Co_T;
  }

  // Root outports get method
  const ExtY_Model_Lateral_Pacejka_Co_T &getExternalOutputs() const { return Model_Lateral_Pacejka_Codegen_Y; }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  Model_Lateral_Pacejka_Codegen();

  // Destructor
  ~Model_Lateral_Pacejka_Codegen();

  // private data and function members
private:
  // External inputs
  ExtU_Model_Lateral_Pacejka_Co_T Model_Lateral_Pacejka_Codegen_U;

  // External outputs
  ExtY_Model_Lateral_Pacejka_Co_T Model_Lateral_Pacejka_Codegen_Y;

  // Block states
  DW_Model_Lateral_Pacejka_Code_T Model_Lateral_Pacejka_Codege_DW;

  // Real-Time Model
  RT_MODEL_Model_Lateral_Pacejk_T Model_Lateral_Pacejka_Codege_M;
};

extern const Model_Lateral_Pacejka_Codegen::ConstB_Model_Lateral_Pacejka__T
    Model_Lateral_Pacejka_Co_ConstB; // constant block i/o

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S19>/Scope' : Unused code path elimination

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'Model_Lateral_Pacejka_Codegen'
//  '<S1>'   : 'Model_Lateral_Pacejka_Codegen/Body'
//  '<S2>'   : 'Model_Lateral_Pacejka_Codegen/States'
//  '<S3>'   : 'Model_Lateral_Pacejka_Codegen/Vertical'
//  '<S4>'   : 'Model_Lateral_Pacejka_Codegen/Wheels'
//  '<S5>'   : 'Model_Lateral_Pacejka_Codegen/Body/Drag'
//  '<S6>'   : 'Model_Lateral_Pacejka_Codegen/Body/Lift'
//  '<S7>'   : 'Model_Lateral_Pacejka_Codegen/Body/Rolling_Resistance'
//  '<S8>'   : 'Model_Lateral_Pacejka_Codegen/States/Subsystem'
//  '<S9>'   : 'Model_Lateral_Pacejka_Codegen/States/X'
//  '<S10>'  : 'Model_Lateral_Pacejka_Codegen/States/X_Rate'
//  '<S11>'  : 'Model_Lateral_Pacejka_Codegen/States/Y'
//  '<S12>'  : 'Model_Lateral_Pacejka_Codegen/States/Y_Rate'
//  '<S13>'  : 'Model_Lateral_Pacejka_Codegen/States/Yaw_Rate'
//  '<S14>'  : 'Model_Lateral_Pacejka_Codegen/States/Yaw_Rate/Torque_Vectoring'
//  '<S15>'  : 'Model_Lateral_Pacejka_Codegen/Vertical/FzFL'
//  '<S16>'  : 'Model_Lateral_Pacejka_Codegen/Vertical/FzFR'
//  '<S17>'  : 'Model_Lateral_Pacejka_Codegen/Vertical/FzRL'
//  '<S18>'  : 'Model_Lateral_Pacejka_Codegen/Vertical/FzRR'
//  '<S19>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FL'
//  '<S20>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FR'
//  '<S21>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RL'
//  '<S22>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RR'
//  '<S23>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FL/Pacejka mu'
//  '<S24>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FL/SA'
//  '<S25>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FR/Pacejka mu'
//  '<S26>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_FR/SA'
//  '<S27>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RL/Pacejka mu'
//  '<S28>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RL/SA'
//  '<S29>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RR/Pacejka mu'
//  '<S30>'  : 'Model_Lateral_Pacejka_Codegen/Wheels/Wheel_RR/SA'

#endif // RTW_HEADER_Model_Lateral_Pacejka_Codegen_h_

//
// File trailer for generated code.
//
// [EOF]
//
