//

// File: CoreStateMachine.h
//
// Code generated for Simulink model 'CoreStateMachine'.
//
// Model version                  : 1.37
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Feb 08 13:43:24 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_CoreStateMachine_h_
# define RTW_HEADER_CoreStateMachine_h_
# include <string.h>
# include <stddef.h>
# ifndef CoreStateMachine_COMMON_INCLUDES_
#  define CoreStateMachine_COMMON_INCLUDES_
#  include "state/rtwtypes.h"
# endif // CoreStateMachine_COMMON_INCLUDES_

# include "state/CoreStateMachine_types.h"

// Macros for accessing real-time model data structure
# ifndef rtmGetErrorStatus
#  define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
# endif // ifndef rtmGetErrorStatus

# ifndef rtmSetErrorStatus
#  define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
# endif // ifndef rtmSetErrorStatus

// Block states (auto storage) for system '<Root>'
typedef struct {
  uint16_T cal_count;                     // '<S1>/DetectState'
  uint8_T  is_active_c1_CoreStateMachine; // '<S1>/StateMachine'
  uint8_T  is_c1_CoreStateMachine;        // '<S1>/StateMachine'
  uint8_T  is_active_c2_CoreStateMachine; // '<S1>/NavState'
  uint8_T  is_c2_CoreStateMachine;        // '<S1>/NavState'
  uint8_T  is_active_c3_CoreStateMachine; // '<S1>/DetectState'
  uint8_T  is_c3_CoreStateMachine;        // '<S1>/DetectState'
} DW_CoreStateMachine_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  boolean_T detect_flag_input; // '<Root>/detect_flag_input'
  uint16_T  filter_count;      // '<Root>/filter_count'
  boolean_T detect_conn;       // '<Root>/detect_conn'
  boolean_T nav_conn;          // '<Root>/nav_conn'
  boolean_T lidar_conn;        // '<Root>/lidar_conn'
  boolean_T tcp_conn;          // '<Root>/tcp_conn'
  uint8_T   recv_msg;          // '<Root>/recv_msg'
} ExtU_CoreStateMachine_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  uint8_T   sys_status;         // '<Root>/sys_status'
  boolean_T detect_flag_output; // '<Root>/detect_flag_output'
  uint8_T   detect_status;      // '<Root>/detect_status'
  uint8_T   nav_status;         // '<Root>/nav_status'
} ExtY_CoreStateMachine_T;

// Parameters (auto storage)
struct P_CoreStateMachine_T_ {
  uint16_T counter_times; // Variable: counter_times
                          //  Referenced by: '<S1>/DetectState'

  uint8_T Memory2_X0;     // Computed Parameter: Memory2_X0
                          //  Referenced by: '<S1>/Memory2'

  uint8_T Memory1_X0;     // Computed Parameter: Memory1_X0
                          //  Referenced by: '<S1>/Memory1'
};

// Real-time Model Data Structure
struct tag_RTM_CoreStateMachine_T {
  const char_T *errorStatus;
};

// Block parameters (auto storage)
# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

extern P_CoreStateMachine_T CoreStateMachine_P;

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

// Block states (auto storage)
extern DW_CoreStateMachine_T CoreStateMachine_DW;

# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

// External inputs (root inport signals with auto storage)
extern ExtU_CoreStateMachine_T CoreStateMachine_U;

// External outputs (root outports fed by signals with auto storage)
extern ExtY_CoreStateMachine_T CoreStateMachine_Y;

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

// Model entry point functions
extern void CoreStateMachine_initialize(void);
extern void CoreStateMachine_step(void);
extern void CoreStateMachine_terminate(void);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

// Real-time Model object
# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

extern RT_MODEL_CoreStateMachine_T *const CoreStateMachine_M;

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

// -
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('SysStateMachine/CoreStateMachine')    - opens subsystem
// SysStateMachine/CoreStateMachine
//  hilite_system('SysStateMachine/CoreStateMachine/Kp') - opens and selects
// block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'SysStateMachine'
//  '<S1>'   : 'SysStateMachine/CoreStateMachine'
//  '<S2>'   : 'SysStateMachine/CoreStateMachine/DetectState'
//  '<S3>'   : 'SysStateMachine/CoreStateMachine/NavState'
//  '<S4>'   : 'SysStateMachine/CoreStateMachine/StateMachine'

#endif // RTW_HEADER_CoreStateMachine_h_

//
// File trailer for generated code.
//
// [EOF]
//
