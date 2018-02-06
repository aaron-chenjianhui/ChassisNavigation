//

// File: StateMachine.h
//
// Code generated for Simulink model 'StateMachine'.
//
// Model version                  : 1.18
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Feb  6 13:44:39 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_StateMachine_h_
# define RTW_HEADER_StateMachine_h_
# include <string.h>
# include <stddef.h>
# ifndef StateMachine_COMMON_INCLUDES_
#  define StateMachine_COMMON_INCLUDES_
#  include "tcp_driver/rtwtypes.h"
# endif // StateMachine_COMMON_INCLUDES_

# include "tcp_driver/StateMachine_types.h"

// Macros for accessing real-time model data structure
# ifndef rtmGetErrorStatus
#  define rtmGetErrorStatus(rtm) ((rtm)->errorStatus)
# endif // ifndef rtmGetErrorStatus

# ifndef rtmSetErrorStatus
#  define rtmSetErrorStatus(rtm, val) ((rtm)->errorStatus = (val))
# endif // ifndef rtmSetErrorStatus

// Block states (auto storage) for system '<Root>'
typedef struct {
  uint8_T is_active_c3_StateMachine; // '<Root>/StateMachine'
  uint8_T is_c3_StateMachine;        // '<Root>/StateMachine'
} DW_StateMachine_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  uint8_T lidar_state;       // '<Root>/lidar_state'
  uint8_T tcp_state;         // '<Root>/tcp_state'
  uint8_T detect_node_state; // '<Root>/detect_node_state'
  uint8_T nav_node_state;    // '<Root>/nav_node_state'
  uint8_T recv_msg_state;    // '<Root>/recv_msg_state'
} ExtU_StateMachine_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  uint8_T sys_state; // '<Root>/sys_state'
} ExtY_StateMachine_T;

// Real-time Model Data Structure
struct tag_RTM_StateMachine_T {
  const char_T *errorStatus;
};

// Block states (auto storage)
extern DW_StateMachine_T StateMachine_DW;

# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

// External inputs (root inport signals with auto storage)
extern ExtU_StateMachine_T StateMachine_U;

// External outputs (root outports fed by signals with auto storage)
extern ExtY_StateMachine_T StateMachine_Y;

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

// Model entry point functions
extern void StateMachine_initialize(void);
extern void StateMachine_step(void);
extern void StateMachine_terminate(void);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

// Real-time Model object
# ifdef __cplusplus

extern "C" {
# endif // ifdef __cplusplus

extern RT_MODEL_StateMachine_T *const StateMachine_M;

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
//  hilite_system('laser_nav_statemachine/StateMachine')    - opens subsystem
// laser_nav_statemachine/StateMachine
//  hilite_system('laser_nav_statemachine/StateMachine/Kp') - opens and selects
// block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'laser_nav_statemachine'
//  '<S1>'   : 'laser_nav_statemachine/StateMachine'

#endif // RTW_HEADER_StateMachine_h_

//
// File trailer for generated code.
//
// [EOF]
//
