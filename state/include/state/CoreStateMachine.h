//
// File: CoreStateMachine.h
//
// Code generated for Simulink model 'CoreStateMachine'.
//
// Model version                  : 1.71
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Jul 25 08:50:57 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_CoreStateMachine_h_
#define RTW_HEADER_CoreStateMachine_h_
#include <string.h>
#include <stddef.h>
#ifndef CoreStateMachine_COMMON_INCLUDES_
# define CoreStateMachine_COMMON_INCLUDES_
#include "state/rtwtypes.h"
#endif                                 // CoreStateMachine_COMMON_INCLUDES_

#include "state/CoreStateMachine_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (auto storage) for system '<Root>'
typedef struct {
	uint16_T	cal_count;                      // '<S2>/DetectState'
	uint16_T	init_count;                     // '<S2>/AMCL_Interface'
	uint16_T	send_count;                     // '<S2>/AMCL_Interface'
	uint16_T	update_count;                   // '<S2>/AMCL_Interface'
	uint8_T		is_active_c1_CoreStateMachine;  // '<S1>/StateMachine'
	uint8_T		is_c1_CoreStateMachine;         // '<S1>/StateMachine'
	uint8_T		is_active_c2_CoreStateMachine;  // '<S1>/NavState'
	uint8_T		is_c2_CoreStateMachine;         // '<S1>/NavState'
	uint8_T		is_active_c3_CoreStateMachine;  // '<S2>/DetectState'
	uint8_T		is_c3_CoreStateMachine;         // '<S2>/DetectState'
	uint8_T		is_active_c4_CoreStateMachine;  // '<S2>/AMCL_Interface'
	uint8_T		is_c4_CoreStateMachine;         // '<S2>/AMCL_Interface'
} DW_CoreStateMachine_T;

// External inputs (root inport signals with auto storage)
typedef struct {
	boolean_T	is_calcd;       // '<Root>/is_calcd'
	uint16_T	filter_count;   // '<Root>/filter_count'
	boolean_T	detect_conn;    // '<Root>/detect_conn'
	boolean_T	nav_conn;       // '<Root>/nav_conn'
	boolean_T	lidar_conn;     // '<Root>/lidar_conn'
	boolean_T	tcp_conn;       // '<Root>/tcp_conn'
	boolean_T	amcl_conn;      // '<Root>/amcl_conn'
	boolean_T	cov_is_small;   // '<Root>/cov_is_small'
	uint8_T		recv_msg;       // '<Root>/recv_msg'
} ExtU_CoreStateMachine_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
	uint8_T		sys_status;     // '<Root>/sys_status'
	boolean_T	calc_request;   // '<Root>/calc_request'
	uint8_T		detect_status;  // '<Root>/detect_status'
	uint8_T		nav_status;     // '<Root>/nav_status'
	uint8_T		amcl_status;    // '<Root>/amcl_status'
} ExtY_CoreStateMachine_T;

// Parameters (auto storage)
struct P_CoreStateMachine_T_ {
	uint16_T	counter_times;  // Variable: counter_times
	                                //  Referenced by: '<S2>/DetectState'

	uint8_T		Memory2_X0;     // Computed Parameter: Memory2_X0
	                                //  Referenced by: '<S1>/Memory2'

	uint8_T		Memory1_X0;     // Computed Parameter: Memory1_X0
	                                //  Referenced by: '<S1>/Memory1'
};

// Real-time Model Data Structure
struct tag_RTM_CoreStateMachine_T {
	const char_T *errorStatus;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {
#endif

extern P_CoreStateMachine_T CoreStateMachine_P;

#ifdef __cplusplus
}
#endif

// Block states (auto storage)
extern DW_CoreStateMachine_T CoreStateMachine_DW;

#ifdef __cplusplus

extern "C" {
#endif

// External inputs (root inport signals with auto storage)
extern ExtU_CoreStateMachine_T CoreStateMachine_U;

// External outputs (root outports fed by signals with auto storage)
extern ExtY_CoreStateMachine_T CoreStateMachine_Y;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

extern "C" {
#endif

// Model entry point functions
extern void CoreStateMachine_initialize(void);
extern void CoreStateMachine_step(void);
extern void CoreStateMachine_terminate(void);

#ifdef __cplusplus
}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {
#endif

extern RT_MODEL_CoreStateMachine_T *const CoreStateMachine_M;

#ifdef __cplusplus
}
#endif

//-
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
//  hilite_system('SysStateMachine/CoreStateMachine')    - opens subsystem SysStateMachine/CoreStateMachine
//  hilite_system('SysStateMachine/CoreStateMachine/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'SysStateMachine'
//  '<S1>'   : 'SysStateMachine/CoreStateMachine'
//  '<S2>'   : 'SysStateMachine/CoreStateMachine/Global Location'
//  '<S3>'   : 'SysStateMachine/CoreStateMachine/NavState'
//  '<S4>'   : 'SysStateMachine/CoreStateMachine/StateMachine'
//  '<S5>'   : 'SysStateMachine/CoreStateMachine/Global Location/AMCL_Interface'
//  '<S6>'   : 'SysStateMachine/CoreStateMachine/Global Location/DetectState'

#endif                                 // RTW_HEADER_CoreStateMachine_h_

//
// File trailer for generated code.
//
// [EOF]
//
